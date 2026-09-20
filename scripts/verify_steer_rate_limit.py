"""验证 LQR（kinematic）高速振荡是"平台不施加 max_steer_rate"造成的伪现象。

背景
----
LQR kinematic 在 7 m/s 档出现转角在 ±0.61 rad 上逐拍等幅振荡（J_delta = 5.49，
是 medium 档的 10.9 倍）。归因排查否定了三个自然解释：

    - 线性化模型失稳：eig(A - BK) 的 max|lambda| 从 v=3 的 0.844 降到 v=7 的 0.706，
      极点始终在单位圆内且随速度更靠内；
    - 上一拍转角构成代数回路：回路增益 K3*v/L 在 v=7 时仅 0.655 < 1；
    - 参考路径重采样混叠：把 ds 从 0.5 扫到 0.05，振荡不消失。

真正的原因是 vdm_lab/common/vehicle.py 的 limit_command() 只 clamp 转角**大小**，
从不施加 max_steer_rate，于是控制器可以命令相邻两拍之间 0.9 rad 的转角跳变
（折合 9 rad/s，而车辆声明的上限是 0.7854 rad/s）。

本脚本在同一个控制器外面套一层**真实的转角速率限制**，跑完全相同的实验，
比较"加速度率限"前后的 J_delta / max_steer / 精度，并输出独立单图。

注意：本脚本**不修改** vdm_lab/common/vehicle.py —— 那一层包装只存在于本脚本内，
用来隔离平台缺陷的影响，平台本身保持原样。

用法：
    python scripts/verify_steer_rate_limit.py
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from vdm_lab.common.geometry import clamp  # noqa: E402
from vdm_lab.common.simulation import load_controller, run_simulation  # noqa: E402
from vdm_lab.common.types import ControlCommand, LabConfig  # noqa: E402

RADIUS = 12.0
TRIM = 15
SPEED_MODES = ("medium", "high")


class RateLimitedController:
    """把任意控制器包一层，强制 |Δsteer| <= max_steer_rate * dt。

    这层包装复现了真实执行器应有的行为：转角不能在一拍之内跳变。
    `previous_control` 是上一拍**已执行**（即已被本包装限制过）的指令，
    所以反馈回控制器 `e_yaw_dot` 的 `previous_control.steer` 也是受限值。
    """

    def __init__(self, base):
        self.base = base
        self.NAME = f"{getattr(base, 'NAME', 'controller')} + rate limit"

    def control(self, state, reference, previous_control, config):
        command = self.base.control(state, reference, previous_control, config)
        max_delta = config.vehicle.max_steer_rate * config.sim.dt
        steer = clamp(
            command.steer,
            previous_control.steer - max_delta,
            previous_control.steer + max_delta,
        )
        return ControlCommand(acceleration=command.acceleration, steer=steer)


def steady_slice(records):
    """取稳态圆弧段：先按曲率筛，再两端各剔除 TRIM 个点。"""
    curvature = np.array([r.curvature for r in records])
    inside = np.where(np.abs(curvature - 1.0 / RADIUS) < 0.005)[0]
    if len(inside) <= 2 * TRIM + 1:
        return records
    return records[inside[0] + TRIM: inside[-1] - TRIM + 1]


def summarise(records, dt, max_steer_rate):
    """J_delta、最大转角、稳态精度、转角速率越界比例。"""
    steer = np.array([r.steer for r in records])
    if len(steer) < 2:
        return None
    delta = np.diff(steer)
    rate_limit = max_steer_rate * dt

    segment = steady_slice(records)
    lateral = np.array([r.lateral_error for r in segment])

    # J_delta 用**稳态圆弧段**口径，与 analyze_circle_speed.py 的表 2 可直接比较。
    segment_steer = np.array([r.steer for r in segment])
    segment_delta = np.diff(segment_steer)
    J_delta_steady = (float(np.mean(np.abs(segment_delta) / dt))
                      if len(segment_delta) else float("nan"))

    return {
        "J_delta": float(np.mean(np.abs(delta) / dt)),
        "J_delta_steady": J_delta_steady,
        "max_dsteer": float(np.max(np.abs(delta))),
        "max_steer": float(np.max(np.abs(steer))),
        "violations": int(np.sum(np.abs(delta) > rate_limit)),
        "steps": int(len(delta)),
        "violation_ratio": float(np.mean(np.abs(delta) > rate_limit)),
        "steady_sigma": float(np.std(lateral)),
        "steady_max_abs": float(np.max(np.abs(lateral))),
        "steady_mean": float(np.mean(lateral)),
        "records": records,
    }


def run_case(speed_mode, use_rate_limit):
    base = load_controller("lqr_kinematic", "student")
    controller = RateLimitedController(base) if use_rate_limit else base

    config = LabConfig()
    config.sim.route_name = "circle"
    config.sim.speed_mode = speed_mode
    _, records, _ = run_simulation(controller, config=config)
    return summarise(records, config.sim.dt, config.vehicle.max_steer_rate)


def main():
    out_dir = REPO_ROOT / "outputs" / "analysis_steer_rate"
    out_dir.mkdir(parents=True, exist_ok=True)

    results = {}
    for speed_mode in SPEED_MODES:
        raw = run_case(speed_mode, use_rate_limit=False)
        limited = run_case(speed_mode, use_rate_limit=True)
        results[speed_mode] = {"raw": raw, "limited": limited}

        print(f"=== LQR kinematic / {speed_mode} ===")
        for label, key in [("原始（平台不施加速率限制）", "raw"),
                           ("加速度率限制后", "limited")]:
            data = results[speed_mode][key]
            print(f"  {label}: J_delta(稳态)={data['J_delta_steady']:.4f}  "
                  f"J_delta(全程)={data['J_delta']:.4f}  "
                  f"max|steer|={data['max_steer']:.4f}  "
                  f"max|Δsteer|={data['max_dsteer']:.4f}  "
                  f"越界={data['violations']}/{data['steps']} "
                  f"({data['violation_ratio']*100:.1f}%)  "
                  f"稳态σ={data['steady_sigma']:.4f}  "
                  f"稳态max|e|={data['steady_max_abs']:.4f}")
        print()

    # ---------------- 表格 ----------------
    lines = [
        "# LQR kinematic：转角速率限制前后的对比",
        "",
        f"车辆声明的最大转角速率 `max_steer_rate = 45°/s = {math.radians(45.0):.4f} rad/s`，",
        f"仿真步长 `dt = 0.1 s`，因此单拍允许的最大转角跳变为 "
        f"`{math.radians(45.0) * 0.1:.4f} rad`。",
        "",
        "| 速度档 | 转速限制 | J_delta（稳态段）/ rad/s | J_delta（全程）/ rad/s | max\\|steer\\| / rad | max\\|Δsteer\\| / rad | 越界拍数 | 越界比例 | 稳态 σ(e) / m | 稳态 max\\|e\\| / m |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for speed_mode in SPEED_MODES:
        for label, key in [("原始", "raw"), ("加速率限", "limited")]:
            data = results[speed_mode][key]
            lines.append(
                f"| {speed_mode} | {label} | {data['J_delta_steady']:.4f} | "
                f"{data['J_delta']:.4f} | "
                f"{data['max_steer']:.4f} | {data['max_dsteer']:.4f} | "
                f"{data['violations']}/{data['steps']} | "
                f"{data['violation_ratio']*100:.1f}% | "
                f"{data['steady_sigma']:.4f} | {data['steady_max_abs']:.4f} |"
            )
    lines.append("")
    raw_high = results["high"]["raw"]
    lim_high = results["high"]["limited"]
    lines.append(
        f"高速档稳态段 `J_delta` 从 {raw_high['J_delta_steady']:.4f} 降到 "
        f"{lim_high['J_delta_steady']:.4f}"
        f"（{raw_high['J_delta_steady']/lim_high['J_delta_steady']:.1f} 倍），"
        f"`max|steer|` 从 {raw_high['max_steer']:.4f} 降到 {lim_high['max_steer']:.4f}"
        f"（不再顶死 ±0.6109 rad 限幅），而精度几乎不变"
        f"（稳态 max|e| {raw_high['steady_max_abs']:.4f} → {lim_high['steady_max_abs']:.4f}）。"
    )
    (out_dir / "rate_limit_compare.md").write_text("\n".join(lines), encoding="utf-8")

    # ---------------- 独立单图 ----------------
    def save(fig, name):
        fig.tight_layout()
        fig.savefig(out_dir / name, dpi=140)
        plt.close(fig)
        print(f"  {out_dir / name}")

    # 图 1：high 档转角时间历程，原始 vs 受限（两条曲线画在同一坐标系里比较）
    raw_records = results["high"]["raw"]["records"]
    lim_records = results["high"]["limited"]["records"]
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    ax.plot([r.time for r in raw_records], [r.steer for r in raw_records],
            color="#dc2626", lw=1.1, label="raw (platform applies no rate limit)")
    ax.plot([r.time for r in lim_records], [r.steer for r in lim_records],
            color="#2563eb", lw=1.4, label="with real steer-rate limit")
    ax.set_title("LQR kinematic at 6.06 m/s: steer command vs time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("steer [rad]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="lower right")
    save(fig, "fig_steer_time_raw_vs_limited_high.png")

    # 图 2：原始转角单独一张，放大看清逐拍交替
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    ax.plot([r.time for r in raw_records], [r.steer for r in raw_records],
            color="#dc2626", lw=1.2)
    ax.set_title("LQR kinematic at 6.06 m/s: the raw oscillation")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("steer [rad]")
    ax.grid(alpha=0.3)
    save(fig, "fig_steer_time_raw_high.png")

    # 图 3：逐拍转角跳变 vs 执行器上限（原始 / 受限），柱状对比
    rate_limit = math.radians(45.0) * 0.1
    labels = ["medium\nraw", "medium\nrate-limited", "high\nraw", "high\nrate-limited"]
    values = [
        results["medium"]["raw"]["max_dsteer"],
        results["medium"]["limited"]["max_dsteer"],
        results["high"]["raw"]["max_dsteer"],
        results["high"]["limited"]["max_dsteer"],
    ]
    colors = ["#dc2626", "#2563eb", "#dc2626", "#2563eb"]
    fig, ax = plt.subplots(figsize=(7.6, 4.8))
    bars = ax.bar(labels, values, color=colors)
    ax.axhline(rate_limit, color="k", ls="--", lw=1.2,
               label=f"actuator limit {rate_limit:.4f} rad/step")
    ax.bar_label(bars, fmt="%.4f", fontsize=9)
    ax.set_title("max per-step steer jump vs actuator limit")
    ax.set_ylabel(r"max $|\Delta$steer$|$ per step [rad]")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=9)
    save(fig, "fig_max_dsteer_vs_limit.png")

    with (out_dir / "rate_limit_compare.csv").open("w", encoding="utf-8-sig",
                                                   newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["speed_mode", "variant", "J_delta_steady", "J_delta_run",
                         "max_steer", "max_dsteer", "violations", "steps",
                         "violation_ratio", "steady_sigma", "steady_max_abs",
                         "steady_mean"])
        for speed_mode in SPEED_MODES:
            for variant in ("raw", "limited"):
                data = results[speed_mode][variant]
                writer.writerow([
                    speed_mode, variant,
                    f"{data['J_delta_steady']:.6f}", f"{data['J_delta']:.6f}",
                    f"{data['max_steer']:.6f}",
                    f"{data['max_dsteer']:.6f}", data["violations"],
                    data["steps"], f"{data['violation_ratio']:.6f}",
                    f"{data['steady_sigma']:.6f}",
                    f"{data['steady_max_abs']:.6f}", f"{data['steady_mean']:.6f}",
                ])

    print(f"\n[输出目录] {out_dir}")


if __name__ == "__main__":
    main()
