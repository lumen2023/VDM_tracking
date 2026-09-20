"""Pure Pursuit 圆弧稳态内切量归因实验。

PP 在恒曲率圆上会稳定在一个同心的、半径更小的圆上（内切）。本脚本通过
三组受控实验定位内切量的来源：

    A. 只改基础前视距离 L0  -> 内切量与前视距离 Lf 的关系
    B. 只改 lf/lr 分配      -> 内切量与车身侧偏角 beta 的关系
    C. 只改重采样间距 ds     -> 内切量与参考路径离散化的关系

结论（由本脚本的实测数据给出）：内切量主要由 beta 决定，满足
    corner_cut ~= Lf * beta
因为 PP 的几何转角公式 delta = atan(2L*sin(alpha)/Lf) 是按无侧偏的单轮
模型推导的，而仿真的运动学自行车模型让被跟踪点沿 psi+beta 方向运动。

用法：
    python scripts/analyze_pp_corner_cut.py
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

from vdm_lab.common.simulation import load_controller, run_simulation  # noqa: E402
from vdm_lab.common.types import LabConfig  # noqa: E402
from vdm_lab.config.vehicle_params import make_vehicle_config  # noqa: E402

RADIUS = 12.0
CENTER = (16.0, 12.0)
SPEED_MODE = "low"
PP_SPEED_GAIN = 0.35
TRIM = 15


def run_case(*, base_lookahead=3.0, lf=1.25, lr=1.25, ds=0.05,
             speed_mode=SPEED_MODE, controller=None):
    """跑一次 circle 实验，返回稳态圆弧段的内切量和相关量。"""
    vehicle = make_vehicle_config("student_car")
    vehicle.lf = lf
    vehicle.lr = lr
    vehicle.wheelbase = lf + lr

    config = LabConfig(vehicle=vehicle)
    config.sim.route_name = "circle"
    config.sim.speed_mode = speed_mode
    config.sim.waypoint_ds = ds
    config.controller.pp_base_lookahead = base_lookahead

    _, records, _ = run_simulation(controller, config=config)

    curvature = np.array([r.curvature for r in records])
    inside = np.where(np.abs(curvature - 1.0 / RADIUS) < 0.005)[0]
    if len(inside) <= 2 * TRIM + 1:
        return None
    segment = records[inside[0] + TRIM: inside[-1] - TRIM + 1]

    lateral = np.array([r.lateral_error for r in segment])
    radius = np.hypot(
        [r.x - CENTER[0] for r in segment],
        [r.y - CENTER[1] for r in segment],
    )
    speed = float(np.mean([r.speed for r in segment]))
    steer = float(np.mean([r.steer for r in segment]))
    beta = math.atan(lr * math.tan(steer) / (lf + lr))

    return {
        "corner_cut": float(np.mean(lateral)),
        "radius": float(np.mean(radius)),
        "speed": speed,
        "steer": steer,
        "beta": beta,
        "lookahead": base_lookahead + PP_SPEED_GAIN * speed,
        "lf": lf,
        "lr": lr,
        "ds": ds,
    }


def linear_fit_through_origin(xs, ys):
    """过原点最小二乘斜率，用于 A 组"内切量是否正比于 Lf"的目视参考线。"""
    denominator = sum(x * x for x in xs)
    if denominator <= 0.0:
        return 0.0
    return sum(x * y for x, y in zip(xs, ys)) / denominator


def linear_regression(xs, ys):
    """普通最小二乘 y = slope*x + intercept，返回 (slope, intercept, R^2, 最大残差)。"""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx <= 0.0:
        return 0.0, mean_y, float("nan"), 0.0
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sxx
    intercept = mean_y - slope * mean_x
    residuals = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    ss_total = sum((y - mean_y) ** 2 for y in ys)
    ss_residual = sum(r * r for r in residuals)
    r_squared = 1.0 - ss_residual / ss_total if ss_total > 0.0 else float("nan")
    return slope, intercept, r_squared, max(abs(r) for r in residuals)


def main():
    controller = load_controller("pp", "student")

    print("=== A. 扫描基础前视距离 L0（v=low, lf=lr=1.25, ds=0.05）===")
    sweep_lookahead = []
    for base in [0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 9.0, 14.0]:
        result = run_case(base_lookahead=base, controller=controller)
        if result is None:
            continue
        sweep_lookahead.append(result)
        print(f"  L0={base:5.1f}  Lf={result['lookahead']:6.2f}  "
              f"内切={result['corner_cut']:+.4f} m  "
              f"半径={result['radius']:.4f} m  beta={result['beta']:.5f}")

    print("\n=== B. 扫描 lf/lr 分配（L0=3.0, v=low, ds=0.05）===")
    sweep_geometry = []
    for lf, lr in [(2.5, 0.0), (2.0, 0.5), (1.25, 1.25), (0.5, 2.0), (0.0, 2.5)]:
        result = run_case(lf=lf, lr=lr, controller=controller)
        if result is None:
            continue
        sweep_geometry.append(result)
        predicted = result["lookahead"] * result["beta"]
        print(f"  (lf,lr)=({lf},{lr})  beta={result['beta']:.5f}  "
              f"内切={result['corner_cut']:+.4f} m  "
              f"Lf*beta={predicted:.4f} m")

    print("\n=== C. 扫描重采样间距 ds（L0=3.0, v=low, lf=lr=1.25）===")
    sweep_ds = []
    for ds in [0.05, 0.1, 0.25, 0.5, 1.0, 2.0]:
        result = run_case(ds=ds, controller=controller)
        if result is None:
            continue
        sweep_ds.append(result)
        print(f"  ds={ds:4.2f}  内切={result['corner_cut']:+.4f} m  "
              f"半径={result['radius']:.4f} m")

    # ---------------- 画图（每张图单独输出，不拼多子图） ----------------
    out_dir = REPO_ROOT / "outputs" / "analysis_pp_corner_cut"
    out_dir.mkdir(parents=True, exist_ok=True)

    def new_axes(title, xlabel, ylabel):
        fig, ax = plt.subplots(figsize=(7.5, 5.0))
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        return fig, ax

    # A. 内切量 vs 前视距离
    lf_values = [r["lookahead"] for r in sweep_lookahead]
    cuts = [r["corner_cut"] for r in sweep_lookahead]
    fig, ax = new_axes(r"corner cutting grows with $L_f$",
                       r"lookahead $L_f = L_0 + k\,v$ [m]", "corner cutting [m]")
    ax.plot(lf_values, cuts, "o-", color="#1f77b4", label="measured")
    if len(lf_values) >= 2:
        slope = linear_fit_through_origin(lf_values, cuts)
        ax.plot(lf_values, [slope * value for value in lf_values], "k--", lw=1.0,
                label=f"linear through origin (slope={slope:.4f})")
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_cut_vs_lookahead.png", dpi=140)
    plt.close(fig)

    # B. 内切量 vs 侧偏角 beta（决定性的对照实验）
    betas = [r["beta"] for r in sweep_geometry]
    cuts_g = [r["corner_cut"] for r in sweep_geometry]
    fig, ax = new_axes(r"corner cutting is driven by $\beta$",
                       r"body slip angle $\beta$ [rad]", "corner cutting [m]")
    ax.plot(betas, cuts_g, "s-", color="#d62728", label="measured")
    if betas:
        reference_lf = sweep_geometry[0]["lookahead"]
        grid = np.linspace(0.0, max(betas) * 1.05, 60)
        ax.plot(grid, [reference_lf * beta for beta in grid], "k--", lw=1.0,
                label=r"reference $L_f \cdot \beta$")
    ax.axhline(0.0, color="0.6", lw=0.8, ls=":")
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_cut_vs_beta.png", dpi=140)
    plt.close(fig)

    # C. 内切量 vs 参考路径重采样间距
    ds_values = [r["ds"] for r in sweep_ds]
    fig, ax = new_axes("discretization adds a smaller term",
                       "reference resampling ds [m]", "corner cutting [m]")
    ax.plot(ds_values, [r["corner_cut"] for r in sweep_ds], "^-",
            color="#2ca02c", label="measured")
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / "fig_cut_vs_ds.png", dpi=140)
    plt.close(fig)

    # D. 主回归：内切量 vs Lf*beta（只用 ds=0.05 的受控点）
    controlled = sweep_lookahead + sweep_geometry
    if len(controlled) >= 3:
        xs = [r["lookahead"] * r["beta"] for r in controlled]
        ys = [r["corner_cut"] for r in controlled]
        slope, intercept, r_squared, max_residual = linear_regression(xs, ys)
        fig, ax = new_axes(
            rf"corner cutting $\approx$ {slope:.4f}$\cdot L_f\beta$ "
            rf"({intercept:+.4f})   $R^2$={r_squared:.5f}",
            r"$L_f \cdot \beta$ [m]", "corner cutting [m]",
        )
        ax.plot(xs, ys, "o", color="#1f77b4", ms=7, label="measured (13 points)")
        grid = np.linspace(0.0, max(xs) * 1.08, 60)
        ax.plot(grid, [slope * value + intercept for value in grid], "k--", lw=1.2,
                label="linear fit")
        ax.legend(fontsize=9)
        fig.tight_layout()
        fig.savefig(out_dir / "fig_cut_regression.png", dpi=140)
        plt.close(fig)
        print(f"\n主回归: cut = {slope:.4f}*(Lf*beta) {intercept:+.4f}  "
              f"R^2={r_squared:.5f}  最大残差={max_residual:.4f} m")

    for name, data in [("sweep_lookahead", sweep_lookahead),
                       ("sweep_geometry", sweep_geometry),
                       ("sweep_ds", sweep_ds)]:
        if not data:
            continue
        with (out_dir / f"{name}.csv").open("w", encoding="utf-8-sig",
                                            newline="") as file:
            writer = csv.DictWriter(file, fieldnames=list(data[0].keys()))
            writer.writeheader()
            writer.writerows(data)

    print(f"\n[输出目录] {out_dir}")


if __name__ == "__main__":
    main()
