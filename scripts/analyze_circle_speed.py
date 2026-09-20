"""圆形路径速度实验分析脚本（对应 vdm_lab/tasks/README.md 实验任务 2）。

从 outputs/ 下读取 circle 路线的实验日志，按
    abs(curvature - 1/R) < tol
筛选出车辆真正在圆弧上行驶的稳态段，剔除切入/驶出的过渡记录，然后：

    1. 输出任务文档 §2.5 的总体指标表；
    2. 输出稳态段的转角/横摆角速度/法向加速度/横向误差统计；
    3. 输出仿真均值与理论值 atan(L*kappa)、v*kappa、v^2*kappa 的相对误差；
    4. 画出法向加速度对速度平方、横向误差对速度的关系图。

用法：
    python scripts/analyze_circle_speed.py                    # 自动扫描 outputs/
    python scripts/analyze_circle_speed.py DIR1 DIR2 ...      # 指定实验目录
"""

import argparse
import csv
import json
import math
import statistics as stats
import sys
from pathlib import Path

# Windows 控制台默认使用 GBK，直接打印 "m/s²" 这类字符会抛 UnicodeEncodeError。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from vdm_lab.config.vehicle_params import make_vehicle_config  # noqa: E402

ALGO_LABELS = {
    "pp": "PP",
    "lqr_kinematic": "LQR kinematic",
    "lqr_dynamic": "LQR dynamic",
    "mpc": "MPC",
}
ALGO_ORDER = ["pp", "lqr_kinematic", "lqr_dynamic", "mpc"]
SPEED_ORDER = ["low", "medium", "high"]


# --------------------------------------------------------------------------
# 读取与筛选
# --------------------------------------------------------------------------
def read_csv_rows(path):
    with path.open(encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


def parse_experiment_dir(name, route):
    """从目录名 {时间戳}_{算法}_{路线}_{速度档} 解析出 (算法, 速度档)。

    算法名自身含下划线（lqr_kinematic），不能用 split 按下标取，
    改为按已知算法名从长到短做子串匹配。
    """
    for algo in sorted(ALGO_LABELS, key=len, reverse=True):
        marker = f"_{algo}_{route}_"
        index = name.find(marker)
        if index == -1:
            continue
        speed = name[index + len(marker):]
        if speed in SPEED_ORDER:
            return algo, speed
    return None


def discover_experiments(outputs_dir, route):
    """扫描 outputs/，按 (算法, 速度档) 归并，同名保留最新的一批。"""
    found = {}
    for entry in sorted(outputs_dir.iterdir()):
        if not entry.is_dir() or not (entry / "trajectory.csv").exists():
            continue
        parsed = parse_experiment_dir(entry.name, route)
        if parsed is None:
            continue
        # 目录名以时间戳开头，字符串序即时间序；保留最新的一次。
        if parsed not in found or entry.name > found[parsed].name:
            found[parsed] = entry
    return found


def find_steady_segment(rows, kappa_ref, tol):
    """返回满足曲率条件的最长连续区间 [start, end]（闭区间）。"""
    best = cur = 0
    best_span = cur_span = (0, -1)
    for i, row in enumerate(rows):
        if abs(float(row["curvature"]) - kappa_ref) < tol:
            if cur == 0:
                cur_span = (i, i)
            cur += 1
            if cur > best:
                best = cur
                best_span = (cur_span[0], i)
        else:
            cur = 0
    return best_span if best > 0 else (0, -1)


def median_dt(rows):
    deltas = [
        float(rows[i]["time"]) - float(rows[i - 1]["time"])
        for i in range(1, len(rows))
        if float(rows[i]["time"]) > float(rows[i - 1]["time"])
    ]
    return stats.median(deltas) if deltas else 0.1


# --------------------------------------------------------------------------
# 统计
# --------------------------------------------------------------------------
def series(rows, key):
    return [float(r[key]) for r in rows]


def steady_statistics(segment, dt):
    lateral = series(segment, "lateral_error")
    steer = series(segment, "steer")
    steer_rate = [
        abs(steer[i] - steer[i - 1]) / dt for i in range(1, len(steer))
    ]
    return {
        "samples": len(segment),
        "v_mean": stats.fmean(series(segment, "speed")),
        "v_target_median": stats.median(series(segment, "target_speed")),
        "lateral_mean": stats.fmean(lateral),
        "lateral_abs_mean": stats.fmean(abs(x) for x in lateral),
        "lateral_abs_max": max(abs(x) for x in lateral),
        "lateral_std": stats.pstdev(lateral),
        "steer_mean": stats.fmean(steer),
        "yaw_rate_mean": stats.fmean(series(segment, "yaw_rate")),
        "normal_accel_mean": stats.fmean(series(segment, "normal_accel")),
        "beta_abs_max": max(abs(x) for x in series(segment, "beta")),
        "steer_rate_mean": stats.fmean(steer_rate) if steer_rate else 0.0,
    }


def whole_run_statistics(rows, dt):
    steer = series(rows, "steer")
    steer_rate = [
        abs(steer[i] - steer[i - 1]) / dt for i in range(1, len(steer))
    ]
    return {
        "j_delta": stats.fmean(steer_rate) if steer_rate else 0.0,
    }


def fit_vehicle_circle(segment):
    """最小二乘（Kasa 代数法）拟合车辆稳态段轨迹所在圆。

    PP 在恒曲率路径上会稳定在一个与参考同心的、半径更小的圆上（内切）。
    拟合出圆心和半径，就能验证 R - r 是否等于实测横向误差均值，
    从而把"内切"确认为稳态几何性质而不是暂态或漂移。
    """
    x = np.array(series(segment, "x"))
    y = np.array(series(segment, "y"))
    if len(x) < 3:
        return None
    design = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(x))])
    solution, *_ = np.linalg.lstsq(design, x * x + y * y, rcond=None)
    cx, cy, constant = solution
    radius_sq = constant + cx * cx + cy * cy
    if radius_sq <= 0:
        return None
    return float(cx), float(cy), math.sqrt(radius_sq)


def theory_values(v, wheelbase, kappa_ref):
    return {
        "steer": math.atan(wheelbase * kappa_ref),
        "yaw_rate": v * kappa_ref,
        "normal_accel": v * v * kappa_ref,
    }


def relative_error(sim, theory):
    if abs(theory) < 1e-12:
        return float("nan")
    return abs(sim - theory) / abs(theory) * 100.0


# --------------------------------------------------------------------------
# 输出
# --------------------------------------------------------------------------
def format_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(c) for c in row) + " |")
    return "\n".join(lines)


def write_csv_file(path, headers, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(rows)


def plot_series(records, out_dir, key, ylabel, title, filename, theory=None):
    """把一组曲线画成单独一张图（不做多子图拼版）。

    theory:
        None      不画理论线
        "v2kappa" 法向加速度 a_n = v^2 * kappa
        "vkappa"  横摆角速度 psi_dot = v * kappa
        "const"   稳态转角 delta = atan(L * kappa)，与速度无关
    """
    fig, ax = plt.subplots(figsize=(7.5, 5.0))

    for algo in ALGO_ORDER:
        points = [r for r in records if r["algo"] == algo]
        if not points:
            continue
        ax.plot([p["v_mean"] for p in points],
                [p[key] for p in points], "o-", label=ALGO_LABELS[algo])

    v_ref = [p["v_mean"] for p in records]
    if theory and v_ref:
        kappa = records[0]["kappa_ref"] if records else 1 / 12.0
        wheelbase = records[0]["wheelbase"] if records else 2.5
        lo, hi = min(v_ref), max(v_ref)
        grid = [lo + (hi - lo) * i / 100 for i in range(101)]
        if theory == "v2kappa":
            values = [v * v * kappa for v in grid]
            label = r"theory $v^2\kappa$"
        elif theory == "vkappa":
            values = [v * kappa for v in grid]
            label = r"theory $v\kappa$"
        else:
            values = [math.atan(wheelbase * kappa)] * len(grid)
            label = r"theory $\arctan(L\kappa)$  (independent of $v$)"
        ax.plot(grid, values, "k--", lw=1.1, label=label)

    ax.set_xlabel("steady-state speed v [m/s]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / filename, dpi=140)
    plt.close(fig)


def plot_results(records, out_dir):
    """每张图单独输出一个文件，不拼多子图。"""
    # 速度的直接代价：a_n = v^2 * kappa
    plot_series(records, out_dir, "normal_accel_mean",
                r"normal accel $a_n$ [m/s$^2$]",
                r"$a_n$ vs $v$   (theory: $a_n=v^2\kappa$)",
                "fig_an_vs_v.png", theory="v2kappa")
    # 不是转向不够用：稳态转角与速度无关
    plot_series(records, out_dir, "steer_mean",
                "steady-state steer [rad]",
                r"steady steer vs $v$   (theory: $\arctan(L\kappa)$, flat)",
                "fig_steer_vs_v.png", theory="const")
    # 横摆角速度线性增长
    plot_series(records, out_dir, "yaw_rate_mean",
                r"steady yaw rate [rad/s]",
                r"yaw rate vs $v$   (theory: $v\kappa$)",
                "fig_yawrate_vs_v.png", theory="vkappa")
    # 误差随速度的增长（均值会被统计口径骗，最大值单调）
    plot_series(records, out_dir, "lateral_abs_mean",
                "mean |lateral error| [m]",
                "steady-state lateral error vs v",
                "fig_lateral_mean_vs_v.png")
    plot_series(records, out_dir, "lateral_abs_max",
                "max |lateral error| [m]",
                "steady-state max lateral error vs v",
                "fig_lateral_max_vs_v.png")


def load_trajectory(directory):
    return read_csv_rows(directory / "trajectory.csv")


def plot_trajectory_single(records, out_dir, algo, speed_mode, xlim, ylim):
    """单个速度档的轨迹（参考 vs 车辆），放大到一段圆弧上让内切量可见。"""
    point = next((r for r in records
                  if r["algo"] == algo and r["speed_mode"] == speed_mode), None)
    if point is None:
        return

    reference = read_csv_rows(out_dir.parent / point["dir"] / "reference_path.csv")
    rows = load_trajectory(out_dir.parent / point["dir"])

    fig, ax = plt.subplots(figsize=(7.5, 6.0))
    ax.plot(series(reference, "x_m"), series(reference, "y_m"),
            "-", color="0.55", lw=2.6, label="reference", zorder=1)
    ax.plot(series(rows, "x"), series(rows, "y"),
            lw=1.8, color="#2563eb", label="vehicle", zorder=2)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("x / East [m]")
    ax.set_ylabel("y / North [m]")
    ax.set_title(f"{ALGO_LABELS[algo]} ({speed_mode}, v={point['v_mean']:.2f} m/s): "
                 f"corner cutting {point['lateral_mean']:+.3f} m")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="upper left")
    fig.tight_layout()
    fig.savefig(out_dir / f"fig_trajectory_{algo}_{speed_mode}.png", dpi=140)
    plt.close(fig)


def plot_radius_vs_time(records, out_dir, algo="pp"):
    """车辆到参考圆心的距离随时间变化——内切量最直观的一张图。"""
    points = [r for r in records if r["algo"] == algo]
    if not points:
        return
    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    for point in points:
        rows = load_trajectory(out_dir.parent / point["dir"])
        radius = [math.hypot(float(r["x"]) - 16.0, float(r["y"]) - 12.0) for r in rows]
        ax.plot(series(rows, "time"), radius, lw=1.6,
                label=f"{point['speed_mode']}  v={point['v_mean']:.2f} m/s")
    ax.axhline(12.0, color="0.45", lw=1.4, ls="--", label="reference R = 12 m")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance to circle centre r [m]")
    ax.set_title(f"{ALGO_LABELS[algo]}: vehicle stays on a smaller concentric circle")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / f"fig_radius_vs_time_{algo}.png", dpi=140)
    plt.close(fig)


def plot_lateral_error_time(records, out_dir, algo="pp"):
    """同一种算法、不同速度档的横向误差时间曲线（单图）。"""
    points = [r for r in records if r["algo"] == algo]
    if not points:
        return
    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    for point in points:
        rows = load_trajectory(out_dir.parent / point["dir"])
        ax.plot(series(rows, "time"), series(rows, "lateral_error"),
                lw=1.6, label=f"{point['speed_mode']}  v={point['v_mean']:.2f} m/s")
    ax.axhline(0.0, color="k", lw=0.8, ls=":")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("lateral error [m]")
    ax.set_title(f"{ALGO_LABELS[algo]}: lateral error vs time")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(out_dir / f"fig_lateral_error_time_{algo}.png", dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="圆形路径速度实验分析")
    parser.add_argument("dirs", nargs="*", type=Path,
                        help="实验目录；不填则自动扫描 outputs/")
    parser.add_argument("--outputs-dir", type=Path,
                        default=REPO_ROOT / "outputs")
    parser.add_argument("--route", default="circle")
    parser.add_argument("--radius", type=float, default=12.0,
                        help="圆弧半径 [m]，用于确定参考曲率 1/R")
    parser.add_argument("--kappa-tol", type=float, default=0.005,
                        help="曲率筛选容差 [1/m]")
    parser.add_argument("--trim-samples", type=int, default=15,
                        help="稳态段两端各剔除的采样点数")
    parser.add_argument("--vehicle", default="student_car")
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    kappa_ref = 1.0 / args.radius
    vehicle = make_vehicle_config(args.vehicle)
    wheelbase = vehicle.wheelbase

    if args.dirs:
        experiments = {}
        for directory in args.dirs:
            parsed = parse_experiment_dir(directory.name, args.route)
            if parsed is None:
                raise SystemExit(
                    f"无法从目录名解析出算法和速度档：{directory.name}"
                    f"（路线需为 {args.route}，可用 --route 指定）"
                )
            experiments[parsed] = directory
    else:
        experiments = discover_experiments(args.outputs_dir, args.route)

    if not experiments:
        raise SystemExit(
            f"在 {args.outputs_dir} 下没有找到 {args.route} 的实验目录。"
        )

    records = []
    skipped = []
    for (algo, speed), directory in sorted(
        experiments.items(),
        key=lambda kv: (ALGO_ORDER.index(kv[0][0])
                        if kv[0][0] in ALGO_ORDER else 99,
                        SPEED_ORDER.index(kv[0][1])),
    ):
        rows = read_csv_rows(directory / "trajectory.csv")
        if len(rows) < 2:
            skipped.append((algo, speed, "日志为空"))
            continue

        dt = median_dt(rows)
        start, end = find_steady_segment(rows, kappa_ref, args.kappa_tol)
        if end - start + 1 <= 2 * args.trim_samples + 1:
            skipped.append((algo, speed, f"稳态段仅 {end - start + 1} 点，无法剔除过渡"))
            seg_start, seg_end = start, end
        else:
            seg_start = start + args.trim_samples
            seg_end = end - args.trim_samples
        segment = rows[seg_start:seg_end + 1]

        if not segment:
            skipped.append((algo, speed, "剔除过渡后稳态段为空"))
            continue

        summary = steady_statistics(segment, dt)
        summary.update(whole_run_statistics(rows, dt))
        circle = fit_vehicle_circle(segment)
        if circle is None:
            summary.update(vehicle_circle_x=None, vehicle_circle_y=None,
                           vehicle_circle_radius=None, corner_cut=None)
        else:
            summary.update(vehicle_circle_x=circle[0], vehicle_circle_y=circle[1],
                           vehicle_circle_radius=circle[2],
                           corner_cut=args.radius - circle[2])
        summary["algo"] = algo
        summary["speed_mode"] = speed
        summary["dir"] = directory.name
        summary["kappa_ref"] = kappa_ref
        summary["wheelbase"] = wheelbase
        summary["steady_index_span"] = (start, end)
        summary["trimmed_span"] = (seg_start, seg_end)

        metrics_file = directory / "metrics.json"
        if metrics_file.exists():
            with metrics_file.open(encoding="utf-8") as file:
                metrics = json.load(file)
            summary["reached_goal"] = bool(metrics.get("reached_goal"))
            summary["overall_mean_lateral"] = float(metrics["mean_lateral_error_m"])
            summary["overall_max_lateral"] = float(metrics["max_lateral_error_m"])
        else:
            summary["reached_goal"] = None
            summary["overall_mean_lateral"] = None
            summary["overall_max_lateral"] = None

        records.append(summary)

    if not records:
        raise SystemExit("没有可用的实验记录。")

    out_dir = args.out_dir or (args.outputs_dir / "analysis_circle_speed")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---- 表 1：总体指标（对应任务文档 §2.5） ----------------------------
    headers1 = ["算法", "速度档", "到达终点", "平均横向误差 / m", "最大横向误差 / m",
                "稳态平均转角 / rad", "稳态平均横摆角速度 / rad/s",
                "稳态平均法向加速度 / m/s²"]
    rows1 = [
        [ALGO_LABELS[r["algo"]], r["speed_mode"],
         "是" if r["reached_goal"] else "否",
         f"{r['overall_mean_lateral']:.4f}", f"{r['overall_max_lateral']:.4f}",
         f"{r['steer_mean']:.5f}", f"{r['yaw_rate_mean']:.5f}",
         f"{r['normal_accel_mean']:.5f}"]
        for r in records
    ]

    # ---- 表 2：稳态段统计 ----------------------------------------------
    headers2 = ["算法", "速度档", "稳态样点数", "稳态速度 / m/s", "横向误差均值 / m",
                "横向误差绝对值均值 / m", "横向误差绝对值最大 / m",
                "横向误差标准差 / m", "max|beta| / rad", "J_delta / rad/s"]
    rows2 = [
        [ALGO_LABELS[r["algo"]], r["speed_mode"], r["samples"],
         f"{r['v_mean']:.3f}", f"{r['lateral_mean']:+.5f}",
         f"{r['lateral_abs_mean']:.5f}", f"{r['lateral_abs_max']:.5f}",
         f"{r['lateral_std']:.5f}", f"{r['beta_abs_max']:.5f}",
         f"{r['steer_rate_mean']:.5f}"]
        for r in records
    ]

    # ---- 表 3：理论值 vs 仿真值 ----------------------------------------
    headers3 = ["算法", "速度档", "v / m/s", "给定转角理论 / rad", "给定转角仿真 / rad",
                "给定转角相对误差 / %", "横摆角速度理论 / rad/s",
                "横摆角速度仿真 / rad/s", "横摆角速度相对误差 / %",
                "法向加速度理论 / m/s²", "法向加速度仿真 / m/s²",
                "法向加速度相对误差 / %"]
    rows3 = []
    for r in records:
        theory = theory_values(r["v_mean"], wheelbase, kappa_ref)
        rows3.append([
            ALGO_LABELS[r["algo"]], r["speed_mode"], f"{r['v_mean']:.3f}",
            f"{theory['steer']:.5f}", f"{r['steer_mean']:.5f}",
            f"{relative_error(r['steer_mean'], theory['steer']):.2f}",
            f"{theory['yaw_rate']:.5f}", f"{r['yaw_rate_mean']:.5f}",
            f"{relative_error(r['yaw_rate_mean'], theory['yaw_rate']):.2f}",
            f"{theory['normal_accel']:.5f}", f"{r['normal_accel_mean']:.5f}",
            f"{relative_error(r['normal_accel_mean'], theory['normal_accel']):.2f}",
        ])

    # ---- 表 4：车辆轨迹圆拟合（内切量归因） ------------------------------
    headers4 = ["算法", "速度档", "拟合圆心 x / m", "拟合圆心 y / m",
                "车辆半径 r / m", "内切量 R-r / m", "横向误差均值 / m",
                "二者之差 / mm"]
    rows4 = []
    for r in records:
        if r["vehicle_circle_radius"] is None:
            rows4.append([ALGO_LABELS[r["algo"]], r["speed_mode"],
                          "-", "-", "-", "-", f"{r['lateral_mean']:+.5f}", "-"])
            continue
        rows4.append([
            ALGO_LABELS[r["algo"]], r["speed_mode"],
            f"{r['vehicle_circle_x']:.4f}", f"{r['vehicle_circle_y']:.4f}",
            f"{r['vehicle_circle_radius']:.4f}", f"{r['corner_cut']:.4f}",
            f"{r['lateral_mean']:+.5f}",
            f"{abs(r['corner_cut'] - r['lateral_mean']) * 1000:.2f}",
        ])

    sections = [
        "# 圆形路径速度实验分析",
        "",
        f"- 参考曲率 kappa = 1/{args.radius:g} = {kappa_ref:.6f} 1/m",
        f"- 稳态段判据 |curvature - kappa| < {args.kappa_tol}",
        f"- 稳态段两端各剔除 {args.trim_samples} 个采样点",
        f"- 轴距 L = {wheelbase:.3f} m",
        "",
        "## 表 1 总体指标（任务文档 §2.5）",
        "",
        format_table(headers1, rows1),
        "",
        "## 表 2 稳态圆弧段统计",
        "",
        format_table(headers2, rows2),
        "",
        "## 表 3 理论值与仿真值对比",
        "",
        "理论值：转角 atan(L*kappa)、横摆角速度 v*kappa、法向加速度 v^2*kappa，",
        "其中 v 取稳态段实测平均速度。",
        "",
        format_table(headers3, rows3),
        "",
        "## 表 4 车辆轨迹圆拟合（内切量归因）",
        "",
        "稳态段轨迹用最小二乘拟合圆。参考圆心为 (16.0000, 12.0000)、半径 R = 12.0000 m；",
        "若内切量 R-r 与实测横向误差均值一致，说明横向误差是**稳态几何偏心**，",
        "而不是暂态或漂移。",
        "",
        format_table(headers4, rows4),
        "",
        "## 各实验目录",
        "",
    ]
    for r in records:
        sections.append(
            f"- {ALGO_LABELS[r['algo']]} / {r['speed_mode']}: `{r['dir']}` "
            f"（稳态段 index {r['trimmed_span'][0]}~{r['trimmed_span'][1]}）"
        )
    if skipped:
        sections += ["", "## 跳过的实验", ""]
        sections += [f"- {a} / {s}: {why}" for a, s, why in skipped]

    report = "\n".join(sections) + "\n"
    (out_dir / "tables.md").write_text(report, encoding="utf-8")
    write_csv_file(out_dir / "table1_overall.csv", headers1, rows1)
    write_csv_file(out_dir / "table2_steady.csv", headers2, rows2)
    write_csv_file(out_dir / "table3_theory_vs_sim.csv", headers3, rows3)
    write_csv_file(out_dir / "table4_vehicle_circle_fit.csv", headers4, rows4)
    plot_results(records, out_dir)
    # 只放大到圆弧右侧一段（x 14~30, y 5~22），内切的 0.4 m 偏移才看得见
    plot_trajectory_single(records, out_dir, "pp", "low", (14.0, 30.0), (5.0, 22.0))
    plot_trajectory_single(records, out_dir, "pp", "high", (14.0, 30.0), (5.0, 22.0))
    plot_radius_vs_time(records, out_dir, algo="pp")
    plot_lateral_error_time(records, out_dir, algo="pp")
    plot_lateral_error_time(records, out_dir, algo="mpc")

    print(report)
    print(f"[输出目录] {out_dir}")


if __name__ == "__main__":
    main()
