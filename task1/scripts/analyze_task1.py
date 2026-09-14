"""Validate task-1 raw logs and produce auditable tables and comparison plots."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import sys

os.environ["MPLBACKEND"] = "Agg"
TASK_ROOT = Path(__file__).resolve().parents[1]
ROOT = TASK_ROOT.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from scipy.signal import find_peaks

from task1.scripts.run_task1 import ALGORITHMS, ROUTES, Case, extended_metrics, make_config
from vdm_lab.common.types import Path as ReferencePath, StepRecord

LABELS = {"pp": "PP", "lqr_kinematic": "运动学 LQR", "mpc": "MPC", "lqr_dynamic": "动力学 LQR"}
TITLES = {"double_lane_change": "双移线", "right_angle": "直角弯", "s_curve": "S 弯", "mixed_course": "综合路线"}
COLORS = {"pp": "#2575b9", "lqr_kinematic": "#d96a22", "mpc": "#16806a", "lqr_dynamic": "#7b52ab"}
STEER_COLORS = {35: "#2575b9", 25: "#16806a", 15: "#d04b40"}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path):
    return np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8"))


def load_runs(source):
    manifest = read_json(source / "manifest.json")
    results = read_json(source / "results.json")
    if len(results) != len(manifest["cases"]):
        raise ValueError("Batch has not finished, or results are missing")
    runs = []
    for result in results:
        directory = source / "runs" / result["name"]
        if result["status"] == "worker_error":
            raise ValueError(f"Worker failure must be investigated, not dropped: {result}")
        trajectory_file = directory / "trajectory.csv"
        if not trajectory_file.exists():
            raise ValueError(f"No trajectory; see preserved diagnostics: {directory}")
        runs.append({**result, "directory": directory, "trajectory": read_csv(trajectory_file),
                     "reference": read_csv(directory / "reference_path.csv"),
                     "saved_config": read_json(directory / "config.json"),
                     "info": read_json(directory / "run_info.json")})
    return manifest, runs


def validate_run(run, speed_mode):
    case = Case(**run["case"])
    config = make_config(case, speed_mode)
    saved = run["saved_config"]["config"]
    if saved["vehicle"]["max_steer"] != config.vehicle.max_steer:
        raise ValueError(f"Configuration mismatch: {run['name']}")
    data, ref = run["trajectory"], run["reference"]
    assert len(data) == run["metrics"]["steps"]
    for field in data.dtype.names:
        assert np.isfinite(data[field]).all(), (run["name"], field)
    assert np.allclose(np.diff(data["time"]), config.sim.dt, atol=1e-10)
    assert np.all(data["speed"] >= -1e-12)
    assert np.all(data["speed"] <= config.vehicle.max_speed + 1e-10)
    assert np.max(np.abs(data["steer"])) <= config.vehicle.max_steer + 1e-10
    assert np.all(data["acceleration"] <= config.vehicle.max_accel + 1e-10)
    assert np.all(data["acceleration"] >= -config.vehicle.max_decel - 1e-10)
    assert np.all(np.diff(data["target_index"]) >= 0)
    indices = data["target_index"].astype(int)
    np.testing.assert_allclose(data["curvature"], ref["curvature_1pm"][indices], atol=1e-12)
    np.testing.assert_allclose(data["normal_accel"], data["speed"]**2 * data["curvature"], atol=1e-10)
    beta = np.arctan(config.vehicle.lr / (config.vehicle.lf + config.vehicle.lr) * np.tan(data["steer"]))
    np.testing.assert_allclose(data["beta"], beta, atol=1e-12)
    np.testing.assert_allclose(data["yaw_rate"], data["speed"] / (config.vehicle.lf + config.vehicle.lr) * np.tan(data["steer"]) * np.cos(beta), atol=1e-12)
    expected_lateral = -(data["x"] - ref["x_m"][indices]) * np.sin(ref["yaw_rad"][indices]) + (data["y"] - ref["y_m"][indices]) * np.cos(ref["yaw_rad"][indices])
    np.testing.assert_allclose(data["lateral_error"], expected_lateral, atol=1e-10)
    # Independently check the recorded state transitions, excluding the final command.
    dt = np.diff(data["time"])
    np.testing.assert_allclose(np.diff(data["x"]), data["speed"][:-1] * np.cos(data["yaw"][:-1] + beta[:-1]) * dt, atol=1e-10)
    np.testing.assert_allclose(np.diff(data["y"]), data["speed"][:-1] * np.sin(data["yaw"][:-1] + beta[:-1]) * dt, atol=1e-10)
    expected_speed = np.clip(data["speed"][:-1] + data["acceleration"][:-1] * dt, config.vehicle.min_speed, config.vehicle.max_speed)
    np.testing.assert_allclose(data["speed"][1:], expected_speed, atol=1e-10)
    records = [StepRecord(**{field: int(row[field]) if field == "target_index" else float(row[field]) for field in data.dtype.names}) for row in data]
    path = ReferencePath(x=ref["x_m"], y=ref["y_m"], yaw=ref["yaw_rad"], curvature=ref["curvature_1pm"], s=ref["s_m"], target_speed=ref["target_speed_mps"])
    recomputed = extended_metrics(path, records, config)
    for key, expected in recomputed.items():
        actual = run["metrics"][key]
        if run["status"] == "error" and key in {"reached_goal", "completion_time_s"}:
            continue
        if expected is None:
            assert actual is None, key
        else:
            assert math.isclose(float(actual), float(expected), rel_tol=1e-9, abs_tol=1e-10), (run["name"], key)
    if not run["metrics"]["reached_goal"]:
        assert run["metrics"]["completion_time_s"] is None
    return {"case": run["name"], "samples": len(data), "status": run["status"], "checks": "PASS"}


def configure_plots():
    available = {font.name for font in font_manager.fontManager.ttflist
                 if font.style == "normal" and font.weight in ("normal", "regular", 400)}
    candidates = ("Noto Sans CJK SC", "Noto Sans SC", "Microsoft YaHei", "SimHei", "DengXian", "Noto Serif SC")
    chosen = next((font for font in candidates if font in available), "DejaVu Sans")
    plt.rcParams.update({"font.family": [chosen, "DejaVu Sans"], "axes.unicode_minus": False,
                         "font.size": 10, "axes.titlesize": 12, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.facecolor": "white", "savefig.facecolor": "white"})
    return chosen


def save_figure(fig, path):
    for ax in fig.axes:
        ax.grid(True, alpha=0.18)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def route_plot(runs, route, destination, speed_mode):
    selected = [next(run for run in runs if run["case"]["group"] == "core" and run["case"]["route"] == route and run["case"]["algo"] == algo) for algo in ALGORITHMS]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    ref = selected[0]["reference"]
    axes[0, 0].plot(ref["x_m"], ref["y_m"], "--", color="#7d8793", label="参考路径", linewidth=2)
    for run in selected:
        data, algo = run["trajectory"], run["case"]["algo"]
        label = LABELS[algo] + ("（未完成）" if not run["metrics"]["reached_goal"] else "")
        style = {"color": COLORS[algo], "label": label, "linewidth": 1.5}
        axes[0, 0].plot(data["x"], data["y"], **style)
        axes[0, 1].plot(data["time"], data["lateral_error"], **style)
        axes[1, 0].plot(data["time"], data["speed"], **style)
        axes[1, 1].plot(data["time"], np.rad2deg(data["steer"]), **style)
    axes[0, 0].set(xlabel="x / m", ylabel="y / m", title="参考路径与实际轨迹")
    axes[0, 0].set_aspect("equal", adjustable="datalim")
    axes[0, 1].set(xlabel="时间 / s", ylabel="横向误差 / m", title="横向误差（带符号）")
    axes[1, 0].set(xlabel="时间 / s", ylabel="速度 / (m/s)", title="实际速度（非强制恒速）")
    axes[1, 0].axhline(ref["target_speed_mps"].max(), color="#7d8793", linestyle=":", label="巡航目标速度")
    axes[1, 1].set(xlabel="时间 / s", ylabel="前轮转角 / °", title="控制输入：幅值与高频变化")
    for ax in axes.flat:
        ax.legend(fontsize=9)
    fig.suptitle(f"{TITLES[route]} · 三算法对比 | {speed_mode}，巡航目标 {ref['target_speed_mps'].max():g} m/s", fontsize=15)
    save_figure(fig, destination)


def metric_plot(runs, destination):
    fields = (("mean_lateral_error_m", "平均绝对横向误差 / m"), ("max_lateral_error_m", "最大绝对横向误差 / m"),
              ("mean_abs_steer_rate_radps", "平均绝对转角变化率 / (rad/s)"))
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6), layout="constrained")
    x = np.arange(len(ROUTES))
    for ax, (field, title) in zip(axes, fields):
        for offset, algo in enumerate(ALGORITHMS):
            cases = [next(run for run in runs if run["case"] == {"group": "core", "route": route, "algo": algo, "max_steer_deg": 35.0}) for route in ROUTES]
            heights = [run["metrics"][field] for run in cases]
            bars = ax.bar(x + (offset - 1) * 0.25, heights, 0.24, color=COLORS[algo], label=LABELS[algo])
            ax.bar_label(bars, labels=[f"{v:.3f}" for v in heights], fontsize=8, rotation=90, padding=3)
            for bar, run in zip(bars, cases):
                if not run["metrics"]["reached_goal"]:
                    bar.set_hatch("///")
        ax.set_xticks(x, [TITLES[route] for route in ROUTES])
        ax.set_title(title)
        ax.margins(y=0.3)
    axes[0].legend(loc="upper left", fontsize=9)
    fig.suptitle("必做 9 组实验：精度与控制平滑性（越低越好；不代表唯一评价标准）", fontsize=14)
    save_figure(fig, destination)


def curvature_plot(runs, route, destination):
    selected = [next(run for run in runs if run["case"]["group"] == "core" and run["case"]["route"] == route and run["case"]["algo"] == algo) for algo in ALGORITHMS]
    fig, axes = plt.subplots(3, 1, figsize=(12, 8.3), layout="constrained")
    for ax, run in zip(axes, selected):
        data, metrics = run["trajectory"], run["metrics"]
        line_error = ax.plot(data["time"], np.abs(data["lateral_error"]), color=COLORS[run["case"]["algo"]], label="|横向误差|")
        ax.set(ylabel="|误差| / m", title=LABELS[run["case"]["algo"]], xlabel="时间 / s")
        other = ax.twinx()
        line_curvature = other.plot(data["time"], np.abs(data["curvature"]), color="#8c939d", linestyle="--", label="|所访问的参考曲率|")
        other.set_ylabel("|参考曲率| / (1/m)")
        peak_line = ax.axvline(metrics["peak_error_time_s"], color="#d04b40", linestyle=":", label="最大误差时刻")
        max_kappa = metrics["max_visited_abs_curvature_1pm"]
        times = data["time"][np.isclose(np.abs(data["curvature"]), max_kappa, rtol=1e-6, atol=1e-10)]
        other.scatter(times, np.full(len(times), max_kappa), color="#444b53", s=18, zorder=4)
        lines = line_error + line_curvature + [peak_line]
        ax.legend(lines, [line.get_label() for line in lines], loc="upper left", ncol=3, fontsize=9)
        ax.margins(y=0.25)
        other.margins(y=0.35)
    fig.suptitle(f"{TITLES[route]} · 最大误差与曲率的位置关系（黑点标记访问曲率的全局最大值）", fontsize=14)
    save_figure(fig, destination)


def steering_plot(runs, destination):
    selected = [run for run in runs if run["case"]["route"] == "right_angle" and run["case"]["algo"] in ("pp", "lqr_kinematic")]
    fig, axes = plt.subplots(4, 2, figsize=(13, 13), layout="constrained")
    for column, algo in enumerate(("pp", "lqr_kinematic")):
        group = sorted((run for run in selected if run["case"]["algo"] == algo), key=lambda run: -run["case"]["max_steer_deg"])
        ref = group[0]["reference"]
        axes[0, column].plot(ref["x_m"], ref["y_m"], "--", color="#7d8793", label="参考路径")
        for run in group:
            data, limit = run["trajectory"], run["case"]["max_steer_deg"]
            style = {"color": STEER_COLORS[limit], "label": f"上限 {limit:g}°", "linewidth": 1.5,
                     "linestyle": ":" if limit == 25 else "-"}
            axes[0, column].plot(data["x"], data["y"], **style)
            axes[1, column].plot(data["time"], data["lateral_error"], **style)
            axes[2, column].plot(data["time"], np.rad2deg(data["beta"]), **style)
            axes[3, column].plot(data["time"], data["yaw_rate"], **style)
        axes[0, column].set(xlabel="x / m", ylabel="y / m", title=LABELS[algo] + "：路径")
        axes[0, column].set_aspect("equal", adjustable="datalim")
        for row, (ylabel, title) in enumerate((("横向误差 / m", "横向误差"), ("β / °", "几何侧偏角"), ("横摆角速度 / (rad/s)", "横摆角速度")), start=1):
            axes[row, column].set(xlabel="时间 / s", ylabel=ylabel, title=LABELS[algo] + "：" + title)
        for ax in axes[:, column]:
            ax.legend(fontsize=9)
    fig.suptitle("车辆参数实验：直角弯，巡航目标 5.5 m/s；只改变最大前轮转角", fontsize=15)
    save_figure(fig, destination)


def dynamic_plot(runs, destination):
    selected = [run for run in runs if run["case"]["route"] == "s_curve" and run["case"]["max_steer_deg"] == 35]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), layout="constrained")
    for run in selected:
        data, algo = run["trajectory"], run["case"]["algo"]
        axes[0].plot(data["time"], data["lateral_error"], label=LABELS[algo], color=COLORS[algo])
        axes[1].plot(data["time"], np.rad2deg(data["steer"]), label=LABELS[algo], color=COLORS[algo])
    axes[0].set(xlabel="时间 / s", ylabel="横向误差 / m", title="跟踪误差")
    axes[1].set(xlabel="时间 / s", ylabel="转角 / °", title="控制输入")
    for ax in axes:
        ax.legend(ncol=4)
    fig.suptitle("补充：S 弯四算法对比（被控车辆仍统一为运动学模型）", fontsize=14)
    save_figure(fig, destination)


def format_value(value):
    if value is None:
        return "—"
    if isinstance(value, (bool, np.bool_)):
        return "是" if value else "否"
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def export_table(directory, name, rows, columns):
    """CSV retains full precision; Markdown rounds to four decimals."""
    with (directory / f"{name}.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=[key for key, _ in columns], extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    escape = lambda value: str(value).replace("|", "\\|").replace("\n", " ")
    lines = ["| " + " | ".join(escape(label) for _, label in columns) + " |",
             "| " + " | ".join("---" for _ in columns) + " |"]
    lines += ["| " + " | ".join(escape(format_value(row.get(key))) for key, _ in columns) + " |" for row in rows]
    text = "\n".join(lines) + "\n"
    (directory / f"{name}.md").write_text(text, encoding="utf-8")
    return text


def make_tables(runs, destination):
    rows = []
    for run in runs:
        case = run["case"]
        rows.append({"case": run["name"], "group": case["group"], "route": case["route"], "route_cn": TITLES[case["route"]],
                     "algo": case["algo"], "algo_cn": LABELS[case["algo"]], "max_steer_deg": case["max_steer_deg"],
                     "cruise_target_mps": float(run["reference"]["target_speed_mps"].max()), "status": run["status"], **run["metrics"]})
    core = sorted((row for row in rows if row["group"] == "core"), key=lambda row: (ROUTES.index(row["route"]), ALGORITHMS.index(row["algo"])))
    steering = sorted((row for row in rows if row["route"] == "right_angle" and row["algo"] in ("pp", "lqr_kinematic")), key=lambda row: (ALGORITHMS.index(row["algo"]), -row["max_steer_deg"]))
    dynamic = [row for row in rows if row["group"] == "supplement"]
    all_columns = [(key, key) for key in rows[0]]
    export_table(destination, "all_metrics", rows, all_columns)
    required = [("route_cn", "路线"), ("algo_cn", "算法"), ("reached_goal", "到达终点"), ("mean_lateral_error_m", "平均误差/m"),
                ("max_lateral_error_m", "最大误差/m"), ("max_steer_rad", "最大转角/rad"),
                ("max_normal_acceleration_mps2", "最大参考法向加速度/(m/s²)"), ("max_yaw_rate_radps", "最大横摆角速度/(rad/s)")]
    smooth = [("route_cn", "路线"), ("algo_cn", "算法"), ("completion_time_s", "完成用时/s"),
              ("mean_abs_steer_rate_radps", "Jδ/(rad/s)"), ("max_abs_steer_rate_radps", "最大变化率/(rad/s)"),
              ("steer_rate_above_config_fraction", "超过45°/s的比例"), ("steer_saturation_fraction", "转角饱和比例")]
    parameters = [("algo_cn", "算法"), ("max_steer_deg", "转角上限/°"), ("reached_goal", "到达终点"),
                  ("mean_lateral_error_m", "平均误差/m"), ("max_lateral_error_m", "最大误差/m"),
                  ("max_side_slip_beta_rad", "最大|β|/rad"), ("max_yaw_rate_radps", "最大|横摆角速度|/(rad/s)"),
                  ("steer_saturation_fraction", "转角饱和比例"), ("mean_abs_steer_rate_radps", "Jδ/(rad/s)")]
    peaks = [("route_cn", "路线"), ("algo_cn", "算法"), ("peak_error_time_s", "最大误差时刻/s"), ("peak_error_reference_s_m", "对应参考s/m"),
             ("abs_curvature_at_peak_error_1pm", "此处|κ|/(1/m)"), ("max_visited_abs_curvature_1pm", "访问曲率最大值/(1/m)"),
             ("first_peak_curvature_time_s", "最大曲率首次时刻/s"), ("last_peak_curvature_time_s", "最大曲率最后时刻/s")]
    contents = ["# 自动生成的数据表\n\n由原始 CSV 重新校验后生成。CSV 保留完整精度；表中的误差均取绝对值。\n"]
    for name, title, data, columns in (("core_metrics", "必做九组", core, required), ("smoothness", "平滑性与完成用时", core, smooth),
                                        ("steering_metrics", "转角上限实验（35°复用基准）", steering, parameters),
                                        ("peak_locations", "最大误差与访问曲率", core, peaks),
                                        ("dynamic_metrics", "动力学 LQR 补充", dynamic, required + [("mean_abs_steer_rate_radps", "Jδ/(rad/s)")])):
        contents.append(f"\n## {title}\n\n" + export_table(destination, name, data, columns))
    contents.append("\n参考 s 是仓库的路径参数：样条路线不严格等于平滑曲线的弧长。曲率最大值按各车实际访问的离散参考点计算，可能跳过某些参考点。\n")
    (destination.parent / "data_tables.md").write_text("\n".join(contents), encoding="utf-8")


def diagnostic_table(runs, destination):
    rows = []
    for run in runs:
        if run["case"]["group"] != "core":
            continue
        data = run["trajectory"]
        error_index = int(np.argmax(np.abs(data["lateral_error"])))
        peaks, _ = find_peaks(np.abs(data["curvature"]), prominence=0.02)
        preceding = peaks[peaks <= error_index]
        previous = int(preceding[-1]) if preceding.size else None
        rates = np.abs(np.diff(data["steer"]) / np.diff(data["time"]))
        rate_index = int(np.argmax(rates)) + 1
        rows.append({"route": run["case"]["route"], "algo": run["case"]["algo"],
                     "local_peak_prominence_1pm": 0.02,
                     "preceding_local_peak_time_s": float(data["time"][previous]) if previous is not None else None,
                     "error_minus_preceding_peak_s": float(data["time"][error_index] - data["time"][previous]) if previous is not None else None,
                     "max_rate_time_s": float(data["time"][rate_index]),
                     "speed_at_max_rate_mps": float(data["speed"][rate_index]),
                     "max_rate_is_terminal_command": bool(rate_index == len(data) - 1),
                     "note": "Plateau peak uses its midpoint; a global peak on another bend is not a causal delay reference."})
    export_table(destination, "local_diagnostics", rows, [(key, key) for key in rows[0]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / "reports")
    parser.add_argument("--refresh-generated", action="store_true", help="Replace generated tables/figures/validation only; never changes report.md")
    args = parser.parse_args()
    source, destination = args.input.resolve(), args.output.resolve()
    if destination.exists() and not args.refresh_generated:
        parser.error(f"Choose a new output directory (will not overwrite a report): {destination}")
    manifest, runs = load_runs(source)
    checks = [validate_run(run, manifest["speed_mode"]) for run in runs]
    # A core route's inputs must match exactly across algorithms.
    for route in ROUTES:
        group = [run for run in runs if run["case"]["route"] == route and run["case"]["group"] == "core"]
        assert len(group) == 3
        for run in group[1:]:
            for field in ("x_m", "y_m", "s_m", "target_speed_mps", "curvature_1pm"):
                np.testing.assert_array_equal(group[0]["reference"][field], run["reference"][field])
            assert run["saved_config"]["config"] == group[0]["saved_config"]["config"]
    figures, tables = destination / "figures", destination / "tables"
    figures.mkdir(parents=True, exist_ok=True)
    tables.mkdir(exist_ok=True)
    chosen_font = configure_plots()
    make_tables(runs, tables)
    diagnostic_table(runs, tables)
    for route in ROUTES:
        route_plot(runs, route, figures / f"comparison_{route}.png", manifest["speed_mode"])
        curvature_plot(runs, route, figures / f"curvature_error_{route}.png")
    metric_plot(runs, figures / "metrics_comparison.png")
    steering_plot(runs, figures / "steering_sensitivity.png")
    if any(run["case"]["group"] == "supplement" for run in runs):
        dynamic_plot(runs, figures / "dynamic_s_curve.png")
    # Include actual repository summary.png examples, not only custom comparison plots.
    for route in ROUTES:
        run = next(run for run in runs if run["case"] == {"group": "core", "route": route, "algo": "mpc", "max_steer_deg": 35.0})
        if (run["directory"] / "summary.png").exists():
            shutil.copy2(run["directory"] / "summary.png", figures / f"summary_mpc_{route}.png")
    validation = {"source": os.path.relpath(source, destination), "plot_font": chosen_font,
                  "checks": checks, "total_runs": len(runs), "reached_goal": sum(run["metrics"]["reached_goal"] for run in runs),
                  "core_input_equality": "PASS", "note": "No failed run removed. Numeric, model, state-transition and metric checks performed on raw CSV."}
    (destination / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Validated {len(runs)} runs; tables and figures: {destination}")


if __name__ == "__main__":
    main()
