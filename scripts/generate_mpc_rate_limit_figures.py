"""Generate Morandi-styled evidence for actuator-aware linear MPC.

The script compares the repository baseline MPC with the student MPC that
enforces the first control action's steering-rate limit.  It emits raw metrics,
trajectory data, PNG/SVG figures, and a small provenance manifest without
modifying the source simulation data.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import LabConfig


PALETTE = {
    "paper": "#F8F5F0",
    "panel": "#FDFBF7",
    "ink": "#45413C",
    "grid": "#DED7CE",
    "baseline": "#C48B7A",
    "enhanced": "#7F9C96",
    "reference": "#625C55",
    "warm": "#C7A96B",
    "purple": "#9B8FA6",
}
LABELS = {
    "baseline_mpc": "基准 MPC",
    "actuator_aware_mpc": "执行器感知 MPC",
}
SPEED_LABELS = {"low": "低速 3 m/s", "medium": "中速 5 m/s", "high": "高速 7 m/s"}


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": ["Microsoft YaHei", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.facecolor": PALETTE["paper"],
            "axes.facecolor": PALETTE["panel"],
            "axes.edgecolor": "#756E65",
            "axes.labelcolor": PALETTE["ink"],
            "xtick.color": "#625C55",
            "ytick.color": "#625C55",
            "text.color": PALETTE["ink"],
            "axes.titleweight": "bold",
            "axes.grid": True,
            "grid.color": PALETTE["grid"],
            "grid.linewidth": 0.6,
            "grid.alpha": 0.9,
            "legend.frameon": False,
        }
    )


def max_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def run_case(condition: str, speed_mode: str) -> tuple[dict, list[dict], dict]:
    config = LabConfig()
    config.sim.route_name = "circle"
    config.sim.speed_mode = speed_mode
    controller = load_controller(
        "mpc", "solution" if condition == "baseline_mpc" else "student"
    )
    path, records, _ = run_simulation(controller, config=config)
    if not records:
        raise RuntimeError(f"{condition}/{speed_mode} 未产生仿真记录")

    times = np.asarray([record.time for record in records], dtype=float)
    steer = np.asarray([record.steer for record in records], dtype=float)
    lateral_error = np.asarray([record.lateral_error for record in records], dtype=float)
    steer_rate = np.diff(steer) / config.sim.dt
    final = records[-1]
    finish_distance = math.hypot(final.x - path.x[-1], final.y - path.y[-1])
    reached_goal = (
        finish_distance < config.sim.stop_distance and final.speed < config.sim.stop_speed
    )
    metrics = {
        "condition": condition,
        "speed_mode": speed_mode,
        "speed_mps": float(path.target_speed[0]),
        "steps": len(records),
        "reached_goal": reached_goal,
        "mean_abs_lateral_error_m": float(np.mean(np.abs(lateral_error))),
        "max_abs_lateral_error_m": max_abs(lateral_error.tolist()),
        "mean_abs_steer_rate_radps": float(np.mean(np.abs(steer_rate))),
        "max_abs_steer_rate_radps": max_abs(steer_rate.tolist()),
        "max_abs_steer_rad": max_abs(steer.tolist()),
        "finish_distance_m": finish_distance,
    }
    trajectory = [
        {
            "condition": condition,
            "speed_mode": speed_mode,
            "time_s": record.time,
            "x_m": record.x,
            "y_m": record.y,
            "lateral_error_m": record.lateral_error,
            "steer_rad": record.steer,
            "steer_rate_radps": 0.0 if index == 0 else steer_rate[index - 1],
        }
        for index, record in enumerate(records)
    ]
    reference = {"x": path.x, "y": path.y}
    return metrics, trajectory, reference


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def draw_dashboard(trajectory_rows: list[dict], reference: dict, output_dir: Path) -> None:
    focus_rows = [row for row in trajectory_rows if row["speed_mode"] == "high"]
    fig = plt.figure(figsize=(13.2, 7.4), dpi=300, facecolor=PALETTE["paper"])
    grid = fig.add_gridspec(2, 2, width_ratios=[1.08, 1.0], hspace=0.35, wspace=0.28)
    ax_path = fig.add_subplot(grid[:, 0])
    ax_steer = fig.add_subplot(grid[0, 1])
    ax_error = fig.add_subplot(grid[1, 1])

    ax_path.plot(reference["x"], reference["y"], "--", color=PALETTE["reference"], lw=1.3, label="参考路径")
    for condition, color in (("baseline_mpc", PALETTE["baseline"]), ("actuator_aware_mpc", PALETTE["enhanced"])):
        rows = [row for row in focus_rows if row["condition"] == condition]
        ax_path.plot([row["x_m"] for row in rows], [row["y_m"] for row in rows], color=color, lw=2.0, label=LABELS[condition])
        ax_path.scatter(rows[0]["x_m"], rows[0]["y_m"], s=30, color=color, zorder=3)
    ax_path.set_title("高速圆形路径：轨迹对比")
    ax_path.set_xlabel("东向坐标（m）")
    ax_path.set_ylabel("北向坐标（m）")
    ax_path.set_aspect("equal", adjustable="box")
    ax_path.legend(loc="best")

    for condition, color in (("baseline_mpc", PALETTE["baseline"]), ("actuator_aware_mpc", PALETTE["enhanced"])):
        rows = [row for row in focus_rows if row["condition"] == condition]
        ax_steer.plot([row["time_s"] for row in rows], [row["steer_rad"] for row in rows], color=color, lw=1.5, label=LABELS[condition])
        ax_error.plot([row["time_s"] for row in rows], [np.abs(row["lateral_error_m"]) for row in rows], color=color, lw=1.5, label=LABELS[condition])
    ax_steer.set_title("前轮转角时序")
    ax_steer.set_xlabel("时间（s）")
    ax_steer.set_ylabel("转角（rad）")
    ax_steer.legend(loc="upper right")
    ax_error.set_title("绝对横向误差时序")
    ax_error.set_xlabel("时间（s）")
    ax_error.set_ylabel("绝对误差（m）")
    ax_error.legend(loc="upper right")

    fig.suptitle("执行器感知 MPC：约束首步转角变化率的效果", fontsize=15, fontweight="bold", y=0.98)
    fig.text(0.5, 0.015, "莫兰迪配色｜基准与增强条件使用同一路线、车辆与速度；未进行显著性检验", ha="center", fontsize=8, color="#625C55")
    fig.savefig(output_dir / "mpc_rate_limit_dashboard.png", bbox_inches="tight")
    fig.savefig(output_dir / "mpc_rate_limit_dashboard.svg", bbox_inches="tight")
    plt.close(fig)


def draw_tradeoff(metrics_rows: list[dict], output_dir: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13.2, 8.6), dpi=300, facecolor=PALETTE["paper"])
    fig.subplots_adjust(hspace=0.38, wspace=0.30, top=0.90)
    speeds = ["low", "medium", "high"]
    x = np.arange(len(speeds))
    by_condition = {
        condition: [next(row for row in metrics_rows if row["condition"] == condition and row["speed_mode"] == speed) for speed in speeds]
        for condition in LABELS
    }

    ax = axes[0, 0]
    for condition, color in (("baseline_mpc", PALETTE["baseline"]), ("actuator_aware_mpc", PALETTE["enhanced"])):
        values = [row["mean_abs_lateral_error_m"] for row in by_condition[condition]]
        ax.plot(x, values, marker="o", ms=6, lw=2, color=color, label=LABELS[condition])
    ax.set_title("跟踪精度随速度变化")
    ax.set_xticks(x, [SPEED_LABELS[speed] for speed in speeds])
    ax.set_ylabel("平均绝对横向误差（m）")
    ax.legend(loc="best")

    ax = axes[0, 1]
    width = 0.34
    for offset, (condition, color) in zip((-width / 2, width / 2), (("baseline_mpc", PALETTE["baseline"]), ("actuator_aware_mpc", PALETTE["enhanced"]))):
        values = [row["mean_abs_steer_rate_radps"] for row in by_condition[condition]]
        ax.bar(x + offset, values, width=width, color=color, label=LABELS[condition])
    ax.set_title("转向平滑性：平均绝对转角变化率")
    ax.set_xticks(x, [SPEED_LABELS[speed] for speed in speeds])
    ax.set_ylabel("平均 |dδ/dt|（rad/s）")
    ax.legend(loc="best")

    ax = axes[1, 0]
    for condition, color, marker in (("baseline_mpc", PALETTE["baseline"], "o"), ("actuator_aware_mpc", PALETTE["enhanced"], "D")):
        for row in by_condition[condition]:
            ax.scatter(row["mean_abs_lateral_error_m"], row["mean_abs_steer_rate_radps"], s=75, marker=marker, color=color, edgecolor=PALETTE["panel"], linewidth=0.8, label=LABELS[condition] if row["speed_mode"] == "low" else None, zorder=3)
            ax.annotate(SPEED_LABELS[row["speed_mode"]].split()[0], (row["mean_abs_lateral_error_m"], row["mean_abs_steer_rate_radps"]), xytext=(6, 5), textcoords="offset points", fontsize=8)
    ax.set_title("精度—平滑性权衡")
    ax.set_xlabel("平均绝对横向误差（m）")
    ax.set_ylabel("平均 |dδ/dt|（rad/s）")
    ax.legend(loc="best")

    metric_keys = ["mean_abs_lateral_error_m", "max_abs_lateral_error_m", "mean_abs_steer_rate_radps", "max_abs_steer_rate_radps"]
    metric_labels = ["平均误差", "最大误差", "平均变化率", "最大变化率"]
    delta = np.array(
        [
            [100.0 * (by_condition["actuator_aware_mpc"][i][key] - by_condition["baseline_mpc"][i][key]) / max(abs(by_condition["baseline_mpc"][i][key]), 1.0e-9) for key in metric_keys]
            for i in range(len(speeds))
        ]
    )
    ax = axes[1, 1]
    bound = max(5.0, float(np.max(np.abs(delta))))
    image = ax.imshow(delta, cmap="PuOr", norm=TwoSlopeNorm(vmin=-bound, vcenter=0.0, vmax=bound), aspect="auto")
    ax.set_title("增强相对基准的变化（%）")
    ax.set_xticks(np.arange(len(metric_labels)), metric_labels)
    ax.set_yticks(np.arange(len(speeds)), [SPEED_LABELS[speed] for speed in speeds])
    for row_index in range(delta.shape[0]):
        for column_index in range(delta.shape[1]):
            ax.text(column_index, row_index, f"{delta[row_index, column_index]:+.1f}", ha="center", va="center", fontsize=8)
    colorbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.04)
    colorbar.set_label("相对变化（%）")

    fig.suptitle("执行器感知 MPC：性能与平滑性的多视图权衡", fontsize=15, fontweight="bold")
    fig.text(0.5, 0.015, "负值代表增强方案较小；热图为描述性相对变化，不表示统计显著性", ha="center", fontsize=8, color="#625C55")
    fig.savefig(output_dir / "mpc_rate_limit_tradeoff.png", bbox_inches="tight")
    fig.savefig(output_dir / "mpc_rate_limit_tradeoff.svg", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="生成执行器感知 MPC 的莫兰迪风格对比图")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mpc_rate_limit_highlight"))
    args = parser.parse_args()
    configure_style()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    metrics_rows: list[dict] = []
    trajectory_rows: list[dict] = []
    high_reference = None
    for condition in LABELS:
        for speed_mode in ("low", "medium", "high"):
            metrics, trajectory, reference = run_case(condition, speed_mode)
            metrics_rows.append(metrics)
            trajectory_rows.extend(trajectory)
            if speed_mode == "high":
                high_reference = reference

    if high_reference is None:
        raise RuntimeError("缺少高速参考路径，无法绘制主图")
    write_csv(args.output_dir / "comparison_metrics.csv", metrics_rows)
    write_csv(args.output_dir / "comparison_trajectory.csv", trajectory_rows)
    draw_dashboard(trajectory_rows, high_reference, args.output_dir)
    draw_tradeoff(metrics_rows, args.output_dir)
    manifest = {
        "question": "首步转角变化率约束是否降低 MPC 控制突变？",
        "route": "circle",
        "conditions": list(LABELS),
        "speed_modes": ["low", "medium", "high"],
        "statistics": "descriptive only; deterministic simulation runs",
        "outputs": [
            "comparison_metrics.csv",
            "comparison_trajectory.csv",
            "mpc_rate_limit_dashboard.png",
            "mpc_rate_limit_dashboard.svg",
            "mpc_rate_limit_tradeoff.png",
            "mpc_rate_limit_tradeoff.svg",
        ],
    }
    (args.output_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"output_dir={args.output_dir}")


if __name__ == "__main__":
    main()
