"""Run the complete NullPointer dorm-to-classroom navigation experiment.

The route is reconstructed in local metres from the user-supplied Amap walking
route screenshot.  Its official displayed length is 868 m.  It is therefore a
screen-derived reference route rather than a survey-grade GPS trajectory.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vdm_lab.common.path import generate_reference_path
from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import LabConfig


OUT = ROOT / "outputs" / "nullpointer_completion"
MAP_SCREENSHOT = Path(r"C:\Users\wblxr\.codex\attachments\f20db65d-8b90-49fc-a418-4dc8eca0efa5\image-1.jpg")

# Coordinates are east/north metres.  They trace the visible route from the
# dormitory-side start marker to the teaching-building-side goal marker.  Short
# intermediate segments round the road turns visible in the Amap screenshot.
ROUTE_WAYPOINTS_M = np.asarray(
    [
        (0.0, 0.0),
        (-45.0, 400.0),
        (-55.0, 425.0),
        (-185.0, 440.0),
        (-205.0, 470.0),
        (-210.0, 500.0),
        (-360.0, 505.0),
        (-380.0, 530.0),
        (-380.0, 590.0),
    ],
    dtype=float,
)

PALETTE = {
    "pp": "#6E8BA0",
    "lqr_kinematic": "#C78B7B",
    "lqr_dynamic": "#8AA6A3",
    "mpc": "#A6B493",
    "ref": "#4F5552",
    "alert": "#A85C55",
    "warm": "#C7B299",
    "paper": "#FAF8F5",
}
LABEL = {"pp": "PP", "lqr_kinematic": "运动学 LQR", "lqr_dynamic": "动力学 LQR", "mpc": "MPC"}


def font_setup() -> None:
    plt.rcParams.update(
        {
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "Arial", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "figure.facecolor": PALETTE["paper"],
            "axes.facecolor": "#FFFFFF",
            "savefig.facecolor": PALETTE["paper"],
        }
    )


def route_path(speed_mps: float):
    """Create a smooth, metre-scale controller reference path."""
    return generate_reference_path(
        waypoints=ROUTE_WAYPOINTS_M,
        ds=1.0,
        target_speed=speed_mps,
    )


def make_config(speed_mps: float, *, max_steer_deg: float = 35.0, wheelbase_m: float = 2.5) -> LabConfig:
    config = LabConfig()
    config.sim.dt = 0.1
    config.sim.max_time = 430.0
    config.sim.stop_distance = 1.5
    config.sim.stop_speed = 0.5
    config.sim.target_speed = speed_mps
    # The navigation experiment treats the requested cruise speed as the route
    # speed limit.  Applying it to every controller prevents MPC from gaining
    # an artificial advantage by accelerating far beyond the reference speed.
    config.vehicle.max_speed = speed_mps
    config.vehicle.max_steer = math.radians(max_steer_deg)
    config.vehicle.lf = wheelbase_m / 2.0
    config.vehicle.lr = wheelbase_m / 2.0
    return config


def metrics(algo: str, factor: str, level: float, path, records, config: LabConfig) -> dict:
    error = np.asarray([record.lateral_error for record in records])
    steer = np.asarray([record.steer for record in records])
    rate = np.diff(steer) / config.sim.dt
    final = records[-1]
    goal_distance = math.hypot(final.x - path.x[-1], final.y - path.y[-1])
    peak_index = int(np.argmax(np.abs(error)))
    return {
        "algorithm": algo,
        "factor": factor,
        "level": float(level),
        "reached_goal": bool(goal_distance < config.sim.stop_distance and final.speed < config.sim.stop_speed),
        "route_length_m": float(path.s[-1]),
        "duration_s": float(final.time),
        "mean_abs_lateral_error_m": float(np.mean(np.abs(error))),
        "max_abs_lateral_error_m": float(np.max(np.abs(error))),
        "mean_abs_steer_rate_radps": float(np.mean(np.abs(rate))) if len(rate) else 0.0,
        "max_abs_steer_rad": float(np.max(np.abs(steer))),
        "peak_time_s": float(records[peak_index].time),
        "peak_x_m": float(records[peak_index].x),
        "peak_y_m": float(records[peak_index].y),
        "peak_lateral_error_m": float(error[peak_index]),
    }


def run_case(algo: str, factor: str, level: float, *, speed=5.0, max_steer=35.0, wheelbase=2.5):
    config = make_config(speed, max_steer_deg=max_steer, wheelbase_m=wheelbase)
    path = route_path(speed)
    path, records, _ = run_simulation(load_controller(algo, "student"), config=config, path_override=path)
    return metrics(algo, factor, level, path, records, config), path, records


def write_csv(rows: list[dict], path: Path) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def axis_style(ax) -> None:
    ax.grid(True, alpha=0.25, linewidth=0.7)
    ax.spines[["top", "right"]].set_visible(False)


def render_route_figure(path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13.2, 6.2), gridspec_kw={"width_ratios": [0.88, 1]})
    if MAP_SCREENSHOT.exists():
        image = plt.imread(MAP_SCREENSHOT)
        axes[0].imshow(image)
        axes[0].set_title("高德步行路线原图（用户提供）", fontsize=12, weight="bold")
        axes[0].axis("off")
    else:
        axes[0].text(0.5, 0.5, "未找到地图截图", ha="center", va="center")
        axes[0].axis("off")

    ax = axes[1]
    ax.plot(path.x, path.y, color=PALETTE["ref"], linewidth=2.0, label="平滑参考路径")
    ax.scatter(path.x[0], path.y[0], s=95, color=PALETTE["pp"], zorder=3, label="寝室起点")
    ax.scatter(path.x[-1], path.y[-1], s=105, color=PALETTE["alert"], marker="*", zorder=3, label="教学楼终点")
    ax.plot(ROUTE_WAYPOINTS_M[:, 0], ROUTE_WAYPOINTS_M[:, 1], "o--", color=PALETTE["warm"], alpha=0.8, label="截图提取路点")
    ax.annotate("起点", (path.x[0], path.y[0]), xytext=(10, -14), textcoords="offset points")
    ax.annotate("终点", (path.x[-1], path.y[-1]), xytext=(10, 5), textcoords="offset points")
    ax.set_title("本地 ENU 坐标中的 868 m 导航参考路径", fontsize=12, weight="bold")
    ax.set_xlabel("东向 x（m）")
    ax.set_ylabel("北向 y（m）")
    ax.set_aspect("equal", adjustable="box")
    axis_style(ax)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"amap_route_reconstruction.{suffix}", dpi=300 if suffix == "png" else None, bbox_inches="tight")
    plt.close(fig)


def render_pp_figure(path, records, row: dict) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 5.4), gridspec_kw={"width_ratios": [1.1, 0.9]})
    ax = axes[0]
    ax.plot(path.x, path.y, color=PALETTE["ref"], linewidth=2.0, label="参考路径")
    ax.plot([r.x for r in records], [r.y for r in records], color=PALETTE["pp"], linewidth=1.6, label="PP 实际轨迹")
    ax.scatter(row["peak_x_m"], row["peak_y_m"], color=PALETTE["alert"], s=58, zorder=4, label="最大横向误差")
    ax.annotate(f"最大误差 {row['max_abs_lateral_error_m']:.2f} m\nt={row['peak_time_s']:.1f} s", (row["peak_x_m"], row["peak_y_m"]), xytext=(12, 14), textcoords="offset points", fontsize=8, arrowprops={"arrowstyle": "->", "color": PALETTE["alert"]})
    ax.set_title("PP 在寝室—教学楼路线上的跟踪轨迹", fontsize=12, weight="bold")
    ax.set_xlabel("东向 x（m）")
    ax.set_ylabel("北向 y（m）")
    ax.set_aspect("equal", adjustable="box")
    axis_style(ax)
    ax.legend(fontsize=8)

    ax = axes[1]
    time = [record.time for record in records]
    ax.plot(time, np.abs([record.lateral_error for record in records]), color=PALETTE["pp"], linewidth=1.4)
    ax.axvline(row["peak_time_s"], color=PALETTE["alert"], linestyle="--", linewidth=1)
    ax.set_title("PP 横向误差时序", fontsize=12, weight="bold")
    ax.set_xlabel("时间（s）")
    ax.set_ylabel("绝对横向误差（m）")
    axis_style(ax)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"pp_navigation_tracking.{suffix}", dpi=300 if suffix == "png" else None, bbox_inches="tight")
    plt.close(fig)


def render_sweep_figure(rows: list[dict]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.8, 9.0))
    speed_rows = [row for row in rows if row["factor"] == "speed"]
    for algo in LABEL:
        data = sorted((row for row in speed_rows if row["algorithm"] == algo), key=lambda row: row["level"])
        if not data:
            continue
        axes[0, 0].plot([row["level"] for row in data], [row["mean_abs_lateral_error_m"] for row in data], "o-", color=PALETTE[algo], label=LABEL[algo])
        axes[0, 1].plot([row["level"] for row in data], [row["mean_abs_steer_rate_radps"] for row in data], "o-", color=PALETTE[algo], label=LABEL[algo])
    axes[0, 0].set_title("变量 1：速度对平均跟踪误差的影响", weight="bold")
    axes[0, 0].set_xlabel("目标速度（m/s）")
    axes[0, 0].set_ylabel("平均绝对横向误差（m）")
    axes[0, 1].set_title("速度对控制激烈程度的影响", weight="bold")
    axes[0, 1].set_xlabel("目标速度（m/s）")
    axes[0, 1].set_ylabel("平均绝对转角变化率（rad/s）")
    for ax in axes[0]:
        axis_style(ax)
        ax.legend(fontsize=8)

    steer_rows = sorted((row for row in rows if row["factor"] == "max_steer"), key=lambda row: row["level"])
    axes[1, 0].plot([row["level"] for row in steer_rows], [row["mean_abs_lateral_error_m"] for row in steer_rows], "o-", color=PALETTE["mpc"])
    for row in steer_rows:
        axes[1, 0].annotate("到达" if row["reached_goal"] else "未到达", (row["level"], row["mean_abs_lateral_error_m"]), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=8)
    axes[1, 0].set_title("变量 2：最大转角对 PP 误差的影响", weight="bold")
    axes[1, 0].set_xlabel("最大前轮转角（deg）")
    axes[1, 0].set_ylabel("平均绝对横向误差（m）")
    axis_style(axes[1, 0])

    wheel_rows = sorted((row for row in rows if row["factor"] == "wheelbase"), key=lambda row: row["level"])
    axes[1, 1].plot([row["level"] for row in wheel_rows], [row["mean_abs_lateral_error_m"] for row in wheel_rows], "o-", color=PALETTE["lqr_dynamic"], label="动力学 LQR")
    axes[1, 1].plot([row["level"] for row in wheel_rows], [row["max_abs_steer_rad"] for row in wheel_rows], "s--", color=PALETTE["warm"], label="最大转角")
    axes[1, 1].set_title("变量 3：轴距对动力学模型的影响", weight="bold")
    axes[1, 1].set_xlabel("轴距（m）")
    axes[1, 1].set_ylabel("误差（m）或转角（rad）")
    axis_style(axes[1, 1])
    axes[1, 1].legend(fontsize=8)
    fig.suptitle("NullPointer 三变量敏感性实验（莫兰迪配色）", fontsize=15, weight="bold", y=0.99)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"three_variable_sweeps.{suffix}", dpi=300 if suffix == "png" else None, bbox_inches="tight")
    plt.close(fig)


def render_model_figure(rows: list[dict], trajectories: dict[str, tuple]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2))
    speed_rows = [row for row in rows if row["factor"] == "speed" and row["algorithm"] in {"lqr_kinematic", "lqr_dynamic"}]
    for algo in ("lqr_kinematic", "lqr_dynamic"):
        data = sorted((row for row in speed_rows if row["algorithm"] == algo), key=lambda row: row["level"])
        axes[0].plot([row["level"] for row in data], [row["mean_abs_lateral_error_m"] for row in data], "o-", color=PALETTE[algo], label=LABEL[algo])
    axes[0].set_title("运动学与动力学 LQR 的速度敏感性", weight="bold")
    axes[0].set_xlabel("目标速度（m/s）")
    axes[0].set_ylabel("平均绝对横向误差（m）")
    axis_style(axes[0])
    axes[0].legend()

    path, _ = trajectories["lqr_kinematic"]
    axes[1].plot(path.x, path.y, color=PALETTE["ref"], linewidth=2, label="参考路径")
    for algo in ("lqr_kinematic", "lqr_dynamic"):
        _, records = trajectories[algo]
        axes[1].plot([r.x for r in records], [r.y for r in records], color=PALETTE[algo], linewidth=1.4, label=LABEL[algo])
    axes[1].set_title("中速导航路线上的模型跟踪对比", weight="bold")
    axes[1].set_xlabel("东向 x（m）")
    axes[1].set_ylabel("北向 y（m）")
    axes[1].set_aspect("equal", adjustable="box")
    axis_style(axes[1])
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    for suffix in ("png", "svg"):
        fig.savefig(OUT / f"kinematic_dynamic_comparison.{suffix}", dpi=300 if suffix == "png" else None, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    font_setup()
    rows: list[dict] = []
    stored: dict[str, tuple] = {}

    # Variable 1: speed.  PP and both LQR variants expose the speed/model
    # mechanisms without repeating a computationally identical MPC QP sweep.
    for speed in (3.0, 5.0, 7.0):
        for algo in ("pp", "lqr_kinematic", "lqr_dynamic"):
            row, path, records = run_case(algo, "speed", speed, speed=speed)
            rows.append(row)
            if speed == 5.0:
                stored[algo] = (path, records)

    # A common 5 m/s baseline includes all four controllers, including the
    # constraint-aware MPC used elsewhere in the report.
    row, path, records = run_case("mpc", "baseline", 5.0, speed=5.0)
    rows.append(row)
    stored["mpc"] = (path, records)

    # Variable 2: front steering authority, with PP as a transparent geometry
    # baseline whose degradation under saturation is easy to interpret.
    for max_steer in (5.0, 8.0, 12.0, 20.0):
        row, _, _ = run_case("pp", "max_steer", max_steer, speed=5.0, max_steer=max_steer)
        rows.append(row)

    # Variable 3: vehicle wheelbase, using the explicitly parameterized dynamic bicycle controller.
    for wheelbase in (2.2, 2.5, 2.8):
        row, _, _ = run_case("lqr_dynamic", "wheelbase", wheelbase, speed=5.0, wheelbase=wheelbase)
        rows.append(row)

    write_csv(rows, OUT / "navigation_sweep_metrics.csv")
    metadata = {
        "route_source": "Amap screenshot supplied by user; manually reconstructed into local ENU waypoints.",
        "displayed_walking_distance_m": 868.0,
        "reference_length_m": float(route_path(5.0).s[-1]),
        "waypoint_count": int(len(ROUTE_WAYPOINTS_M)),
        "coordinate_frame": "Local east/north metres; not survey-grade longitude/latitude.",
        "speed_mps": [3.0, 5.0, 7.0],
        "max_steer_deg": [20.0, 25.0, 30.0, 35.0],
        "wheelbase_m": [2.2, 2.5, 2.8],
    }
    (OUT / "navigation_metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    pp_row = next(row for row in rows if row["algorithm"] == "pp" and row["factor"] == "speed" and row["level"] == 5.0)
    render_route_figure(route_path(5.0))
    render_pp_figure(*stored["pp"], pp_row)
    render_sweep_figure(rows)
    render_model_figure(rows, stored)
    print(f"output={OUT}")


if __name__ == "__main__":
    main()
