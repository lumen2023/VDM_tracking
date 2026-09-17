#!/usr/bin/env python3
"""车辆运动学 / 动力学模型对比实验。

在相同的参考路径、相同的控制器（Pure Pursuit）和相同的目标速度下，比较
三种被控车辆模型：

1. ``kinematic``  : 标准运动学自行车模型（含几何侧偏角 beta）
2. ``simplified`` : 简化运动学模型（忽略侧偏角，beta = 0）
3. ``dynamic``    : 线性二自由度动力学模型（含轮胎侧偏力）

本脚本复用仓库的统一仿真框架（``run_simulation`` + ``ReferenceTracker`` +
``solutions.pure_pursuit``），因此得到的轨迹、横向误差和日志字段与
``run_experiment.py`` 完全一致，可以直接和 PP / LQR / MPC 的实验结果对比。
对每个 (路线, 速度) 组合分别运行三种模型，输出轨迹对比图、误差曲线和
汇总 CSV。

运行方式（仓库根目录）：

    python vdm_lab/tasks/compare_models.py
    python vdm_lab/tasks/compare_models.py --route circle --speed medium
"""

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path as FsPath

import matplotlib

matplotlib.use("Agg")  # 无界面后端，便于批量和服务器运行

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(FsPath(__file__).resolve().parents[2]))

from vdm_lab.common.logging import (  # noqa: E402
    compute_metrics,
    save_records,
    save_reference_path,
)
from vdm_lab.common.path import generate_reference_path  # noqa: E402
from vdm_lab.common.simulation import run_simulation  # noqa: E402
from vdm_lab.common.types import LabConfig  # noqa: E402
from vdm_lab.common.vehicle_backend import (  # noqa: E402
    DynamicBicycleBackend,
    KinematicBicycleBackend,
    SimplifiedKinematicBackend,
)
from vdm_lab.config.speed_profiles import available_speed_modes  # noqa: E402
from vdm_lab.config.vehicle_params import (  # noqa: E402
    DEFAULT_VEHICLE_NAME,
    make_vehicle_config,
)
from vdm_lab.solutions import pure_pursuit  # noqa: E402


# 模型注册表：名称 -> (中文标签, 英文图例, 后端类, 颜色)
MODELS = {
    "kinematic": (
        "标准运动学 (含 beta)",
        "Kinematic (with beta)",
        KinematicBicycleBackend,
        "#1f77b4",
    ),
    "simplified": (
        "简化运动学 (beta=0)",
        "Simplified kinematic (beta=0)",
        SimplifiedKinematicBackend,
        "#d62728",
    ),
    "dynamic": (
        "二自由度动力学",
        "2-DOF dynamic (tire forces)",
        DynamicBicycleBackend,
        "#2ca02c",
    ),
}

# 默认实验矩阵：路线 -> 速度档位
DEFAULT_EXPERIMENTS = {
    "circle": ["low", "medium", "high"],
    "s_curve": ["low", "medium", "high"],
    "right_angle": ["low", "medium", "high"],
}

METRIC_ROWS = [
    ("mean_lateral_error_m", "平均横向误差/m", "{:.4f}"),
    ("max_lateral_error_m", "最大横向误差/m", "{:.4f}"),
    ("finish_error_m", "终点误差/m", "{:.4f}"),
    ("max_steer_rad", "最大转角/rad", "{:.4f}"),
    ("max_side_slip_beta_rad", "最大侧偏角/rad", "{:.4f}"),
    ("max_yaw_rate_radps", "最大横摆角速度/rad/s", "{:.4f}"),
]


def parse_args():
    parser = argparse.ArgumentParser(description="车辆运动学/动力学模型对比实验")
    parser.add_argument(
        "--route",
        choices=list(DEFAULT_EXPERIMENTS.keys()),
        default=None,
        help="只运行指定路线；默认运行 circle / s_curve / right_angle 三条路线",
    )
    parser.add_argument(
        "--speed",
        choices=available_speed_modes(),
        default=None,
        help="只运行指定速度档；默认运行 low / medium / high 三档",
    )
    parser.add_argument(
        "--vehicle",
        choices=[DEFAULT_VEHICLE_NAME],
        default=DEFAULT_VEHICLE_NAME,
        help="车辆参数组",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="输出目录；默认为 outputs/<时间戳>_model_comparison",
    )
    return parser.parse_args()


def _records_to_arrays(records):
    return {
        "x": np.array([r.x for r in records]),
        "y": np.array([r.y for r in records]),
        "lateral_error": np.array([r.lateral_error for r in records]),
        "steer": np.array([r.steer for r in records]),
        "beta": np.array([r.beta for r in records]),
        "yaw_rate": np.array([r.yaw_rate for r in records]),
        "speed": np.array([r.speed for r in records]),
    }


def run_one_model(model_key, path, config):
    """用同一路径和同一控制器运行一种车辆模型，返回记录和指标。"""
    backend_cls = MODELS[model_key][2]
    _, records, _ = run_simulation(
        pure_pursuit,
        config=config,
        vehicle_backend=backend_cls(),
        path_override=path,
    )
    return records, compute_metrics(path, records)


def plot_comparison(path, results, title, filename):
    """绘制轨迹、横向误差、转角和侧偏/横摆对比图。"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # (0,0) 轨迹对比
    ax = axes[0, 0]
    ax.plot(path.x, path.y, "k--", linewidth=2, label="Reference")
    for model_key, (_, label, _, color) in MODELS.items():
        data = results[model_key]["arrays"]
        ax.plot(data["x"], data["y"], color=color, linewidth=1.8, label=label)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory")
    ax.legend(fontsize=8)
    ax.grid(True)
    ax.set_aspect("equal", adjustable="datalim")

    # (0,1) 横向误差
    ax = axes[0, 1]
    for model_key, (_, label, _, color) in MODELS.items():
        ax.plot(results[model_key]["arrays"]["lateral_error"], color=color, label=label)
    ax.set_xlabel("Time step")
    ax.set_ylabel("Lateral error (m)")
    ax.set_title("Lateral error")
    ax.legend(fontsize=8)
    ax.grid(True)

    # (1,0) 前轮转角
    ax = axes[1, 0]
    for model_key, (_, label, _, color) in MODELS.items():
        ax.plot(results[model_key]["arrays"]["steer"], color=color, label=label)
    ax.set_xlabel("Time step")
    ax.set_ylabel("Front steering angle (rad)")
    ax.set_title("Steering angle")
    ax.legend(fontsize=8)
    ax.grid(True)

    # (1,1) 侧偏角与横摆角速度
    ax = axes[1, 1]
    for model_key, (_, label, _, color) in MODELS.items():
        arrays = results[model_key]["arrays"]
        ax.plot(
            arrays["beta"],
            color=color,
            linestyle="-",
            label=f"beta {label}",
        )
    ax.set_xlabel("Time step")
    ax.set_ylabel("Body sideslip beta (rad)")
    ax.set_title("Body sideslip angle")
    ax.legend(fontsize=8)
    ax.grid(True)

    fig.suptitle(title, fontsize=16, fontweight="bold")
    fig.tight_layout()
    fig.savefig(filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  已保存图像: {filename}")


def print_metric_table(results, title):
    print(f"\n{title}")
    header = f"{'指标':<22}" + "".join(
        f"{zh:>22}" for zh, _, _, _ in MODELS.values()
    )
    print(header)
    print("-" * len(header))
    for key, cn_name, fmt in METRIC_ROWS:
        row = f"{cn_name:<22}"
        for model_key in MODELS:
            value = results[model_key]["metrics"][key]
            row += f"{fmt.format(value):>22}"
        print(row)
    row = f"{'reached_goal':<22}"
    for model_key in MODELS:
        row += f"{str(results[model_key]['metrics']['reached_goal']):>22}"
    print(row)


def main():
    args = parse_args()

    routes = [args.route] if args.route else list(DEFAULT_EXPERIMENTS.keys())
    speed_modes = [args.speed] if args.speed else available_speed_modes()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        FsPath(args.output)
        if args.output
        else FsPath("outputs") / f"{stamp}_model_comparison"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    config = LabConfig(vehicle=make_vehicle_config(args.vehicle))
    config.sim.animate = False

    summary_rows = []

    print("=" * 72)
    print("车辆运动学 / 动力学模型对比实验")
    print(f"控制器: {pure_pursuit.NAME} (三条路线使用同一控制器)")
    print(f"车辆参数组: {args.vehicle}")
    print(f"输出目录: {output_dir}")
    print("=" * 72)

    for route in routes:
        for speed_mode in DEFAULT_EXPERIMENTS.get(route, speed_modes):
            if args.speed and speed_mode != args.speed:
                continue

            path = generate_reference_path(
                route_name=route,
                speed_mode=speed_mode,
                ds=config.sim.waypoint_ds,
            )
            config.sim.route_name = route
            config.sim.speed_mode = speed_mode

            label = f"{route} @ {speed_mode}"
            print(f"\n实验: {label}  (路径长度 {path.s[-1]:.1f} m)")
            print("-" * 72)

            results = {}
            for model_key, (model_label, _, _, _) in MODELS.items():
                records, metrics = run_one_model(model_key, path, config)
                results[model_key] = {
                    "arrays": _records_to_arrays(records),
                    "metrics": metrics,
                    "records": records,
                }
                print(
                    f"  {model_label:<24} 平均误差 {metrics['mean_lateral_error_m']:.4f} m, "
                    f"最大误差 {metrics['max_lateral_error_m']:.4f} m, "
                    f"beta_max {metrics['max_side_slip_beta_rad']:.4f} rad"
                )
                summary_rows.append(
                    {
                        "route": route,
                        "speed_mode": speed_mode,
                        "model": model_key,
                        **metrics,
                    }
                )

            experiment_dir = output_dir / f"{route}_{speed_mode}"
            experiment_dir.mkdir(parents=True, exist_ok=True)

            save_reference_path(experiment_dir, path)
            for model_key in MODELS:
                model_dir = experiment_dir / model_key
                model_dir.mkdir(parents=True, exist_ok=True)
                save_records(model_dir, results[model_key]["records"])

            plot_comparison(
                path,
                results,
                f"{route} @ {speed_mode}",
                experiment_dir / "model_comparison.png",
            )

            print_metric_table(results, f"{label} 指标表")

    # 汇总 CSV
    summary_path = output_dir / "model_comparison_metrics.csv"
    fieldnames = list(summary_rows[0].keys())
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_rows)

    print("\n" + "=" * 72)
    print("实验完成！")
    print(f"汇总指标: {summary_path}")
    print("=" * 72)


if __name__ == "__main__":
    main()
