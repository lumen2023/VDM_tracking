"""Collect reproducible metrics used by the NullPointer lab report."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from vdm_lab.common.simulation import build_reference_path, load_controller, run_simulation
from vdm_lab.common.types import LabConfig


def run_case(algo: str, route: str, speed_mode: str, *, max_steer: float | None = None) -> dict:
    config = LabConfig()
    config.sim.route_name = route
    config.sim.speed_mode = speed_mode
    if max_steer is not None:
        config.vehicle.max_steer = max_steer
    path, records, _ = run_simulation(load_controller(algo, "student"), config=config)
    if not records:
        raise RuntimeError(f"{algo}/{route}/{speed_mode} 未生成记录")
    final = records[-1]
    finish_distance = math.hypot(final.x - path.x[-1], final.y - path.y[-1])
    reached_goal = finish_distance < config.sim.stop_distance and final.speed < config.sim.stop_speed
    steer = np.asarray([record.steer for record in records])
    lateral_error = np.asarray([record.lateral_error for record in records])
    yaw_rate = np.asarray([record.yaw_rate for record in records])
    normal_accel = np.asarray([record.normal_accel for record in records])
    beta = np.asarray([record.beta for record in records])
    candidate = np.flatnonzero(np.abs(np.asarray([record.curvature for record in records]) - 1.0 / 12.0) < 0.005)
    if len(candidate) > 10:
        trim = max(1, round(len(candidate) * 0.10))
        candidate = candidate[trim:-trim]
    steady = candidate if len(candidate) else np.arange(len(records))
    delta = np.diff(steer) / config.sim.dt
    return {
        "algorithm": algo,
        "route": route,
        "speed_mode": speed_mode,
        "target_speed_mps": float(path.target_speed[0]),
        "steps": len(records),
        "reached_goal": reached_goal,
        "finish_error_m": finish_distance,
        "mean_lateral_error_m": float(np.mean(np.abs(lateral_error))),
        "max_lateral_error_m": float(np.max(np.abs(lateral_error))),
        "max_steer_rad": float(np.max(np.abs(steer))),
        "max_normal_accel_mps2": float(np.max(np.abs(normal_accel))),
        "max_yaw_rate_radps": float(np.max(np.abs(yaw_rate))),
        "mean_abs_steer_rate_radps": float(np.mean(np.abs(delta))) if len(delta) else 0.0,
        "steady_mean_steer_rad": float(np.mean(steer[steady])),
        "steady_mean_yaw_rate_radps": float(np.mean(yaw_rate[steady])),
        "steady_mean_normal_accel_mps2": float(np.mean(normal_accel[steady])),
        "steady_mean_lateral_error_m": float(np.mean(np.abs(lateral_error[steady]))),
        "steady_max_lateral_error_m": float(np.max(np.abs(lateral_error[steady]))),
        "steady_std_lateral_error_m": float(np.std(lateral_error[steady])),
        "steady_max_beta_rad": float(np.max(np.abs(beta[steady]))),
    }


def load_innovation_metrics(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("outputs/nullpointer_report_data/report_data.json"))
    parser.add_argument(
        "--innovation-csv",
        type=Path,
        default=Path("outputs/mpc_rate_limit_highlight/comparison_metrics.csv"),
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    task1 = [
        run_case(algo, route, "medium")
        for route in ("double_lane_change", "right_angle", "s_curve")
        for algo in ("pp", "lqr_kinematic", "mpc")
    ]
    task2 = [
        run_case(algo, "circle", speed_mode)
        for algo in ("pp", "lqr_kinematic", "mpc")
        for speed_mode in ("low", "medium", "high")
    ]
    nominal_limit = math.radians(35.0)
    restricted_limit = math.radians(25.0)
    parameter_study = [
        {**run_case(algo, "double_lane_change", "medium", max_steer=limit), "max_steer_limit_deg": math.degrees(limit)}
        for algo in ("pp", "lqr_kinematic")
        for limit in (nominal_limit, restricted_limit)
    ]

    gpx_config = LabConfig()
    gpx_config.sim.gpx_file = "data/gpx/homework_route_1.gpx"
    gpx_config.sim.waypoint_ds = 1.0
    gpx_config.sim.target_speed = 8.0
    gpx_path = build_reference_path(gpx_config)
    theory = {
        mode: {
            "speed_mps": speed,
            "delta_rad": math.atan(gpx_config.vehicle.wheelbase / 12.0),
            "yaw_rate_radps": speed / 12.0,
            "normal_accel_mps2": speed * speed / 12.0,
        }
        for mode, speed in (("low", 3.0), ("medium", 5.0), ("high", 7.0))
    }
    report_data = {
        "group_name": "NullPointer",
        "task1": task1,
        "task2": task2,
        "parameter_study": parameter_study,
        "theory_circle": theory,
        "gpx_example": {
            "file": gpx_config.sim.gpx_file,
            "reference_points": len(gpx_path.x),
            "route_length_m": float(gpx_path.s[-1]),
            "origin_lon_lat": [118.8145, 31.8885],
        },
        "innovation": load_innovation_metrics(args.innovation_csv),
    }
    args.output.write_text(
        json.dumps(
            report_data,
            ensure_ascii=False,
            indent=2,
            default=lambda value: value.item() if isinstance(value, np.generic) else str(value),
        ),
        encoding="utf-8",
    )
    print(f"output={args.output}")


if __name__ == "__main__":
    main()
