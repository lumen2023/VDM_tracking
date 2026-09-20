"""问题 4 组 D：Pure Pursuit 前视距离 L_f 扫描。

PP 的转向律

    delta = atan2(2 L sin(alpha) / L_f)

只有一个可调参数 —— 前视距离。它决定一个两难：

    L_f 小 -> 贴线紧，但高速时增益 2L/L_f 过大，转向抖动甚至振荡
     L_f 大 -> 平顺，但转弯半径被"提前切"掉，弯道内侧出现系统性偏差

本脚本在速度 x 前视距离网格上各跑一次闭环仿真，把每个格点的横向误差
峰值记下来，写成 CSV 并画成热力图，同时给出误差峰值出现的位置。

用法：
    python scripts/sweep_lookahead.py
    python scripts/sweep_lookahead.py --speeds 3 5 8 --lookaheads 1 2 3 5 8 12
"""

import argparse
import csv
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

from vdm_lab.common.simulation import build_reference_path, load_controller, run_simulation  # noqa: E402
from vdm_lab.common.types import LabConfig  # noqa: E402
from vdm_lab.config.vehicle_params import make_vehicle_config  # noqa: E402

DEFAULT_GPX = "data/gpx/dorm_to_classroom.gpx"
ORIGIN = (31.8885, 118.8145)

DEFAULT_SPEEDS = [3.0, 5.0, 8.0]
DEFAULT_LOOKAHEADS = [1.0, 2.0, 3.0, 5.0, 8.0, 12.0]


def parse_args():
    parser = argparse.ArgumentParser(description="PP 前视距离扫描")
    parser.add_argument("--gpx", default=DEFAULT_GPX)
    parser.add_argument("--speeds", nargs="+", type=float, default=DEFAULT_SPEEDS)
    parser.add_argument(
        "--lookaheads", nargs="+", type=float, default=DEFAULT_LOOKAHEADS
    )
    parser.add_argument("--speed-gain", type=float, default=0.35, help="pp_speed_gain")
    parser.add_argument("--algo", default="pp", help="控制器，默认 pp")
    parser.add_argument(
        "--output",
        default="outputs/dorm_lookahead_sweep.csv",
        help="扫描结果 CSV",
    )
    return parser.parse_args()


def make_config(gpx, speed):
    config = LabConfig(vehicle=make_vehicle_config("student_car"))
    config.sim.gpx_file = gpx
    config.sim.coordinate_origin_lat = ORIGIN[0]
    config.sim.coordinate_origin_lon = ORIGIN[1]
    config.sim.speed_profile = "curvature"
    config.sim.target_speed = float(speed)
    return config


def run_point(controller, base_path, config, lookahead):
    config.controller.pp_base_lookahead = float(lookahead)
    path, records, _ = run_simulation(
        controller, config=config, path_override=base_path
    )
    error = np.array([r.lateral_error for r in records], dtype=float)
    steer = np.array([r.steer for r in records], dtype=float)
    time = np.array([r.time for r in records], dtype=float)
    rate = np.rad2deg(np.abs(np.diff(steer)) / np.maximum(np.diff(time), 1.0e-9))
    peak = int(np.argmax(np.abs(error)))
    # 转向抖动用带死区的方向反转次数，避免把趋零噪声算成反转。
    delta = np.diff(steer)
    moving = np.sign(np.where(np.abs(delta) > np.deg2rad(0.5), delta, 0.0))
    moving = moving[moving != 0.0]
    reversals = int(np.sum(moving[1:] != moving[:-1])) if len(moving) > 1 else 0
    return {
        "mean_lateral_error_m": float(np.mean(np.abs(error))),
        "max_lateral_error_m": float(np.max(np.abs(error))),
        "peak_at_s_m": float(base_path.s[records[peak].target_index]),
        "steer_rate_p95_deg_s": float(np.percentile(rate, 95)) if len(rate) else 0.0,
        "steer_rate_peak_deg_s": float(rate.max()) if len(rate) else 0.0,
        "steer_reversals": reversals,
        "finish_error_m": float(
            np.hypot(records[-1].x - base_path.x[-1], records[-1].y - base_path.y[-1])
        ),
    }


def main():
    args = parse_args()
    controller = load_controller(args.algo, "solution")

    rows = []
    print(f"{'v [m/s]':>8} {'L_f [m]':>8} {'实际前视 [m]':>12} "
          f"{'误差均值':>9} {'误差峰值':>9} {'峰值位置 [m]':>12} "
          f"{'转向速率P95':>11} {'转向反向':>8}")
    for speed in args.speeds:
        config = make_config(args.gpx, speed)
        config.controller.pp_speed_gain = args.speed_gain
        base_path = build_reference_path(config)

        for lookahead in args.lookaheads:
            metrics = run_point(controller, base_path, config, lookahead)
            row = {
                "speed_mps": speed,
                "base_lookahead_m": lookahead,
                "lookahead_at_top_speed_m": lookahead
                + args.speed_gain * min(speed, float(base_path.target_speed.max())),
                **metrics,
            }
            rows.append(row)
            print(
                f"{speed:8.1f} {lookahead:8.1f} "
                f"{row['lookahead_at_top_speed_m']:12.2f} "
                f"{row['mean_lateral_error_m']:9.3f} "
                f"{row['max_lateral_error_m']:9.3f} "
                f"{row['peak_at_s_m']:12.1f} "
                f"{row['steer_rate_p95_deg_s']:11.1f} "
                f"{row['steer_reversals']:8d}"
            )

    output = REPO_ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[输出] {output}")


if __name__ == "__main__":
    main()
