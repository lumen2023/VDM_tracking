# -*- coding: utf-8 -*-
"""
提取 task3 六组实验的最大横向偏差点详情（时刻、里程、经纬度、曲率、速度、转角）。

运行方式（仓库根目录）：
    python task3/scripts/extract_peaks.py
"""
import csv
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "20260916_task3"

dirs = {
    "PP_s8": DATA_DIR / "pp__speed8",
    "LQR_s8": DATA_DIR / "lqr_kinematic__speed8",
    "MPC_s8": DATA_DIR / "mpc__speed8",
    "PP_s5": DATA_DIR / "pp__speed5",
    "LQR_s5": DATA_DIR / "lqr_kinematic__speed5",
    "MPC_s5": DATA_DIR / "mpc__speed5",
}

for name, d in dirs.items():
    with open(d / "trajectory.csv", encoding="utf-8") as f:
        traj = list(csv.DictReader(f))
    with open(d / "reference_path.csv", encoding="utf-8") as f:
        ref = list(csv.DictReader(f))
    peak = max(traj, key=lambda r: abs(float(r["lateral_error"])))
    ti = min(int(peak["target_index"]), len(ref) - 1)
    near = ref[ti]
    dur = float(traj[-1]["time"]) - float(traj[0]["time"])
    ts = float(traj[0]["target_speed"])
    print(
        f"{name}: speed={ts}, dur={dur:.1f}s, "
        f"peak_time={float(peak['time']):.1f}s, "
        f"peak_s={float(near['s_m']):.1f}m, "
        f"lon={near['lon_deg']}, lat={near['lat_deg']}, "
        f"kappa={float(near['curvature_1pm']):.6f}, "
        f"v_peak={float(peak['speed']):.3f}, "
        f"steer_peak={float(peak['steer']):.6f}"
    )
