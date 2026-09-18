"""
扫描 outputs/ 目录下指定算法和路线的实验结果，
生成 algorithm_comparison.csv（含全部 7 个指标 + 补充信息）。
"""
import os
import json
import glob
import csv
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUTS_DIR = os.path.join(PROJECT_ROOT, "outputs")
RESULT_CSV = os.path.join(PROJECT_ROOT, "algorithm_comparison.csv")

ALGOS = ["pp", "lqr_kinematic", "mpc"]
ROUTES = ["double_lane_change", "right_angle", "s_curve"]
SPEED = "medium"


def find_run_dir(algo, route):
    pattern = os.path.join(OUTPUTS_DIR, f"*_{algo}_{route}_{SPEED}")
    cands = sorted(glob.glob(pattern), key=os.path.getmtime)
    return cands[-1] if cands else None


def load_metrics(run_dir):
    with open(os.path.join(run_dir, "metrics.json"), "r", encoding="utf-8") as f:
        return json.load(f)


def load_trajectory(run_dir):
    path = os.path.join(run_dir, "trajectory.csv")
    return np.genfromtxt(path, delimiter=",", names=True)


def compute_j_delta(traj):
    steer = np.asarray(traj["steer"], dtype=float)
    t = np.asarray(traj["time"], dtype=float)
    if len(steer) < 2:
        return float("nan")
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    return float(np.nanmean(np.abs(np.diff(steer)) / dt))


def main():
    rows = []
    for route in ROUTES:
        for algo in ALGOS:
            run_dir = find_run_dir(algo, route)
            if run_dir is None:
                print(f"[MISS] {algo} / {route}")
                continue

            m = load_metrics(run_dir)
            traj = load_trajectory(run_dir)
            j_delta = compute_j_delta(traj)

            rows.append({
                "route": route,
                "algo": algo,
                "reached_goal": m.get("reached_goal"),
                "mean_lateral_error_m": m.get("mean_lateral_error_m"),
                "max_lateral_error_m": m.get("max_lateral_error_m"),
                "max_steer_rad": m.get("max_steer_rad"),
                "max_beta_rad": m.get("max_side_slip_beta_rad"),
                "max_yaw_rate_radps": m.get("max_yaw_rate_radps"),
                "J_delta": j_delta,
                "steps": m.get("steps"),
                "finish_error_m": m.get("finish_error_m"),
                "mean_heading_error_rad": m.get("mean_heading_error_rad"),
                "max_normal_acceleration_mps2": m.get("max_normal_acceleration_mps2"),
                "run_dir": os.path.basename(run_dir),
            })

    fields = list(rows[0].keys())
    with open(RESULT_CSV, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print(f"[OK] 写入 {RESULT_CSV}\n")
    header = (f"{'route':<20}{'algo':<15}{'reached':<8}"
              f"{'mean_err':<10}{'max_err':<10}{'max_steer':<10}"
              f"{'max_beta':<10}{'max_yaw':<10}{'J_delta':<10}")
    print(header)
    print("-" * len(header))
    for r in rows:
        print(f"{r['route']:<20}{r['algo']:<15}"
              f"{str(r['reached_goal']):<8}"
              f"{r['mean_lateral_error_m']:<10.4f}"
              f"{r['max_lateral_error_m']:<10.4f}"
              f"{r['max_steer_rad']:<10.4f}"
              f"{r['max_beta_rad']:<10.4f}"
              f"{r['max_yaw_rate_radps']:<10.4f}"
              f"{r['J_delta']:<10.4f}")


if __name__ == "__main__":
    main()