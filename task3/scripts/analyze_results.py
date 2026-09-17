# -*- coding: utf-8 -*-
"""
批量分析 task3/data/20260916_task3/ 下所有实验的指标，
输出三算法对比表和最大偏差位置详情。
速度 8 用于三算法对比，速度 5 用于改进前后对比。

运行方式（仓库根目录）：
    python task3/scripts/analyze_results.py
"""
import csv
import json
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "20260916_task3"


def analyze_one(output_dir: Path) -> dict:
    with (output_dir / "trajectory.csv").open(encoding="utf-8") as f:
        trajectory = list(csv.DictReader(f))
    with (output_dir / "reference_path.csv").open(encoding="utf-8") as f:
        reference = list(csv.DictReader(f))
    with (output_dir / "metrics.json").open(encoding="utf-8") as f:
        metrics = json.load(f)

    if not trajectory or not reference:
        return {"error": "empty log"}

    # 目标速度
    target_speed = float(trajectory[0]["target_speed"])

    # 最大横向偏差
    peak = max(trajectory, key=lambda row: abs(float(row["lateral_error"])))
    target_index = min(int(peak["target_index"]), len(reference) - 1)
    nearest = reference[target_index]

    duration = float(trajectory[-1]["time"]) - float(trajectory[0]["time"])
    route_length = float(reference[-1]["s_m"])

    # 最大转角
    max_steer = max(abs(float(r["steer"])) for r in trajectory)

    # 最大法向加速度 (v^2 * kappa)
    max_lat_accel = max(
        abs(float(r["speed"]) ** 2 * float(r["curvature"])) for r in trajectory
    )

    return {
        "target_speed": target_speed,
        "reached_goal": metrics["reached_goal"],
        "duration_s": round(duration, 3),
        "route_length_m": round(route_length, 3),
        "mean_lat_err_m": round(metrics["mean_lateral_error_m"], 4),
        "max_lat_err_m": round(metrics["max_lateral_error_m"], 4),
        "finish_err_m": round(metrics["finish_error_m"], 4),
        "max_steer_rad": round(max_steer, 4),
        "max_lat_accel_mps2": round(max_lat_accel, 4),
        "peak_time_s": round(float(peak["time"]), 3),
        "peak_s_m": round(float(nearest["s_m"]), 3),
        "peak_lon": nearest["lon_deg"],
        "peak_lat": nearest["lat_deg"],
        "peak_curvature_1pm": round(float(nearest["curvature_1pm"]), 6),
        "peak_speed_mps": round(float(peak["speed"]), 3),
        "peak_steer_rad": round(float(peak["steer"]), 6),
    }


def pick_latest(results, algo_key, speed):
    """从 results 中选出指定算法、指定速度的实验"""
    candidates = []
    for name, data in results.items():
        if "error" in data:
            continue
        if not name.startswith(f"{algo_key}__"):
            continue
        if data["target_speed"] != speed:
            continue
        candidates.append((name, data))
    if not candidates:
        return None, None
    candidates.sort(key=lambda x: x[0])
    return candidates[-1]


def print_table(title, rows, algo_keys, latest_pairs):
    print("=" * 100)
    print(title)
    print("=" * 100)
    headers = ["指标"] + [ak.upper() for ak in algo_keys]
    print(f"{headers[0]:<28}" + "".join(f"{h:<22}" for h in headers[1:]))
    print("-" * 100)

    for label, key in rows:
        vals = []
        for ak in algo_keys:
            pair = latest_pairs.get(ak)
            if pair and pair[1] is not None:
                _, data = pair
                if key == "dir":
                    vals.append(data["dir"][:18] + "...")
                else:
                    vals.append(str(data[key]))
            else:
                vals.append("N/A")
        print(f"{label:<28}" + "".join(f"{v:<22}" for v in vals))
    print()


def main():
    results = {}
    for d in sorted(DATA_DIR.iterdir()):
        if not d.is_dir():
            continue
        if not (d / "metrics.json").exists():
            continue
        try:
            data = analyze_one(d)
            data["dir"] = d.name
            results[d.name] = data
        except Exception as e:
            results[d.name] = {"error": str(e)}

    algo_keys = ["pp", "lqr_kinematic", "mpc"]

    # 三算法对比（速度 8） 
    latest_pairs_s8 = {ak: pick_latest(results, ak, 8.0) for ak in algo_keys}

    table_rows = [
        ("实验目录", "dir"),
        ("reached_goal", "reached_goal"),
        ("用时 (s)", "duration_s"),
        ("路线长度 (m)", "route_length_m"),
        ("平均横向误差 (m)", "mean_lat_err_m"),
        ("最大横向误差 (m)", "max_lat_err_m"),
        ("终点误差 (m)", "finish_err_m"),
        ("最大转角 (rad)", "max_steer_rad"),
        ("最大法向加速度 (m/s^2)", "max_lat_accel_mps2"),
    ]
    print_table("三算法对比 (目标速度 8 m/s)", table_rows, algo_keys, latest_pairs_s8)

    # 最大偏差位置详情（速度 8）
    peak_rows = [
        ("出现时间 (s)", "peak_time_s"),
        ("路线里程 s (m)", "peak_s_m"),
        ("经度 (lon)", "peak_lon"),
        ("纬度 (lat)", "peak_lat"),
        ("曲率 (1/m)", "peak_curvature_1pm"),
        ("当时速度 (m/s)", "peak_speed_mps"),
        ("当时转角 (rad)", "peak_steer_rad"),
    ]
    print_table("最大偏差位置详情 (目标速度 8 m/s)", peak_rows, algo_keys, latest_pairs_s8)

    # 调低速度改进前后对比（三算法, 速度 8 vs 速度 5）
    algo_labels = {"pp": "PP", "lqr_kinematic": "LQR", "mpc": "MPC"}

    for ak in algo_keys:
        s8 = pick_latest(results, ak, 8.0)
        s5 = pick_latest(results, ak, 5.0)
        if not s8 or not s5:
            continue

        print("=" * 100)
        print(f"调低速度改进前后对比 ({algo_labels[ak]} 算法)")
        print("=" * 100)
        print(f"{'指标':<28}{'速度 8 m/s':<22}{'速度 5 m/s':<22}{'变化':<22}")
        print("-" * 100)

        compare_rows = [
            ("reached_goal", "reached_goal"),
            ("用时 (s)", "duration_s"),
            ("平均横向误差 (m)", "mean_lat_err_m"),
            ("最大横向误差 (m)", "max_lat_err_m"),
            ("终点误差 (m)", "finish_err_m"),
            ("最大法向加速度 (m/s^2)", "max_lat_accel_mps2"),
            ("最大偏差出现时间 (s)", "peak_time_s"),
            ("最大偏差路线里程 (m)", "peak_s_m"),
            ("最大偏差处速度 (m/s)", "peak_speed_mps"),
            ("最大偏差处转角 (rad)", "peak_steer_rad"),
        ]

        for label, key in compare_rows:
            v8 = s8[1].get(key, "N/A") if s8 and s8[1] else "N/A"
            v5 = s5[1].get(key, "N/A") if s5 and s5[1] else "N/A"
            if v8 != "N/A" and v5 != "N/A":
                try:
                    f8, f5 = float(v8), float(v5)
                    if f8 != 0:
                        change = f"{(f5 - f8) / f8 * 100:+.1f}%"
                    else:
                        change = f"{f5 - f8:+.3f}"
                    v8 = str(v8)
                    v5 = str(v5)
                except (ValueError, TypeError):
                    change = "-"
            else:
                change = "-"
            print(f"{label:<28}{str(v8):<22}{str(v5):<22}{change:<22}")
        print()

    # 所有实验列表
    print("=" * 100)
    print("所有实验列表")
    print("=" * 100)
    for name, data in results.items():
        if "error" in data:
            print(f"  {name}: ERROR - {data['error']}")
        else:
            print(
                f"  {name}: speed={data['target_speed']:.0f}, "
                f"reached={data['reached_goal']}, "
                f"max_err={data['max_lat_err_m']:.3f}m, "
                f"mean_err={data['mean_lat_err_m']:.4f}m, "
                f"time={data['duration_s']:.1f}s"
            )


if __name__ == "__main__":
    main()
