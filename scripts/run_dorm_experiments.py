"""问题 4 实验矩阵：寝室 -> 教学楼 导航任务（离线，无需联网）。

按四个问题组织，每组只改一个自变量：

  A  参考路径设置   2x2  A/B：原始折线 / 倒圆角  x  匀速 / 曲率限速
  B  速度影响       曲率限速下 PP 分别跑 3 / 5 / 7 m/s
  C  算法对比       pp / lqr_kinematic / mpc，同一参考路径
  D  前视距离        PP 的 base_lookahead 扫描

用法：
    python scripts/run_dorm_experiments.py            # 全部
    python scripts/run_dorm_experiments.py --groups A B
    python scripts/run_dorm_experiments.py --groups D --dry-run
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass


PYTHON = sys.executable
DENSE_GPX = "data/gpx/dorm_to_classroom.gpx"
RAW_GPX = "data/gpx/dorm_to_classroom_raw.gpx"
BASEMAP_FILE = "data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz"

COMMON = [
    "--save-log",
    # 与离线地图的局部坐标原点保持一致。不给的话参考路径会退化成以 GPX 首点
    # 为原点，轨迹和底图就会整体错开 700 多米（仿真结果不变，图会错）。
    "--map-origin", "118.8145", "31.8885",
]

# 每组实验的 label -> 输出目录，供 analyze_dorm_task.py 按名字取数据。
INDEX_FILE = "outputs/dorm_experiments.json"

# (组, 标签, run_experiment.py 参数)
EXPERIMENTS = [
    # --- A 参考路径设置: 每次只改一件事的阶梯 ---
    # A0 是仓库原版行为：对线性插值后的航向直接微分，不做平滑。
    ("A", "A0_repo_baseline",    ["--gpx", RAW_GPX,   "--algo", "pp", "--target-speed", "8",
                                  "--speed-profile", "constant",
                                  "--curvature-smooth-m", "0"]),
    ("A", "A1_raw_smooth_const", ["--gpx", RAW_GPX,   "--algo", "pp", "--target-speed", "8",
                                  "--speed-profile", "constant"]),
    ("A", "A2_raw_smooth_curv",  ["--gpx", RAW_GPX,   "--algo", "pp", "--target-speed", "8",
                                  "--speed-profile", "curvature"]),
    ("A", "A3_fillet_const",     ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "8",
                                  "--speed-profile", "constant"]),
    ("A", "A4_fillet_curv",      ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "8",
                                  "--speed-profile", "curvature"]),
    # LQR 的曲率进前馈项，PP 不读曲率 —— 用同一对配置对照。
    ("A", "A5_lqr_unsmoothed",   ["--gpx", RAW_GPX,   "--algo", "lqr_kinematic",
                                  "--target-speed", "8", "--speed-profile", "constant",
                                  "--curvature-smooth-m", "0"]),
    ("A", "A6_lqr_smoothed",     ["--gpx", RAW_GPX,   "--algo", "lqr_kinematic",
                                  "--target-speed", "8", "--speed-profile", "constant"]),

    # --- B 速度影响: 曲率限速, 只改目标速度 ---
    ("B", "B1_pp_curv_3", ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "3",
                           "--speed-profile", "curvature"]),
    ("B", "B2_pp_curv_5", ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "5",
                           "--speed-profile", "curvature"]),
    ("B", "B3_pp_curv_7", ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "7",
                           "--speed-profile", "curvature"]),

    # --- C 算法对比: 同一参考路径 ---
    ("C", "C1_pp_curv_8",  ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "8",
                            "--speed-profile", "curvature"]),
    ("C", "C2_lqr_curv_8", ["--gpx", DENSE_GPX, "--algo", "lqr_kinematic", "--target-speed", "8",
                            "--speed-profile", "curvature"]),
    ("C", "C3_mpc_curv_8", ["--gpx", DENSE_GPX, "--algo", "mpc", "--target-speed", "8",
                            "--speed-profile", "curvature"]),
    ("C", "C4_pp_curv_5",  ["--gpx", DENSE_GPX, "--algo", "pp", "--target-speed", "5",
                            "--speed-profile", "curvature"]),
    ("C", "C5_lqr_curv_5", ["--gpx", DENSE_GPX, "--algo", "lqr_kinematic", "--target-speed", "5",
                            "--speed-profile", "curvature"]),
    ("C", "C6_mpc_curv_5", ["--gpx", DENSE_GPX, "--algo", "mpc", "--target-speed", "5",
                            "--speed-profile", "curvature"]),
]

def _trajectory_stats(output_dir):
    """补充 compute_metrics 里没有、但报告需要的量。"""
    import csv
    import math

    rows = list(
        csv.DictReader(
            (output_dir / "trajectory.csv").read_text(encoding="utf-8").splitlines()
        )
    )
    if not rows:
        return {}
    speed = [float(r["speed"]) for r in rows]
    steer = [float(r["steer"]) for r in rows]
    time = [float(r["time"]) for r in rows]
    rate = [
        abs(steer[i] - steer[i - 1]) / max(time[i] - time[i - 1], 1.0e-9)
        for i in range(1, len(steer))
    ]
    return {
        "max_speed_mps": max(speed),
        "mean_speed_mps": sum(speed) / len(speed),
        "max_steer_deg": math.degrees(max(abs(s) for s in steer)),
        "max_steer_rate_deg_s": math.degrees(max(rate)) if rate else 0.0,
        "duration_s": time[-1],
    }


def parse_args():
    parser = argparse.ArgumentParser(description="问题 4 实验矩阵批量运行")
    parser.add_argument(
        "--groups", nargs="+", default=["A", "B", "C", "D"],
        help="要跑的组，默认全部",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印命令，不执行")
    return parser.parse_args()


def run_one(label, extra, dry_run):
    command = [PYTHON, "run_experiment.py"] + COMMON + extra
    print(f"\n=== {label} ===")
    print("  " + " ".join(command))
    if dry_run:
        return None

    proc = subprocess.run(
        command, cwd=str(REPO_ROOT), capture_output=True, text=True, encoding="utf-8"
    )
    if proc.returncode != 0:
        print(proc.stdout[-2000:])
        print(proc.stderr[-2000:])
        raise SystemExit(f"{label} 运行失败。")

    output_dir = None
    for line in proc.stdout.splitlines():
        if line.startswith("output_dir="):
            output_dir = REPO_ROOT / line.split("=", 1)[1].strip()
        if line.startswith(("steps=", "mean_lateral_error=", "max_lateral_error=",
                            "finish_error=", "min_speed=")):
            print("  " + line)

    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    metrics.update(_trajectory_stats(output_dir))
    metrics["output_dir"] = output_dir.name
    metrics["_label"] = label
    return metrics


def main():
    args = parse_args()
    groups = set(args.groups)

    results = []
    for group, label, extra in EXPERIMENTS:
        if group not in groups:
            continue
        metrics = run_one(label, extra, args.dry_run)
        if metrics is not None:
            results.append((group, label, metrics))

    if not results:
        return

    table_path = REPO_ROOT / "outputs" / "dorm_experiments.md"
    lines = [
        "| 组 | 实验 | 到达终点 | 横向误差均值 [m] | 横向误差峰值 [m] | 终点误差 [m] "
        "| 最高车速 [m/s] | 峰值侧向加速度 [m/s^2] | 最大前轮转角 [deg] | 耗时 [s] |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for group, label, metrics in results:
        lines.append(
            f"| {group} | {index_label(label)} "
            f"| {'是' if metrics.get('reached_goal') else '否'} "
            f"| {_fmt(metrics, 'mean_lateral_error_m')} "
            f"| {_fmt(metrics, 'max_lateral_error_m')} "
            f"| {_fmt(metrics, 'finish_error_m')} "
            f"| {_fmt(metrics, 'max_speed_mps')} "
            f"| {_fmt(metrics, 'max_normal_acceleration_mps2')} "
            f"| {_fmt(metrics, 'max_steer_deg')} "
            f"| {_fmt(metrics, 'duration_s')} |"
        )
    table_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n[汇总] {table_path}")

    index_path = REPO_ROOT / INDEX_FILE
    index_path.write_text(
        json.dumps(
            {
                label: {"group": group, "output_dir": metrics["output_dir"],
                        **{k: v for k, v in metrics.items() if k != "output_dir"}}
                for group, label, metrics in results
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"[索引] {index_path}")


def index_label(label):
    return label.split("_", 1)[1].replace("_", " ")


def _fmt(metrics, key):
    value = metrics.get(key)
    if value is None:
        return "-"
    if isinstance(value, bool):
        return str(value)
    return f"{value:.3f}"


if __name__ == "__main__":
    main()
