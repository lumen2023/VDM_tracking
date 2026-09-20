"""把单次实验的轨迹日志拆成**独立单图**输出。

`run_experiment.py --save-fig` 生成的 `summary.png` 是一张 2x2 拼版图。
本脚本从同一个 `trajectory.csv` / `reference_path.csv` 出发，把四个面板
分别存成单独的文件，方便直接插进实验报告：

    <run>_trajectory.png    轨迹（参考 vs 车辆）
    <run>_lateral_error.png 横向误差
    <run>_speed.png         速度（含目标速度）
    <run>_control.png       转角 / 侧偏角 / 法向加速度

用法：
    python scripts/plot_run_panels.py                    # 扫描 outputs/ 下所有 circle 实验
    python scripts/plot_run_panels.py DIR1 DIR2 ...      # 只处理指定实验目录
"""

import argparse
import csv
import sys
from pathlib import Path

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent

PANELS = (
    ("trajectory", "Trajectory"),
    ("lateral_error", "Lateral error"),
    ("speed", "Speed"),
    ("control", "Control and bicycle-model terms"),
)


def read_csv_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def series(rows, key):
    return [float(row[key]) for row in rows]


def new_axes(title, xlabel, ylabel):
    fig, ax = plt.subplots(figsize=(7.5, 5.2))
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(alpha=0.3)
    return fig, ax


def save(fig, out_dir, run_name, panel):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{run_name}_{panel}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path


def plot_trajectory(reference, rows, title, out_dir, run_name):
    fig, ax = new_axes(f"{title} — trajectory", "x / East [m]", "y / North [m]")
    ax.plot(series(reference, "x_m"), series(reference, "y_m"),
            color="#6b7280", lw=1.7, label="reference")
    ax.plot(series(rows, "x"), series(rows, "y"),
            color="#2563eb", lw=1.7, label="vehicle")
    ax.axis("equal")
    ax.legend()
    return save(fig, out_dir, run_name, "trajectory")


def plot_lateral_error(rows, title, out_dir, run_name):
    fig, ax = new_axes(f"{title} — lateral error", "time [s]", "lateral error [m]")
    ax.plot(series(rows, "time"), series(rows, "lateral_error"), color="#dc2626")
    ax.axhline(0.0, color="k", lw=0.8, ls=":")
    return save(fig, out_dir, run_name, "lateral_error")


def plot_speed(rows, title, out_dir, run_name):
    fig, ax = new_axes(f"{title} — speed", "time [s]", "m/s")
    ax.plot(series(rows, "time"), series(rows, "speed"),
            color="#16a34a", label="speed")
    ax.plot(series(rows, "time"), series(rows, "target_speed"),
            color="#6b7280", ls="--", label="target")
    ax.legend()
    return save(fig, out_dir, run_name, "speed")


def plot_control(rows, title, out_dir, run_name):
    fig, ax = new_axes(f"{title} — control and bicycle-model terms",
                       "time [s]", "value")
    times = series(rows, "time")
    ax.plot(times, series(rows, "steer"), color="#9333ea", label="steer [rad]")
    ax.plot(times, series(rows, "beta"), color="#0f766e", label="beta [rad]")
    ax.plot(times, series(rows, "normal_accel"),
            color="#ea580c", label="normal accel [m/s²]")
    ax.legend()
    return save(fig, out_dir, run_name, "control")


def process_run(directory, out_dir):
    trajectory_file = directory / "trajectory.csv"
    reference_file = directory / "reference_path.csv"
    if not trajectory_file.exists() or not reference_file.exists():
        return []

    rows = read_csv_rows(trajectory_file)
    reference = read_csv_rows(reference_file)
    if not rows:
        return []

    run_name = directory.name
    # 目录名形如 20260912_151820_pp_circle_high，去掉时间戳更易读
    title = run_name[16:] if len(run_name) > 16 and run_name[8] == "_" else run_name

    return [
        plot_trajectory(reference, rows, title, out_dir, run_name),
        plot_lateral_error(rows, title, out_dir, run_name),
        plot_speed(rows, title, out_dir, run_name),
        plot_control(rows, title, out_dir, run_name),
    ]


def main():
    parser = argparse.ArgumentParser(description="把单次实验拆成独立单图")
    parser.add_argument("dirs", nargs="*", type=Path,
                        help="实验目录；不填则扫描 outputs/ 下所有 circle 实验")
    parser.add_argument("--outputs-dir", type=Path, default=REPO_ROOT / "outputs")
    parser.add_argument("--route", default="circle")
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    if args.dirs:
        directories = list(args.dirs)
    else:
        directories = sorted(
            entry for entry in args.outputs_dir.iterdir()
            if entry.is_dir() and args.route in entry.name
            and (entry / "trajectory.csv").exists()
        )

    if not directories:
        raise SystemExit(f"在 {args.outputs_dir} 下没有找到可处理的实验目录。")

    out_dir = args.out_dir or (args.outputs_dir / "panels")
    written = []
    for directory in directories:
        written.extend(process_run(directory, out_dir))

    for path in written:
        print(f"  {path.name}")
    print(f"\n共 {len(written)} 张单图 -> {out_dir}")


if __name__ == "__main__":
    main()
