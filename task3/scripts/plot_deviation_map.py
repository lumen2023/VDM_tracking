# -*- coding: utf-8 -*-
"""
在路线图上标出三种算法的最大横向偏差位置。
读取 task3/data/20260916_task3/ 下的实验日志，输出：
    task3/report/figures/deviation_map.png

运行方式（仓库根目录）：
    python task3/scripts/plot_deviation_map.py
"""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA_DIR = Path(__file__).resolve().parents[1] / "data" / "20260916_task3"
OUT_DIR = Path(__file__).resolve().parents[1] / "report" / "figures"

# 标注位置：xytext 为相对点位的偏移（单位 pt），ha 控制文本对齐，
# 底部三个点靠得很近，分别朝左上/右下/左下引出，避免互相遮挡。
RUNS = {
    "pp__speed8": ("PP (8 m/s)", "#d62728", "o", (14, 12), "left"),
    "lqr_kinematic__speed8": ("LQR (8 m/s)", "#1f77b4", "s", (-12, 30), "right"),
    "mpc__speed8": ("MPC (8 m/s)", "#2ca02c", "^", (58, 30), "left"),
    "pp__speed5": ("PP (5 m/s)", "#e15759", "D", (-72, -2), "right"),
}


def load(run):
    with open(DATA_DIR / run / "trajectory.csv", encoding="utf-8") as f:
        traj = list(csv.DictReader(f))
    with open(DATA_DIR / run / "reference_path.csv", encoding="utf-8") as f:
        ref = list(csv.DictReader(f))
    peak = max(traj, key=lambda r: abs(float(r["lateral_error"])))
    ti = min(int(peak["target_index"]), len(ref) - 1)
    return traj, ref, peak, ref[ti]


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    _, ref, _, _ = load("pp__speed8")
    rx = [float(r["x_m"]) for r in ref]
    ry = [float(r["y_m"]) for r in ref]

    fig, ax = plt.subplots(figsize=(11.5, 7))
    ax.plot(rx, ry, color="#356115", lw=1.5, label="Reference route (774.1 m)")
    ax.plot(rx[0], ry[0], marker="*", ms=16, color="#356115", ls="none", label="Start (dorm)")
    ax.plot(rx[-1], ry[-1], marker="P", ms=12, color="#9467bd", ls="none", label="Goal (classroom)")

    for run, (label, color, marker, offset, ha) in RUNS.items():
        _, _, peak, near = load(run)
        err = abs(float(peak["lateral_error"]))
        ax.plot(
            float(peak["x"]), float(peak["y"]),
            marker=marker, ms=11, color=color, ls="none", mec="white", mew=0.8,
            label=f"{label}: max dev {err:.2f} m @ s={float(near['s_m']):.0f} m, t={float(peak['time']):.1f} s",
        )
        ax.annotate(
            f"{label.split(' (')[0]}: {err:.2f} m",
            (float(peak["x"]), float(peak["y"])),
            textcoords="offset points", xytext=offset, ha=ha,
            fontsize=10, fontweight="bold", color=color,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=color, lw=0.9, alpha=0.9),
            arrowprops=dict(arrowstyle="-", color=color, lw=0.9, shrinkB=7),
            zorder=5,
        )

    ax.set_title("Task 3: Maximum Lateral Deviation Locations (dorm-to-classroom route)")
    ax.set_xlabel("x (m, local frame, origin 118.8145E 31.8885N)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.margins(x=0.06, y=0.05)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="center left", bbox_to_anchor=(1.01, 0.5))
    fig.tight_layout()
    out = OUT_DIR / "deviation_map.png"
    fig.savefig(out, dpi=160)
    print(f"saved: {out}")


if __name__ == "__main__":
    main()
