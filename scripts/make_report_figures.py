"""把实验产物汇总成实验报告用的**独立单图**集合（docs/figures/）。

设计原则：**一张图一个文件，不做多子图拼版**。每张图只讲一件事。

来源分三类：
    1. outputs/analysis_circle_speed/    —— 速度扫描（表 1-4）
    2. outputs/analysis_pp_corner_cut/   —— PP 内切量受控实验
    3. outputs/analysis_steer_rate/      —— 转角速率限制前后对比
    4. outputs/panels/                   —— 单次运行的轨迹/误差/速度/控制面板
    5. 本脚本直接算的                    —— 三算法转角速率越界比例

用法：
    python scripts/make_report_figures.py            # 汇总 + 清理旧拼版图
    python scripts/make_report_figures.py --no-clean  # 不删除旧文件
"""

import argparse
import csv
import math
import shutil
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OUTPUTS = REPO_ROOT / "outputs"
FIG_DIR = REPO_ROOT / "docs" / "figures"

SPEED_ANALYSIS = OUTPUTS / "analysis_circle_speed"
CORNER_CUT = OUTPUTS / "analysis_pp_corner_cut"
STEER_RATE = OUTPUTS / "analysis_steer_rate"
PANELS = OUTPUTS / "panels"

# 本脚本直接计算、不来自 COPY_PLAN 的图
COMPUTED = ("fig16_steer_rate_violations.png",)

# (源文件, 报告用文件名)，编号顺序 = 报告里的出现顺序
COPY_PLAN = (
    # 一、速度改变了什么：转角不变、横摆角速度线性、法向加速度平方
    (SPEED_ANALYSIS / "fig_steer_vs_v.png", "fig01_steer_vs_v.png"),
    (SPEED_ANALYSIS / "fig_yawrate_vs_v.png", "fig02_yawrate_vs_v.png"),
    (SPEED_ANALYSIS / "fig_an_vs_v.png", "fig03_an_vs_v.png"),
    # 二、误差随速度的增长（最大值单调，均值会骗人）
    (SPEED_ANALYSIS / "fig_lateral_max_vs_v.png", "fig04_lateral_max_vs_v.png"),
    (SPEED_ANALYSIS / "fig_lateral_mean_vs_v.png", "fig05_lateral_mean_vs_v.png"),
    # 三、PP 稳态内切：车辆跑在同心的偏心圆上
    (SPEED_ANALYSIS / "fig_trajectory_pp_low.png", "fig06_trajectory_pp_low.png"),
    (SPEED_ANALYSIS / "fig_trajectory_pp_high.png", "fig07_trajectory_pp_high.png"),
    (SPEED_ANALYSIS / "fig_radius_vs_time_pp.png", "fig08_radius_vs_time_pp.png"),
    # 四、内切量归因的三组受控实验
    (CORNER_CUT / "fig_cut_vs_lookahead.png", "fig09_cut_vs_lookahead.png"),
    (CORNER_CUT / "fig_cut_vs_beta.png", "fig10_cut_vs_beta.png"),
    (CORNER_CUT / "fig_cut_regression.png", "fig11_cut_regression.png"),
    (CORNER_CUT / "fig_cut_vs_ds.png", "fig12_cut_vs_ds.png"),
    # 五、平台缺陷一：limit_command() 从不施加 max_steer_rate
    (STEER_RATE / "fig_steer_time_raw_vs_limited_high.png",
     "fig13_steer_raw_vs_limited.png"),
    (STEER_RATE / "fig_max_dsteer_vs_limit.png", "fig14_max_dsteer_vs_limit.png"),
    # 六、平台缺陷二：终点限速用欧氏直线距离
    (PANELS / "20260912_151824_lqr_kinematic_circle_high_speed.png",
     "fig15_lqrk_high_speed.png"),
    # 七、单次运行的误差细节
    (PANELS / "20260912_151820_pp_circle_high_lateral_error.png",
     "fig17_pp_high_lateral_error.png"),
    (PANELS / "20260912_151903_mpc_circle_high_lateral_error.png",
     "fig18_mpc_high_lateral_error.png"),
)
# fig16 由本脚本直接计算（三算法转角速率越界比例）

# 三算法「每拍转角跳变越界比例」用到的高速档实验目录
HIGH_RUN_DIRS = {
    "PP": "20260912_151820_pp_circle_high",
    "MPC": "20260912_151903_mpc_circle_high",
    "LQR kinematic": "20260912_151824_lqr_kinematic_circle_high",
}

MAX_STEER_RATE = math.radians(45.0)
DT = 0.1


def read_csv_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def steer_rate_stats(run_dir):
    """返回 (max|Δsteer|, 越界拍数, 总拍数, 越界比例)。"""
    rows = read_csv_rows(run_dir / "trajectory.csv")
    steer = np.array([float(row["steer"]) for row in rows])
    delta = np.abs(np.diff(steer))
    limit = MAX_STEER_RATE * DT
    return (float(np.max(delta)), int(np.sum(delta > limit)),
            int(len(delta)), float(np.mean(delta > limit)))


def plot_violations(out_dir):
    names = list(HIGH_RUN_DIRS)
    stats = {name: steer_rate_stats(OUTPUTS / run) for name, run in HIGH_RUN_DIRS.items()}
    limit = MAX_STEER_RATE * DT

    fig, ax = plt.subplots(figsize=(7.8, 4.8))
    xs = np.arange(len(names))
    ratios = [stats[name][3] * 100.0 for name in names]
    bars = ax.bar(xs, ratios, color=["#16a34a", "#ea580c", "#dc2626"], width=0.55)
    ax.bar_label(bars, labels=[f"{value:.1f}%" for value in ratios], fontsize=10)
    ax.set_xticks(xs)
    ax.set_xticklabels([
        f"{name}\nmax|Δ|={stats[name][0]:.3f} rad" for name in names
    ])
    ax.set_title("high speed: steering steps that exceed the actuator rate limit")
    ax.set_ylabel("share of steps over limit [%]")
    ax.set_ylim(0.0, max(ratios) * 1.28)
    ax.axhline(0.0, color="k", lw=0.8)
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_dir / "fig16_steer_rate_violations.png", dpi=140)
    plt.close(fig)

    print(f"  fig16_steer_rate_violations.png  "
          f"(执行器上限 {limit:.4f} rad/拍 = {MAX_STEER_RATE:.4f} rad/s × {DT} s)")
    for name in names:
        mx, over, total, _ = stats[name]
        print(f"    {name:<14} max|Δsteer|={mx:.4f}  越界 {over}/{total}")


def main():
    parser = argparse.ArgumentParser(description="汇总报告的独立单图")
    parser.add_argument("--no-clean", action="store_true",
                        help="不删除已淘汰的旧拼版图")
    args = parser.parse_args()

    FIG_DIR.mkdir(parents=True, exist_ok=True)

    copied = []
    missing = []
    for source, target in COPY_PLAN:
        if not source.exists():
            missing.append(source)
            continue
        shutil.copyfile(source, FIG_DIR / target)
        copied.append(target)

    print(f"[复制] {len(copied)} 张")
    for name in copied:
        print(f"  {name}")

    print("\n[计算] 三算法转角速率越界比例（高速档）")
    plot_violations(FIG_DIR)

    # 清理：docs/figures 下不属于本次集合的 .png 一律删掉（含旧版拼版图与被改名的旧编号）
    if not args.no_clean:
        expected = {target for _, target in COPY_PLAN} | set(COMPUTED) | set(copied)
        removed = sorted(
            path.name for path in FIG_DIR.glob("*.png") if path.name not in expected
        )
        for name in removed:
            (FIG_DIR / name).unlink()
        if removed:
            print(f"\n[清理] 删除 {len(removed)} 张不在本次集合里的旧图")
            for name in removed:
                print(f"  {name}")

    if missing:
        print(f"\n[缺少源文件] {len(missing)} 张 —— 请先跑对应的分析脚本：")
        for path in missing:
            print(f"  {path.relative_to(REPO_ROOT)}")

    print(f"\n[输出目录] {FIG_DIR}")


if __name__ == "__main__":
    main()
