"""问题 4 结果分析与绘图。

读取 scripts/run_dorm_experiments.py 与 scripts/sweep_lookahead.py 的产物，
生成报告用的独立图片（每张图一个文件，不做多子图拼版）。

    python scripts/run_dorm_experiments.py
    python scripts/sweep_lookahead.py
    python scripts/analyze_dorm_task.py

输出目录：outputs/dorm_figures/
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

from vdm_lab.common.basemap import load_basemap  # noqa: E402
from vdm_lab.common.gpx import generate_gpx_path  # noqa: E402
from vdm_lab.common.road_graph import (  # noqa: E402
    build_road_graph,
    named_feature_points,
)
from vdm_lab.common.types import LabConfig  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 130

INDEX_FILE = "outputs/dorm_experiments.json"
SWEEP_FILE = "outputs/dorm_lookahead_sweep.csv"
DENSE_GPX = "data/gpx/dorm_to_classroom.gpx"
RAW_GPX = "data/gpx/dorm_to_classroom_raw.gpx"
META_FILE = "data/gpx/dorm_to_classroom.meta.json"
ORIGIN = (31.8885, 118.8145)
MAP_FILE = "data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz"

OUTPUT_DIR = REPO_ROOT / "outputs" / "dorm_figures"

COLORS = {"pp": "#1f77b4", "lqr_kinematic": "#d62728", "mpc": "#2ca02c"}
NAMES = {"pp": "Pure Pursuit", "lqr_kinematic": "LQR (运动学)", "mpc": "MPC"}


# --------------------------------------------------------------------------
# 数据读取
# --------------------------------------------------------------------------
def load_index():
    return json.loads((REPO_ROOT / INDEX_FILE).read_text(encoding="utf-8"))


def load_run(label):
    index = load_index()
    directory = REPO_ROOT / "outputs" / index[label]["output_dir"]
    return read_run_dir(directory), index[label]


def read_run_dir(directory):
    def rows(name):
        text = (directory / name).read_text(encoding="utf-8").splitlines()
        return [
            {k: (v if v else "nan") for k, v in row.items()}
            for row in csv.DictReader(text)
        ]

    return {"trajectory": rows("trajectory.csv"), "reference": rows("reference_path.csv")}


def column(rows, key):
    return np.array([float(row[key]) for row in rows], dtype=float)


def reference_frame(reference):
    return {key: column(reference, key) for key in reference[0]}


def mileage_of_run(run):
    """每条仿真记录对应的参考路径里程。

    轨迹比参考路径点多（车冲过终点后 target_index 钉在最后一格，仿真还会
    继续跑），所以不能按序号对齐，必须按 target_index 取里程。
    """
    s = reference_frame(run["reference"])["s_m"]
    index = column(run["trajectory"], "target_index").astype(int)
    return s[np.clip(index, 0, len(s) - 1)]


def load_basemap_for():
    config = LabConfig()
    config.sim.coordinate_origin_lat = ORIGIN[0]
    config.sim.coordinate_origin_lon = ORIGIN[1]
    config.sim.basemap = "geojson"
    config.sim.basemap_file = MAP_FILE
    config.sim.basemap_opacity = 0.72
    path = generate_gpx_path(
        DENSE_GPX, ds=0.5, target_speed=5.0,
        origin_lat=ORIGIN[0], origin_lon=ORIGIN[1],
    )
    return load_basemap(path, config.sim)


def add_basemap(ax, basemap, opacity=0.75):
    if basemap is None:
        return
    ax.imshow(
        basemap.image, extent=basemap.extent, origin="upper",
        alpha=opacity, interpolation="bilinear", zorder=0,
    )


def set_route_limits(ax, x, y, margin=45.0):
    """把视窗裁到路线附近，底图整块瓦片的范围没有意义。"""
    ax.set_xlim(float(np.min(x)) - margin, float(np.max(x)) + margin)
    ax.set_ylim(float(np.min(y)) - margin, float(np.max(y)) + margin)


def finish(fig, ax, name, xlabel, ylabel, title=None, legend=True, grid=True):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    if legend and ax.get_legend_handles_labels()[0]:
        ax.legend(loc="best", fontsize=9)
    if grid:
        ax.grid(alpha=0.25, linestyle=":", linewidth=0.7)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / name, bbox_inches="tight")
    plt.close(fig)
    print(f"[图] {name}")


# --------------------------------------------------------------------------
# 图 1：路网构建
# --------------------------------------------------------------------------
def figure_road_graph():
    graph = build_road_graph(MAP_FILE, ORIGIN[0], ORIGIN[1])
    landmarks = named_feature_points(
        MAP_FILE, ["橘园8舍", "教学楼1"], ORIGIN[0], ORIGIN[1]
    )
    route = graph.route(landmarks["橘园8舍"], landmarks["教学楼1"])
    route_xy = graph.polyline(route)

    fig, ax = plt.subplots(figsize=(9, 8))
    for node, neighbours in enumerate(graph.adjacency):
        for other, _ in neighbours:
            if other > node:
                ax.plot(
                    [graph.xy[node, 0], graph.xy[other, 0]],
                    [graph.xy[node, 1], graph.xy[other, 1]],
                    color="#9aa4b2", linewidth=0.4, zorder=1,
                )
    ax.plot(route_xy[:, 0], route_xy[:, 1], color="#d62728", linewidth=2.2,
            zorder=3, label=f"最短路径 {_route_length(route_xy):.0f} m")
    for name, (x, y) in landmarks.items():
        ax.plot(x, y, marker="*", markersize=16, color="#135200",
                markeredgecolor="white", zorder=4)
        ax.annotate(name, (x, y), textcoords="offset points", xytext=(8, 6),
                    fontsize=10, fontweight="bold", zorder=5)
    ax.set_aspect("equal")
    finish(fig, ax, "fig1_road_graph.png", "东向 x [m]", "北向 y [m]",
           f"离线 OSM 可通行路网：{graph.node_count} 节点 / {graph.edge_count} 边 / "
           f"{graph.total_length_m / 1000:.1f} km")


def _route_length(xy):
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))


# --------------------------------------------------------------------------
# 图 2：底图上的参考路径
# --------------------------------------------------------------------------
def figure_reference_path(basemap):
    meta = json.loads((REPO_ROOT / META_FILE).read_text(encoding="utf-8"))
    reference = read_run_dir(REPO_ROOT / "outputs" /
                             load_index()["C1_pp_curv_8"]["output_dir"])["reference"]
    frame = reference_frame(reference)

    fig, ax = plt.subplots(figsize=(9, 8))
    add_basemap(ax, basemap)
    ax.plot(frame["x_m"], frame["y_m"], color="#d62728", linewidth=2.0, zorder=3,
            label="参考路径（倒圆角 + 0.5 m 重采样）")
    for corner in meta["corners"]:
        i = int(np.argmin(np.abs(frame["s_m"] - corner["s_m"])))
        ax.plot(frame["x_m"][i], frame["y_m"][i], marker="o", markersize=6,
                color="#ff7f0e", markeredgecolor="white", zorder=4)
        if corner["radius_m"] <= 5.0:
            ax.annotate(f"R{corner['radius_m']:.1f} m @ s={corner['s_m']:.0f} m",
                        (frame["x_m"][i], frame["y_m"][i]),
                        textcoords="offset points", xytext=(8, -14),
                        fontsize=9, color="#b34700", fontweight="bold", zorder=5)
    ax.plot(frame["x_m"][0], frame["y_m"][0], marker="*", markersize=17,
            color="#135200", markeredgecolor="white", zorder=5)
    ax.plot(frame["x_m"][-1], frame["y_m"][-1], marker="*", markersize=17,
            color="#a8071a", markeredgecolor="white", zorder=5)
    set_route_limits(ax, frame["x_m"], frame["y_m"])
    ax.set_aspect("equal")
    finish(fig, ax, "fig2_reference_path.png", "东向 x [m]", "北向 y [m]",
           f"参考路径：Dijkstra 最短路 {frame['s_m'][-1]:.0f} m，"
           f"倒圆角 {len(meta['corners'])} 处")


# --------------------------------------------------------------------------
# 图 3：曲率
# --------------------------------------------------------------------------
def figure_curvature():
    raw_unsmoothed = generate_gpx_path(
        RAW_GPX, ds=0.5, target_speed=8.0, origin_lat=ORIGIN[0], origin_lon=ORIGIN[1],
        speed_profile="curvature", curvature_smooth_m=0.0,
    )
    raw_smoothed = generate_gpx_path(
        RAW_GPX, ds=0.5, target_speed=8.0, origin_lat=ORIGIN[0], origin_lon=ORIGIN[1],
        speed_profile="curvature", curvature_smooth_m=12.0,
    )
    fillet = generate_gpx_path(
        DENSE_GPX, ds=0.5, target_speed=8.0, origin_lat=ORIGIN[0], origin_lon=ORIGIN[1],
        speed_profile="curvature", curvature_smooth_m=12.0,
    )
    limit = np.tan(np.deg2rad(35.0)) / 2.5

    peak_raw = float(np.abs(raw_unsmoothed.curvature).max())

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(raw_unsmoothed.s, np.abs(raw_unsmoothed.curvature), color="#d62728",
            linewidth=0.8, label=f"不平滑（仓库原版）峰值 {peak_raw:.3f} 1/m，远超坐标范围")
    ax.plot(raw_smoothed.s, np.abs(raw_smoothed.curvature), color="#1f77b4",
            linewidth=1.4, label=f"12 m 平滑，原始折线 峰值 {np.abs(raw_smoothed.curvature).max():.3f} 1/m")
    ax.plot(fillet.s, np.abs(fillet.curvature), color="#2ca02c",
            linewidth=1.4, label=f"12 m 平滑 + 倒圆角 峰值 {np.abs(fillet.curvature).max():.3f} 1/m")
    ax.axhline(limit, color="#111827", linestyle="--", linewidth=1.4,
               label=f"车辆物理极限 {limit:.3f} 1/m（前轮 35°）")
    # 不平滑的尖峰能到 1.5 1/m，线性轴上会把其余三条压成一条线，所以裁掉。
    ax.set_ylim(0.0, limit * 1.18)
    ax.annotate("红色尖峰超出本图范围\n（最高 1.47 1/m ≈ 半径 0.68 m）",
                xy=(250, limit * 1.10), xytext=(430, limit * 0.92),
                fontsize=9, color="#a8071a",
                arrowprops=dict(arrowstyle="->", color="#a8071a", linewidth=0.9))
    finish(fig, ax, "fig3_curvature.png", "沿路径里程 s [m]", "曲率 |k| [1/m]",
           "参考路径曲率：不平滑 vs 平滑 vs 倒圆角")


# --------------------------------------------------------------------------
# 图 4 / 5：速度曲线与横向加速度
# --------------------------------------------------------------------------
def figure_speed_and_accel():
    const = load_run("A3_fillet_const")[0]
    curv = load_run("A4_fillet_curv")[0]
    c_ref = reference_frame(const["reference"])
    k_ref = reference_frame(curv["reference"])

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(c_ref["s_m"], c_ref["target_speed_mps"], color="#d62728", linewidth=1.6,
            label="参考速度：匀速 8 m/s")
    ax.plot(k_ref["s_m"], k_ref["target_speed_mps"], color="#1f77b4", linewidth=1.6,
            label="参考速度：曲率限速（$a_n \\leq 2.5$ m/s²）")
    ax.plot(k_ref["s_m"], k_ref["s_m"] * 0 + 4.47, color="#9aa4b2",
            linestyle=":", linewidth=1.0)
    ax.annotate("最急弯 R2.4 m 处限速 4.47 m/s", xy=(589, 4.47),
                xytext=(760, 2.2), fontsize=9, color="#1f4e79",
                arrowprops=dict(arrowstyle="->", color="#1f4e79", linewidth=0.9))
    finish(fig, ax, "fig4_speed_profile.png", "沿路径里程 s [m]", "参考速度 [m/s]",
           "参考路径的速度设置")

    fig, ax = plt.subplots(figsize=(11, 5))
    for run, color, label in ((const, "#d62728", "匀速 8 m/s"),
                              (curv, "#1f77b4", "曲率限速")):
        ax.plot(mileage_of_run(run), column(run["trajectory"], "normal_accel"),
                color=color, linewidth=1.2, label=label)
    ax.axhline(2.5, color="#111827", linestyle="--", linewidth=1.2,
               label="舒适上限 2.5 m/s²")
    ax.axhline(7.85, color="#7f7f7f", linestyle=":", linewidth=1.2,
               label="轮胎附着上限 ≈ 0.8 g")
    finish(fig, ax, "fig5_lateral_accel.png", "沿路径里程 s [m]", "侧向加速度 $v^2k$ [m/s²]",
           "实际侧向加速度：限速前 vs 限速后")


# --------------------------------------------------------------------------
# 图 6 / 7 / 8：三种算法
# --------------------------------------------------------------------------
def figure_algorithms(basemap):
    runs = {
        "pp": load_run("C4_pp_curv_5")[0],
        "lqr_kinematic": load_run("C5_lqr_curv_5")[0],
        "mpc": load_run("C6_mpc_curv_5")[0],
    }
    reference = reference_frame(runs["pp"]["reference"])

    fig, ax = plt.subplots(figsize=(9, 8))
    add_basemap(ax, basemap)
    ax.plot(reference["x_m"], reference["y_m"], color="#111827", linewidth=2.6,
            zorder=2, label="参考路径")
    # 三条轨迹几乎重合，用递减线宽画，否则后画的会把先画的完全盖住。
    for (algo, run), width in zip(runs.items(), (3.4, 2.2, 1.1)):
        ax.plot(column(run["trajectory"], "x"), column(run["trajectory"], "y"),
                color=COLORS[algo], linewidth=width, zorder=3, label=NAMES[algo])
    set_route_limits(ax, reference["x_m"], reference["y_m"])
    ax.set_aspect("equal")
    finish(fig, ax, "fig6_algorithms_map.png", "东向 x [m]", "北向 y [m]",
           "三种算法的实际轨迹（目标速度 5 m/s）")

    # 最急弯局部放大
    cx, cy = reference["x_m"][int(np.argmin(np.abs(reference["s_m"] - 589.0)))], \
             reference["y_m"][int(np.argmin(np.abs(reference["s_m"] - 589.0)))]
    fig, ax = plt.subplots(figsize=(8, 7))
    add_basemap(ax, basemap)
    ax.plot(reference["x_m"], reference["y_m"], color="#111827", linewidth=2.6,
            zorder=2, label="参考路径")
    for algo, run in runs.items():
        ax.plot(column(run["trajectory"], "x"), column(run["trajectory"], "y"),
                color=COLORS[algo], linewidth=1.8, zorder=3, label=NAMES[algo])
    ax.set_xlim(cx - 22, cx + 22)
    ax.set_ylim(cy - 22, cy + 22)
    ax.set_aspect("equal")
    finish(fig, ax, "fig7_corner_zoom.png", "东向 x [m]", "北向 y [m]",
           "s ≈ 589 m 最急弯（圆角 R2.4 m）局部放大")

    fig, ax = plt.subplots(figsize=(11, 5))
    for algo, run in runs.items():
        ax.plot(mileage_of_run(run), column(run["trajectory"], "lateral_error"),
                color=COLORS[algo], linewidth=1.3, label=NAMES[algo])
    ax.axhline(0.0, color="#111827", linewidth=0.8)
    ax.axvline(589.0, color="#ff7f0e", linestyle="--", linewidth=1.0)
    ax.annotate("最急弯 s≈589 m", xy=(589, 0.9), xytext=(700, 0.95),
                fontsize=9, color="#b34700")
    finish(fig, ax, "fig8_lateral_error.png", "沿路径里程 s [m]", "横向误差 [m]",
           "三种算法的横向误差（目标速度 5 m/s）")


# --------------------------------------------------------------------------
# 图 9 / 10：速度影响与前视距离权衡
# --------------------------------------------------------------------------
def figure_speed_effect():
    labels = ["B1_pp_curv_3", "B2_pp_curv_5", "B3_pp_curv_7", "C1_pp_curv_8"]
    speeds, means, peaks, accels = [], [], [], []
    for label in labels:
        _, metrics = load_run(label)
        speeds.append(metrics["max_speed_mps"])
        means.append(metrics["mean_lateral_error_m"])
        peaks.append(metrics["max_lateral_error_m"])
        accels.append(metrics["max_normal_acceleration_mps2"])

    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.plot(speeds, peaks, marker="o", markersize=8, color="#d62728", linewidth=1.8,
            label="横向误差峰值")
    ax.plot(speeds, means, marker="s", markersize=8, color="#1f77b4", linewidth=1.8,
            label="横向误差均值")
    for x, y in zip(speeds, peaks):
        ax.annotate(f"{y:.2f} m", (x, y), textcoords="offset points",
                    xytext=(0, 9), ha="center", fontsize=9, color="#d62728")
    finish(fig, ax, "fig9_error_vs_speed.png", "目标速度 [m/s]", "横向误差 [m]",
           "Pure Pursuit：速度对跟踪精度的影响（曲率限速）")

    fig, ax = plt.subplots(figsize=(8, 5.5))
    ax.plot(speeds, accels, marker="o", markersize=8, color="#2ca02c", linewidth=1.8)
    ax.axhline(2.5, color="#111827", linestyle="--", linewidth=1.2,
               label="舒适上限 2.5 m/s²")
    for x, y in zip(speeds, accels):
        ax.annotate(f"{y:.2f}", (x, y), textcoords="offset points",
                    xytext=(0, 9), ha="center", fontsize=9, color="#2ca02c")
    finish(fig, ax, "fig10_accel_vs_speed.png", "目标速度 [m/s]", "峰值侧向加速度 [m/s²]",
           "Pure Pursuit：速度对侧向加速度的影响")


def figure_lookahead():
    rows = list(csv.DictReader(
        (REPO_ROOT / SWEEP_FILE).read_text(encoding="utf-8").splitlines()
    ))
    speeds = sorted({float(r["speed_mps"]) for r in rows})

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for speed in speeds:
        subset = [r for r in rows if float(r["speed_mps"]) == speed]
        ax.plot([float(r["base_lookahead_m"]) for r in subset],
                [float(r["max_lateral_error_m"]) for r in subset],
                marker="o", markersize=7, linewidth=1.8, label=f"{speed:.0f} m/s")
    finish(fig, ax, "fig11_error_vs_lookahead.png", "前视距离基准 $L_{f0}$ [m]",
           "横向误差峰值 [m]",
           "前视距离越大，切弯越狠（误差峰值全部出现在 s≈589 m 的同一弯）")

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    for speed in speeds:
        subset = [r for r in rows if float(r["speed_mps"]) == speed]
        ax.plot([float(r["base_lookahead_m"]) for r in subset],
                [float(r["steer_rate_p95_deg_s"]) for r in subset],
                marker="s", markersize=7, linewidth=1.8, label=f"{speed:.0f} m/s")
    ax.axhline(45.0, color="#111827", linestyle="--", linewidth=1.2,
               label="转向执行器上限 45 °/s")
    ax.set_yscale("log")
    finish(fig, ax, "fig12_steerrate_vs_lookahead.png", "前视距离基准 $L_{f0}$ [m]",
           "转向角速率 P95 [°/s]（对数轴）",
           "前视距离越小，转向指令越抖")


def main():
    parser = argparse.ArgumentParser(description="问题 4 结果绘图")
    parser.add_argument("--skip-map", action="store_true", help="跳过需要底图的图")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    basemap = None if args.skip_map else load_basemap_for()

    figure_road_graph()
    figure_reference_path(basemap)
    figure_curvature()
    figure_speed_and_accel()
    figure_algorithms(basemap)
    figure_speed_effect()
    figure_lookahead()
    print(f"\n[输出目录] {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
