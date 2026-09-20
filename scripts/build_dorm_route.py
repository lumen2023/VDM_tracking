"""寝室 -> 教学楼 参考路径生成（离线，无需联网）。

从仓库自带的离线 OSM GeoJSON 提取可通行路网，建图后用 Dijkstra 求
「橘园 8 舍 -> 教学楼 1」的最短路，再对拐点倒圆角，最后写成一份稠密
GPX。产物可直接喂给 run_experiment.py：

    python scripts/build_dorm_route.py
    python run_experiment.py --algo pp \\
      --gpx data/gpx/dorm_to_classroom.gpx \\
      --basemap geojson \\
      --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \\
      --speed-profile curvature --save-log --save-fig

为什么要倒圆角：OSM 里两条道路在路口共用一个节点，折线在该点的转向
接近 90 度。直接对这样的折线重采样并微分求曲率，会得到 1 m^-1 量级的
尖峰（转弯半径不到 1 m），远超车辆物理上限，控制器会一直处于饱和。
倒圆角把每个尖角替换成一段圆弧，曲率才回到可信范围。

用法：
    python scripts/build_dorm_route.py
    python scripts/build_dorm_route.py --radius 6 --target-speed 6
"""

import argparse
import json
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

from vdm_lab.common import reference_path as rp  # noqa: E402
from vdm_lab.common.gpx import local_xy_to_latlon  # noqa: E402
from vdm_lab.common.road_graph import (  # noqa: E402
    build_road_graph,
    named_feature_points,
)

# 仓库离线地图自带边界，几何中心即局部坐标原点。
DEFAULT_MAP = "data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz"
DEFAULT_ORIGIN = (31.8885, 118.8145)

START_LANDMARK = "橘园8舍"
GOAL_LANDMARK = "教学楼1"

DEFAULT_OUTPUT = "data/gpx/dorm_to_classroom.gpx"


def parse_args():
    parser = argparse.ArgumentParser(
        description="从离线 OSM 路网生成寝室到教学楼的参考路径"
    )
    parser.add_argument("--map", default=DEFAULT_MAP, help="离线 OSM GeoJSON(.xz)")
    parser.add_argument("--start", default=START_LANDMARK, help="起点地标名称")
    parser.add_argument("--goal", default=GOAL_LANDMARK, help="终点地标名称")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="输出 GPX 路径")
    parser.add_argument(
        "--ds", type=float, default=0.5, help="参考路径重采样间距 [m]，默认 0.5"
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=rp.DEFAULT_CORNER_RADIUS_M,
        help="拐点圆角半径 [m]，默认 8",
    )
    parser.add_argument(
        "--min-turn",
        type=float,
        default=rp.DEFAULT_MIN_TURN_DEG,
        help="转角超过该值才倒圆角 [deg]，默认 20",
    )
    return parser.parse_args()


def write_gpx(path, xy, lat, lon, start_name, goal_name):
    """Write a dense GPX track usable by run_experiment.py --gpx."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<gpx xmlns="http://www.topografix.com/GPX/1/1" version="1.1"',
        '     creator="VDM_tracking build_dorm_route.py">',
        " <trk>",
        f"  <name>{start_name}_to_{goal_name}</name>",
        "  <trkseg>",
    ]
    for x, y, la, lo in zip(xy[:, 0], xy[:, 1], lat, lon):
        lines.append(f'   <trkpt lat="{la:.7f}" lon="{lo:.7f}">')
        lines.append("   </trkpt>")
    lines += ["  </trkseg>", " </trk>", "</gpx>", ""]

    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def main():
    args = parse_args()
    origin_lat, origin_lon = DEFAULT_ORIGIN

    print(f"[地图] {args.map}")
    graph = build_road_graph(args.map, origin_lat, origin_lon)
    print(
        f"       可通行路网: {graph.node_count} 节点, {graph.edge_count} 边, "
        f"{graph.total_length_m / 1000.0:.1f} km, "
        f"{graph.component_count()} 个连通分量"
    )

    landmarks = named_feature_points(
        args.map, [args.start, args.goal], origin_lat, origin_lon
    )
    for name in (args.start, args.goal):
        if name not in landmarks:
            raise SystemExit(f"离线地图中找不到地标「{name}」，无法生成路径。")

    snaps = {}
    for name, (x, y) in landmarks.items():
        node, offset = graph.snap(x, y)
        snaps[name] = {"node": node, "offset_m": offset, "xy": (x, y)}
        print(
            f"[吸附] {name}: 质心 ({x:.1f}, {y:.1f}) -> 路网节点 {node}, "
            f"偏离 {offset:.1f} m, 所在分量 {graph.component_size(node)} 节点"
        )

    route_nodes = graph.route(landmarks[args.start], landmarks[args.goal])
    if route_nodes is None:
        raise SystemExit("起点与终点在可通行路网上不连通，无法生成路径。")

    raw_xy = graph.polyline(route_nodes)
    raw_length = float(rp.polyline_length(raw_xy)[-1])

    rounded_xy, corners = rp.round_corners(
        raw_xy, radius=args.radius, min_turn_deg=args.min_turn
    )
    dense_xy, _ = rp.resample_polyline(rounded_xy, args.ds)
    dense_length = float(rp.polyline_length(dense_xy)[-1])

    print(
        f"[寻路] {len(route_nodes)} 个路网节点, 原始折线 {raw_length:.1f} m, "
        f"倒圆角后 {dense_length:.1f} m"
    )
    # 拐点在稠密路径上的里程。上游坐标依赖建图时的原点，而实验里参考路径
    # 会以 GPX 首点为原点重新投影，所以坐标不能跨脚本使用 —— 里程可以。
    dense_s = rp.polyline_length(dense_xy)
    for corner in corners:
        nearest = int(
            np.argmin(np.hypot(dense_xy[:, 0] - corner["x"], dense_xy[:, 1] - corner["y"]))
        )
        corner["s_m"] = float(dense_s[nearest])

    print(f"[圆角] 处理 {len(corners)} 个拐点")
    for corner in corners:
        print(
            f"       顶点 #{corner['vertex_index']:<3d} "
            f"s={corner['s_m']:7.1f} m  "
            f"转角 {corner['turn_deg']:+7.1f} deg  ->  "
            f"半径 {corner['radius_m']:5.1f} m, 弧长 {corner['arc_length_m']:5.1f} m"
        )

    # 曲率对比：直接微分 vs 倒圆角后
    _, kappa_raw, _ = rp.curvature_profile(
        rp.resample_polyline(raw_xy, args.ds)[0], args.ds, smooth_m=0.0
    )
    _, kappa_dense, _ = rp.curvature_profile(
        dense_xy, args.ds, smooth_m=rp.DEFAULT_CURVATURE_SMOOTH_M
    )
    _report_curvature("原始折线(直接微分)", kappa_raw)
    _report_curvature("倒圆角 + 平滑", kappa_dense)

    lat, lon = local_xy_to_latlon(dense_xy[:, 0], dense_xy[:, 1], origin_lat, origin_lon)
    gpx_path = write_gpx(args.output, dense_xy, lat, lon, args.start, args.goal)
    print(f"[输出] {gpx_path}  ({len(dense_xy)} 个轨迹点, 间距 {args.ds} m)")

    meta_path = gpx_path.with_suffix(".meta.json")
    meta_path.write_text(
        json.dumps(
            {
                "start": args.start,
                "goal": args.goal,
                "map": args.map,
                "origin_lat": origin_lat,
                "origin_lon": origin_lon,
                "graph_nodes": graph.node_count,
                "graph_edges": graph.edge_count,
                "graph_length_m": graph.total_length_m,
                "snap": {
                    name: {"offset_m": info["offset_m"], "node": info["node"]}
                    for name, info in snaps.items()
                },
                "raw_length_m": raw_length,
                "dense_length_m": dense_length,
                "point_count": int(len(dense_xy)),
                "ds_m": args.ds,
                "corner_radius_m": args.radius,
                "min_turn_deg": args.min_turn,
                "curvature_max_raw": float(np.abs(kappa_raw).max()),
                "curvature_max_dense": float(np.abs(kappa_dense).max()),
                "corners": corners,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"[输出] {meta_path}")


def _report_curvature(label, curvature):
    peak = float(np.abs(curvature).max())
    radius = 1.0 / peak if peak > 0 else float("inf")
    print(
        f"[曲率] {label}: |k| 最大 {peak:.4f} 1/m (转弯半径 {radius:.2f} m), "
        f"中位 {np.median(np.abs(curvature)):.5f}"
    )


if __name__ == "__main__":
    main()
