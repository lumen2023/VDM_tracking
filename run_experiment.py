import argparse
from pathlib import Path as FsPath

from vdm_lab.common.basemap import load_basemap
from vdm_lab.common.logging import (
    compute_metrics,
    create_output_dir,
    save_metrics,
    save_predictions,
    save_records,
    save_reference_path,
)
from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import LabConfig
from vdm_lab.common.vector_map import infer_geojson_bounds
from vdm_lab.common.visualization import (
    save_gif,
    save_gpx_overview,
    save_summary,
)
from vdm_lab.config.routes import DEFAULT_ROUTE_NAME, available_route_names
from vdm_lab.config.speed_profiles import DEFAULT_SPEED_MODE, available_speed_modes
from vdm_lab.config.vehicle_params import (
    DEFAULT_VEHICLE_NAME,
    available_vehicle_names,
    make_vehicle_config,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="VDM 学生路径跟踪仿真实验"
    )
    parser.add_argument(
        "--algo",
        choices=["pp", "lqr_kinematic", "lqr_dynamic", "mpc"],
        required=True,
    )
    parser.add_argument(
        "--version",
        choices=["solution", "student"],
        default="solution",
    )
    parser.add_argument(
        "--route",
        choices=available_route_names(),
        default=DEFAULT_ROUTE_NAME,
        help="选择内置跟踪路线；如果指定 --gpx，则 GPX 优先",
    )

    # ---------------- GPX extension ----------------
    parser.add_argument(
        "--gpx",
        type=str,
        default=None,
        help="GPX 路径文件。指定后覆盖 --route，例如 data/gpx/homework_route_1.gpx",
    )
    parser.add_argument(
        "--waypoint-ds",
        type=float,
        default=0.5,
        help="参考路径重采样间距 [m]，默认 0.5",
    )
    parser.add_argument(
        "--gpx-gap-warning",
        type=float,
        default=50.0,
        help="GPX 原始相邻点超过该距离时发出稀疏路径警告 [m]",
    )
    parser.add_argument(
        "--view-mode",
        choices=["auto", "full", "follow"],
        default="auto",
        help="可视化视角：auto 长路线自动跟随；full 全局；follow 局部跟随+全局小窗",
    )
    parser.add_argument(
        "--follow-radius",
        type=float,
        default=45.0,
        help="follow 模式车辆周围显示半径 [m]",
    )
    parser.add_argument(
        "--max-time",
        type=float,
        default=None,
        help="手动覆盖最大仿真时间 [s]；GPX 默认会按路线长度自动延长",
    )
    parser.add_argument(
        "--basemap",
        choices=["none", "osm", "local", "geojson"],
        default="none",
        help="GPX 地理底图；可选 none/osm/local/geojson",
    )
    parser.add_argument(
        "--basemap-zoom",
        type=int,
        default=16,
        help="OSM 底图缩放级别 0..19，默认 16",
    )
    parser.add_argument(
        "--basemap-opacity",
        type=float,
        default=0.72,
        help="底图透明度 0..1，默认 0.72",
    )
    parser.add_argument(
        "--basemap-padding",
        type=float,
        default=100.0,
        help="GPX 边界外额外加载的地图范围 [m]，默认 100",
    )
    parser.add_argument(
        "--basemap-url",
        type=str,
        default=None,
        help=(
            "自定义 XYZ 瓦片地址，必须包含 {z}/{x}/{y}；"
            "仅使用你有权访问的地图服务"
        ),
    )
    parser.add_argument(
        "--basemap-retries",
        type=int,
        default=3,
        help="每张瓦片的网络重试次数，默认 3",
    )
    parser.add_argument(
        "--basemap-file",
        type=str,
        default=None,
        help="local 的 .npz 包或 geojson 的 .geojson[.xz] 文件",
    )
    parser.add_argument(
        "--basemap-max-pixels",
        type=int,
        default=2200,
        help="GeoJSON 栅格化后的最长边像素，默认 2200",
    )
    parser.add_argument(
        "--map-origin",
        nargs=2,
        type=float,
        metavar=("LON", "LAT"),
        default=None,
        help="局部米制坐标原点；GeoJSON 默认从文件名边界取中心",
    )
    parser.add_argument(
        "--basemap-strict",
        action="store_true",
        help="瓦片下载失败时终止；默认仅警告并继续仿真",
    )

    parser.add_argument(
        "--vehicle",
        choices=available_vehicle_names(),
        default=DEFAULT_VEHICLE_NAME,
        help="选择车辆参数组",
    )
    parser.add_argument(
        "--speed-mode",
        choices=available_speed_modes(),
        default=DEFAULT_SPEED_MODE,
        help="目标速度档位，默认 low 低速",
    )
    parser.add_argument(
        "--target-speed",
        type=float,
        default=None,
        help="手动覆盖速度档位，单位 m/s",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="显示实时动画",
    )
    parser.add_argument(
        "--save-log",
        action="store_true",
        help="保存 trajectory.csv / reference_path.csv / metrics.json",
    )
    parser.add_argument(
        "--save-fig",
        action="store_true",
        help="保存 summary.png；GPX 额外保存 gpx_overview.png",
    )
    parser.add_argument(
        "--save-gif",
        action="store_true",
        help="保存 animation.gif 教学演示动图",
    )
    parser.add_argument(
        "--history-ghosts",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "在实时动画和 GIF 中显示历史车辆姿态虚影，默认开启；"
            "关闭用 --no-history-ghosts"
        ),
    )
    parser.add_argument(
        "--ghost-stride",
        type=int,
        default=12,
        help="历史虚影采样间隔，单位为仿真步",
    )
    parser.add_argument(
        "--ghost-count",
        type=int,
        default=0,
        help="最多显示的历史虚影数量；0 表示从起点开始一直保留",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.basemap != "none" and not args.gpx:
        raise SystemExit("--basemap 地理底图必须和 --gpx 一起使用。")
    if args.basemap in {"local", "geojson"} and not args.basemap_file:
        raise SystemExit(
            f"--basemap {args.basemap} 必须指定 --basemap-file。"
        )
    if not 0 <= args.basemap_zoom <= 19:
        raise SystemExit("--basemap-zoom 必须在 0..19 之间。")
    if not 0.0 <= args.basemap_opacity <= 1.0:
        raise SystemExit("--basemap-opacity 必须在 0..1 之间。")
    if args.basemap_padding < 0.0:
        raise SystemExit("--basemap-padding 不能为负数。")
    if args.basemap_retries < 0:
        raise SystemExit("--basemap-retries 不能为负数。")
    if args.basemap_max_pixels < 256:
        raise SystemExit("--basemap-max-pixels 不能小于 256。")

    map_origin = args.map_origin
    if map_origin is None and args.basemap == "geojson":
        bounds = infer_geojson_bounds(args.basemap_file)
        if bounds is None:
            raise SystemExit(
                "无法从 GeoJSON 文件名识别边界，请指定 "
                "--map-origin LON LAT。"
            )
        west, south, east, north = bounds
        map_origin = ((west + east) / 2.0, (south + north) / 2.0)

    controller = load_controller(args.algo, args.version)

    route_label = (
        FsPath(args.gpx).stem
        if args.gpx
        else args.route
    )

    output_dir = None
    if args.save_log or args.save_fig or args.save_gif:
        output_dir = create_output_dir(
            f"{args.algo}_{route_label}_{args.speed_mode}"
        )

    config = LabConfig(
        vehicle=make_vehicle_config(args.vehicle)
    )
    config.sim.route_name = args.route
    config.sim.gpx_file = args.gpx
    config.sim.waypoint_ds = args.waypoint_ds
    config.sim.gpx_gap_warning_m = args.gpx_gap_warning
    if map_origin is not None:
        config.sim.coordinate_origin_lon = map_origin[0]
        config.sim.coordinate_origin_lat = map_origin[1]
    config.sim.speed_mode = args.speed_mode
    config.sim.view_mode = args.view_mode
    config.sim.follow_radius = args.follow_radius
    config.sim.basemap = args.basemap
    config.sim.basemap_zoom = args.basemap_zoom
    config.sim.basemap_opacity = args.basemap_opacity
    config.sim.basemap_padding_m = args.basemap_padding
    config.sim.basemap_url = args.basemap_url
    config.sim.basemap_file = args.basemap_file
    config.sim.basemap_max_pixels = args.basemap_max_pixels
    config.sim.basemap_retries = args.basemap_retries
    config.sim.basemap_strict = args.basemap_strict
    config.sim.show_history_ghosts = args.history_ghosts
    config.sim.history_ghost_stride = args.ghost_stride
    config.sim.history_ghost_count = args.ghost_count

    if args.target_speed is not None:
        config.sim.target_speed = args.target_speed
    if args.max_time is not None:
        config.sim.max_time = args.max_time
        config.sim.auto_extend_gpx_time = False

    live_gif_path = (
        output_dir / "animation.gif"
        if args.animate and args.save_gif
        else None
    )

    path, records, predictions = run_simulation(
        controller,
        config=config,
        animate=args.animate,
        gif_path=live_gif_path,
    )

    metrics = compute_metrics(path, records)

    needs_saved_basemap = args.save_fig or (
        args.save_gif and live_gif_path is None
    )
    basemap = (
        load_basemap(path, config.sim)
        if needs_saved_basemap
        else None
    )

    if args.save_log:
        save_records(output_dir, records)
        save_reference_path(output_dir, path)
        save_predictions(output_dir, predictions)
        save_metrics(output_dir, metrics)

    if args.save_fig:
        save_summary(
            path,
            records,
            output_dir / "summary.png",
            getattr(controller, "NAME", args.algo),
            basemap=basemap,
            basemap_opacity=config.sim.basemap_opacity,
        )
        if args.gpx:
            save_gpx_overview(
                path,
                output_dir / "gpx_overview.png",
                basemap=basemap,
                basemap_opacity=config.sim.basemap_opacity,
            )

    if args.save_gif and live_gif_path is None:
        save_gif(
            path,
            records,
            predictions,
            output_dir / "animation.gif",
            getattr(controller, "NAME", args.algo),
            config.vehicle,
            show_history_ghosts=config.sim.show_history_ghosts,
            ghost_stride=config.sim.history_ghost_stride,
            ghost_count=config.sim.history_ghost_count,
            view_mode=config.sim.view_mode,
            follow_radius=config.sim.follow_radius,
            basemap=basemap,
            basemap_opacity=config.sim.basemap_opacity,
        )

    print(f"algo={args.algo}, version={args.version}")
    print(f"speed_mode={args.speed_mode}, vehicle={args.vehicle}")
    if args.basemap != "none":
        print(f"basemap={args.basemap}")
    if map_origin is not None:
        print(
            f"map_origin_lon={map_origin[0]:.7f}, "
            f"map_origin_lat={map_origin[1]:.7f}"
        )

    if args.gpx:
        print(f"route_source=GPX, gpx={args.gpx}")
        metadata = getattr(path, "gpx_metadata", None)
        if metadata:
            print(
                "gpx_raw_points="
                f"{metadata['raw_point_count']}, "
                "reference_points="
                f"{metadata['resampled_point_count']}"
            )
            print(
                f"route_length={metadata['route_length_m']:.1f} m, "
                f"max_raw_gap={metadata['max_raw_gap_m']:.1f} m"
            )
    else:
        print(f"route={args.route}")

    print(f"steps={metrics['steps']}, reached_goal={metrics['reached_goal']}")
    print(
        f"mean_lateral_error="
        f"{metrics['mean_lateral_error_m']:.3f} m"
    )
    print(
        f"max_lateral_error="
        f"{metrics['max_lateral_error_m']:.3f} m"
    )
    print(
        f"finish_error="
        f"{metrics['finish_error_m']:.3f} m"
    )
    print(
        f"min_speed="
        f"{metrics['min_speed_mps']:.3f} m/s"
    )

    if output_dir is not None:
        print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
