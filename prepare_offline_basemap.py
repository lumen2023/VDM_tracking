"""Build a portable offline basemap bundle for one GPX route."""

import argparse

from vdm_lab.common.basemap import (
    DEFAULT_USER_AGENT,
    load_osm_basemap,
    save_basemap_bundle,
)
from vdm_lab.common.gpx import generate_gpx_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="为 GPX 路线生成 VDM Lab .npz 离线地图包",
    )
    parser.add_argument("--gpx", required=True, help="GPX 路径文件")
    parser.add_argument("--output", required=True, help="输出 .npz 文件")
    parser.add_argument(
        "--tile-url",
        required=True,
        help=(
            "允许离线使用的 XYZ 瓦片地址，"
            "必须包含 {z}/{x}/{y}"
        ),
    )
    parser.add_argument(
        "--attribution",
        required=True,
        help="地图服务要求显示的归属文本",
    )
    parser.add_argument("--zoom", type=int, default=16)
    parser.add_argument("--padding", type=float, default=100.0)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--user-agent", default=DEFAULT_USER_AGENT)
    return parser.parse_args()


def main():
    args = parse_args()
    path = generate_gpx_path(
        args.gpx,
        ds=5.0,
        gap_warning_m=None,
    )
    basemap = load_osm_basemap(
        path,
        zoom=args.zoom,
        padding_m=args.padding,
        tile_url=args.tile_url,
        user_agent=args.user_agent,
        retries=args.retries,
        strict=True,
    )
    basemap = type(basemap)(
        image=basemap.image,
        extent=basemap.extent,
        attribution=args.attribution,
        geographic_bounds=basemap.geographic_bounds,
    )
    output = save_basemap_bundle(basemap, args.output)
    print(f"offline_basemap={output}")
    print(f"image_shape={basemap.image.shape}")
    print(f"geographic_bounds={basemap.geographic_bounds}")


if __name__ == "__main__":
    main()
