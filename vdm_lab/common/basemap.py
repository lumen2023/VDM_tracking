"""OpenStreetMap background tiles aligned with a GPX reference path."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import io
import math
import os
from pathlib import Path as FsPath
import time
import urllib.error
import urllib.parse
import urllib.request
import warnings

import numpy as np
from PIL import Image


EARTH_RADIUS_M = 6378137.0
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSM_ATTRIBUTION = "© OpenStreetMap contributors"
DEFAULT_USER_AGENT = "VDM-Lab/1.0 (educational GPX visualization)"
MAX_TILE_COUNT = 128


@dataclass(frozen=True)
class BasemapImage:
    """A raster basemap expressed in the simulation's local x/y frame."""

    image: np.ndarray
    extent: tuple[float, float, float, float]
    attribution: str = OSM_ATTRIBUTION
    geographic_bounds: tuple[float, float, float, float] | None = None


_MEMORY_CACHE = {}


def _tile_x(lon_deg, zoom):
    return (float(lon_deg) + 180.0) / 360.0 * (2**zoom)


def _tile_y(lat_deg, zoom):
    latitude = np.clip(float(lat_deg), -85.05112878, 85.05112878)
    latitude = math.radians(latitude)
    return (
        1.0
        - math.asinh(math.tan(latitude)) / math.pi
    ) / 2.0 * (2**zoom)


def _tile_lon(tile_x, zoom):
    return float(tile_x) / (2**zoom) * 360.0 - 180.0


def _tile_lat(tile_y, zoom):
    mercator_y = math.pi * (1.0 - 2.0 * float(tile_y) / (2**zoom))
    return math.degrees(math.atan(math.sinh(mercator_y)))


def _default_cache_dir():
    cache_home = os.environ.get("XDG_CACHE_HOME")
    if cache_home:
        return FsPath(cache_home) / "vdm_lab" / "osm"
    return FsPath.home() / ".cache" / "vdm_lab" / "osm"


def _download_tile(
    url,
    cache_path,
    user_agent,
    retries=3,
    timeout=10.0,
):
    if cache_path.exists():
        with Image.open(cache_path) as image:
            return image.convert("RGB")

    retries = max(0, int(retries))
    last_error = None
    for attempt in range(retries + 1):
        request = urllib.request.Request(
            url,
            headers={
                "User-Agent": user_agent,
                "Accept": "image/png,image/*;q=0.8,*/*;q=0.1",
                "Connection": "close",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                payload = response.read()
            with Image.open(io.BytesIO(payload)) as image:
                tile = image.convert("RGB")
            break
        except (OSError, urllib.error.URLError) as exc:
            last_error = exc
            if attempt < retries:
                time.sleep(min(2.0**attempt, 4.0))
    else:
        raise RuntimeError(
            f"地图瓦片在 {retries + 1} 次尝试后仍下载失败: "
            f"{last_error}"
        ) from last_error

    cache_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = cache_path.with_suffix(".tmp")
    tile.save(temporary, format="PNG")
    temporary.replace(cache_path)
    return tile


def _geographic_bounds(path, padding_m):
    lat = np.asarray(path.lat, dtype=float)
    lon = np.asarray(path.lon, dtype=float)
    if lat.size == 0 or lon.size == 0:
        raise ValueError("OSM 底图需要带经纬度的 GPX 路径。")
    if not np.all(np.isfinite(lat)) or not np.all(np.isfinite(lon)):
        raise ValueError("GPX 经纬度包含非有限数，无法加载 OSM 底图。")

    mean_lat = math.radians(float(np.mean(lat)))
    lat_padding = math.degrees(float(padding_m) / EARTH_RADIUS_M)
    lon_padding = math.degrees(
        float(padding_m) / (EARTH_RADIUS_M * max(math.cos(mean_lat), 1.0e-6))
    )
    return (
        float(np.min(lon)) - lon_padding,
        float(np.min(lat)) - lat_padding,
        float(np.max(lon)) + lon_padding,
        float(np.max(lat)) + lat_padding,
    )


def _origin(path):
    metadata = getattr(path, "gpx_metadata", {})
    origin_lat = float(metadata.get("origin_lat", path.lat[0]))
    origin_lon = float(metadata.get("origin_lon", path.lon[0]))
    return origin_lat, origin_lon


def _lon_to_local_x(lon, origin_lon, origin_lat):
    return (
        math.radians(float(lon) - origin_lon)
        * math.cos(math.radians(origin_lat))
        * EARTH_RADIUS_M
    )


def _lat_to_local_y(lat, origin_lat):
    return math.radians(float(lat) - origin_lat) * EARTH_RADIUS_M


def load_osm_basemap(
    path,
    zoom=16,
    padding_m=100.0,
    cache_dir=None,
    tile_url=OSM_TILE_URL,
    user_agent=DEFAULT_USER_AGENT,
    retries=3,
    strict=False,
):
    """
    Download/cache the visible OSM tiles and return one local-metric raster.

    Only tiles covering the GPX route and requested padding are requested.
    Cached tiles are reused on subsequent runs.
    """
    if getattr(path, "lat", None) is None or getattr(path, "lon", None) is None:
        raise ValueError("--basemap osm 只能和带经纬度的 GPX 路径一起使用。")
    if not 0 <= int(zoom) <= 19:
        raise ValueError("basemap_zoom 必须在 0..19 之间。")
    if padding_m < 0:
        raise ValueError("basemap_padding_m 不能为负数。")
    if retries < 0:
        raise ValueError("basemap_retries 不能为负数。")
    if not all(token in tile_url for token in ("{z}", "{x}", "{y}")):
        raise ValueError(
            "basemap_url 必须包含 {z}、{x}和 {y} XYZ 占位符。"
        )
    scheme = urllib.parse.urlparse(tile_url).scheme.lower()
    if scheme not in {"http", "https"}:
        raise ValueError("basemap_url 只支持 http/https XYZ 地图服务。")

    zoom = int(zoom)
    west, south, east, north = _geographic_bounds(path, padding_m)
    tile_limit = 2**zoom - 1
    x_min = max(0, min(tile_limit, int(math.floor(_tile_x(west, zoom)))))
    x_max = max(0, min(tile_limit, int(math.floor(_tile_x(east, zoom)))))
    y_min = max(0, min(tile_limit, int(math.floor(_tile_y(north, zoom)))))
    y_max = max(0, min(tile_limit, int(math.floor(_tile_y(south, zoom)))))

    columns = x_max - x_min + 1
    rows = y_max - y_min + 1
    tile_count = columns * rows
    if tile_count > MAX_TILE_COUNT:
        raise ValueError(
            f"当前范围在 zoom={zoom} 需要 {tile_count} 张瓦片，"
            f"超过安全上限 {MAX_TILE_COUNT}。请降低 --basemap-zoom。"
        )

    cache_root = FsPath(cache_dir) if cache_dir else _default_cache_dir()
    provider_key = hashlib.sha256(tile_url.encode("utf-8")).hexdigest()[:12]
    origin_lat, origin_lon = _origin(path)
    key = (
        zoom,
        x_min,
        x_max,
        y_min,
        y_max,
        str(cache_root),
        tile_url,
        origin_lat,
        origin_lon,
        int(retries),
        bool(strict),
    )
    if key in _MEMORY_CACHE:
        return _MEMORY_CACHE[key]

    mosaic = Image.new("RGB", (columns * 256, rows * 256), "#f3f4f6")
    failures = []
    completed_tiles = 0
    stop_downloading = False
    for row, tile_y in enumerate(range(y_min, y_max + 1)):
        for column, tile_x in enumerate(range(x_min, x_max + 1)):
            url = tile_url.format(z=zoom, x=tile_x, y=tile_y)
            cache_path = (
                cache_root
                / provider_key
                / str(zoom)
                / str(tile_x)
                / f"{tile_y}.png"
            )
            if stop_downloading and not cache_path.exists():
                continue
            try:
                tile = _download_tile(
                    url,
                    cache_path,
                    user_agent,
                    retries=retries,
                )
            except RuntimeError as exc:
                failures.append((url, exc))
                if strict:
                    raise RuntimeError(
                        f"{exc}。可用 --basemap-retries 增加重试，"
                        "用 --basemap-url 指定其他合法 XYZ 服务，"
                        "或去掉 --basemap-strict 允许仿真继续。"
                    ) from exc
                stop_downloading = True
                continue
            if tile.size != (256, 256):
                tile = tile.resize((256, 256), Image.Resampling.BILINEAR)
            mosaic.paste(tile, (column * 256, row * 256))
            completed_tiles += 1

    if failures:
        missing_tiles = tile_count - completed_tiles
        warnings.warn(
            (
                f"底图有 {missing_tiles}/{tile_count} 张瓦片未加载；"
                "对应区域将留白，仿真继续运行。"
                "请检查网络/代理，或用 --basemap-url 指定你有权使用的 "
                "XYZ 地图服务。第一个错误: "
                f"{failures[0][1]}"
            ),
            RuntimeWarning,
        )

    tile_west = _tile_lon(x_min, zoom)
    tile_east = _tile_lon(x_max + 1, zoom)
    tile_north = _tile_lat(y_min, zoom)
    tile_south = _tile_lat(y_max + 1, zoom)
    extent = (
        _lon_to_local_x(tile_west, origin_lon, origin_lat),
        _lon_to_local_x(tile_east, origin_lon, origin_lat),
        _lat_to_local_y(tile_south, origin_lat),
        _lat_to_local_y(tile_north, origin_lat),
    )
    result = BasemapImage(
        image=np.asarray(mosaic),
        extent=extent,
        geographic_bounds=(
            tile_west,
            tile_south,
            tile_east,
            tile_north,
        ),
    )
    _MEMORY_CACHE[key] = result
    return result


def save_basemap_bundle(basemap, output_file):
    """Save a downloaded basemap as a portable, offline ``.npz`` bundle."""
    if basemap.geographic_bounds is None:
        raise ValueError("底图没有地理边界，无法生成离线包。")

    output_file = FsPath(output_file)
    if output_file.suffix.lower() != ".npz":
        raise ValueError("离线底图输出文件必须使用 .npz 扩展名。")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_file.with_suffix(".npz.tmp")
    with temporary.open("wb") as file_handle:
        np.savez_compressed(
            file_handle,
            format_version=np.asarray(1, dtype=np.int64),
            image=np.asarray(basemap.image, dtype=np.uint8),
            geographic_bounds=np.asarray(
                basemap.geographic_bounds,
                dtype=np.float64,
            ),
            attribution=np.asarray(basemap.attribution),
        )
    temporary.replace(output_file)
    return output_file


def load_basemap_bundle(path, bundle_file):
    """Load a portable basemap bundle and align it to this GPX origin."""
    bundle_file = FsPath(bundle_file)
    if not bundle_file.exists():
        raise FileNotFoundError(f"离线底图文件不存在: {bundle_file}")

    try:
        with np.load(bundle_file, allow_pickle=False) as bundle:
            version = int(bundle["format_version"])
            image = np.asarray(bundle["image"], dtype=np.uint8)
            bounds = tuple(
                float(value) for value in bundle["geographic_bounds"]
            )
            attribution = str(bundle["attribution"].item())
    except (KeyError, OSError, ValueError) as exc:
        raise ValueError(f"离线底图包损坏或格式不正确: {bundle_file}") from exc

    if version != 1:
        raise ValueError(f"不支持的离线底图版本: {version}")
    if image.ndim != 3 or image.shape[2] not in {3, 4}:
        raise ValueError("离线底图图像必须是 RGB/RGBA 数组。")
    if len(bounds) != 4 or not all(math.isfinite(value) for value in bounds):
        raise ValueError("离线底图缺少有效的地理边界。")

    west, south, east, north = bounds
    if not west < east or not south < north:
        raise ValueError("离线底图的 west/south/east/north 边界无效。")
    origin_lat, origin_lon = _origin(path)
    extent = (
        _lon_to_local_x(west, origin_lon, origin_lat),
        _lon_to_local_x(east, origin_lon, origin_lat),
        _lat_to_local_y(south, origin_lat),
        _lat_to_local_y(north, origin_lat),
    )
    return BasemapImage(
        image=image,
        extent=extent,
        attribution=attribution,
        geographic_bounds=bounds,
    )


def load_basemap(path, simulation_config):
    """Resolve the configured basemap provider for a reference path."""
    provider = simulation_config.basemap
    if provider == "none":
        return None
    if provider == "osm":
        return load_osm_basemap(
            path,
            zoom=simulation_config.basemap_zoom,
            padding_m=simulation_config.basemap_padding_m,
            tile_url=simulation_config.basemap_url or OSM_TILE_URL,
            retries=simulation_config.basemap_retries,
            strict=simulation_config.basemap_strict,
        )
    if provider == "local":
        if not simulation_config.basemap_file:
            raise ValueError("--basemap local 必须同时指定 --basemap-file。")
        return load_basemap_bundle(path, simulation_config.basemap_file)
    if provider == "geojson":
        if not simulation_config.basemap_file:
            raise ValueError("--basemap geojson 必须同时指定 --basemap-file。")
        from vdm_lab.common.vector_map import load_geojson_basemap

        return load_geojson_basemap(
            path,
            simulation_config.basemap_file,
            max_pixels=simulation_config.basemap_max_pixels,
            padding_m=simulation_config.basemap_padding_m,
        )
    raise ValueError(f"不支持的底图类型: {provider}")
