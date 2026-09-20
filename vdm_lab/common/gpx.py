"""
GPX route import for VDM_tracking.

The GPX track is converted from WGS84 latitude/longitude into a local
Cartesian frame used by all existing VDM tracking controllers:

    +x = East [m]
    +y = North [m]

The first valid GPX point is used as the local origin.

No new third-party dependency is required.  For several-kilometre experiments
the local tangent/equirectangular approximation is sufficiently accurate.
If centimetre-level geodesy is required later, replace latlon_to_local_xy()
with pyproj/ENU while keeping the returned Path interface unchanged.
"""

from __future__ import annotations

import warnings
import xml.etree.ElementTree as ET
from pathlib import Path as FsPath

import numpy as np

from vdm_lab.common.types import Path


EARTH_RADIUS_M = 6378137.0

# Generic speed presets used by GPX routes when --target-speed is not supplied.
GPX_TARGET_SPEEDS = {
    "low": 4.0,
    "medium": 7.0,
    "high": 9.0,
}


def _tag_name(tag):
    """Strip an XML namespace from a GPX tag."""
    return tag.split("}")[-1]


def load_gpx_points(gpx_file):
    """
    Read GPX points.

    Search priority:
        trkpt -> rtept -> wpt

    Returns
    -------
    lat, lon, elevation : np.ndarray
        WGS84 latitude/longitude [deg] and elevation [m].
        Missing elevation is stored as NaN.
    """
    gpx_file = FsPath(gpx_file)
    if not gpx_file.exists():
        raise FileNotFoundError(f"GPX 文件不存在: {gpx_file}")

    root = ET.parse(gpx_file).getroot()

    selected = []
    selected_type = None
    for point_type in ("trkpt", "rtept", "wpt"):
        selected = [
            node for node in root.iter()
            if _tag_name(node.tag) == point_type
            and "lat" in node.attrib
            and "lon" in node.attrib
        ]
        if selected:
            selected_type = point_type
            break

    if len(selected) < 2:
        raise ValueError(
            f"GPX 至少需要 2 个有效轨迹点，当前文件无法生成路径: {gpx_file}"
        )

    lat = []
    lon = []
    elevation = []

    for point in selected:
        lat.append(float(point.attrib["lat"]))
        lon.append(float(point.attrib["lon"]))

        ele = np.nan
        for child in point:
            if _tag_name(child.tag) == "ele" and child.text:
                try:
                    ele = float(child.text)
                except ValueError:
                    pass
        elevation.append(ele)

    return (
        np.asarray(lat, dtype=float),
        np.asarray(lon, dtype=float),
        np.asarray(elevation, dtype=float),
        selected_type,
    )


def latlon_to_local_xy(lat, lon, origin_lat=None, origin_lon=None):
    """
    WGS84 latitude/longitude -> local East/North coordinates [m].

    This uses a local tangent/equirectangular approximation.  The first point
    is the default origin.
    """
    lat = np.asarray(lat, dtype=float)
    lon = np.asarray(lon, dtype=float)

    if len(lat) != len(lon) or len(lat) == 0:
        raise ValueError("lat/lon 数组必须非空且长度一致。")

    if origin_lat is None:
        origin_lat = float(lat[0])
    if origin_lon is None:
        origin_lon = float(lon[0])

    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    lat0 = np.deg2rad(float(origin_lat))
    lon0 = np.deg2rad(float(origin_lon))

    x = (lon_rad - lon0) * np.cos(lat0) * EARTH_RADIUS_M
    y = (lat_rad - lat0) * EARTH_RADIUS_M
    return x, y


def local_xy_to_latlon(x, y, origin_lat, origin_lon):
    """
    Local East/North coordinates [m] -> WGS84 latitude/longitude [deg].

    Exact inverse of latlon_to_local_xy for the same origin.  Needed when a
    path is generated in the local frame (for example from the road graph)
    but must still be drawn on the geographic basemap.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)

    lat0 = np.deg2rad(float(origin_lat))
    lon0 = np.deg2rad(float(origin_lon))

    lat = lat0 + y / EARTH_RADIUS_M
    lon = lon0 + x / (EARTH_RADIUS_M * np.cos(lat0))
    return np.rad2deg(lat), np.rad2deg(lon)


def _remove_consecutive_duplicates(x, y, lat, lon, elevation):
    step = np.hypot(np.diff(x), np.diff(y))
    keep = np.ones(len(x), dtype=bool)
    keep[1:] = step > 1.0e-6

    return (
        x[keep],
        y[keep],
        lat[keep],
        lon[keep],
        elevation[keep],
    )


def _fill_elevation(s, elevation):
    valid = np.isfinite(elevation)
    if not np.any(valid):
        return np.full_like(s, np.nan, dtype=float)
    return np.interp(s, s[valid], elevation[valid])


def _forward_speed_profile(s, target_speed):
    """
    Same stop-at-goal idea as the built-in route generator:
    cruise at target_speed and decelerate near the end.
    """
    remaining = np.maximum(s[-1] - s, 0.0)
    speed = np.minimum(float(target_speed), np.sqrt(2.0 * 0.9 * remaining))
    speed[-1] = 0.0
    return speed


def resolve_gpx_target_speed(speed_mode="low", override_speed=None):
    if override_speed is not None:
        return float(override_speed)
    try:
        return float(GPX_TARGET_SPEEDS[speed_mode])
    except KeyError as exc:
        choices = ", ".join(GPX_TARGET_SPEEDS)
        raise ValueError(
            f"未知 GPX 速度档位 {speed_mode}，可选值为: {choices}"
        ) from exc


def generate_gpx_path(
    gpx_file,
    ds=0.5,
    target_speed=4.0,
    gap_warning_m=50.0,
    origin_lat=None,
    origin_lon=None,
    speed_profile="constant",
    curvature_smooth_m=12.0,
    lateral_accel_limit=2.5,
    max_accel=2.0,
    max_decel=3.5,
):
    """
    Convert a GPX route into the existing vdm_lab.common.types.Path.

    Processing:
        GPX/WGS84
            -> local East/North x/y [m]
            -> remove duplicate points
            -> cumulative arc length s
            -> linear resampling at ds
            -> yaw / curvature
            -> target speed profile

    Linear interpolation is intentional.  It does not invent road geometry
    between sparse GPX vertices and avoids spline overshoot at road corners.
    """
    if ds <= 0:
        raise ValueError("GPX 重采样间距 ds 必须大于 0。")

    lat, lon, elevation, point_type = load_gpx_points(gpx_file)
    if (origin_lat is None) != (origin_lon is None):
        raise ValueError("origin_lat 和 origin_lon 必须同时指定。")
    resolved_origin_lat = float(lat[0] if origin_lat is None else origin_lat)
    resolved_origin_lon = float(lon[0] if origin_lon is None else origin_lon)
    x, y = latlon_to_local_xy(
        lat,
        lon,
        origin_lat=resolved_origin_lat,
        origin_lon=resolved_origin_lon,
    )

    x, y, lat, lon, elevation = _remove_consecutive_duplicates(
        x, y, lat, lon, elevation
    )
    if len(x) < 2:
        raise ValueError("去除重复点后 GPX 有效点不足 2 个。")

    raw_ds = np.hypot(np.diff(x), np.diff(y))
    raw_s = np.concatenate(([0.0], np.cumsum(raw_ds)))

    max_gap = float(np.max(raw_ds))
    median_gap = float(np.median(raw_ds))

    if gap_warning_m is not None and max_gap > gap_warning_m:
        warnings.warn(
            (
                f"GPX 存在较稀疏路段：最大相邻点距离 {max_gap:.1f} m "
                f"(中位数 {median_gap:.1f} m)。重采样只能在线段之间加点，"
                "不能恢复 GPX 中本来不存在的真实道路弯曲。建议尽量导出更高密度 GPX。"
            ),
            RuntimeWarning,
        )

    s = np.arange(0.0, raw_s[-1], ds, dtype=float)
    if len(s) == 0 or s[-1] < raw_s[-1]:
        s = np.append(s, raw_s[-1])

    x_new = np.interp(s, raw_s, x)
    y_new = np.interp(s, raw_s, y)
    lat_new = np.interp(s, raw_s, lat)
    lon_new = np.interp(s, raw_s, lon)

    elevation_filled = _fill_elevation(raw_s, elevation)
    if np.all(np.isnan(elevation_filled)):
        elevation_new = np.full_like(s, np.nan)
    else:
        elevation_new = np.interp(s, raw_s, elevation_filled)

    # Heading and curvature in the same conventions as existing VDM routes.
    dx = np.gradient(x_new, s, edge_order=1)
    dy = np.gradient(y_new, s, edge_order=1)
    yaw = np.unwrap(np.arctan2(dy, dx))

    if len(s) >= 3:
        # Curvature is a property of the reference path, not of the speed
        # profile: it feeds the lateral-acceleration metric and the LQR
        # feedforward as well as the speed limit.  It is therefore always
        # smoothed.  Differentiating the heading directly would make the
        # constant and the curvature speed profile produce different
        # reference paths, which confounds any A/B comparison between them.
        from vdm_lab.common.reference_path import curvature_profile

        _, curvature, _ = curvature_profile(
            np.column_stack([x_new, y_new]), ds, smooth_m=curvature_smooth_m
        )
    else:
        curvature = np.zeros_like(s)

    # Store yaw in [-pi, pi] for compatibility/readability.
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

    if speed_profile == "curvature":
        from vdm_lab.common.reference_path import curvature_speed_profile

        target = curvature_speed_profile(
            s,
            curvature,
            target_speed=target_speed,
            lateral_accel_limit=lateral_accel_limit,
            max_accel=max_accel,
            max_decel=max_decel,
        )
    else:
        target = _forward_speed_profile(s, target_speed)

    path = Path(
        x=x_new,
        y=y_new,
        yaw=yaw,
        curvature=curvature,
        s=s,
        target_speed=target,
        lat=lat_new,
        lon=lon_new,
        elevation=elevation_new,
        source=f"gpx:{FsPath(gpx_file).name}",
    )

    # Lightweight metadata useful for CLI output and debugging.
    path.gpx_metadata = {
        "file": str(FsPath(gpx_file)),
        "point_type": point_type,
        "raw_point_count": int(len(x)),
        "resampled_point_count": int(len(s)),
        "route_length_m": float(s[-1]),
        "median_raw_gap_m": median_gap,
        "max_raw_gap_m": max_gap,
        "origin_lat": resolved_origin_lat,
        "origin_lon": resolved_origin_lon,
    }
    return path
