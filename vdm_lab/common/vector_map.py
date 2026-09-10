"""Render a local OSM GeoJSON extract as a cached Matplotlib basemap image."""

from __future__ import annotations

import json
import lzma
import math
from pathlib import Path as FsPath
import re

import numpy as np
from PIL import Image, ImageDraw

from vdm_lab.common.basemap import (
    BasemapImage,
    EARTH_RADIUS_M,
    OSM_ATTRIBUTION,
    _geographic_bounds,
    _lat_to_local_y,
    _lon_to_local_x,
    _origin,
)


_BOUNDS_PATTERN = re.compile(
    r"_(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    r"_(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)"
    r"(?:\.osm)?\.geojson(?:\.xz)?$",
    re.IGNORECASE,
)

_LAND_COLORS = {
    "forest": "#cfe8c8",
    "wood": "#cfe8c8",
    "grass": "#dcefcf",
    "meadow": "#e1f0cf",
    "scrub": "#d8e8c6",
    "residential": "#eceae6",
    "industrial": "#e5e3df",
    "retail": "#eee6e2",
    "brownfield": "#e7dfd5",
    "greenfield": "#e2efd7",
    "construction": "#eadfc8",
}

_ROAD_STYLES = {
    "motorway": ("#dc8f9f", 9),
    "motorway_link": ("#e5a5af", 7),
    "trunk": ("#e9a27f", 8),
    "trunk_link": ("#efb393", 6),
    "primary": ("#f5b38f", 8),
    "primary_link": ("#f6c19f", 6),
    "secondary": ("#f4ce93", 7),
    "secondary_link": ("#f4d8ab", 5),
    "tertiary": ("#f3e6ae", 6),
    "tertiary_link": ("#f3e9bf", 5),
    "residential": ("#ffffff", 5),
    "unclassified": ("#ffffff", 5),
    "living_street": ("#ffffff", 4),
    "service": ("#ffffff", 3),
    "pedestrian": ("#eee5dc", 3),
    "cycleway": ("#84b6df", 2),
    "footway": ("#d8a5a5", 2),
    "path": ("#c9a77d", 2),
    "steps": ("#b99a7a", 2),
}

_ROAD_ORDER = {
    name: index
    for index, name in enumerate(
        (
            "path",
            "footway",
            "steps",
            "cycleway",
            "service",
            "living_street",
            "residential",
            "unclassified",
            "tertiary_link",
            "tertiary",
            "secondary_link",
            "secondary",
            "primary_link",
            "primary",
            "trunk_link",
            "trunk",
            "motorway_link",
            "motorway",
        )
    )
}


def _read_geojson(map_file):
    map_file = FsPath(map_file)
    if not map_file.exists():
        raise FileNotFoundError(f"GeoJSON 地图文件不存在: {map_file}")
    opener = lzma.open if map_file.suffix.lower() == ".xz" else open
    try:
        with opener(map_file, "rt", encoding="utf-8") as file_handle:
            data = json.load(file_handle)
    except (OSError, json.JSONDecodeError, lzma.LZMAError) as exc:
        raise ValueError(f"GeoJSON 地图损坏或无法解压: {map_file}") from exc
    if data.get("type") != "FeatureCollection":
        raise ValueError("GeoJSON 顶层必须是 FeatureCollection。")
    return data


def infer_geojson_bounds(map_file):
    """Infer west/south/east/north from the map extract filename."""
    match = _BOUNDS_PATTERN.search(FsPath(map_file).name)
    if match is None:
        return None
    lon1, lat1, lon2, lat2 = (float(value) for value in match.groups())
    return (
        min(lon1, lon2),
        min(lat1, lat2),
        max(lon1, lon2),
        max(lat1, lat2),
    )


def _iter_coordinate_sequences(geometry):
    geometry_type = geometry.get("type")
    coordinates = geometry.get("coordinates", [])
    if geometry_type == "LineString":
        yield coordinates
    elif geometry_type == "MultiLineString":
        yield from coordinates
    elif geometry_type == "Polygon":
        yield from coordinates
    elif geometry_type == "MultiPolygon":
        for polygon in coordinates:
            yield from polygon


def _feature_intersects_bounds(feature, bounds):
    geometry = feature.get("geometry") or {}
    geometry_type = geometry.get("type")
    coordinates = geometry.get("coordinates", [])
    west, south, east, north = bounds
    if geometry_type == "Point":
        points = [coordinates]
    else:
        points = [
            point
            for sequence in _iter_coordinate_sequences(geometry)
            for point in sequence
        ]
    valid = [
        point for point in points
        if isinstance(point, (list, tuple)) and len(point) >= 2
    ]
    if not valid:
        return False
    min_lon = min(point[0] for point in valid)
    max_lon = max(point[0] for point in valid)
    min_lat = min(point[1] for point in valid)
    max_lat = max(point[1] for point in valid)
    return not (
        max_lon < west
        or min_lon > east
        or max_lat < south
        or min_lat > north
    )


def _canvas_size(bounds, max_pixels):
    west, south, east, north = bounds
    mean_lat = math.radians((south + north) / 2.0)
    width_m = math.radians(east - west) * math.cos(mean_lat) * EARTH_RADIUS_M
    height_m = math.radians(north - south) * EARTH_RADIUS_M
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError("GeoJSON 地图边界无效。")
    scale = float(max_pixels) / max(width_m, height_m)
    return (
        max(256, int(round(width_m * scale))),
        max(256, int(round(height_m * scale))),
    )


def _pixel_transform(bounds, width, height):
    west, south, east, north = bounds

    def transform(point):
        x = (float(point[0]) - west) / (east - west) * (width - 1)
        y = (north - float(point[1])) / (north - south) * (height - 1)
        return int(round(x)), int(round(y))

    return transform


def _polygon_style(properties):
    if properties.get("natural") == "water" or properties.get("water"):
        return "#aad3df", "#8cb9c7"
    if properties.get("waterway") in {"riverbank", "dock"}:
        return "#aad3df", "#8cb9c7"
    if properties.get("building"):
        return "#d9d0c9", "#b8aaa0"
    landuse = properties.get("landuse")
    if landuse in _LAND_COLORS:
        return _LAND_COLORS[landuse], "#cad1c2"
    leisure = properties.get("leisure")
    if leisure in {"park", "garden", "pitch"}:
        return "#cfe8c8", "#b7d6ad"
    amenity = properties.get("amenity")
    if amenity in {"school", "university", "hospital", "kindergarten"}:
        return "#efe3dd", "#d7c6be"
    if amenity == "parking":
        return "#e3e7ea", "#c9d0d5"
    return None


def _draw_polygon(draw, geometry, transform, fill, outline):
    geometry_type = geometry.get("type")
    polygons = (
        [geometry.get("coordinates", [])]
        if geometry_type == "Polygon"
        else geometry.get("coordinates", [])
    )
    for polygon in polygons:
        if not polygon:
            continue
        outer = [transform(point) for point in polygon[0] if len(point) >= 2]
        if len(outer) >= 3:
            draw.polygon(outer, fill=fill, outline=outline, width=1)


def _draw_line_feature(draw, feature, transform):
    properties = feature.get("properties") or {}
    geometry = feature.get("geometry") or {}
    highway = properties.get("highway")
    if highway in _ROAD_STYLES:
        color, width = _ROAD_STYLES[highway]
        casing = "#c7c4bf"
        for sequence in _iter_coordinate_sequences(geometry):
            points = [transform(point) for point in sequence if len(point) >= 2]
            if len(points) >= 2:
                draw.line(points, fill=casing, width=width + 2, joint="curve")
                draw.line(points, fill=color, width=width, joint="curve")
        return

    waterway = properties.get("waterway")
    if waterway:
        width = 4 if waterway == "river" else 2
        color = "#79b7d2"
    elif properties.get("railway"):
        width, color = 2, "#777777"
    else:
        return
    for sequence in _iter_coordinate_sequences(geometry):
        points = [transform(point) for point in sequence if len(point) >= 2]
        if len(points) >= 2:
            draw.line(points, fill=color, width=width, joint="curve")


def _draw_point_feature(draw, feature, transform):
    properties = feature.get("properties") or {}
    coordinates = (feature.get("geometry") or {}).get("coordinates", [])
    if len(coordinates) < 2:
        return
    if properties.get("highway") == "traffic_signals":
        color, radius = "#dc2626", 3
    elif properties.get("highway") == "bus_stop":
        color, radius = "#2563eb", 2
    elif properties.get("amenity") in {"hospital", "school", "university"}:
        color, radius = "#7c3aed", 3
    else:
        return
    x, y = transform(coordinates)
    draw.ellipse(
        (x - radius, y - radius, x + radius, y + radius),
        fill=color,
        outline="#ffffff",
        width=1,
    )


def load_geojson_basemap(
    path,
    map_file,
    bounds=None,
    max_pixels=2200,
    padding_m=100.0,
):
    """Rasterize an OSM GeoJSON/GeoJSON.XZ extract in the GPX local frame."""
    if getattr(path, "lat", None) is None or getattr(path, "lon", None) is None:
        raise ValueError("GeoJSON 底图只能和 GPX 地理路径一起使用。")
    if int(max_pixels) < 256:
        raise ValueError("basemap_max_pixels 不能小于 256。")

    data = _read_geojson(map_file)
    if bounds is None:
        bounds = infer_geojson_bounds(map_file)
    if bounds is None:
        bounds = _geographic_bounds(path, padding_m)
    bounds = tuple(float(value) for value in bounds)
    if len(bounds) != 4:
        raise ValueError("GeoJSON 边界必须是 west/south/east/north。")

    width, height = _canvas_size(bounds, int(max_pixels))
    image = Image.new("RGB", (width, height), "#f4f2ed")
    draw = ImageDraw.Draw(image)
    transform = _pixel_transform(bounds, width, height)
    features = [
        feature
        for feature in data.get("features", [])
        if _feature_intersects_bounds(feature, bounds)
    ]

    for feature in features:
        geometry = feature.get("geometry") or {}
        if geometry.get("type") not in {"Polygon", "MultiPolygon"}:
            continue
        style = _polygon_style(feature.get("properties") or {})
        if style:
            _draw_polygon(draw, geometry, transform, *style)

    line_features = [
        feature
        for feature in features
        if (feature.get("geometry") or {}).get("type")
        in {"LineString", "MultiLineString"}
    ]
    line_features.sort(
        key=lambda feature: _ROAD_ORDER.get(
            (feature.get("properties") or {}).get("highway"),
            -1,
        )
    )
    for feature in line_features:
        _draw_line_feature(draw, feature, transform)

    for feature in features:
        if (feature.get("geometry") or {}).get("type") == "Point":
            _draw_point_feature(draw, feature, transform)

    west, south, east, north = bounds
    origin_lat, origin_lon = _origin(path)
    extent = (
        _lon_to_local_x(west, origin_lon, origin_lat),
        _lon_to_local_x(east, origin_lon, origin_lat),
        _lat_to_local_y(south, origin_lat),
        _lat_to_local_y(north, origin_lat),
    )
    return BasemapImage(
        image=np.asarray(image),
        extent=extent,
        attribution=OSM_ATTRIBUTION,
        geographic_bounds=bounds,
    )
