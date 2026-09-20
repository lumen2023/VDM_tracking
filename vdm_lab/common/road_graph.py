"""
Offline OSM road graph for VDM_tracking.

Builds a weighted undirected graph from the repository's offline OSM GeoJSON
extract, so that a reference path can be generated between two geographic
landmarks (for example a dormitory and a teaching building) along the real
road network without any network access.

The graph shares the same local East/North metric frame as
vdm_lab.common.gpx, so graph routes and the rasterized basemap use one origin
and overlay correctly.

Only road classes a forward-only car can physically use are kept; footways,
cycleways, steps and motorways are excluded.  On the current extract this is
also the cleaner choice: the drivable subset is a single connected component
while the full network fragments into nine.
"""

from __future__ import annotations

import heapq
import json
import lzma
from dataclasses import dataclass
from pathlib import Path as FsPath

import numpy as np

from vdm_lab.common.gpx import latlon_to_local_xy


# Road classes a forward-only car may use.
DRIVABLE_HIGHWAYS = frozenset(
    {
        "primary",
        "secondary",
        "tertiary",
        "unclassified",
        "residential",
        "service",
        "living_street",
        "primary_link",
        "secondary_link",
        "tertiary_link",
    }
)

# Road vertices closer than this are merged into a single graph node [m].
DEFAULT_MERGE_TOLERANCE_M = 1.0


def read_geojson(map_file):
    """Read a plain or xz-compressed GeoJSON file into a dict."""
    map_file = FsPath(map_file)
    if not map_file.exists():
        raise FileNotFoundError(f"地图文件不存在: {map_file}")

    if map_file.suffix == ".xz":
        with lzma.open(map_file, "rt", encoding="utf-8") as handle:
            return json.load(handle)

    with open(map_file, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _iter_positions(geometry):
    """Yield every (lon, lat) position of any LineString/MultiLineString."""
    coords = geometry.get("coordinates")
    if coords is None:
        return

    stack = [coords]
    while stack:
        item = stack.pop()
        if not item:
            continue
        head = item[0]
        if isinstance(head, (int, float)):
            yield item
        else:
            stack.extend(item)


def road_vertex_arrays(geojson, highway_filter=DRIVABLE_HIGHWAYS):
    """
    Extract road polylines as local metric arrays.

    Returns
    -------
    list of (highway, name, xy)
        xy is an (n, 2) array in local East/North metres.
    """
    roads = []
    for feature in geojson.get("features", []):
        properties = feature.get("properties", {})
        highway = properties.get("highway")
        if highway is None or highway not in highway_filter:
            continue

        positions = list(_iter_positions(feature.get("geometry", {})))
        if len(positions) < 2:
            continue

        array = np.asarray(positions, dtype=float)
        x, y = latlon_to_local_xy(array[:, 1], array[:, 0])
        roads.append((highway, properties.get("name"), np.column_stack([x, y])))

    return roads


@dataclass
class RoadGraph:
    """Weighted undirected graph over merged OSM road vertices."""

    xy: np.ndarray
    lat: np.ndarray
    lon: np.ndarray
    adjacency: list
    origin_lat: float
    origin_lon: float
    highway_counts: dict

    _tree: object = None

    # ---------------- construction helpers ----------------

    @property
    def node_count(self):
        return len(self.xy)

    @property
    def edge_count(self):
        return sum(len(neighbours) for neighbours in self.adjacency) // 2

    @property
    def total_length_m(self):
        return sum(
            weight
            for neighbours in self.adjacency
            for _, weight in neighbours
        ) / 2.0

    # ---------------- queries ----------------

    def _index(self):
        if self._tree is None:
            from scipy.spatial import cKDTree

            self._tree = cKDTree(self.xy)
        return self._tree

    def snap(self, x, y):
        """
        Nearest graph node to a local metric point.

        Returns
        -------
        node_index, offset_m
        """
        distance, index = self._index().query([float(x), float(y)])
        return int(index), float(distance)

    def nearest_road_offset(self, x, y):
        """Distance from a point to the road network [m]."""
        return self.snap(x, y)[1]

    def route(self, start_xy, goal_xy):
        """
        Dijkstra shortest path between two local metric points.

        Both endpoints are snapped to their nearest node first.  Returns the
        list of node indices, or None when the two points are not connected.
        """
        start, _ = self.snap(*start_xy)
        goal, _ = self.snap(*goal_xy)
        return self.route_nodes(start, goal)

    def route_nodes(self, start, goal):
        if start == goal:
            return [start]

        distance = {start: 0.0}
        previous = {}
        queue = [(0.0, start)]

        while queue:
            current_distance, node = heapq.heappop(queue)
            if node == goal:
                break
            if current_distance > distance.get(node, np.inf):
                continue

            for neighbour, weight in self.adjacency[node]:
                candidate = current_distance + weight
                if candidate < distance.get(neighbour, np.inf):
                    distance[neighbour] = candidate
                    previous[neighbour] = node
                    heapq.heappush(queue, (candidate, neighbour))

        if goal not in distance:
            return None

        path = [goal]
        while path[-1] != start:
            path.append(previous[path[-1]])
        path.reverse()
        return path

    def polyline(self, node_ids):
        """Node indices -> (n, 2) local metric polyline."""
        return self.xy[np.asarray(node_ids, dtype=int)]

    def polyline_latlon(self, node_ids):
        """Node indices -> (lat, lon) arrays."""
        index = np.asarray(node_ids, dtype=int)
        return self.lat[index], self.lon[index]

    def component_size(self, node):
        """Number of nodes reachable from `node` (connectivity check)."""
        seen = {node}
        stack = [node]
        while stack:
            current = stack.pop()
            for neighbour, _ in self.adjacency[current]:
                if neighbour not in seen:
                    seen.add(neighbour)
                    stack.append(neighbour)
        return len(seen)

    def component_count(self):
        seen = set()
        count = 0
        for node in range(self.node_count):
            if node in seen:
                continue
            count += 1
            stack = [node]
            seen.add(node)
            while stack:
                current = stack.pop()
                for neighbour, _ in self.adjacency[current]:
                    if neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        return count


def build_road_graph(
    map_file,
    origin_lat,
    origin_lon,
    merge_tolerance_m=DEFAULT_MERGE_TOLERANCE_M,
    highway_filter=DRIVABLE_HIGHWAYS,
):
    """
    Build a RoadGraph from an offline OSM GeoJSON extract.

    Parameters
    ----------
    map_file : path
        Plain or ``.xz`` GeoJSON file.
    origin_lat, origin_lon : float
        WGS84 origin of the local East/North frame.  Must match the basemap
        origin, otherwise the route and the background will not overlay.
    merge_tolerance_m : float
        Road vertices closer than this become one node.  Roads meet at shared
        nodes in OSM, so a small tolerance is enough to weld the network.
    highway_filter : set of str
        Road classes to keep.
    """
    geojson = read_geojson(map_file)

    tolerance = float(merge_tolerance_m)
    origin_lat = float(origin_lat)
    origin_lon = float(origin_lon)

    cells = {}
    coords = []
    lats = []
    lons = []
    neighbours = []
    edges = set()
    highway_counts = {}

    def node_of(lon, lat):
        """WGS84 -> merged node index in the caller's local frame."""
        x, y = latlon_to_local_xy(
            np.asarray([lat]), np.asarray([lon]), origin_lat, origin_lon
        )
        key = (int(np.floor(x[0] / tolerance)), int(np.floor(y[0] / tolerance)))
        node = cells.get(key)
        if node is None:
            node = len(coords)
            cells[key] = node
            coords.append((float(x[0]), float(y[0])))
            lats.append(float(lat))
            lons.append(float(lon))
        return node

    for feature in geojson.get("features", []):
        properties = feature.get("properties", {})
        highway = properties.get("highway")
        if highway is None or highway not in highway_filter:
            continue

        positions = list(_iter_positions(feature.get("geometry", {})))
        if len(positions) < 2:
            continue

        highway_counts[highway] = highway_counts.get(highway, 0) + 1
        ids = [node_of(position[0], position[1]) for position in positions]

        while len(neighbours) < len(coords):
            neighbours.append([])

        for first, second in zip(ids[:-1], ids[1:]):
            if first == second:
                continue
            key = (min(first, second), max(first, second))
            if key in edges:
                continue
            edges.add(key)

            p = np.asarray(coords[first])
            q = np.asarray(coords[second])
            weight = float(np.hypot(*(q - p)))
            neighbours[first].append((second, weight))
            neighbours[second].append((first, weight))

    if not coords:
        raise ValueError(f"地图中未找到可通行道路: {map_file}")

    while len(neighbours) < len(coords):
        neighbours.append([])

    return RoadGraph(
        xy=np.asarray(coords, dtype=float),
        lat=np.asarray(lats, dtype=float),
        lon=np.asarray(lons, dtype=float),
        adjacency=neighbours,
        origin_lat=float(origin_lat),
        origin_lon=float(origin_lon),
        highway_counts=highway_counts,
    )


def named_feature_points(map_file, names, origin_lat, origin_lon):
    """
    Centroid of every named OSM feature whose name is in `names`.

    Used to locate landmarks such as a dormitory or a teaching building in the
    same local frame as the road graph.

    Returns
    -------
    dict {name: (x, y)}
    """
    wanted = set(names)
    geojson = read_geojson(map_file)
    found = {}

    for feature in geojson.get("features", []):
        name = feature.get("properties", {}).get("name")
        if name not in wanted:
            continue

        positions = list(_iter_positions(feature.get("geometry", {})))
        if not positions:
            continue

        array = np.asarray(positions, dtype=float)
        x, y = latlon_to_local_xy(array[:, 1], array[:, 0], origin_lat, origin_lon)
        found.setdefault(name, []).append((float(x.mean()), float(y.mean())))

    return {name: points[0] for name, points in found.items()}
