from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RouteSpec:
    name: str
    title: str
    kind: str
    waypoints: np.ndarray | None = None
    target_speeds: dict[str, float] | None = None


ROUTES = {
    "double_lane_change": RouteSpec(
        name="double_lane_change",
        title="双移线",
        kind="spline",
        waypoints=np.array(
            [
                [0.0, 0.0],
                [15.0, 0.0],
                [27.0, 3.5],
                [42.0, 3.5],
                [54.0, 0.0],
                [72.0, 0.0],
            ]
        ),
        target_speeds={"low": 4.0, "medium": 7.0, "high": 9.0},
    ),
    "right_angle": RouteSpec(
        name="right_angle",
        title="直角弯",
        kind="right_angle",
        target_speeds={"low": 3.5, "medium": 5.5, "high": 7.0},
    ),
    "s_curve": RouteSpec(
        name="s_curve",
        title="S 弯道",
        kind="spline",
        waypoints=np.array(
            [
                [0.0, 0.0],
                [12.0, 3.0],
                [24.0, -4.0],
                [36.0, 4.0],
                [48.0, -3.0],
                [62.0, 0.0],
            ]
        ),
        target_speeds={"low": 4.0, "medium": 6.5, "high": 8.0},
    ),
    "mixed_course": RouteSpec(
        name="mixed_course",
        title="综合路线",
        kind="spline",
        waypoints=np.array(
            [
                [0.0, 0.0],
                [12.0, 0.0],
                [24.0, 8.0],
                [36.0, 8.0],
                [48.0, -3.0],
                [60.0, 0.0],
                [72.0, 10.0],
                [84.0, 10.0],
            ]
        ),
        target_speeds={"low": 4.0, "medium": 7.0, "high": 9.0},
    ),
}


DEFAULT_ROUTE_NAME = "mixed_course"


def available_route_names():
    return tuple(ROUTES.keys())


def get_route_spec(name=DEFAULT_ROUTE_NAME):
    try:
        return ROUTES[name]
    except KeyError as exc:
        choices = ", ".join(available_route_names())
        raise ValueError(f"未知路线 {name}，可选值为: {choices}") from exc
