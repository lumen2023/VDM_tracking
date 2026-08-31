import numpy as np
from scipy.interpolate import CubicSpline

from vdm_lab.config.routes import DEFAULT_ROUTE_NAME, get_route_spec
from vdm_lab.config.speed_profiles import DEFAULT_SPEED_MODE, resolve_target_speed
from vdm_lab.common.types import Path


def generate_reference_path(
    route_name=DEFAULT_ROUTE_NAME,
    speed_mode=DEFAULT_SPEED_MODE,
    waypoints=None,
    ds=0.5,
    target_speed=None,
):
    route = get_route_spec(route_name)
    target_speed = resolve_target_speed(route, speed_mode, target_speed)

    if route.kind == "right_angle" and waypoints is None:
        return _generate_right_angle_path(ds=ds, target_speed=target_speed)

    points = route.waypoints if waypoints is None else np.asarray(waypoints, dtype=float)
    if len(points) < 3:
        raise ValueError("至少需要 3 个路点，才能生成用于路径跟踪的平滑路线。")

    distances = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    s_waypoints = np.concatenate(([0.0], np.cumsum(distances)))
    s = np.arange(0.0, s_waypoints[-1] + ds, ds)
    s[-1] = min(s[-1], s_waypoints[-1])

    spline_x = CubicSpline(s_waypoints, points[:, 0], bc_type="natural")
    spline_y = CubicSpline(s_waypoints, points[:, 1], bc_type="natural")

    x = spline_x(s)
    y = spline_y(s)
    dx = spline_x(s, 1)
    dy = spline_y(s, 1)
    ddx = spline_x(s, 2)
    ddy = spline_y(s, 2)

    yaw = np.arctan2(dy, dx)
    denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1.0e-8)
    curvature = (dx * ddy - dy * ddx) / denominator

    speed = _forward_speed_profile(s, target_speed)

    return Path(x=x, y=y, yaw=yaw, curvature=curvature, s=s, target_speed=speed)


def _generate_right_angle_path(ds=0.5, target_speed=5.5):
    radius = 5.0
    straight_in = 14.0
    straight_out = 22.0

    x_in = np.arange(0.0, straight_in, ds)
    y_in = np.zeros_like(x_in)
    yaw_in = np.zeros_like(x_in)
    curvature_in = np.zeros_like(x_in)

    theta = np.arange(-0.5 * np.pi, 0.0, ds / radius)
    x_arc = straight_in + radius * np.cos(theta)
    y_arc = radius + radius * np.sin(theta)
    yaw_arc = theta + 0.5 * np.pi
    curvature_arc = np.full_like(theta, 1.0 / radius)

    y_out = np.arange(radius, radius + straight_out + ds, ds)
    x_out = np.full_like(y_out, straight_in + radius)
    yaw_out = np.full_like(y_out, 0.5 * np.pi)
    curvature_out = np.zeros_like(y_out)

    x = np.concatenate([x_in, x_arc, x_out])
    y = np.concatenate([y_in, y_arc, y_out])
    yaw = np.concatenate([yaw_in, yaw_arc, yaw_out])
    curvature = np.concatenate([curvature_in, curvature_arc, curvature_out])
    segment_lengths = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    target = _forward_speed_profile(s, target_speed)
    return Path(x=x, y=y, yaw=yaw, curvature=curvature, s=s, target_speed=target)


def _forward_speed_profile(s, target_speed):
    remaining = np.maximum(s[-1] - s, 0.0)
    speed = np.minimum(target_speed, np.sqrt(2.0 * 0.9 * remaining))
    speed[-1] = 0.0
    return speed
