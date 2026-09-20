"""
Reference path construction for VDM_tracking.

Turns a raw road polyline into a reference path a tracking controller can
actually follow.  The repository's original GPX pipeline resamples a polyline
at a fixed spacing and differentiates the heading to get curvature, which
leaves two defects that matter on a real campus road:

1. Junctions between two OSM ways are single vertices, so the heading changes
   by ~90 degrees within one sample.  Differentiating that produces curvature
   spikes of the order of 1 m^-1, i.e. a turning radius below one metre, which
   is not physical and saturates any steering controller.
2. Differentiating a linearly interpolated heading is noisy even on gentle
   geometry, so the curvature is dominated by interpolation artefacts rather
   than by the road.

This module fixes both and derives a curvature-limited speed profile, which
is what "reference path setting" means for a navigation task: the car should
slow down for bends instead of cruising at a constant target speed.
"""

from __future__ import annotations

import numpy as np

from vdm_lab.common.gpx import local_xy_to_latlon
from vdm_lab.common.types import Path


DEFAULT_CORNER_RADIUS_M = 8.0
DEFAULT_MIN_TURN_DEG = 20.0
DEFAULT_CURVATURE_SMOOTH_M = 12.0
DEFAULT_LATERAL_ACCEL_LIMIT = 2.5
DEFAULT_TARGET_SPEED = 8.0


def polyline_length(xy):
    """Cumulative arc length of a polyline."""
    xy = np.asarray(xy, dtype=float)
    steps = np.hypot(*np.diff(xy, axis=0).T)
    return np.concatenate([[0.0], np.cumsum(steps)])


def resample_polyline(xy, ds):
    """Resample an (n, 2) polyline at a fixed arc-length spacing."""
    xy = np.asarray(xy, dtype=float)
    s = polyline_length(xy)

    keep = np.concatenate([[True], np.diff(s) > 1.0e-9])
    xy = xy[keep]
    s = s[keep]
    if len(xy) < 2:
        raise ValueError("折线有效点不足 2 个，无法重采样。")

    targets = np.arange(0.0, s[-1], float(ds))
    if len(targets) == 0 or targets[-1] < s[-1]:
        targets = np.append(targets, s[-1])

    return np.column_stack(
        [np.interp(targets, s, xy[:, 0]), np.interp(targets, s, xy[:, 1])]
    ), targets


def _signed_turn(vin, vout):
    """Signed deflection angle [rad] from direction vin to direction vout."""
    cross = vin[0] * vout[1] - vin[1] * vout[0]
    dot = float(vin[0] * vout[0] + vin[1] * vout[1])
    return float(np.arctan2(cross, dot))


def round_corners(
    xy,
    radius=DEFAULT_CORNER_RADIUS_M,
    min_turn_deg=DEFAULT_MIN_TURN_DEG,
    arc_step_m=0.5,
):
    """
    Replace sharp polyline vertices with circular fillet arcs.

    Each vertex whose deflection exceeds `min_turn_deg` is cut back by a
    tangent length chosen for the requested radius, and the corner is
    replaced by a circular arc.  The tangent length is capped at 45% of each
    adjacent segment so neighbouring corners cannot overlap.

    Returns
    -------
    xy_rounded : (m, 2) array
    corners : list of dict
        One record per filleted corner, for reporting.
    """
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 3:
        return xy.copy(), []

    sin_min = np.sin(np.deg2rad(float(min_turn_deg)))
    step = max(float(arc_step_m), 0.05)
    output = [xy[0]]
    corners = []

    for index in range(1, len(xy) - 1):
        previous = xy[index - 1]
        corner = xy[index]
        following = xy[index + 1]

        incoming = corner - previous
        outgoing = following - corner
        length_in = float(np.hypot(*incoming))
        length_out = float(np.hypot(*outgoing))
        if length_in < 1.0e-6 or length_out < 1.0e-6:
            output.append(corner)
            continue

        turn = _signed_turn(incoming / length_in, outgoing / length_out)
        if abs(np.sin(turn)) < sin_min:
            output.append(corner)
            continue

        # Tangent length for the requested radius, capped so adjacent
        # corners cannot consume each other's segments.
        tangent = float(radius) * np.tan(abs(turn) / 2.0)
        tangent = min(tangent, 0.45 * length_in, 0.45 * length_out)
        if tangent < 1.0e-3:
            output.append(corner)
            continue

        effective_radius = tangent / np.tan(abs(turn) / 2.0)
        unit_in = incoming / length_in
        unit_out = outgoing / length_out

        start = corner - unit_in * tangent
        end = corner + unit_out * tangent

        bisector = unit_out - unit_in
        norm = float(np.hypot(*bisector))
        if norm < 1.0e-9:
            output.append(corner)
            continue
        bisector = bisector / norm

        centre = corner + bisector * (effective_radius / np.cos(abs(turn) / 2.0))

        start_angle = np.arctan2(start[1] - centre[1], start[0] - centre[0])
        end_angle = np.arctan2(end[1] - centre[1], end[0] - centre[0])
        sweep = (end_angle - start_angle + np.pi) % (2.0 * np.pi) - np.pi
        if np.sign(sweep) != np.sign(turn):
            sweep = -sweep

        arc_length = abs(sweep) * effective_radius
        count = max(int(np.ceil(arc_length / step)), 2)
        angles = start_angle + sweep * np.linspace(0.0, 1.0, count)

        output.append(start)
        for angle in angles[1:-1]:
            output.append(
                centre + effective_radius * np.array([np.cos(angle), np.sin(angle)])
            )
        output.append(end)

        corners.append(
            {
                "vertex_index": index,
                "x": float(corner[0]),
                "y": float(corner[1]),
                "turn_deg": float(np.rad2deg(turn)),
                "radius_m": float(effective_radius),
                "tangent_m": float(tangent),
                "arc_length_m": float(arc_length),
            }
        )

    output.append(xy[-1])
    return np.asarray(output, dtype=float), corners


def curvature_profile(xy, ds, smooth_m=DEFAULT_CURVATURE_SMOOTH_M):
    """
    Curvature along a resampled polyline [1/m].

    The raw derivative of a linearly interpolated heading is dominated by
    interpolation noise, so the result is smoothed over `smooth_m` metres.
    """
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 3:
        raise ValueError("至少需要 3 个点才能计算曲率。")

    s = polyline_length(xy)
    dx = np.gradient(xy[:, 0], s, edge_order=1)
    dy = np.gradient(xy[:, 1], s, edge_order=1)
    yaw = np.unwrap(np.arctan2(dy, dx))
    raw = np.gradient(yaw, s, edge_order=1)

    sigma = max(float(smooth_m), 0.0) / max(float(ds), 1.0e-6) / 2.355
    if sigma > 0.5:
        from scipy.ndimage import gaussian_filter1d

        curvature = gaussian_filter1d(raw, sigma=sigma, mode="nearest")
    else:
        curvature = raw

    return s, curvature, raw


def curvature_speed_profile(
    s,
    curvature,
    target_speed=DEFAULT_TARGET_SPEED,
    lateral_accel_limit=DEFAULT_LATERAL_ACCEL_LIMIT,
    max_accel=2.0,
    max_decel=3.5,
    final_speed=0.0,
    plan_decel_ratio=0.5,
):
    """
    Speed profile limited by lateral acceleration on bends.

    The cornering limit is a_n = v^2 * kappa, so a bend with curvature kappa
    caps the speed at sqrt(a_lat_limit / kappa).  A backward pass then makes
    sure the car can decelerate in time before each bend and a forward pass
    keeps the acceleration within the vehicle limit.

    `final_speed` is applied *before* the two passes.  The backward pass can
    only plan a braking ramp if it starts from the speed the car must have at
    the end of the path; setting it afterwards leaves the profile at cruise
    speed right up to the last waypoint and the car overshoots the goal.

    `plan_decel_ratio` plans the braking at only this fraction of the vehicle
    deceleration limit.  A ramp scheduled at exactly the limit cannot be
    tracked: any controller lag turns into overshoot of the goal.  The
    default of 0.5 (1.75 m/s^2 on the student car) is chosen so that the
    profile is also feasible for the tightest controller in the repository -
    the MPC, whose QP constrains |a| <= max_accel and therefore cannot brake
    harder than 2.0 m/s^2.
    """
    s = np.asarray(s, dtype=float)
    curvature = np.asarray(curvature, dtype=float)

    kappa = np.maximum(np.abs(curvature), 1.0e-6)
    speed = np.minimum(float(target_speed), np.sqrt(lateral_accel_limit / kappa))
    speed = np.clip(speed, 0.0, float(target_speed))
    speed[0] = min(speed[0], float(target_speed))
    speed[-1] = float(final_speed)

    brake = float(max_decel) * float(plan_decel_ratio)
    if len(s) > 1:
        # Both passes are recurrences: each point is limited by the *already
        # limited* speed of its neighbour, so the ramp propagates one sample
        # at a time.  Written as a single array expression the limit is
        # computed from the un-limited profile and never propagates, which
        # leaves the car at cruise speed right up to the final waypoint.
        for i in range(len(speed) - 2, -1, -1):
            reachable = np.sqrt(
                max(speed[i + 1] ** 2 + 2.0 * brake * (s[i + 1] - s[i]), 0.0)
            )
            if speed[i] > reachable:
                speed[i] = reachable

        for i in range(1, len(speed)):
            reachable = np.sqrt(
                max(speed[i - 1] ** 2 + 2.0 * max_accel * (s[i] - s[i - 1]), 0.0)
            )
            if speed[i] > reachable:
                speed[i] = reachable

    speed = np.clip(speed, 0.0, float(target_speed))
    speed[-1] = float(final_speed)
    return speed


def constant_speed_profile(s, target_speed=DEFAULT_TARGET_SPEED, decel=0.9):
    """
    The repository's original profile: cruise at a constant target speed and
    decelerate over the final metres.  Kept as the baseline for comparison.
    """
    s = np.asarray(s, dtype=float)
    remaining = np.maximum(s[-1] - s, 0.0)
    speed = np.minimum(float(target_speed), np.sqrt(2.0 * decel * remaining))
    speed[-1] = 0.0
    return speed


def build_reference_path(
    xy,
    ds,
    origin_lat=None,
    origin_lon=None,
    target_speed=DEFAULT_TARGET_SPEED,
    speed_profile="curvature",
    corner_radius=DEFAULT_CORNER_RADIUS_M,
    min_turn_deg=DEFAULT_MIN_TURN_DEG,
    smooth_m=DEFAULT_CURVATURE_SMOOTH_M,
    lateral_accel_limit=DEFAULT_LATERAL_ACCEL_LIMIT,
    max_accel=2.0,
    max_decel=3.5,
    source="road_graph",
):
    """
    Build a controller-ready Path from a raw road polyline.

    Steps: fillet the sharp corners -> resample -> curvature -> speed profile.

    `speed_profile` is either "curvature" (lateral-acceleration limited) or
    "constant" (the repository's original behaviour).
    """
    rounded, corners = round_corners(
        xy, radius=corner_radius, min_turn_deg=min_turn_deg
    )
    resampled, s = resample_polyline(rounded, ds)
    _, curvature, _ = curvature_profile(resampled, ds, smooth_m=smooth_m)

    if speed_profile == "curvature":
        speed = curvature_speed_profile(
            s,
            curvature,
            target_speed=target_speed,
            lateral_accel_limit=lateral_accel_limit,
            max_accel=max_accel,
            max_decel=max_decel,
        )
    elif speed_profile == "constant":
        speed = constant_speed_profile(s, target_speed=target_speed)
    else:
        raise ValueError(
            f"未知速度曲线 {speed_profile}，可选值为: curvature, constant"
        )

    dx = np.gradient(resampled[:, 0], s, edge_order=1)
    dy = np.gradient(resampled[:, 1], s, edge_order=1)
    yaw = np.unwrap(np.arctan2(dy, dx))
    yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

    lat = lon = None
    if origin_lat is not None and origin_lon is not None:
        lat, lon = local_xy_to_latlon(
            resampled[:, 0], resampled[:, 1], origin_lat, origin_lon
        )

    path = Path(
        x=resampled[:, 0],
        y=resampled[:, 1],
        yaw=yaw,
        curvature=curvature,
        s=s,
        target_speed=speed,
        lat=lat,
        lon=lon,
        elevation=None,
        source=source,
    )
    path.corners = corners
    return path
