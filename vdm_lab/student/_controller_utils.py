"""Student-controller-only validation and path helpers.

This module deliberately lives under :mod:`vdm_lab.student`.  The shared
simulation and vehicle interfaces may also be used by physical backends, so
the student exercises must not mutate those modules to compensate for
controller behavior.
"""

import math

import numpy as np


def finite_float(value, name):
    """Return *value* as a finite float or raise a descriptive ValueError."""
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validated_wheelbase(vehicle):
    """Return the plant wheelbase after checking duplicated geometry fields.

    The shared bicycle plant derives its wheelbase from ``lf + lr`` while the
    controller configuration also carries ``wheelbase``.  Silently accepting
    disagreement would make all steering models inconsistent with the plant.
    """
    configured = finite_float(vehicle.wheelbase, "wheelbase")
    lf = finite_float(vehicle.lf, "lf")
    lr = finite_float(vehicle.lr, "lr")
    plant_wheelbase = lf + lr
    if configured <= 0.0 or lf <= 0.0 or lr <= 0.0:
        raise ValueError("wheelbase, lf, and lr must be positive")
    tolerance = max(1.0e-9, 1.0e-6 * plant_wheelbase)
    if abs(configured - plant_wheelbase) > tolerance:
        raise ValueError(
            "vehicle.wheelbase must agree with vehicle.lf + vehicle.lr"
        )
    return plant_wheelbase


def remaining_path_distance(path, nearest_index):
    """Return forward arc length from the tracked path index to its endpoint.

    ``path.s`` is preferred because it is the route's progress coordinate.  A
    geometric segment sum is retained for direct/unit callers that construct a
    minimal path without a usable ``s`` array.  Endpoint Euclidean distance is
    intentionally not used: it causes premature braking on loops and routes
    that pass near their endpoint before they are complete.
    """
    point_count = min(len(path.x), len(path.y))
    if point_count < 1:
        raise ValueError("reference path is empty")
    try:
        index = int(nearest_index)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("nearest path index is invalid") from exc
    index = min(max(index, 0), point_count - 1)

    try:
        path_s = np.asarray(getattr(path, "s", ()), dtype=float).reshape(-1)
    except (TypeError, ValueError):
        path_s = np.array([], dtype=float)
    if (
        path_s.size >= point_count
        and np.all(np.isfinite(path_s[:point_count]))
        and np.all(np.diff(path_s[:point_count]) >= 0.0)
    ):
        return max(0.0, float(path_s[point_count - 1] - path_s[index]))

    x = np.asarray(path.x[:point_count], dtype=float)
    y = np.asarray(path.y[:point_count], dtype=float)
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("reference path coordinates must be finite")
    if index >= point_count - 1:
        return 0.0
    return float(np.hypot(np.diff(x[index:]), np.diff(y[index:])).sum())


def _reference_path_progress(reference):
    """Return monotonic path progress, the tracked index, and curvature samples."""
    path = reference.path
    point_count = min(len(path.x), len(path.y), len(path.curvature))
    if point_count < 1:
        raise ValueError("reference path has no curvature samples")
    try:
        index = int(reference.nearest_index)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("nearest path index is invalid") from exc
    index = min(max(index, 0), point_count - 1)
    curvature = np.asarray(path.curvature[:point_count], dtype=float)
    if not np.all(np.isfinite(curvature)):
        raise ValueError("path curvature must be finite")
    try:
        path_s = np.asarray(getattr(path, "s", ()), dtype=float).reshape(-1)
    except (TypeError, ValueError):
        path_s = np.array([], dtype=float)
    if (
        path_s.size < point_count
        or not np.all(np.isfinite(path_s[:point_count]))
        or np.any(np.diff(path_s[:point_count]) < 0.0)
    ):
        x = np.asarray(path.x[:point_count], dtype=float)
        y = np.asarray(path.y[:point_count], dtype=float)
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise ValueError("path coordinates must be finite")
        path_s = np.concatenate(
            ([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y))))
        )
    else:
        path_s = path_s[:point_count]
    return path_s, index, curvature, point_count


def preview_path_curvature(reference, distance):
    """Sample path curvature at a nonnegative forward arc-length offset."""
    path_s, index, curvature, point_count = _reference_path_progress(reference)
    offset = max(0.0, finite_float(distance, "preview distance"))
    target_s = path_s[index] + offset
    preview_index = min(
        int(np.searchsorted(path_s, target_s, side="left")),
        point_count - 1,
    )
    return float(curvature[preview_index])


def preview_max_abs_curvature(reference, distance):
    """Return max |curvature| from the tracked index through a forward offset."""
    path_s, index, curvature, point_count = _reference_path_progress(reference)
    offset = max(0.0, finite_float(distance, "preview distance"))
    target_s = path_s[index] + offset
    end_index = min(
        int(np.searchsorted(path_s, target_s, side="left")),
        point_count - 1,
    )
    start_index = min(index, end_index)
    stop_index = max(index, end_index) + 1
    return float(np.max(np.abs(curvature[start_index:stop_index])))


def tracking_speed_limit(target_speed, curvature, vehicle):
    """Cap cruise speed so the required steer remains rate-feasible.

    Built-in lab routes at the default 45 deg/s steer rate keep their design
    speeds. Sharp GPX corners and reduced steer-rate experiments are slowed
    because the vehicle would otherwise demand more steering than it can slew.
    """
    try:
        target = float(target_speed)
    except (TypeError, ValueError, OverflowError):
        target = 0.0
    if not math.isfinite(target):
        target = 0.0
    target = max(0.0, target)

    try:
        kappa = abs(float(curvature))
    except (TypeError, ValueError, OverflowError):
        kappa = 0.0
    if not math.isfinite(kappa):
        kappa = 0.0

    try:
        min_speed = max(0.0, float(vehicle.min_speed))
    except (TypeError, ValueError, OverflowError):
        min_speed = 0.0
    if not math.isfinite(min_speed):
        min_speed = 0.0
    try:
        max_speed = float(vehicle.max_speed)
    except (TypeError, ValueError, OverflowError):
        max_speed = min_speed
    if not math.isfinite(max_speed):
        max_speed = min_speed
    max_speed = max(min_speed, max_speed)
    cap = min(target, max_speed)
    if kappa <= 1.0e-9:
        return cap

    try:
        wheelbase = float(vehicle.wheelbase)
        max_steer = abs(float(vehicle.max_steer))
        max_rate = abs(float(vehicle.max_steer_rate))
    except (TypeError, ValueError, OverflowError):
        return cap
    if not all(math.isfinite(value) for value in (wheelbase, max_steer, max_rate)):
        return cap
    if wheelbase <= 0.0 or max_steer <= 0.0 or max_rate <= 0.0:
        return cap

    delta = min(math.atan(wheelbase * kappa), max_steer)
    if delta > 1.0e-9:
        # Allow about two curvature radii of travel while slewing onto delta.
        cap = min(cap, 2.0 * max_rate / (delta * kappa))
    return max(min_speed, cap)


def command_target_speed(target_speed, reference, measured_speed, vehicle):
    """Path target after a short-horizon curvature preview."""
    try:
        speed = float(measured_speed)
    except (TypeError, ValueError, OverflowError):
        speed = 0.0
    if not math.isfinite(speed):
        speed = 0.0
    preview_distance = max(10.0, max(0.0, speed) * 1.5)
    try:
        kappa = preview_max_abs_curvature(reference, preview_distance)
    except ValueError:
        try:
            kappa = abs(float(getattr(reference, "curvature", 0.0)))
        except (TypeError, ValueError, OverflowError):
            kappa = 0.0
        if not math.isfinite(kappa):
            kappa = 0.0
    return tracking_speed_limit(target_speed, kappa, vehicle)
