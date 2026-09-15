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


def preview_path_curvature(reference, distance):
    """Sample path curvature at a nonnegative forward arc-length offset."""
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
    offset = max(0.0, finite_float(distance, "preview distance"))
    target_s = path_s[index] + offset
    preview_index = min(
        int(np.searchsorted(path_s, target_s, side="left")),
        point_count - 1,
    )
    return float(curvature[preview_index])
