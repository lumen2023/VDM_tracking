"""Student-controller-only validation, limits, and path helpers.

This module lives under :mod:`vdm_lab.student`.  Shared simulation and vehicle
interfaces may be used by physical backends, so student exercises must not
mutate those modules to compensate for controller behavior.
"""

import math

import numpy as np

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


def finite(value, default=0.0):
    """Return *value* as a finite float, otherwise *default*."""
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def finite_float(value, name):
    """Return *value* as a finite float or raise a descriptive ValueError."""
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def vehicle_speed_bounds(vehicle):
    """Return ``(min_speed, max_speed)`` after sanitizing vehicle limits."""
    min_speed = max(0.0, finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, finite(vehicle.max_speed, min_speed))
    return min_speed, max_speed


def state_in_speed_envelope(state, vehicle):
    """True when pose/speed are finite and speed is inside vehicle limits."""
    if not all(
        math.isfinite(finite(value, math.nan))
        for value in (state.x, state.y, state.yaw, state.v)
    ):
        return False
    min_speed, max_speed = vehicle_speed_bounds(vehicle)
    speed = finite(state.v, math.nan)
    return min_speed - 1.0e-8 <= speed <= max_speed + 1.0e-8


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


def validate_safety_config(config):
    """Raise if actuator, timing, or geometry limits cannot be used safely."""
    vehicle = config.vehicle
    validated_wheelbase(vehicle)
    values = {
        "dt": config.sim.dt,
        "min_speed": vehicle.min_speed,
        "max_speed": vehicle.max_speed,
        "max_accel": vehicle.max_accel,
        "max_decel": vehicle.max_decel,
        "max_steer": vehicle.max_steer,
        "max_steer_rate": vehicle.max_steer_rate,
        "wheelbase": vehicle.wheelbase,
    }
    checked = {}
    for name, value in values.items():
        try:
            checked[name] = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"invalid safety configuration: {name} must be numeric"
            ) from exc
        if not math.isfinite(checked[name]):
            raise ValueError(
                f"invalid safety configuration: {name} must be finite"
            )
    if checked["dt"] <= 0.0:
        raise ValueError("invalid safety configuration: dt must be positive")
    if checked["min_speed"] < 0.0:
        raise ValueError(
            "invalid safety configuration: min_speed cannot be negative"
        )
    if checked["max_speed"] <= checked["min_speed"]:
        raise ValueError(
            "invalid safety configuration: max_speed must exceed min_speed"
        )
    if checked["max_accel"] <= 0.0 or checked["max_decel"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: acceleration limits must be positive"
        )
    if not 0.0 < checked["max_steer"] < 0.5 * math.pi:
        raise ValueError(
            "invalid safety configuration: max_steer must be in (0, pi/2)"
        )
    if checked["max_steer_rate"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: max_steer_rate must be positive"
        )
    if checked["wheelbase"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: wheelbase must be positive"
        )


def bounded_steer(desired, previous, config):
    """Apply both the steering-angle and per-sample steering-rate limits."""
    vehicle = config.vehicle
    max_steer = max(0.0, finite(vehicle.max_steer, 0.0))
    max_rate = max(0.0, finite(vehicle.max_steer_rate, 0.0))
    dt = max(0.0, finite(config.sim.dt, 0.0))
    previous = clamp(finite(previous, 0.0), -max_steer, max_steer)
    desired = clamp(finite(desired, 0.0), -max_steer, max_steer)
    change = max_rate * dt
    return clamp(desired, previous - change, previous + change)


def bounded_acceleration(desired, speed, config, speed_limit=None):
    """Respect acceleration limits without requesting reverse motion next step."""
    vehicle = config.vehicle
    max_accel = max(0.0, finite(vehicle.max_accel, 0.0))
    max_decel = max(0.0, finite(vehicle.max_decel, 0.0))
    min_speed, max_speed = vehicle_speed_bounds(vehicle)
    if speed_limit is None:
        cruise_limit = max_speed
    else:
        cruise_limit = min(max_speed, max(min_speed, finite(speed_limit, max_speed)))
    dt = finite(config.sim.dt, 0.0)
    speed = finite(speed, math.nan)
    if not math.isfinite(speed):
        return -max_decel
    if speed < min_speed:
        return 0.0
    lower, upper = -max_decel, max_accel
    if dt > 0.0:
        lower = max(lower, (min_speed - speed) / dt)
        upper = min(upper, (cruise_limit - speed) / dt)
    if lower > upper:
        return -max_decel
    return clamp(finite(desired, -max_decel), lower, upper)


def stop_command(state, previous_control, config):
    """Brake at the configured limit and slew steering toward zero."""
    return ControlCommand(
        acceleration=bounded_acceleration(
            -config.vehicle.max_decel, state.v, config
        ),
        steer=bounded_steer(
            0.0, getattr(previous_control, "steer", 0.0), config
        ),
    )


def matrix_solve(matrix, rhs):
    """Use a direct solve when well conditioned and an SVD fallback otherwise."""
    try:
        condition = np.linalg.cond(matrix)
        if np.isfinite(condition) and condition < 1.0e12:
            return np.linalg.solve(matrix, rhs)
    except np.linalg.LinAlgError:
        pass
    return np.linalg.pinv(matrix, rcond=1.0e-10) @ rhs


def bilinear_discrete(A_c, B_c, dt):
    """Tustin discretization of ``x' = A_c x + B_c u``."""
    A_c = np.asarray(A_c, dtype=float)
    B_c = np.asarray(B_c, dtype=float)
    identity = np.eye(A_c.shape[0])
    left = identity - 0.5 * dt * A_c
    A = matrix_solve(left, identity + 0.5 * dt * A_c)
    B = matrix_solve(left, dt * B_c)
    if not np.all(np.isfinite(A)) or not np.all(np.isfinite(B)):
        raise FloatingPointError("bilinear discretization is non-finite")
    return A, B


def bilinear_step(A, B, state, input_value, dt):
    """One Tustin step of ``x' = A x + B u`` for a scalar input."""
    identity = np.eye(A.shape[0])
    left = identity - 0.5 * dt * A
    right = (identity + 0.5 * dt * A) @ state + dt * (B * input_value)
    nxt = matrix_solve(left, right)
    if not np.all(np.isfinite(nxt)):
        raise FloatingPointError("bilinear step is non-finite")
    return nxt


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

    path_s = _monotonic_path_s(path, point_count)
    if path_s is not None:
        return max(0.0, float(path_s[point_count - 1] - path_s[index]))

    x = np.asarray(path.x[:point_count], dtype=float)
    y = np.asarray(path.y[:point_count], dtype=float)
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("reference path coordinates must be finite")
    if index >= point_count - 1:
        return 0.0
    return float(np.hypot(np.diff(x[index:]), np.diff(y[index:])).sum())


def _monotonic_path_s(path, point_count):
    try:
        path_s = np.asarray(getattr(path, "s", ()), dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if (
        path_s.size >= point_count
        and np.all(np.isfinite(path_s[:point_count]))
        and np.all(np.diff(path_s[:point_count]) >= 0.0)
    ):
        return path_s[:point_count]
    return None


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
    path_s = _monotonic_path_s(path, point_count)
    if path_s is None:
        x = np.asarray(path.x[:point_count], dtype=float)
        y = np.asarray(path.y[:point_count], dtype=float)
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise ValueError("path coordinates must be finite")
        path_s = np.concatenate(
            ([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y))))
        )
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
    min_speed, max_speed = vehicle_speed_bounds(vehicle)
    cap = min(max(0.0, finite(target_speed, 0.0)), max_speed)
    kappa = abs(finite(curvature, 0.0))
    if kappa <= 1.0e-9:
        return cap

    wheelbase = finite(vehicle.wheelbase, math.nan)
    max_steer = abs(finite(vehicle.max_steer, math.nan))
    max_rate = abs(finite(vehicle.max_steer_rate, math.nan))
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
    speed = max(0.0, finite(measured_speed, 0.0))
    preview_distance = max(10.0, speed * 1.5)
    try:
        kappa = preview_max_abs_curvature(reference, preview_distance)
    except ValueError:
        kappa = abs(finite(getattr(reference, "curvature", 0.0), 0.0))
    return tracking_speed_limit(target_speed, kappa, vehicle)


def longitudinal_command(state, reference, config, *, tracking_ok):
    """Speed PID plus actuator bounds; fail closed to a stop when tracking is off."""
    vehicle = config.vehicle
    measured = max(0.0, finite(state.v, 0.0))
    if tracking_ok:
        try:
            goal_distance = remaining_path_distance(
                reference.path, reference.nearest_index
            )
            target_speed = command_target_speed(
                finite(reference.target_speed, 0.0),
                reference,
                measured,
                vehicle,
            )
        except ValueError:
            goal_distance = 0.0
            target_speed = 0.0
    else:
        goal_distance = 0.0
        target_speed = 0.0
    acceleration = speed_pid(
        target_speed,
        measured,
        goal_distance,
        config.controller,
        vehicle,
    )
    acceleration = bounded_acceleration(
        acceleration, state.v, config, speed_limit=target_speed
    )
    return acceleration, target_speed


def lqr_error_state(reference, speed, previous_steer, config):
    """Four-state lateral error used by both kinematic and dynamic LQR."""
    e_y = finite(reference.lateral_error, math.nan)
    raw_heading_error = finite(reference.heading_error, math.nan)
    curvature = finite(reference.curvature, math.nan)
    if not all(
        math.isfinite(value)
        for value in (e_y, raw_heading_error, curvature)
    ):
        raise FloatingPointError("LQR reference is non-finite")
    e_yaw = pi_to_pi(raw_heading_error)
    wheelbase = validated_wheelbase(config.vehicle)
    max_steer = max(0.0, finite(config.vehicle.max_steer, 0.0))
    previous_for_model = clamp(
        finite(previous_steer, 0.0), -max_steer, max_steer
    )
    e_yaw_dot = speed / wheelbase * math.tan(previous_for_model) - speed * curvature
    return np.array(
        [[e_y], [speed * math.sin(e_yaw)], [e_yaw], [e_yaw_dot]]
    )


def lqr_control(
    state,
    reference,
    previous_control,
    config,
    *,
    build_model,
    solve,
    feedforward,
    preview_attr,
    default_preview,
):
    """Shared LQR path-tracking loop; only the plant/feedforward differ."""
    validate_safety_config(config)
    controller = config.controller
    vehicle = config.vehicle
    measured_speed = max(0.0, finite(state.v, 0.0))
    model_floor = max(1.0e-3, finite(controller.lqr_min_model_speed, 0.5))
    speed = max(measured_speed, model_floor)
    previous_steer = getattr(previous_control, "steer", 0.0)
    state_ok = state_in_speed_envelope(state, vehicle)

    try:
        if not state_ok:
            raise FloatingPointError("vehicle state is invalid")
        A, B = build_model(speed, config)
        K = solve(
            A,
            B,
            controller.lqr_q,
            controller.lqr_r,
            controller.lqr_eps,
            controller.lqr_max_iter,
        )
        closed_loop_radius = float(np.max(np.abs(np.linalg.eigvals(A - B @ K))))
        if not math.isfinite(closed_loop_radius) or closed_loop_radius >= 1.0:
            raise FloatingPointError("LQR closed loop is not stable")
        error_state = lqr_error_state(reference, speed, previous_steer, config)
        feedback = float(-(K @ error_state)[0, 0])
        preview_time = finite(
            getattr(controller, preview_attr, default_preview),
            default_preview,
        )
        kappa = preview_path_curvature(
            reference, max(0.0, measured_speed) * max(0.0, preview_time)
        )
        desired_steer = feedback + feedforward(speed, kappa, K, config)
        if not math.isfinite(desired_steer):
            raise FloatingPointError("steering command is non-finite")
        lateral_ok = True
    except (ValueError, FloatingPointError, np.linalg.LinAlgError, OverflowError):
        desired_steer = 0.0
        lateral_ok = False

    steer = bounded_steer(desired_steer, previous_steer, config)
    tracking_ok = bool(
        len(reference.path.x)
        and len(reference.path.y)
        and state_ok
        and lateral_ok
    )
    acceleration, _ = longitudinal_command(
        state, reference, config, tracking_ok=tracking_ok
    )
    return ControlCommand(acceleration=acceleration, steer=steer)
