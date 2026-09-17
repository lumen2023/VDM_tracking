import math

import numpy as np

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid
from vdm_lab.student._controller_utils import (
    command_target_speed,
    preview_path_curvature,
    remaining_path_distance,
    validated_wheelbase,
)


NAME = "LQR Kinematic Student"

# The default is deliberately module-local so the shared ControllerConfig and
# any physical-backend configuration remain untouched.  Tests may override it
# on a ControllerConfig instance with ``lqr_kinematic_curvature_preview_time``.
DEFAULT_CURVATURE_PREVIEW_TIME = 0.25


def _finite(value, default):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def _validate_safety_config(config):
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
        raise ValueError("invalid safety configuration: min_speed cannot be negative")
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


def _matrix_solve(matrix, rhs):
    """Use a direct solve when well conditioned and an SVD fallback otherwise."""
    try:
        condition = np.linalg.cond(matrix)
        if np.isfinite(condition) and condition < 1.0e12:
            return np.linalg.solve(matrix, rhs)
    except np.linalg.LinAlgError:
        pass
    return np.linalg.pinv(matrix, rcond=1.0e-10) @ rhs


def solve_lqr(A, B, Q, R, eps, max_iter):
    """Solve the discrete algebraic Riccati equation by fixed-point iteration."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be square")
    n = A.shape[0]
    if B.ndim != 2 or B.shape[0] != n:
        raise ValueError("B has an incompatible shape")
    m = B.shape[1]
    if Q.shape != (n, n) or R.shape != (m, m):
        raise ValueError("Q or R has an incompatible shape")
    if not all(np.all(np.isfinite(item)) for item in (A, B, Q, R)):
        raise FloatingPointError("LQR matrices must be finite")

    tolerance = max(0.0, _finite(eps, 0.0))
    iterations = max(1, int(_finite(max_iter, 1)))
    P = 0.5 * (Q + Q.T)
    for _ in range(iterations):
        gain = _matrix_solve(R + B.T @ P @ B, B.T @ P @ A)
        P_next = A.T @ P @ A - A.T @ P @ B @ gain + Q
        P_next = 0.5 * (P_next + P_next.T)
        if not np.all(np.isfinite(P_next)):
            raise FloatingPointError("Riccati iteration produced a non-finite matrix")
        difference = float(np.max(np.abs(P_next - P)))
        P = P_next
        if difference <= tolerance:
            break

    K = _matrix_solve(R + B.T @ P @ B, B.T @ P @ A)
    if not np.all(np.isfinite(K)):
        raise FloatingPointError("LQR gain is non-finite")
    return K


def build_kinematic_model(speed, config):
    dt = _finite(config.sim.dt, 0.0)
    wheelbase = validated_wheelbase(config.vehicle)
    speed = max(0.0, _finite(speed, 0.0))
    if dt <= 0.0 or wheelbase <= 0.0:
        raise ValueError("dt and wheelbase must be positive")

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = speed
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = speed / wheelbase
    return A, B


def _bounded_steer(desired, previous, config):
    vehicle = config.vehicle
    max_steer = max(0.0, _finite(vehicle.max_steer, 0.0))
    max_rate = max(0.0, _finite(vehicle.max_steer_rate, 0.0))
    dt = max(0.0, _finite(config.sim.dt, 0.0))
    previous = clamp(_finite(previous, 0.0), -max_steer, max_steer)
    desired = clamp(_finite(desired, 0.0), -max_steer, max_steer)
    change = max_rate * dt
    return clamp(desired, previous - change, previous + change)


def _bounded_acceleration(desired, speed, config, speed_limit=None):
    vehicle = config.vehicle
    max_accel = max(0.0, _finite(vehicle.max_accel, 0.0))
    max_decel = max(0.0, _finite(vehicle.max_decel, 0.0))
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    if speed_limit is None:
        cruise_limit = max_speed
    else:
        cruise_limit = min(max_speed, max(min_speed, _finite(speed_limit, max_speed)))
    dt = _finite(config.sim.dt, 0.0)
    speed = _finite(speed, math.nan)
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
    return clamp(_finite(desired, -max_decel), lower, upper)


def _preview_curvature(reference, speed, config):
    preview_time = _finite(
        getattr(
            config.controller,
            "lqr_kinematic_curvature_preview_time",
            DEFAULT_CURVATURE_PREVIEW_TIME,
        ),
        DEFAULT_CURVATURE_PREVIEW_TIME,
    )
    preview_distance = max(0.0, speed) * max(0.0, preview_time)
    return preview_path_curvature(reference, preview_distance)


def control(state, reference, previous_control, config):
    _validate_safety_config(config)
    controller = config.controller
    vehicle = config.vehicle
    measured_speed = max(0.0, _finite(state.v, 0.0))
    model_floor = max(1.0e-3, _finite(controller.lqr_min_model_speed, 0.5))
    speed = max(measured_speed, model_floor)
    previous_steer = getattr(previous_control, "steer", 0.0)

    desired_steer = 0.0
    state_is_finite = all(
        math.isfinite(_finite(value, math.nan))
        for value in (state.x, state.y, state.yaw, state.v)
    )
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    state_speed = _finite(state.v, math.nan)
    state_is_valid = (
        state_is_finite
        and state_speed >= min_speed - 1.0e-8
        and state_speed <= max_speed + 1.0e-8
    )
    try:
        if not state_is_valid:
            raise FloatingPointError("vehicle state is invalid")
        A, B = build_kinematic_model(speed, config)
        K = solve_lqr(
            A,
            B,
            controller.lqr_q,
            controller.lqr_r,
            controller.lqr_eps,
            controller.lqr_max_iter,
        )
        closed_loop_radius = float(
            np.max(np.abs(np.linalg.eigvals(A - B @ K)))
        )
        if not math.isfinite(closed_loop_radius) or closed_loop_radius >= 1.0:
            raise FloatingPointError("LQR closed loop is not stable")

        e_y = _finite(reference.lateral_error, math.nan)
        raw_heading_error = _finite(reference.heading_error, math.nan)
        curvature = _finite(reference.curvature, math.nan)
        if not all(
            math.isfinite(value)
            for value in (e_y, raw_heading_error, curvature)
        ):
            raise FloatingPointError("LQR reference is non-finite")
        e_yaw = pi_to_pi(raw_heading_error)
        e_y_dot = speed * math.sin(e_yaw)
        wheelbase = validated_wheelbase(vehicle)
        max_steer = max(0.0, _finite(vehicle.max_steer, 0.0))
        previous_for_model = clamp(
            _finite(previous_steer, 0.0), -max_steer, max_steer
        )
        e_yaw_dot = (
            speed / wheelbase * math.tan(previous_for_model)
            - speed * curvature
        )
        error_state = np.array([[e_y], [e_y_dot], [e_yaw], [e_yaw_dot]])

        feedback = float(-(K @ error_state)[0, 0])
        feedforward_curvature = _preview_curvature(
            reference, measured_speed, config
        )
        feedforward = math.atan(wheelbase * feedforward_curvature)
        desired_steer = feedback + feedforward
        if not math.isfinite(desired_steer):
            raise FloatingPointError("steering command is non-finite")
        lateral_control_ok = True
    except (ValueError, FloatingPointError, np.linalg.LinAlgError, OverflowError):
        desired_steer = 0.0
        lateral_control_ok = False

    steer = _bounded_steer(desired_steer, previous_steer, config)
    if (
        len(reference.path.x)
        and len(reference.path.y)
        and state_is_valid
        and lateral_control_ok
    ):
        try:
            goal_distance = remaining_path_distance(
                reference.path, reference.nearest_index
            )
            target_speed = command_target_speed(
                _finite(reference.target_speed, 0.0),
                reference,
                measured_speed,
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
        measured_speed,
        goal_distance,
        controller,
        vehicle,
    )
    acceleration = _bounded_acceleration(
        acceleration, state.v, config, speed_limit=target_speed
    )
    return ControlCommand(acceleration=acceleration, steer=steer)
