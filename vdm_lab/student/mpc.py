import math
import warnings
from dataclasses import dataclass

import numpy as np

from vdm_lab.common.geometry import clamp
from vdm_lab.common.types import ControlCommand
from vdm_lab.student._controller_utils import validated_wheelbase


NAME = "Linear MPC Student"


class MPCSolverError(RuntimeError):
    """Raised when the QP cannot provide a finite, usable control sequence."""

    def __init__(
        self,
        message,
        *,
        status=None,
        iterations=0,
        solve_time_ms=0.0,
    ):
        super().__init__(message)
        self.status = status
        self.iterations = int(iterations or 0)
        self.solve_time_ms = float(solve_time_ms or 0.0)


@dataclass(frozen=True)
class MPCSolveInfo:
    status: str
    iterations: int
    solve_time_ms: float


@dataclass(frozen=True)
class MPCDiagnostics:
    """Controller-owned telemetry that survives shared command limiting."""

    control_calls: int = 0
    fallback_count: int = 0
    controller_status: str = "not_run"
    fallback_reason: str | None = None
    solver_status: str = "not_run"
    solver_iterations: int = 0
    solver_solve_time_ms: float = 0.0


_DIAGNOSTICS = MPCDiagnostics()


def reset_diagnostics():
    """Reset cumulative MPC telemetry before an experiment."""
    global _DIAGNOSTICS
    _DIAGNOSTICS = MPCDiagnostics()


def get_diagnostics():
    """Return the latest immutable diagnostics snapshot."""
    return _DIAGNOSTICS


def _record_diagnostics(
    command,
    *,
    fallback,
    reason=None,
    solver_status="not_available",
    solver_iterations=0,
    solver_solve_time_ms=0.0,
):
    global _DIAGNOSTICS
    fallback_count = _DIAGNOSTICS.fallback_count + int(bool(fallback))
    _DIAGNOSTICS = MPCDiagnostics(
        control_calls=_DIAGNOSTICS.control_calls + 1,
        fallback_count=fallback_count,
        controller_status="fallback" if fallback else "ok",
        fallback_reason=None if reason is None else str(reason),
        solver_status=str(solver_status or "not_available"),
        solver_iterations=int(solver_iterations or 0),
        solver_solve_time_ms=float(solver_solve_time_ms or 0.0),
    )

    # ControlCommand intentionally has only the actuator fields.  Dynamic
    # attributes preserve diagnostics for direct callers without changing the
    # shared dataclass used by other backends.
    command.controller_status = _DIAGNOSTICS.controller_status
    command.fallback_reason = _DIAGNOSTICS.fallback_reason
    command.fallback_count = _DIAGNOSTICS.fallback_count
    command.solver_status = _DIAGNOSTICS.solver_status
    command.solver_iterations = _DIAGNOSTICS.solver_iterations
    command.solver_solve_time_ms = _DIAGNOSTICS.solver_solve_time_ms

    if fallback and (
        fallback_count == 1
        or (fallback_count & (fallback_count - 1)) == 0
    ):
        warnings.warn(
            f"MPC fallback #{fallback_count}: {reason} "
            f"(solver_status={_DIAGNOSTICS.solver_status})",
            RuntimeWarning,
            stacklevel=2,
        )
    return command


class _MPCWorkspace:
    """Reusable parameterized CVXPY problem for one controller configuration."""

    def __init__(
        self,
        cp,
        horizon,
        q,
        qf,
        r,
        rd,
        min_speed,
        max_speed,
        max_accel,
        max_decel,
        max_steer,
        max_steer_change,
        state_scales,
        input_scales,
    ):
        self.z_ref = cp.Parameter((4, horizon + 1))
        self.z0 = cp.Parameter(4)
        self.applied_steer = cp.Parameter()
        self.A = [cp.Parameter((4, 4)) for _ in range(horizon)]
        self.B = [cp.Parameter((4, 2)) for _ in range(horizon)]
        self.C = [cp.Parameter(4) for _ in range(horizon)]
        self.z = cp.Variable((4, horizon + 1))
        self.u = cp.Variable((2, horizon))

        q_scaled = np.diag(state_scales) @ q @ np.diag(state_scales)
        qf_scaled = np.diag(state_scales) @ qf @ np.diag(state_scales)
        r_scaled = np.diag(input_scales) @ r @ np.diag(input_scales)
        rd_scaled = np.diag(input_scales) @ rd @ np.diag(input_scales)
        cost = 0.0
        constraints = [self.z[:, 0] == self.z0]
        for t in range(horizon):
            cost += cp.quad_form(
                self.z_ref[:, t] - self.z[:, t], q_scaled
            )
            cost += cp.quad_form(self.u[:, t], r_scaled)
            constraints.append(
                self.z[:, t + 1]
                == self.A[t] @ self.z[:, t]
                + self.B[t] @ self.u[:, t]
                + self.C[t]
            )
            if t == 0:
                constraints.append(
                    cp.abs(self.u[1, t] - self.applied_steer)
                    <= max_steer_change / input_scales[1]
                )
            if t < horizon - 1:
                cost += cp.quad_form(
                    self.u[:, t + 1] - self.u[:, t], rd_scaled
                )
                constraints.append(
                    cp.abs(self.u[1, t + 1] - self.u[1, t])
                    <= max_steer_change / input_scales[1]
                )
        cost += cp.quad_form(
            self.z_ref[:, horizon] - self.z[:, horizon], qf_scaled
        )
        constraints += [
            self.z[2, 1:] >= min_speed / state_scales[2],
            self.z[2, 1:] <= max_speed / state_scales[2],
            self.u[0, :] <= max_accel / input_scales[0],
            self.u[0, :] >= -max_decel / input_scales[0],
            cp.abs(self.u[1, :]) <= max_steer / input_scales[1],
        ]
        self.problem = cp.Problem(cp.Minimize(cost), constraints)


_WORKSPACE_CACHE = {}


def _workspace_key(
    horizon,
    q,
    qf,
    r,
    rd,
    bounds,
    state_scales,
    input_scales,
):
    return (
        horizon,
        *tuple(np.asarray(q).ravel()),
        *tuple(np.asarray(qf).ravel()),
        *tuple(np.asarray(r).ravel()),
        *tuple(np.asarray(rd).ravel()),
        *tuple(bounds),
        *tuple(state_scales),
        *tuple(input_scales),
    )


def _solver_info(problem):
    stats = getattr(problem, "solver_stats", None)
    status = str(getattr(problem, "status", None) or "unknown")
    iterations = int(getattr(stats, "num_iters", 0) or 0)
    solve_time = float(getattr(stats, "solve_time", 0.0) or 0.0)
    return MPCSolveInfo(
        status=status,
        iterations=iterations,
        solve_time_ms=1000.0 * solve_time,
    )


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


def _horizon(config):
    value = _finite(config.controller.mpc_horizon, 0.0)
    horizon = int(value)
    if horizon < 1 or value != horizon:
        raise ValueError("mpc_horizon must be a positive integer")
    return horizon


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _bounded_steer(desired, previous, config):
    vehicle = config.vehicle
    max_steer = max(0.0, _finite(vehicle.max_steer, 0.0))
    max_rate = max(0.0, _finite(vehicle.max_steer_rate, 0.0))
    dt = max(0.0, _finite(config.sim.dt, 0.0))
    previous = clamp(_finite(previous, 0.0), -max_steer, max_steer)
    desired = clamp(_finite(desired, 0.0), -max_steer, max_steer)
    max_change = max_rate * dt
    return clamp(desired, previous - max_change, previous + max_change)


def _bounded_acceleration(desired, speed, config):
    vehicle = config.vehicle
    max_accel = max(0.0, _finite(vehicle.max_accel, 0.0))
    max_decel = max(0.0, _finite(vehicle.max_decel, 0.0))
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    dt = _finite(config.sim.dt, 0.0)
    speed = _finite(speed, math.nan)
    if not math.isfinite(speed):
        return -max_decel
    if speed < min_speed:
        return 0.0

    lower, upper = -max_decel, max_accel
    if dt > 0.0:
        lower = max(lower, (min_speed - speed) / dt)
        upper = min(upper, (max_speed - speed) / dt)
    if lower > upper:
        return -max_decel
    return clamp(_finite(desired, -max_decel), lower, upper)


def nearest_horizon_reference(state, reference, config):
    """Build an unwrapped [x, y, speed, yaw] reference over the MPC horizon."""
    path = reference.path
    horizon = _horizon(config)
    point_count = min(
        len(path.x), len(path.y), len(path.yaw), len(path.target_speed)
    )
    if point_count < 1:
        raise ValueError("reference path is empty or inconsistent")

    try:
        base_index = int(reference.nearest_index)
    except (TypeError, ValueError, OverflowError):
        base_index = 0
    base_index = int(clamp(base_index, 0, point_count - 1))
    z_ref = np.zeros((4, horizon + 1))

    preview_target = max(
        0.0,
        _finite(reference.target_speed, path.target_speed[base_index]),
    )
    preview_speed = max(
        0.0,
        _finite(state.v, 0.0),
        0.5 * preview_target,
    )
    dt = _finite(config.sim.dt, 0.0)
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    waypoint_ds = _finite(config.sim.waypoint_ds, 0.0)
    if waypoint_ds <= 0.0:
        raise ValueError("waypoint_ds must be positive")

    path_s = np.asarray(path.s[:point_count], dtype=float)
    use_path_s = (
        path_s.shape == (point_count,)
        and np.all(np.isfinite(path_s))
        and np.all(np.diff(path_s) >= 0.0)
    )
    distance = 0.0
    previous_yaw = _finite(state.yaw, path.yaw[base_index])
    if not math.isfinite(previous_yaw):
        raise ValueError("state and reference yaw are non-finite")

    for i in range(horizon + 1):
        if i > 0:
            distance += preview_speed * dt
        if use_path_s:
            target_s = path_s[base_index] + distance
            index = int(np.searchsorted(path_s, target_s, side="left"))
            index = min(max(index, base_index), point_count - 1)
        else:
            offset = int(round(distance / waypoint_ds))
            index = min(base_index + max(offset, 0), point_count - 1)

        x_ref = _finite(path.x[index], math.nan)
        y_ref = _finite(path.y[index], math.nan)
        raw_yaw = _finite(path.yaw[index], math.nan)
        if not all(math.isfinite(value) for value in (x_ref, y_ref, raw_yaw)):
            raise ValueError("reference path contains non-finite values")
        yaw_ref = previous_yaw + _wrap_angle(raw_yaw - previous_yaw)
        target_speed = clamp(
            _finite(path.target_speed[index], 0.0),
            max(0.0, _finite(config.vehicle.min_speed, 0.0)),
            max(
                max(0.0, _finite(config.vehicle.min_speed, 0.0)),
                _finite(config.vehicle.max_speed, 0.0),
            ),
        )
        z_ref[:, i] = [x_ref, y_ref, target_speed, yaw_ref]
        previous_yaw = yaw_ref
    return z_ref


def update_kinematic_array(z, acceleration, steer, config):
    """One bounded prediction step matching the shared beta-based CG plant."""
    z = np.asarray(z, dtype=float).reshape(-1)
    if z.shape != (4,) or not np.all(np.isfinite(z)):
        raise ValueError("predicted state must contain four finite values")
    dt = _finite(config.sim.dt, 0.0)
    vehicle = config.vehicle
    wheelbase = validated_wheelbase(vehicle)
    if dt <= 0.0 or wheelbase <= 0.0:
        raise ValueError("dt and wheelbase must be positive")

    x, y, speed, yaw = z
    lr = _finite(vehicle.lr, 0.0)
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    max_steer = min(
        max(0.0, _finite(vehicle.max_steer, 0.0)),
        0.5 * math.pi - 1.0e-3,
    )
    steer = clamp(_finite(steer, 0.0), -max_steer, max_steer)
    acceleration = _bounded_acceleration(acceleration, speed, config)

    tangent = math.tan(steer)
    beta = math.atan2(lr * tangent, wheelbase)
    yaw_rate = speed / wheelbase * tangent * math.cos(beta)
    result = np.array(
        [
            x + speed * math.cos(yaw + beta) * dt,
            y + speed * math.sin(yaw + beta) * dt,
            clamp(speed + acceleration * dt, min_speed, max_speed),
            yaw + yaw_rate * dt,
        ]
    )
    if not np.all(np.isfinite(result)):
        raise FloatingPointError("nonlinear MPC prediction is non-finite")
    return result


def predict_motion(z0, acceleration, steer, z_ref, config):
    horizon = _horizon(config)
    acceleration = np.asarray(acceleration, dtype=float).reshape(-1)
    steer = np.asarray(steer, dtype=float).reshape(-1)
    if len(acceleration) != horizon or len(steer) != horizon:
        raise ValueError("control sequences do not match the MPC horizon")
    z_bar = np.zeros((4, horizon + 1))
    state = np.asarray(z0, dtype=float).reshape(-1)
    if state.shape != (4,) or not np.all(np.isfinite(state)):
        raise ValueError("initial MPC state must contain four finite values")
    z_bar[:, 0] = state
    for i in range(horizon):
        state = update_kinematic_array(
            state, acceleration[i], steer[i], config
        )
        z_bar[:, i + 1] = state
    return z_bar


def linear_model(v, yaw, steer, config):
    """Linearize the exact student MPC predictor about one nominal point."""
    dt = _finite(config.sim.dt, 0.0)
    vehicle = config.vehicle
    wheelbase = validated_wheelbase(vehicle)
    v = _finite(v, math.nan)
    yaw = _finite(yaw, math.nan)
    if dt <= 0.0 or wheelbase <= 0.0:
        raise ValueError("dt and wheelbase must be positive")
    if not math.isfinite(v) or not math.isfinite(yaw):
        raise ValueError("linearization state must be finite")

    max_steer = min(
        max(0.0, _finite(vehicle.max_steer, 0.0)),
        0.5 * math.pi - 1.0e-3,
    )
    lr = _finite(vehicle.lr, 0.0)
    steer = clamp(_finite(steer, 0.0), -max_steer, max_steer)
    tangent = math.tan(steer)
    secant_squared = 1.0 / max(math.cos(steer) ** 2, 1.0e-12)
    ratio = lr * tangent / wheelbase
    beta = math.atan(ratio)
    beta_derivative = (
        lr * secant_squared / wheelbase / (1.0 + ratio * ratio)
    )
    direction = yaw + beta
    yaw_curvature = tangent * math.cos(beta) / wheelbase
    yaw_curvature_derivative = (
        secant_squared * math.cos(beta)
        - tangent * math.sin(beta) * beta_derivative
    ) / wheelbase

    A = np.eye(4)
    A[0, 2] = dt * math.cos(direction)
    A[0, 3] = -dt * v * math.sin(direction)
    A[1, 2] = dt * math.sin(direction)
    A[1, 3] = dt * v * math.cos(direction)
    A[3, 2] = dt * yaw_curvature

    B = np.zeros((4, 2))
    B[0, 1] = -dt * v * math.sin(direction) * beta_derivative
    B[1, 1] = dt * v * math.cos(direction) * beta_derivative
    B[2, 0] = dt
    B[3, 1] = dt * v * yaw_curvature_derivative

    nominal_state = np.array([0.0, 0.0, v, yaw])
    nominal_input = np.array([0.0, steer])
    nominal_next = np.array(
        [
            dt * v * math.cos(direction),
            dt * v * math.sin(direction),
            v,
            yaw + dt * v * yaw_curvature,
        ]
    )
    C = nominal_next - A @ nominal_state - B @ nominal_input
    if not all(np.all(np.isfinite(item)) for item in (A, B, C)):
        raise FloatingPointError("linearized MPC model is non-finite")
    return A, B, C


def _weight_matrix(value, size, name):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (size, size) or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite {size}x{size} matrix")
    matrix = 0.5 * (matrix + matrix.T)
    if float(np.min(np.linalg.eigvalsh(matrix))) < -1.0e-9:
        raise ValueError(f"{name} must be positive semidefinite")
    return matrix


def _nominal_steer_sequence(previous_steer, horizon):
    sequence = np.asarray(previous_steer, dtype=float).reshape(-1)
    if sequence.size == 1:
        sequence = np.full(horizon, sequence.item())
    if sequence.shape != (horizon,) or not np.all(np.isfinite(sequence)):
        raise ValueError("nominal steering sequence is invalid")
    return sequence


def _solve_linear_mpc(
    z_ref,
    z_bar,
    z0,
    nominal_steer,
    applied_steer,
    config,
):
    try:
        import cvxpy as cp
    except ImportError as exc:
        raise ImportError(
            "MPC requires CVXPY: pip install -r requirements.txt"
        ) from exc

    horizon = _horizon(config)
    vehicle = config.vehicle
    controller = config.controller
    z_ref = np.asarray(z_ref, dtype=float)
    z_bar = np.asarray(z_bar, dtype=float)
    z0 = np.asarray(z0, dtype=float).reshape(-1)
    nominal_steer = _nominal_steer_sequence(nominal_steer, horizon)
    if (
        z_ref.shape != (4, horizon + 1)
        or z_bar.shape != (4, horizon + 1)
        or z0.shape != (4,)
        or not np.all(np.isfinite(z_ref))
        or not np.all(np.isfinite(z_bar))
        or not np.all(np.isfinite(z0))
    ):
        raise ValueError("MPC states and references have invalid shapes or values")

    # The physical problem is translation invariant.  Solve in a frame whose
    # origin is the current vehicle position so large map coordinates cannot
    # dominate OSQP's residuals.
    position_origin = z0[:2].copy()
    z_ref = z_ref.copy()
    z_bar = z_bar.copy()
    z0 = z0.copy()
    z_ref[:2, :] -= position_origin[:, None]
    z_bar[:2, :] -= position_origin[:, None]
    z0[:2] -= position_origin

    q = _weight_matrix(controller.mpc_q, 4, "mpc_q")
    qf = _weight_matrix(controller.mpc_qf, 4, "mpc_qf")
    r = _weight_matrix(controller.mpc_r, 2, "mpc_r")
    rd = _weight_matrix(controller.mpc_rd, 2, "mpc_rd")
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    max_accel = max(0.0, _finite(vehicle.max_accel, 0.0))
    max_decel = max(0.0, _finite(vehicle.max_decel, 0.0))
    max_steer = max(0.0, _finite(vehicle.max_steer, 0.0))
    max_steer_rate = max(0.0, _finite(vehicle.max_steer_rate, 0.0))
    dt = _finite(config.sim.dt, 0.0)
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    applied_steer = clamp(
        _finite(applied_steer, 0.0), -max_steer, max_steer
    )
    max_steer_change = max_steer_rate * dt

    # Use dimensionless optimization variables.  Transforming both the model
    # and weights preserves the original physical cost exactly.
    state_scales = np.array(
        [
            max(1.0, max_speed * dt * horizon),
            max(1.0, max_speed * dt * horizon),
            max(1.0, max_speed),
            math.pi,
        ]
    )
    input_scales = np.array(
        [max(1.0, max_accel, max_decel), max(1.0e-3, max_steer)]
    )
    z_ref_scaled = z_ref / state_scales[:, None]
    z0_scaled = z0 / state_scales

    A_values = []
    B_values = []
    C_values = []
    for t in range(horizon):
        A, B, C = linear_model(
            z_bar[2, t], z_bar[3, t], nominal_steer[t], config
        )
        A_values.append(
            (A * state_scales[np.newaxis, :])
            / state_scales[:, np.newaxis]
        )
        B_values.append(
            (B * input_scales[np.newaxis, :])
            / state_scales[:, np.newaxis]
        )
        C_values.append(C / state_scales)

    bounds = (
        min_speed,
        max_speed,
        max_accel,
        max_decel,
        max_steer,
        max_steer_change,
    )
    key = _workspace_key(
        horizon,
        q,
        qf,
        r,
        rd,
        bounds,
        state_scales,
        input_scales,
    )
    workspace = _WORKSPACE_CACHE.get(key)
    if workspace is None:
        workspace = _MPCWorkspace(
            cp,
            horizon,
            q,
            qf,
            r,
            rd,
            min_speed,
            max_speed,
            max_accel,
            max_decel,
            max_steer,
            max_steer_change,
            state_scales,
            input_scales,
        )
        if len(_WORKSPACE_CACHE) >= 4:
            _WORKSPACE_CACHE.pop(next(iter(_WORKSPACE_CACHE)))
        _WORKSPACE_CACHE[key] = workspace

    workspace.z_ref.value = z_ref_scaled
    workspace.z0.value = z0_scaled
    workspace.applied_steer.value = applied_steer / input_scales[1]
    for t in range(horizon):
        workspace.A[t].value = A_values[t]
        workspace.B[t].value = B_values[t]
        workspace.C[t].value = C_values[t]

    problem = workspace.problem
    try:
        problem.solve(
            solver=cp.OSQP,
            warm_start=True,
            verbose=False,
            eps_abs=1.0e-4,
            eps_rel=1.0e-4,
            max_iter=10000,
            polishing=True,
        )
    except cp.error.SolverError as exc:
        info = _solver_info(problem)
        raise MPCSolverError(
            f"OSQP failed: {exc}",
            status=info.status,
            iterations=info.iterations,
            solve_time_ms=info.solve_time_ms,
        ) from exc

    info = _solver_info(problem)
    if problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        raise MPCSolverError(
            f"MPC solve status is {problem.status}",
            status=info.status,
            iterations=info.iterations,
            solve_time_ms=info.solve_time_ms,
        )
    if workspace.u.value is None or workspace.z.value is None:
        raise MPCSolverError(
            "MPC solver returned no solution",
            status=info.status,
            iterations=info.iterations,
            solve_time_ms=info.solve_time_ms,
        )
    controls = input_scales[:, None] * np.asarray(
        workspace.u.value, dtype=float
    )
    acceleration = controls[0, :]
    steer = controls[1, :]
    prediction = state_scales[:, None] * np.asarray(
        workspace.z.value, dtype=float
    )
    prediction[:2, :] += position_origin[:, None]
    if not all(
        np.all(np.isfinite(item))
        for item in (acceleration, steer, prediction)
    ):
        raise MPCSolverError(
            "MPC solver returned non-finite values",
            status=info.status,
            iterations=info.iterations,
            solve_time_ms=info.solve_time_ms,
        )
    return acceleration, steer, prediction, info


def solve_linear_mpc(z_ref, z_bar, z0, previous_steer, config):
    """Public single-QP API retained for the assignment and unit checks."""
    _validate_safety_config(config)
    horizon = _horizon(config)
    nominal_steer = _nominal_steer_sequence(previous_steer, horizon)
    acceleration, steer, _, _ = _solve_linear_mpc(
        z_ref,
        z_bar,
        z0,
        nominal_steer,
        nominal_steer[0],
        config,
    )
    acceleration, steer = _sanitize_controls(
        acceleration, steer, z0[2], nominal_steer[0], config
    )
    prediction = predict_motion(
        z0, acceleration, steer, z_ref, config
    )
    return acceleration, steer, prediction


def _sanitize_controls(acceleration, steer, initial_speed, applied_steer, config):
    horizon = _horizon(config)
    acceleration = np.asarray(acceleration, dtype=float).reshape(-1)
    steer = np.asarray(steer, dtype=float).reshape(-1)
    if acceleration.shape != (horizon,) or steer.shape != (horizon,):
        raise MPCSolverError("MPC control sequence has the wrong shape")

    safe_acceleration = np.zeros(horizon)
    safe_steer = np.zeros(horizon)
    speed = _finite(initial_speed, math.nan)
    if not math.isfinite(speed):
        raise MPCSolverError("initial speed is non-finite")
    previous = _finite(applied_steer, 0.0)
    dt = _finite(config.sim.dt, 0.0)
    for i in range(horizon):
        safe_acceleration[i] = _bounded_acceleration(
            acceleration[i], speed, config
        )
        safe_steer[i] = _bounded_steer(steer[i], previous, config)
        if dt > 0.0:
            speed += safe_acceleration[i] * dt
        speed = clamp(
            speed,
            max(0.0, _finite(config.vehicle.min_speed, 0.0)),
            max(
                max(0.0, _finite(config.vehicle.min_speed, 0.0)),
                _finite(config.vehicle.max_speed, 0.0),
            ),
        )
        previous = safe_steer[i]
    return safe_acceleration, safe_steer


def _iterative_mpc_with_info(z_ref, z0, previous_control, config):
    _validate_safety_config(config)
    horizon = _horizon(config)
    applied_steer = _finite(getattr(previous_control, "steer", 0.0), 0.0)
    previous_acceleration = _finite(
        getattr(previous_control, "acceleration", 0.0), 0.0
    )
    acceleration = np.full(horizon, previous_acceleration)
    steer = np.full(horizon, applied_steer)
    acceleration, steer = _sanitize_controls(
        acceleration, steer, z0[2], applied_steer, config
    )
    prediction = predict_motion(z0, acceleration, steer, z_ref, config)

    iteration_limit = max(
        1, int(max(1.0, _finite(config.controller.mpc_iter_max, 1.0)))
    )
    threshold = max(
        0.0, _finite(config.controller.mpc_du_threshold, 0.0)
    )
    total_solver_iterations = 0
    total_solve_time_ms = 0.0
    final_status = "not_run"
    for _ in range(iteration_limit):
        z_bar = predict_motion(z0, acceleration, steer, z_ref, config)
        old_acceleration = acceleration.copy()
        old_steer = steer.copy()
        try:
            acceleration, steer, _, solve_info = _solve_linear_mpc(
                z_ref,
                z_bar,
                z0,
                steer,
                applied_steer,
                config,
            )
        except MPCSolverError as exc:
            exc.iterations += total_solver_iterations
            exc.solve_time_ms += total_solve_time_ms
            raise
        total_solver_iterations += solve_info.iterations
        total_solve_time_ms += solve_info.solve_time_ms
        final_status = solve_info.status
        acceleration, steer = _sanitize_controls(
            acceleration, steer, z0[2], applied_steer, config
        )
        prediction = predict_motion(z0, acceleration, steer, z_ref, config)
        change = max(
            float(np.max(np.abs(acceleration - old_acceleration))),
            float(np.max(np.abs(steer - old_steer))),
        )
        if change <= threshold:
            break
    return (
        acceleration,
        steer,
        prediction,
        MPCSolveInfo(
            status=final_status,
            iterations=total_solver_iterations,
            solve_time_ms=total_solve_time_ms,
        ),
    )


def iterative_mpc(z_ref, z0, previous_control, config):
    """Public iterative-MPC API retained with its original three results."""
    acceleration, steer, prediction, _ = _iterative_mpc_with_info(
        z_ref, z0, previous_control, config
    )
    return acceleration, steer, prediction


def _safe_stop_command(
    state,
    previous_control,
    config,
    *,
    reason="controller fallback",
    solver_status=None,
    solver_iterations=0,
    solver_solve_time_ms=0.0,
):
    try:
        horizon = _horizon(config)
    except ValueError:
        horizon = 1
    speed = _finite(state.v, math.nan)
    max_decel = max(0.0, _finite(config.vehicle.max_decel, 0.0))
    acceleration = _bounded_acceleration(-max_decel, speed, config)
    steer = _bounded_steer(
        0.0, getattr(previous_control, "steer", 0.0), config
    )
    command = ControlCommand(acceleration=acceleration, steer=steer)

    safe_state = np.array(
        [
            _finite(state.x, 0.0),
            _finite(state.y, 0.0),
            clamp(
                _finite(state.v, 0.0),
                max(0.0, _finite(config.vehicle.min_speed, 0.0)),
                max(
                    max(0.0, _finite(config.vehicle.min_speed, 0.0)),
                    _finite(config.vehicle.max_speed, 0.0),
                ),
            ),
            _finite(state.yaw, 0.0),
        ]
    )
    acceleration_sequence = np.full(horizon, acceleration)
    steer_sequence = np.full(horizon, steer)
    try:
        acceleration_sequence, steer_sequence = _sanitize_controls(
            acceleration_sequence,
            steer_sequence,
            safe_state[2],
            getattr(previous_control, "steer", 0.0),
            config,
        )
        command.prediction = predict_motion(
            safe_state,
            acceleration_sequence,
            steer_sequence,
            np.tile(safe_state.reshape(4, 1), (1, horizon + 1)),
            config,
        )
    except (ValueError, FloatingPointError, MPCSolverError, OverflowError):
        command.prediction = np.tile(
            safe_state.reshape(4, 1), (1, horizon + 1)
        )
    return _record_diagnostics(
        command,
        fallback=True,
        reason=reason,
        solver_status=solver_status,
        solver_iterations=solver_iterations,
        solver_solve_time_ms=solver_solve_time_ms,
    )


def control(state, reference, previous_control, config):
    _validate_safety_config(config)
    z0 = np.array(
        [
            _finite(state.x, math.nan),
            _finite(state.y, math.nan),
            _finite(state.v, math.nan),
            _finite(state.yaw, math.nan),
        ]
    )
    min_speed = max(0.0, _finite(config.vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(config.vehicle.max_speed, min_speed))
    if (
        not np.all(np.isfinite(z0))
        or z0[2] < min_speed - 1.0e-8
        or z0[2] > max_speed + 1.0e-8
    ):
        return _safe_stop_command(
            state,
            previous_control,
            config,
            reason="invalid vehicle state",
            solver_status="not_run",
        )

    try:
        z_ref = nearest_horizon_reference(state, reference, config)
        acceleration, steer, prediction, solve_info = _iterative_mpc_with_info(
            z_ref, z0, previous_control, config
        )
    except ImportError:
        raise
    except (
        MPCSolverError,
        ValueError,
        FloatingPointError,
        np.linalg.LinAlgError,
        OverflowError,
    ) as exc:
        return _safe_stop_command(
            state,
            previous_control,
            config,
            reason=f"{type(exc).__name__}: {exc}",
            solver_status=getattr(exc, "status", None) or "not_available",
            solver_iterations=getattr(exc, "iterations", 0),
            solver_solve_time_ms=getattr(exc, "solve_time_ms", 0.0),
        )

    command = ControlCommand(
        acceleration=float(acceleration[0]),
        steer=float(steer[0]),
    )
    command.prediction = prediction
    return _record_diagnostics(
        command,
        fallback=False,
        solver_status=solve_info.status,
        solver_iterations=solve_info.iterations,
        solver_solve_time_ms=solve_info.solve_time_ms,
    )
