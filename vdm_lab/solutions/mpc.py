import math

import numpy as np

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand


NAME = "Linear MPC"


def nearest_horizon_reference(state, reference, config):
    path = reference.path
    horizon = config.controller.mpc_horizon
    z_ref = np.zeros((4, horizon + 1))
    base_index = reference.nearest_index
    distance = 0.0
    preview_speed = max(reference.target_speed, 1.0)
    for i in range(horizon + 1):
        if i > 0:
            distance += max(state.v, preview_speed * 0.5) * config.sim.dt
        offset = int(round(distance / config.sim.waypoint_ds))
        index = min(base_index + offset, len(path.x) - 1)
        z_ref[:, i] = [path.x[index], path.y[index], path.target_speed[index], path.yaw[index]]
    return z_ref


def update_kinematic_array(z, acceleration, steer, config):
    dt = config.sim.dt
    vehicle = config.vehicle
    x, y, v, yaw = z
    steer = clamp(steer, -vehicle.max_steer, vehicle.max_steer)
    next_x = x + v * math.cos(yaw) * dt
    next_y = y + v * math.sin(yaw) * dt
    next_v = clamp(v + acceleration * dt, vehicle.min_speed, vehicle.max_speed)
    next_yaw = pi_to_pi(yaw + v / vehicle.wheelbase * math.tan(steer) * dt)
    return np.array([next_x, next_y, next_v, next_yaw])


def predict_motion(z0, acceleration, steer, z_ref, config):
    z_bar = np.zeros_like(z_ref)
    z_bar[:, 0] = z0
    state = np.array(z0, dtype=float)
    for i in range(config.controller.mpc_horizon):
        state = update_kinematic_array(state, acceleration[i], steer[i], config)
        z_bar[:, i + 1] = state
    return z_bar


def linear_model(v, yaw, steer, config):
    dt = config.sim.dt
    wheelbase = config.vehicle.wheelbase
    A = np.array(
        [
            [1.0, 0.0, dt * math.cos(yaw), -dt * v * math.sin(yaw)],
            [0.0, 1.0, dt * math.sin(yaw), dt * v * math.cos(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, dt * math.tan(steer) / wheelbase, 1.0],
        ]
    )
    B = np.array(
        [
            [0.0, 0.0],
            [0.0, 0.0],
            [dt, 0.0],
            [0.0, dt * v / (wheelbase * math.cos(steer) ** 2)],
        ]
    )
    C = np.array(
        [
            dt * v * math.sin(yaw) * yaw,
            -dt * v * math.cos(yaw) * yaw,
            0.0,
            -dt * v * steer / (wheelbase * math.cos(steer) ** 2),
        ]
    )
    return A, B, C


def solve_linear_mpc(z_ref, z_bar, z0, previous_steer, config):
    try:
        import cvxpy as cp
    except ImportError as exc:
        raise ImportError("MPC 需要安装 cvxpy：pip install -r requirements.txt") from exc

    horizon = config.controller.mpc_horizon
    vehicle = config.vehicle
    controller = config.controller
    z = cp.Variable((4, horizon + 1))
    u = cp.Variable((2, horizon))
    cost = 0.0
    constraints = [z[:, 0] == z0]

    for t in range(horizon):
        cost += cp.quad_form(z_ref[:, t] - z[:, t], controller.mpc_q)
        cost += cp.quad_form(u[:, t], controller.mpc_r)
        A, B, C = linear_model(z_bar[2, t], z_bar[3, t], previous_steer[t], config)
        constraints.append(z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C)
        if t < horizon - 1:
            cost += cp.quad_form(u[:, t + 1] - u[:, t], controller.mpc_rd)
            constraints.append(cp.abs(u[1, t + 1] - u[1, t]) <= vehicle.max_steer_rate * config.sim.dt)

    cost += cp.quad_form(z_ref[:, horizon] - z[:, horizon], controller.mpc_qf)
    constraints += [
        z[2, :] >= vehicle.min_speed,
        z[2, :] <= vehicle.max_speed,
        cp.abs(u[0, :]) <= vehicle.max_accel,
        u[0, :] >= -vehicle.max_decel,
        cp.abs(u[1, :]) <= vehicle.max_steer,
    ]

    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)

    if problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
        raise RuntimeError(f"MPC 求解失败，状态为 {problem.status}")
    return u.value[0, :], u.value[1, :], z.value


def iterative_mpc(z_ref, z0, previous_control, config):
    horizon = config.controller.mpc_horizon
    acceleration = np.full(horizon, previous_control.acceleration)
    steer = np.full(horizon, previous_control.steer)
    prediction = None

    for _ in range(config.controller.mpc_iter_max):
        z_bar = predict_motion(z0, acceleration, steer, z_ref, config)
        old_acceleration = acceleration.copy()
        old_steer = steer.copy()
        acceleration, steer, prediction = solve_linear_mpc(z_ref, z_bar, z0, steer, config)
        if max(np.max(np.abs(acceleration - old_acceleration)), np.max(np.abs(steer - old_steer))) < config.controller.mpc_du_threshold:
            break
    return acceleration, steer, prediction


def control(state, reference, previous_control, config):
    z_ref = nearest_horizon_reference(state, reference, config)
    z0 = np.array([state.x, state.y, state.v, state.yaw])
    acceleration, steer, prediction = iterative_mpc(z_ref, z0, previous_control, config)
    command = ControlCommand(acceleration=float(acceleration[0]), steer=float(steer[0]))
    command.prediction = prediction
    return command
