"""从用户 MPC/src/mpc_tracking.py 适配的含侧偏角线性时变 MPC。

将原定速模型扩展为状态 [x, y, v, yaw]、输入 [acceleration, steer]。
车辆参数、采样周期、预测时域和权重统一读取课程配置。
"""

import math

import numpy as np

from vdm_lab.common.types import ControlCommand


NAME = "Linear MPC Student"


def nearest_horizon_reference(state, reference, config):
    # 1. 沿路径里程插值 T+1 个参考点，保留原程序的连续航向处理。
    path = reference.path
    horizon = config.controller.mpc_horizon
    speed = max(0.0, state.v, reference.target_speed)
    query_s = np.minimum(
        path.s[reference.nearest_index] + np.arange(horizon + 1) * speed * config.sim.dt,
        path.s[-1],
    )
    z_ref = np.vstack([
        np.interp(query_s, path.s, path.x),
        np.interp(query_s, path.s, path.y),
        np.interp(query_s, path.s, path.target_speed),
        np.interp(query_s, path.s, np.unwrap(path.yaw)),
    ])
    z_ref[3] += 2.0 * math.pi * round((state.yaw - z_ref[3, 0]) / (2.0 * math.pi))
    # 依据剩余里程限制参考速度，使原来的定速控制支持终点停车。
    remaining = np.maximum(0.0, path.s[-1] - query_s)
    z_ref[2] = np.minimum(z_ref[2], np.sqrt(2.0 * config.vehicle.max_decel * remaining))
    z_ref[2] = np.clip(z_ref[2], 0.0, config.vehicle.max_speed)
    return z_ref


def _model_step(z, acceleration, steer, config):
    """质心自行车模型的 Euler 预测；迭代期间保持 yaw 连续。"""
    x, y, v, yaw = z
    dt = config.sim.dt
    vehicle = config.vehicle
    wheelbase = vehicle.lf + vehicle.lr
    beta = math.atan(vehicle.lr / wheelbase * math.tan(steer))
    return np.array([
        x + dt * v * math.cos(yaw + beta),
        y + dt * v * math.sin(yaw + beta),
        v + dt * acceleration,
        yaw + dt * v / wheelbase * math.cos(beta) * math.tan(steer),
    ])


def linear_model(v, yaw, steer, config):
    # 2. 原解析 Jacobian 增加速度状态和加速度输入，z_next = A z + B u + C。
    dt = config.sim.dt
    wheelbase = config.vehicle.lf + config.vehicle.lr
    k = config.vehicle.lr / wheelbase
    tan_delta = math.tan(steer)
    sec2_delta = 1.0 / math.cos(steer) ** 2
    beta = math.atan(k * tan_delta)
    dbeta = k * sec2_delta / (1.0 + (k * tan_delta) ** 2)
    heading = yaw + beta

    a = np.eye(4)
    a[0, 2] = dt * math.cos(heading)
    a[1, 2] = dt * math.sin(heading)
    a[0, 3] = -dt * v * math.sin(heading)
    a[1, 3] = dt * v * math.cos(heading)
    a[3, 2] = dt / wheelbase * math.cos(beta) * tan_delta

    b = np.zeros((4, 2))
    b[2, 0] = dt
    b[0, 1] = -dt * v * math.sin(heading) * dbeta
    b[1, 1] = dt * v * math.cos(heading) * dbeta
    dg = -math.sin(beta) * dbeta * tan_delta + math.cos(beta) * sec2_delta
    b[3, 1] = dt * v / wheelbase * dg
    nominal = np.array([0.0, 0.0, v, yaw])
    c = _model_step(nominal, 0.0, steer, config) - a @ nominal - b[:, 1] * steer
    return a, b, c


def solve_linear_mpc(z_ref, z_bar, z0, previous_steer, config, steer_bar=None):
    try:
        import cvxpy as cp
    except ImportError as exc:
        raise ImportError("MPC 需要安装 cvxpy：pip install -r requirements.txt") from exc

    controller = config.controller
    vehicle = config.vehicle
    horizon = controller.mpc_horizon
    if steer_bar is None:
        steer_bar = np.full(horizon, previous_steer)

    # 保留原程序的局部坐标变换，改善大坐标路线的数值条件。
    origin = np.array([z0[0], z0[1], 0.0, 0.0])
    local_ref = z_ref - origin[:, None]
    z = cp.Variable((4, horizon + 1))
    u = cp.Variable((2, horizon))
    constraints = [z[:, 0] == z0 - origin]
    cost = 0
    max_change = vehicle.max_steer_rate * config.sim.dt
    for t in range(horizon):
        # 3. 跟踪误差、控制代价、控制变化量和终端误差。
        cost += cp.quad_form(z[:, t] - local_ref[:, t], controller.mpc_q)
        cost += cp.quad_form(u[:, t], controller.mpc_r)
        a, b, c = linear_model(z_bar[2, t], z_bar[3, t], steer_bar[t], config)
        constraints.append(z[:, t + 1] == a @ z[:, t] + b @ u[:, t] + c)
        if t == 0:
            cost += controller.mpc_rd[1, 1] * cp.square(u[1, t] - previous_steer)
            constraints.append(cp.abs(u[1, t] - previous_steer) <= max_change)
        else:
            cost += cp.quad_form(u[:, t] - u[:, t - 1], controller.mpc_rd)
            constraints.append(cp.abs(u[1, t] - u[1, t - 1]) <= max_change)
    cost += cp.quad_form(z[:, horizon] - local_ref[:, horizon], controller.mpc_qf)
    # 4. 非负速度、最大速度、加减速度、转角和转角变化率约束。
    constraints += [
        z[2, :] >= 0.0,
        z[2, :] <= vehicle.max_speed,
        u[0, :] >= -vehicle.max_decel,
        u[0, :] <= vehicle.max_accel,
        cp.abs(u[1, :]) <= vehicle.max_steer,
    ]
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP, warm_start=True, verbose=False,
                  eps_abs=1e-5, eps_rel=1e-5, max_iter=20000)
    if problem.status not in {cp.OPTIMAL, cp.OPTIMAL_INACCURATE} or u.value is None or z.value is None:
        raise RuntimeError(f"MPC QP failed with status {problem.status}")
    return np.asarray(u.value[0]).copy(), np.asarray(u.value[1]).copy(), np.asarray(z.value) + origin[:, None]


def control(state, reference, previous_control, config):
    z_ref = nearest_horizon_reference(state, reference, config)
    z0 = np.array([state.x, state.y, state.v, state.yaw])
    horizon = config.controller.mpc_horizon
    acceleration = np.full(horizon, previous_control.acceleration, dtype=float)
    steer = np.full(horizon, previous_control.steer, dtype=float)
    prediction = np.tile(z0.reshape(4, 1), (1, horizon + 1))

    if horizon < 1 or config.controller.mpc_iter_max < 1:
        raise ValueError("mpc_horizon 和 mpc_iter_max 必须为正整数。")

    # 5. 非线性预测 -> 线性化 -> 求解 QP，收敛或达到迭代上限后执行第一步。
    for _ in range(config.controller.mpc_iter_max):
        z_bar = np.empty_like(prediction)
        z_bar[:, 0] = z0
        for t in range(horizon):
            z_bar[:, t + 1] = _model_step(z_bar[:, t], acceleration[t], steer[t], config)
        new_acceleration, new_steer, prediction = solve_linear_mpc(
            z_ref, z_bar, z0, previous_control.steer, config, steer_bar=steer,
        )
        change = max(np.max(np.abs(new_acceleration - acceleration)), np.max(np.abs(new_steer - steer)))
        acceleration, steer = new_acceleration, new_steer
        if change < config.controller.mpc_du_threshold:
            break

    command = ControlCommand(acceleration=float(acceleration[0]), steer=float(steer[0]))
    command.prediction = prediction
    return command
