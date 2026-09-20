import math

import numpy as np

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand


NAME = "Linear MPC Student"


def nearest_horizon_reference(state, reference, config):
    # 1. 从最近点出发，沿路径向前取 T+1 个参考点，构造 [x, y, v, yaw] 参考轨迹。
    #    相邻参考点之间的推进距离用当前车速估计（低速时至少按 0.5 倍目标速度
    #    前进，避免起步阶段参考点挤在一起），再按重采样间距换成索引步长。
    path = reference.path
    horizon = config.controller.mpc_horizon
    z_ref = np.zeros((4, horizon + 1))
    base_index = reference.nearest_index
    distance = 0.0
    preview_speed = max(reference.target_speed, 1.0)
    # 圆形路径的 yaw 在 pi / -pi 处会跳变，逐点累加增量把它连续化，
    # 否则预测时域里会出现约 2*pi 的假跳变，QP 会算出错误的转角。
    previous_yaw = state.yaw
    for i in range(horizon + 1):
        if i > 0:
            distance += max(state.v, preview_speed * 0.5) * config.sim.dt
        offset = int(round(distance / config.sim.waypoint_ds))
        index = min(base_index + offset, len(path.x) - 1)
        yaw_ref = previous_yaw + pi_to_pi(path.yaw[index] - previous_yaw)
        z_ref[:, i] = [path.x[index], path.y[index], path.target_speed[index], yaw_ref]
        previous_yaw = yaw_ref
    return z_ref


def linear_model(v, yaw, steer, config):
    # 2. 围绕名义点 (v, yaw, steer) 对运动学自行车模型做一阶泰勒展开：
    #        z_{k+1} = A z_k + B u_k + C
    #    状态 z = [x, y, v, yaw]，输入 u = [a, delta]，dt 为仿真步长。
    #    A 是状态雅可比，B 是输入雅可比，C 是把展开点代回原非线性式后
    #    得到的常数补偿项，保证线性模型在名义点处与原模型精确相等。
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

    # 3. 二次型目标：跟踪误差 + 控制量幅值 + 控制量变化率。
    #    末点单独用权重更大的 mpc_qf 收尾，让时域末尾也收敛到参考轨迹。
    #    变化率项惩罚 u[t+1] - u[t]，作用是抑制转角抖动，代价是响应变慢。
    cost = 0.0
    constraints = [z[:, 0] == z0]

    for t in range(horizon):
        cost += cp.quad_form(z_ref[:, t] - z[:, t], controller.mpc_q)
        cost += cp.quad_form(u[:, t], controller.mpc_r)
        A, B, C = linear_model(z_bar[2, t], z_bar[3, t], previous_steer[t], config)
        constraints.append(z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C)
        if t < horizon - 1:
            cost += cp.quad_form(u[:, t + 1] - u[:, t], controller.mpc_rd)
            constraints.append(
                cp.abs(u[1, t + 1] - u[1, t]) <= vehicle.max_steer_rate * config.sim.dt
            )

    cost += cp.quad_form(z_ref[:, horizon] - z[:, horizon], controller.mpc_qf)

    # 4. 约束：速度非负且不超上限；加速度受驱动能力和制动能力分别限制；
    #    转角限幅到车辆物理极限 max_steer。
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


def control(state, reference, previous_control, config):
    z_ref = nearest_horizon_reference(state, reference, config)
    z0 = np.array([state.x, state.y, state.v, state.yaw])
    horizon = config.controller.mpc_horizon
    vehicle = config.vehicle
    dt = config.sim.dt
    acceleration = np.full(horizon, previous_control.acceleration)
    steer = np.full(horizon, previous_control.steer)
    prediction = np.tile(z0.reshape(4, 1), (1, horizon + 1))

    # 5. 滚动优化：迭代"预测 -> 线性化 -> 求解 QP"。
    #    控制序列初值取上一拍的解，比从零开始更快收敛。
    #    每轮先用当前控制序列把 z0 前向积分得到名义轨迹 z_bar，
    #    再围绕 z_bar 线性化并求解 QP；当解的变化小于阈值时提前退出。
    #    积分用的是运动学自行车模型 z_{k+1} = f(z_k, u_k)，与 linear_model
    #    对同一个 f 做泰勒展开，两者必须一致，否则迭代会发散。
    for _ in range(config.controller.mpc_iter_max):
        z_bar = np.zeros_like(z_ref)
        z_bar[:, 0] = z0
        nominal = np.array(z0, dtype=float)
        for i in range(horizon):
            x, y, v, yaw = nominal
            steer_i = clamp(steer[i], -vehicle.max_steer, vehicle.max_steer)
            nominal = np.array(
                [
                    x + v * math.cos(yaw) * dt,
                    y + v * math.sin(yaw) * dt,
                    clamp(v + acceleration[i] * dt, vehicle.min_speed, vehicle.max_speed),
                    yaw + v / vehicle.wheelbase * math.tan(steer_i) * dt,
                ]
            )
            z_bar[:, i + 1] = nominal

        previous_acceleration = acceleration.copy()
        previous_steer_sequence = steer.copy()
        acceleration, steer, prediction = solve_linear_mpc(z_ref, z_bar, z0, steer, config)
        if max(
            np.max(np.abs(acceleration - previous_acceleration)),
            np.max(np.abs(steer - previous_steer_sequence)),
        ) < config.controller.mpc_du_threshold:
            break

    # 只执行时域第一个控制量，下一拍带着新的状态重新优化（滚动时域）。
    command = ControlCommand(acceleration=float(acceleration[0]), steer=float(steer[0]))
    command.prediction = prediction
    return command
