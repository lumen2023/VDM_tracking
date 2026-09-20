import math

import numpy as np

from vdm_lab.common.geometry import clamp
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "LQR Kinematic Student"


def solve_lqr(A, B, Q, R, eps, max_iter):
    # 1. 离散 Riccati 迭代：反复代入代数 Riccati 方程，直到 P 的变化小于 eps。
    #    P 初值取 Q；max_iter 只是防止不收敛时死循环的保护。
    P = Q.copy()
    for _ in range(max_iter):
        P_next = (
            A.T @ P @ A
            - A.T @ P @ B @ np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A
            + Q
        )
        if np.max(np.abs(P_next - P)) < eps:
            P = P_next
            break
        P = P_next
    # 最优反馈增益 K = (R + B^T P B)^-1 B^T P A
    return np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A


def build_kinematic_model(speed, config):
    dt = config.sim.dt
    wheelbase = config.vehicle.wheelbase
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    # 2. 误差状态 [e_y, e_y_dot, e_yaw, e_yaw_dot] 的离散运动学模型。
    #    e_y 的导数就是 e_y_dot              -> A[0, 1] = dt
    #    e_y_dot 由航向误差驱动              -> A[1, 2] = v
    #    e_yaw 的导数就是 e_yaw_dot          -> A[2, 3] = dt
    #    e_yaw_dot 由前轮转角驱动            -> B[3, 0] = v / L
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = speed
    A[2, 2] = 1.0
    A[2, 3] = dt

    B[3, 0] = speed / wheelbase
    return A, B


def control(state, reference, previous_control, config):
    controller = config.controller
    vehicle = config.vehicle
    speed = max(state.v, controller.lqr_min_model_speed)

    A, B = build_kinematic_model(speed, config)
    K = solve_lqr(A, B, controller.lqr_q, controller.lqr_r, controller.lqr_eps, controller.lqr_max_iter)

    # 3. 误差状态。注意速度和曲率进入的是误差的"变化率"：
    #    e_y_dot 取车速在路径法向的分量 v * sin(e_yaw)；
    #    e_yaw_dot 是"车身实际横摆角速度 - 参考路径横摆角速度"，
    #    前者由上一拍转角给出（v / L * tan(delta)），后者等于 v * kappa。
    #    同一组 K 在不同车速下闭环效果不同，这就是高速更难控的根源之一。
    e_y = reference.lateral_error
    e_y_dot = speed * math.sin(reference.heading_error)
    e_yaw = reference.heading_error
    e_yaw_dot = (
        speed / vehicle.wheelbase * math.tan(previous_control.steer)
        - speed * reference.curvature
    )
    error_state = np.array([[e_y], [e_y_dot], [e_yaw], [e_yaw_dot]])

    # 4. 反馈转角 + 曲率前馈转角。
    #    反馈项 -K e 负责把误差拉回零；但纯反馈在恒曲率路径上会留下稳态误差，
    #    所以叠加前馈项 L * kappa（小角度下 atan(L * kappa) 的近似），
    #    让车辆即使误差为零也能跟上路径本身的曲率。
    feedback = float(-(K @ error_state)[0, 0])
    feedforward = vehicle.wheelbase * reference.curvature
    steer = feedback + feedforward

    goal_distance = math.hypot(state.x - reference.path.x[-1], state.y - reference.path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(
        acceleration=acceleration,
        steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer),
    )
