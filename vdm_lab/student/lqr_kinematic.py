import math

import numpy as np

from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "LQR Kinematic Student"


def solve_lqr(A, B, Q, R, eps, max_iter):
    # TODO 学生填写 1：用离散 Riccati 迭代求解 P，再计算 K。
    raise NotImplementedError("请先填写 LQR 的 Riccati 迭代和反馈矩阵 K。")


def build_kinematic_model(speed, config):
    dt = config.sim.dt
    wheelbase = config.vehicle.wheelbase
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    # TODO 学生填写 2：建立运动学误差模型的离散 A、B 矩阵。
    raise NotImplementedError("请先填写运动学 LQR 的 A、B 矩阵。")

    return A, B


def control(state, reference, previous_control, config):
    controller = config.controller
    vehicle = config.vehicle
    speed = max(state.v, controller.lqr_min_model_speed)

    A, B = build_kinematic_model(speed, config)
    K = solve_lqr(A, B, controller.lqr_q, controller.lqr_r, controller.lqr_eps, controller.lqr_max_iter)

    # TODO 学生填写 3：构造误差状态 [横向误差, 横向误差变化率, 航向误差, 航向误差变化率]^T。
    e_y = reference.lateral_error
    e_y_dot = 0.0
    e_yaw = reference.heading_error
    e_yaw_dot = 0.0
    error_state = np.array([[e_y], [e_y_dot], [e_yaw], [e_yaw_dot]])

    # TODO 学生填写 4：组合反馈转角和曲率前馈转角。
    steer = 0.0

    goal_distance = math.hypot(state.x - reference.path.x[-1], state.y - reference.path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=steer)
