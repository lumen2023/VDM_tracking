import math

import numpy as np

from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "LQR Dynamic Student"


def solve_lqr(A, B, Q, R, eps, max_iter):
    # TODO 学生填写 1：可参考运动学 LQR，完成 Riccati 迭代与 K 的计算。
    raise NotImplementedError("请先填写动力学 LQR 的 Riccati 迭代。")


def build_dynamic_model(speed, config):
    vehicle = config.vehicle
    controller = config.controller

    # TODO 学生填写 2：车速过低时会导致 1/v 发散，请给 v 设置下限保护。
    raise NotImplementedError("请先填写动力学模型的车速保护。")

    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    # TODO 学生填写 3：根据线性二自由度车辆模型填写 A、B 矩阵，并离散化。
    return A, B


def dynamic_feedforward(speed, curvature, K, config):
    # TODO 学生填写 4：填写动力学 LQR 曲率前馈项，思考稳态横摆误差如何补偿。
    raise NotImplementedError("请先填写动力学 LQR 的前馈转角。")


def control(state, reference, previous_control, config):
    controller = config.controller
    vehicle = config.vehicle
    speed = max(state.v, controller.lqr_min_model_speed)

    A, B = build_dynamic_model(speed, config)
    K = solve_lqr(A, B, controller.lqr_q, controller.lqr_r, controller.lqr_eps, controller.lqr_max_iter)

    e_y = reference.lateral_error
    e_y_dot = speed * math.sin(reference.heading_error)
    e_yaw = reference.heading_error
    e_yaw_dot = speed / vehicle.wheelbase * math.tan(previous_control.steer) - speed * reference.curvature
    error_state = np.array([[e_y], [e_y_dot], [e_yaw], [e_yaw_dot]])

    steer = float(-(K @ error_state)[0, 0] + dynamic_feedforward(speed, reference.curvature, K, config))
    goal_distance = math.hypot(state.x - reference.path.x[-1], state.y - reference.path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=steer)
