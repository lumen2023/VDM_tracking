import math

import numpy as np

from vdm_lab.common.geometry import clamp
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "LQR Kinematic"


def solve_lqr(A, B, Q, R, eps, max_iter):
    P = Q.copy()
    for _ in range(max_iter):
        P_next = A.T @ P @ A - A.T @ P @ B @ np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A + Q
        if np.max(np.abs(P_next - P)) < eps:
            P = P_next
            break
        P = P_next
    return np.linalg.pinv(R + B.T @ P @ B) @ B.T @ P @ A


def build_kinematic_model(speed, config):
    dt = config.sim.dt
    wheelbase = config.vehicle.wheelbase
    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = speed
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = speed / wheelbase
    return A, B


def control(state, reference, previous_control, config):
    controller = config.controller
    vehicle = config.vehicle
    speed = max(state.v, controller.lqr_min_model_speed)

    A, B = build_kinematic_model(speed, config)
    K = solve_lqr(A, B, controller.lqr_q, controller.lqr_r, controller.lqr_eps, controller.lqr_max_iter)

    e_y = reference.lateral_error
    e_y_dot = speed * math.sin(reference.heading_error)
    e_yaw = reference.heading_error
    e_yaw_dot = speed / vehicle.wheelbase * math.tan(previous_control.steer) - speed * reference.curvature
    error_state = np.array([[e_y], [e_y_dot], [e_yaw], [e_yaw_dot]])

    feedback = float(-(K @ error_state)[0, 0])
    feedforward = vehicle.wheelbase * reference.curvature
    steer = feedback + feedforward

    goal_distance = math.hypot(state.x - reference.path.x[-1], state.y - reference.path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer))
