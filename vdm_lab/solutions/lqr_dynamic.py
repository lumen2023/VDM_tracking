import math

import numpy as np

from vdm_lab.common.geometry import clamp
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid
from vdm_lab.solutions.lqr_kinematic import solve_lqr


NAME = "LQR Dynamic"


def build_dynamic_model(speed, config):
    vehicle = config.vehicle
    controller = config.controller
    dt = config.sim.dt
    v = max(speed, controller.lqr_min_model_speed)
    mass = vehicle.mass
    iz = vehicle.inertia_z
    lf = vehicle.lf
    lr = vehicle.lr
    cf = vehicle.cf
    cr = vehicle.cr

    A_c = np.zeros((4, 4))
    A_c[0, 1] = 1.0
    A_c[1, 1] = -(cf + cr) / mass / v
    A_c[1, 2] = (cf + cr) / mass
    A_c[1, 3] = (lr * cr - lf * cf) / mass / v
    A_c[2, 3] = 1.0
    A_c[3, 1] = (lr * cr - lf * cf) / iz / v
    A_c[3, 2] = (lf * cf - lr * cr) / iz
    A_c[3, 3] = -(lf * lf * cf + lr * lr * cr) / iz / v

    identity = np.eye(4)
    A = np.linalg.pinv(identity - 0.5 * dt * A_c) @ (identity + 0.5 * dt * A_c)
    B_c = np.zeros((4, 1))
    B_c[1, 0] = cf / mass
    B_c[3, 0] = lf * cf / iz
    B = B_c * dt
    return A, B


def dynamic_feedforward(speed, curvature, K, config):
    vehicle = config.vehicle
    v = max(speed, config.controller.lqr_min_model_speed)
    mass = vehicle.mass
    wheelbase = vehicle.lf + vehicle.lr
    kv = vehicle.lr * mass / (2.0 * vehicle.cf * wheelbase) - vehicle.lf * mass / (2.0 * vehicle.cr * wheelbase)
    yaw_steady_error = vehicle.lr * curvature - vehicle.lf * mass * v * v * curvature / (2.0 * vehicle.cr * wheelbase)
    return wheelbase * curvature + kv * v * v * curvature - K[0, 2] * yaw_steady_error


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

    feedback = float(-(K @ error_state)[0, 0])
    feedforward = float(dynamic_feedforward(speed, reference.curvature, K, config))
    steer = feedback + feedforward

    goal_distance = math.hypot(state.x - reference.path.x[-1], state.y - reference.path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer))
