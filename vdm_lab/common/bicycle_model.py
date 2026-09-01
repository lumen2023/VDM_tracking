import math


def front_steer_slip_angle(steer, vehicle):
    """PDF 中的 beta: 前轮转向、后轮不转向时的车身侧偏角。"""
    wheelbase = vehicle.lf + vehicle.lr
    return math.atan2(vehicle.lr * math.tan(steer), wheelbase)


def yaw_rate_from_steer(speed, steer, vehicle):
    """PDF 中的 psi_dot = v / (lf + lr) * tan(delta_f) * cos(beta)。"""
    beta = front_steer_slip_angle(steer, vehicle)
    wheelbase = vehicle.lf + vehicle.lr
    return speed / wheelbase * math.tan(steer) * math.cos(beta)


def kinematic_derivatives(state, steer, vehicle):
    """连续时间运动学自行车模型: x_dot, y_dot, psi_dot。"""
    beta = front_steer_slip_angle(steer, vehicle)
    yaw_rate = yaw_rate_from_steer(state.v, steer, vehicle)
    x_dot = state.v * math.cos(state.yaw + beta)
    y_dot = state.v * math.sin(state.yaw + beta)
    return x_dot, y_dot, yaw_rate, beta


def normal_acceleration(speed, curvature):
    """圆周运动关系: a_n = v^2 / rho = v^2 * kappa。"""
    return speed * speed * curvature
