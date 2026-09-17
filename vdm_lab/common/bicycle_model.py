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


# ========== 新增：简化运动学模型（忽略侧偏角）==========

def simplified_kinematic_derivatives(state, steer, vehicle):
    """
    简化运动学模型：忽略车身侧偏角 beta。
    
    假设：
    1. 纯滚动、无侧滑
    2. 忽略侧偏角（beta ≈ 0）
    3. 速度方向与车身朝向一致
    
    公式：
        x_dot = v * cos(psi)
        y_dot = v * sin(psi)
        psi_dot = v / L * tan(delta_f)
    """
    wheelbase = vehicle.lf + vehicle.lr
    # 忽略侧偏角，速度方向与车身朝向一致
    x_dot = state.v * math.cos(state.yaw)
    y_dot = state.v * math.sin(state.yaw)
    # 横摆角速度简化为纯几何关系
    yaw_rate = state.v / wheelbase * math.tan(steer)
    return x_dot, y_dot, yaw_rate, 0.0  # beta = 0


# ========== 动力学模型（线性二自由度自行车模型）==========
#
# 与运动学模型的关键区别：这里把车身横向速度 v_y 和横摆角速度 r 作为独立
# 状态，由线性轮胎侧偏力驱动，因此能够反映轮胎侧偏、横摆响应滞后以及高速
# 下的不足转向特性。运动学模型只用几何关系直接给出横摆角速度，没有这些
# 动态过程。
#
# 线性轮胎侧偏角（小角度假设）:
#     alpha_f = delta_f - (v_y + lf * r) / v
#     alpha_r = -(v_y - lr * r) / v
# 线性轮胎力:
#     F_yf = Cf * alpha_f,   F_yr = Cr * alpha_r
# 车身动力学:
#     m  * (v_y_dot + v * r) = F_yf + F_yr
#     Iz * r_dot             = lf * F_yf - lr * F_yr
# 全局位置（把车身速度 (v, v_y) 旋转到全局坐标系）:
#     x_dot = v * cos(psi) - v_y * sin(psi)
#     y_dot = v * sin(psi) + v_y * cos(psi)

DYNAMIC_MIN_SPEED = 0.5


def dynamic_lateral_accel(speed, v_y, yaw_rate, steer, vehicle):
    """返回线性二自由度模型的横向、横摆角加速度 (v_y_dot, r_dot)。"""
    u = max(speed, DYNAMIC_MIN_SPEED)
    alpha_f = steer - (v_y + vehicle.lf * yaw_rate) / u
    alpha_r = -(v_y - vehicle.lr * yaw_rate) / u
    f_yf = vehicle.cf * alpha_f
    f_yr = vehicle.cr * alpha_r
    v_y_dot = (f_yf + f_yr) / vehicle.mass - u * yaw_rate
    r_dot = (vehicle.lf * f_yf - vehicle.lr * f_yr) / vehicle.inertia_z
    return v_y_dot, r_dot


def dynamic_sideslip(speed, v_y):
    """动力学模型的车身侧偏角 beta = atan2(v_y, v)。"""
    return math.atan2(v_y, max(abs(speed), 1.0e-6))


def tire_slip_angles(speed, v_y, yaw_rate, steer, vehicle):
    """前、后轮胎侧偏角 (alpha_f, alpha_r)，供实验分析使用。"""
    u = max(speed, DYNAMIC_MIN_SPEED)
    alpha_f = steer - (v_y + vehicle.lf * yaw_rate) / u
    alpha_r = -(v_y - vehicle.lr * yaw_rate) / u
    return alpha_f, alpha_r
