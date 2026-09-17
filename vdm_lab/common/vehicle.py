from vdm_lab.common.bicycle_model import (
    kinematic_derivatives,
    simplified_kinematic_derivatives,
)
from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand, VehicleConfig, VehicleState


# 支持的车辆模型。
# 注意：动力学模型带有额外的内部状态 (v_y, r)，无法在无状态的 update_state
# 中积分，因此由 DynamicBicycleBackend 负责实现；这里只保留可直接由当前
# 状态求导的两个运动学模型。
VEHICLE_MODELS = {
    "kinematic": "标准运动学模型（含侧偏角 beta）",
    "simplified": "简化运动学模型（忽略侧偏角，beta = 0）",
    "dynamic": "线性二自由度动力学模型（含轮胎侧偏力）",
}


def limit_command(command, vehicle_config):
    acceleration = clamp(
        command.acceleration,
        -vehicle_config.max_decel,
        vehicle_config.max_accel,
    )
    steer = clamp(command.steer, -vehicle_config.max_steer, vehicle_config.max_steer)
    limited = ControlCommand(acceleration=acceleration, steer=steer)
    if hasattr(command, "prediction"):
        limited.prediction = command.prediction
    return limited


def update_state(state, command, vehicle_config, dt, model_type="kinematic"):
    """
    更新车辆状态（无状态运动学模型）。

    参数:
        state: 当前状态
        command: 控制指令
        vehicle_config: 车辆参数
        dt: 时间步长
        model_type: 模型类型
            - "kinematic": 标准运动学模型（含侧偏角 beta）
            - "simplified": 简化运动学模型（忽略侧偏角，beta = 0）

    注意：动力学模型需要额外的内部状态 (v_y, r)，请使用
    vdm_lab.common.vehicle_backend.DynamicBicycleBackend，而不是本函数。

    返回:
        next_state, command
    """
    command = limit_command(command, vehicle_config)

    # 根据模型类型选择不同的导数计算方法
    if model_type == "simplified":
        x_dot, y_dot, yaw_rate, beta = simplified_kinematic_derivatives(
            state, command.steer, vehicle_config
        )
    elif model_type == "kinematic":
        x_dot, y_dot, yaw_rate, beta = kinematic_derivatives(
            state, command.steer, vehicle_config
        )
    else:
        raise ValueError(
            f"update_state 不支持模型 {model_type!r}；动力学模型请使用 "
            "DynamicBicycleBackend。可选值为: kinematic, simplified"
        )

    next_x = state.x + x_dot * dt
    next_y = state.y + y_dot * dt
    next_yaw = pi_to_pi(state.yaw + yaw_rate * dt)
    next_v = clamp(
        state.v + command.acceleration * dt,
        vehicle_config.min_speed,
        vehicle_config.max_speed,
    )
    return VehicleState(x=next_x, y=next_y, yaw=next_yaw, v=next_v), command


def speed_pid(target_speed, current_speed, distance_to_goal, config, vehicle_config):
    if distance_to_goal < 14.0:
        target_speed = min(target_speed, max(0.0, (2.0 * 0.9 * distance_to_goal) ** 0.5))
    if distance_to_goal < 1.0:
        target_speed = 0.0
    acceleration = config.kp_speed * (target_speed - current_speed)
    return clamp(acceleration, -vehicle_config.max_decel, vehicle_config.max_accel)
