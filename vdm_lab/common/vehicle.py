from vdm_lab.common.bicycle_model import kinematic_derivatives
from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand, VehicleConfig, VehicleState


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


def update_state(state, command, vehicle_config, dt):
    command = limit_command(command, vehicle_config)
    x_dot, y_dot, yaw_rate, _ = kinematic_derivatives(state, command.steer, vehicle_config)
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
