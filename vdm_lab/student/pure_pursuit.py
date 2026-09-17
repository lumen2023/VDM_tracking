import math

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid
from vdm_lab.student._controller_utils import (
    bounded_acceleration,
    bounded_steer,
    command_target_speed,
    finite,
    remaining_path_distance,
    state_in_speed_envelope,
    stop_command,
    validate_safety_config,
    validated_wheelbase,
)


NAME = "Pure Pursuit Student"


def control(state, reference, previous_control, config):
    validate_safety_config(config)
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle
    previous_steer = getattr(previous_control, "steer", 0.0)

    if (
        not state_in_speed_envelope(state, vehicle)
        or len(path.x) == 0
        or len(path.y) == 0
    ):
        return stop_command(state, previous_control, config)

    speed = max(0.0, finite(state.v, math.nan))
    base_lookahead = finite(controller.pp_base_lookahead, math.nan)
    speed_gain = finite(controller.pp_speed_gain, math.nan)
    waypoint_ds = finite(config.sim.waypoint_ds, math.nan)
    wheelbase = validated_wheelbase(vehicle)
    if (
        not math.isfinite(base_lookahead)
        or not math.isfinite(speed_gain)
        or not math.isfinite(waypoint_ds)
        or not math.isfinite(wheelbase)
        or base_lookahead <= 0.0
        or speed_gain < 0.0
        or waypoint_ds <= 0.0
        or wheelbase <= 0.0
    ):
        return stop_command(state, previous_control, config)
    lookahead = max(base_lookahead + speed_gain * speed, waypoint_ds, 1.0e-3)

    last_index = min(len(path.x), len(path.y)) - 1
    try:
        target_index = int(reference.nearest_index)
    except (TypeError, ValueError, OverflowError):
        target_index = 0
    target_index = int(clamp(target_index, 0, last_index))
    state_x = finite(state.x, path.x[target_index])
    state_y = finite(state.y, path.y[target_index])
    while target_index < last_index:
        distance = math.hypot(
            finite(path.x[target_index], state_x) - state_x,
            finite(path.y[target_index], state_y) - state_y,
        )
        if distance >= lookahead:
            break
        target_index += 1

    target_x = finite(path.x[target_index], math.nan)
    target_y = finite(path.y[target_index], math.nan)
    if not all(math.isfinite(value) for value in (target_x, target_y)):
        return stop_command(state, previous_control, config)
    yaw = finite(state.yaw, 0.0)
    alpha = pi_to_pi(math.atan2(target_y - state_y, target_x - state_x) - yaw)
    desired_steer = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead)
    steer = bounded_steer(desired_steer, previous_steer, config)

    try:
        goal_distance = remaining_path_distance(path, reference.nearest_index)
    except ValueError:
        return stop_command(state, previous_control, config)
    target_speed = command_target_speed(
        finite(reference.target_speed, 0.0),
        reference,
        speed,
        vehicle,
    )
    acceleration = speed_pid(
        target_speed, speed, goal_distance, controller, vehicle
    )
    acceleration = bounded_acceleration(
        acceleration, state.v, config, speed_limit=target_speed
    )
    return ControlCommand(acceleration=acceleration, steer=steer)
