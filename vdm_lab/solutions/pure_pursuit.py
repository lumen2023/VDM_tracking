import math

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "Pure Pursuit"


def control(state, reference, previous_control, config):
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    lookahead = controller.pp_base_lookahead + controller.pp_speed_gain * state.v
    target_index = reference.nearest_index
    while target_index < len(path.x) - 1:
        distance = math.hypot(path.x[target_index] - state.x, path.y[target_index] - state.y)
        if distance >= lookahead:
            break
        target_index += 1

    target_x = path.x[target_index]
    target_y = path.y[target_index]
    alpha = pi_to_pi(math.atan2(target_y - state.y, target_x - state.x) - state.yaw)
    steer = math.atan2(2.0 * vehicle.wheelbase * math.sin(alpha), lookahead)

    goal_distance = math.hypot(state.x - path.x[-1], state.y - path.y[-1])
    acceleration = speed_pid(path.target_speed[reference.nearest_index], state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer))
