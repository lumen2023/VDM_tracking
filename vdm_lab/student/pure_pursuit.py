import math

from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "Pure Pursuit Student"


def control(state, reference, previous_control, config):
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    # 1. 原 PP 的速度相关前视距离，参数统一读取课程配置。
    lookahead = max(1.0e-6, controller.pp_base_lookahead + controller.pp_speed_gain * state.v)

    # 2. 从最近点向前搜索，末端使用最后一个路径点。
    target_index = reference.nearest_index
    while target_index < len(path.x) - 1:
        distance = math.hypot(path.x[target_index] - state.x, path.y[target_index] - state.y)
        if distance >= lookahead:
            break
        target_index += 1

    # 3. 计算目标方向相对车身航向的夹角。
    target_x = path.x[target_index]
    target_y = path.y[target_index]
    alpha = math.atan2(target_y - state.y, target_x - state.x) - state.yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

    # 4. 几何转角，并保留原程序的转角及转角变化率限制。
    steer = math.atan2(2.0 * (vehicle.lf + vehicle.lr) * math.sin(alpha), lookahead)
    steer = max(-vehicle.max_steer, min(vehicle.max_steer, steer))
    max_change = vehicle.max_steer_rate * config.sim.dt
    steer = previous_control.steer + max(-max_change, min(max_change, steer - previous_control.steer))

    goal_distance = math.hypot(state.x - path.x[-1], state.y - path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=steer)
