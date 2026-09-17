import math

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "Pure Pursuit Student"


def control(state, reference, previous_control, config):
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    # 学生填写 1：前视距离 = 固定前视距离 + 速度增益 * 当前速度。
    lookahead = controller.pp_base_lookahead + controller.pp_speed_gain * state.v

    # 学生填写 2：从最近点开始向前搜索，找到距离车辆不小于 Lf 的目标点。
    target_index = reference.nearest_index
    while target_index < len(path.x) - 1:
        distance = math.hypot(path.x[target_index] - state.x, path.y[target_index] - state.y)
        if distance >= lookahead:
            break
        target_index += 1

    # 学生填写 3：目标点方向与车身航向的夹角 alpha，归一化到 [-pi, pi]。
    target_x = path.x[target_index]
    target_y = path.y[target_index]
    alpha = pi_to_pi(math.atan2(target_y - state.y, target_x - state.x) - state.yaw)

    # 学生填写 4：Pure Pursuit 几何关系 delta = atan2(2L sin(alpha), Lf)。
    steer = math.atan2(2.0 * vehicle.wheelbase * math.sin(alpha), lookahead)

    goal_distance = math.hypot(state.x - path.x[-1], state.y - path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer))
