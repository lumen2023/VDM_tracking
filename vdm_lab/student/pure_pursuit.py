import math

from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "Pure Pursuit Student"


def control(state, reference, previous_control, config):
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    # TODO 学生填写 1：根据车速计算前视距离 Lf = 固定前视距离 + 速度增益 * 当前速度。
    raise NotImplementedError("请先填写 Pure Pursuit 的前视距离公式。")

    # TODO 学生填写 2：从最近点开始向前搜索，找到距离车辆不小于 Lf 的目标点。
    target_index = reference.nearest_index

    # TODO 学生填写 3：计算目标点方向与车身航向之间的夹角 alpha。
    target_x = path.x[target_index]
    target_y = path.y[target_index]
    alpha = math.atan2(target_y - state.y, target_x - state.x) - state.yaw

    # TODO 学生填写 4：根据几何关系计算前轮转角 delta = atan2(2L sin(alpha), Lf)。
    steer = 0.0

    goal_distance = math.hypot(state.x - path.x[-1], state.y - path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    return ControlCommand(acceleration=acceleration, steer=steer)
