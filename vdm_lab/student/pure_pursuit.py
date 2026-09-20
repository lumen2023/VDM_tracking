import math

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid


NAME = "Pure Pursuit Student"


def control(state, reference, previous_control, config):
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    # 1. 前视距离 Lf = 固定前视距离 + 速度增益 * 当前速度。
    #    车速越高看得越远，避免高速时频繁修正引起振荡；
    #    代价是前视点越远、弯道内切越明显，稳态横向误差随之增大。
    lookahead = controller.pp_base_lookahead + controller.pp_speed_gain * state.v

    # 2. 从最近点开始向前搜索，找到第一个到车辆距离不小于 Lf 的点。
    #    最近点由 ReferenceTracker 给出；必须向前搜索而不能取全局最近点，
    #    否则路径自交或横向偏差较大时可能选中车身侧后方的点。
    target_index = reference.nearest_index
    while target_index < len(path.x) - 1:
        distance = math.hypot(
            path.x[target_index] - state.x,
            path.y[target_index] - state.y,
        )
        if distance >= lookahead:
            break
        target_index += 1

    # 3. 前视点方向与车身航向之间的夹角 alpha。
    #    pi_to_pi 把角度归一到 (-pi, pi]；由于下一步只取 sin(alpha)，
    #    归一化不改变控制量，但能让中间量在调试时保持可读。
    target_x = path.x[target_index]
    target_y = path.y[target_index]
    alpha = pi_to_pi(
        math.atan2(target_y - state.y, target_x - state.x) - state.yaw
    )

    # 4. 几何转角公式：车辆沿一段弦长为 Lf、与航向夹角为 alpha 的圆弧前进，
    #    对应曲率 kappa = 2 * sin(alpha) / Lf，再由运动学自行车模型
    #    delta = atan(L * kappa) 得到下式。
    steer = math.atan2(2.0 * vehicle.wheelbase * math.sin(alpha), lookahead)

    goal_distance = math.hypot(state.x - path.x[-1], state.y - path.y[-1])
    acceleration = speed_pid(reference.target_speed, state.v, goal_distance, controller, vehicle)
    # 仿真循环里的 limit_command 已经限幅，这里再夹一次是为了让 control()
    # 被单独调用时同样满足执行器约束。
    return ControlCommand(
        acceleration=acceleration,
        steer=clamp(steer, -vehicle.max_steer, vehicle.max_steer),
    )
