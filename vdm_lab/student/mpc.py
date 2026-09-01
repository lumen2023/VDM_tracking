import numpy as np

from vdm_lab.common.types import ControlCommand


NAME = "Linear MPC Student"


def nearest_horizon_reference(state, reference, config):
    # TODO 学生填写 1：从最近点开始，为预测时域 T+1 个点构造 [x, y, v, yaw] 参考轨迹。
    # 提示：圆形路径会跨越 pi/-pi，参考 yaw 要相对当前航向做连续化处理。
    raise NotImplementedError("请先填写 MPC 的预测时域参考轨迹构造。")


def linear_model(v, yaw, steer, config):
    # TODO 学生填写 2：围绕预测状态线性化运动学自行车模型，得到 A、B、C。
    raise NotImplementedError("请先填写 MPC 的线性化模型 A、B、C。")


def solve_linear_mpc(z_ref, z_bar, z0, previous_steer, config):
    try:
        import cvxpy as cp
    except ImportError as exc:
        raise ImportError("MPC 需要安装 cvxpy：pip install -r requirements.txt") from exc

    # TODO 学生填写 3：建立二次型目标函数。
    # TODO 学生填写 4：建立约束，包括速度非负、转角限幅、加速度限幅、转角变化率限幅。
    raise NotImplementedError("请先填写 MPC 的优化目标和约束。")


def control(state, reference, previous_control, config):
    z_ref = nearest_horizon_reference(state, reference, config)
    z0 = np.array([state.x, state.y, state.v, state.yaw])
    horizon = config.controller.mpc_horizon
    acceleration = np.full(horizon, previous_control.acceleration)
    steer = np.full(horizon, previous_control.steer)
    prediction = np.tile(z0.reshape(4, 1), (1, horizon + 1))

    # TODO 学生填写 5：迭代“预测 -> 线性化 -> 求解 QP”，取第一个控制量执行。
    raise NotImplementedError("请先填写 MPC 的滚动优化流程。")

    command = ControlCommand(acceleration=float(acceleration[0]), steer=float(steer[0]))
    command.prediction = prediction
    return command
