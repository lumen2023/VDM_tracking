"""
Vehicle-model backend interface.

The original VDM_tracking simulation uses the built-in kinematic bicycle
model.  This adapter keeps that behavior as the default but creates one clean
boundary for connecting TruckSim, CarSim, ROS, MetaDrive, a dynamic bicycle
model, or another external simulator later.

Controllers do NOT need to change. They still consume VehicleState and return
ControlCommand.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod

from vdm_lab.common.bicycle_model import (
    kinematic_derivatives,
    simplified_kinematic_derivatives,
    dynamic_lateral_accel,
    dynamic_sideslip,
)
from vdm_lab.common.geometry import pi_to_pi
from vdm_lab.common.types import ControlCommand, VehicleState
from vdm_lab.common.vehicle import limit_command, update_state


class VehicleBackend(ABC):
    """Common interface between controller/simulation and a vehicle model."""

    def reset(self, initial_state, config):
        """
        Optional reset hook.

        External simulators can override this to reset their own model and then
        return the state actually reported by the simulator.
        """
        return initial_state

    @abstractmethod
    def dynamics_terms(self, state, command, config):
        """
        Return (yaw_rate, beta) for logging.

        If an external simulator does not provide beta, returning float("nan")
        is acceptable.
        """
        raise NotImplementedError

    @abstractmethod
    def step(self, state, command, config, dt):
        """
        Advance the vehicle by one control step.

        Returns
        -------
        next_state : VehicleState
        applied_command : ControlCommand
            The command after any backend/model-specific saturation.
        """
        raise NotImplementedError


class KinematicBicycleBackend(VehicleBackend):
    """Wrapper around the repository's existing vehicle model."""

    def dynamics_terms(self, state, command, config):
        _, _, yaw_rate, beta = kinematic_derivatives(
            state,
            command.steer,
            config.vehicle,
        )
        return float(yaw_rate), float(beta)

    def step(self, state, command, config, dt):
        return update_state(
            state,
            command,
            config.vehicle,
            dt,
            model_type="kinematic",
        )


class SimplifiedKinematicBackend(VehicleBackend):
    """简化运动学模型：忽略侧偏角。"""

    def dynamics_terms(self, state, command, config):
        _, _, yaw_rate, beta = simplified_kinematic_derivatives(
            state,
            command.steer,
            config.vehicle,
        )
        return float(yaw_rate), float(beta)

    def step(self, state, command, config, dt):
        return update_state(
            state,
            command,
            config.vehicle,
            dt,
            model_type="simplified",
        )


class DynamicBicycleBackend(VehicleBackend):
    """线性二自由度动力学模型：考虑轮胎侧偏力。

    与运动学模型不同，本模型把车身横向速度 ``v_y`` 和横摆角速度 ``r`` 作为
    内部状态，用线性轮胎侧偏力积分得到真实的侧偏、横摆滞后和不足转向响应。
    外部接口仍然是标准的 ``VehicleState -> ControlCommand``，控制器无需改动。

    积分使用固定子步长的四阶 Runge-Kutta。低速时轮胎侧偏刚度使横向动力学
    变得刚性（时间常数约 2 ms），因此子步长必须足够小才能稳定；默认 2 ms
    在 v >= DYNAMIC_MIN_SPEED 的全部工况下都有稳定裕度。

    局限（务必在报告中注明）：

    - 线性轮胎 ``F_y = C * alpha``，没有轮胎饱和、载荷转移和松弛长度；
      大侧偏角（约 > 5 deg）时结果不再可信。
    - 没有摩擦圆/附着极限，不能推断真实车辆的打滑边界。
    - 仿真采样周期 ``dt = 0.1 s``，本模型用 2 ms 子步长内部积分。
    - 纵向速度低于 ``DYNAMIC_MIN_SPEED = 0.5 m/s`` 时使用该地板值计算
      侧偏角，低速段是数值近似而非精确物理。
    - 默认 ``run_experiment.py`` 的被控对象仍是运动学模型；动力学模型需要
      显式选择 ``--vehicle-model dynamic``。``lqr_dynamic`` 只改变控制器
      内部的预测模型，并不改变被控车辆模型。
    """

    def __init__(self, substep=0.002):
        self.substep = float(substep)
        self._v_y = 0.0
        self._r = 0.0

    def reset(self, initial_state, config):
        self._v_y = 0.0
        self._r = 0.0
        return initial_state

    def dynamics_terms(self, state, command, config):
        return float(self._r), float(dynamic_sideslip(state.v, self._v_y))

    def _derivative(self, state, steer, acceleration, config):
        """状态向量 [x, y, yaw, v, v_y, r] 的时间导数。"""
        x, y, yaw, v, v_y, r = state
        vehicle = config.vehicle
        v_y_dot, r_dot = dynamic_lateral_accel(v, v_y, r, steer, vehicle)
        x_dot = v * math.cos(yaw) - v_y * math.sin(yaw)
        y_dot = v * math.sin(yaw) + v_y * math.cos(yaw)
        return (x_dot, y_dot, r, acceleration, v_y_dot, r_dot)

    def step(self, state, command, config, dt):
        command = limit_command(command, config.vehicle)
        vehicle = config.vehicle

        substep = max(self.substep, 1.0e-4)
        steps = max(1, int(math.ceil(dt / substep)))
        h = dt / steps

        vector = (state.x, state.y, state.yaw, state.v, self._v_y, self._r)

        for _ in range(steps):
            k1 = self._derivative(vector, command.steer, command.acceleration, config)
            v2 = tuple(vector[i] + 0.5 * h * k1[i] for i in range(6))
            k2 = self._derivative(v2, command.steer, command.acceleration, config)
            v3 = tuple(vector[i] + 0.5 * h * k2[i] for i in range(6))
            k3 = self._derivative(v3, command.steer, command.acceleration, config)
            v4 = tuple(vector[i] + h * k3[i] for i in range(6))
            k4 = self._derivative(v4, command.steer, command.acceleration, config)
            vector = tuple(
                vector[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
                for i in range(6)
            )
            # 纵向速度限幅，保证仅前进且不超过车辆上限。
            vector = (
                vector[0],
                vector[1],
                vector[2],
                min(max(vector[3], vehicle.min_speed), vehicle.max_speed),
                vector[4],
                vector[5],
            )

        self._v_y = vector[4]
        self._r = vector[5]

        if not all(math.isfinite(value) for value in vector):
            # 数值发散时复位横向状态，避免把 inf/nan 传播到公共框架
            # （pi_to_pi 对 inf 会陷入死循环）。
            self._v_y = 0.0
            self._r = 0.0
            raise FloatingPointError(
                "动力学模型积分发散，请减小 DynamicBicycleBackend 的子步长。"
            )

        next_state = VehicleState(
            x=vector[0],
            y=vector[1],
            yaw=pi_to_pi(vector[2]),
            v=vector[3],
        )
        return next_state, command


class ExternalVehicleBackend(VehicleBackend):
    """
    Template for connecting a real external simulator.

    A concrete TruckSim/CarSim/ROS adapter should typically implement:

        reset()
            reset simulator -> read initial state -> VehicleState

        dynamics_terms()
            read yaw rate / beta from simulator (or return NaN)

        step()
            1. convert command.steer [rad] and acceleration [m/s^2]
               into the simulator's input convention;
            2. advance/wait one simulation step;
            3. read x/y/yaw/speed;
            4. return VehicleState(...), applied ControlCommand(...)

    The VDM controllers therefore remain completely simulator-independent.
    """

    def dynamics_terms(self, state, command, config):
        return float("nan"), float("nan")

    def step(self, state, command, config, dt):
        raise NotImplementedError(
            "请继承 ExternalVehicleBackend，并实现具体仿真器的 step() 接口。"
        )
