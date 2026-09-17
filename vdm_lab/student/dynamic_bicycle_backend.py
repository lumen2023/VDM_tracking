"""Student dynamic-bicycle plant with internal lateral velocity and yaw rate.

``VehicleState`` only stores ``x, y, yaw, v``.  Sideslip velocity ``v_y`` and
yaw rate ``r`` live on this backend.  Tire forces use the same axle-stiffness
convention as ``lqr_dynamic.build_dynamic_model``:

    alpha_f = (v_y + lf * r) / v_x - delta_f
    alpha_r = (v_y - lr * r) / v_x
    F_y = -C * alpha

Below ``KINEMATIC_SPEED_FLOOR`` the step falls back to the shared kinematic
bicycle so the stiff tire model is not integrated at near-zero speed.
``dynamics_terms`` always reports the internal yaw rate and
``beta = atan2(v_y, v_x)``, never the purely geometric ``beta(delta)``.
"""

from __future__ import annotations

import math

import numpy as np

from vdm_lab.common.bicycle_model import kinematic_derivatives
from vdm_lab.common.geometry import pi_to_pi
from vdm_lab.common.types import VehicleState
from vdm_lab.common.vehicle import limit_command, update_state
from vdm_lab.common.vehicle_backend import VehicleBackend
from vdm_lab.student._controller_utils import bilinear_step, finite


KINEMATIC_SPEED_FLOOR = 0.5


def _lateral_matrices(v_x, vehicle):
    """Continuous-time matrices for [v_y, r] with F_y = -C alpha."""
    mass = finite(vehicle.mass)
    inertia_z = finite(vehicle.inertia_z)
    lf = finite(vehicle.lf)
    lr = finite(vehicle.lr)
    cf = finite(vehicle.cf)
    cr = finite(vehicle.cr)
    if mass <= 0.0 or inertia_z <= 0.0:
        raise ValueError("dynamic vehicle mass and inertia_z must be positive")

    a = np.zeros((2, 2))
    a[0, 0] = -(cf + cr) / (mass * v_x)
    a[0, 1] = (lr * cr - lf * cf) / (mass * v_x) - v_x
    a[1, 0] = (lr * cr - lf * cf) / (inertia_z * v_x)
    a[1, 1] = -(lf * lf * cf + lr * lr * cr) / (inertia_z * v_x)

    b = np.zeros((2, 1))
    b[0, 0] = cf / mass
    b[1, 0] = lf * cf / inertia_z
    return a, b


class DynamicBicycleBackend(VehicleBackend):
    """Linear-tire dynamic bicycle with a low-speed kinematic floor."""

    def __init__(self):
        self.v_y = 0.0
        self.r = 0.0

    def reset(self, initial_state, config):
        self.v_y = 0.0
        self.r = 0.0
        return initial_state

    def dynamics_terms(self, state, command, config):
        v_x = finite(getattr(state, "v", 0.0))
        beta = math.atan2(self.v_y, v_x)
        return float(self.r), float(beta)

    def step(self, state, command, config, dt):
        vehicle = config.vehicle
        command = limit_command(command, vehicle)
        dt = finite(dt)
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        speed = finite(state.v)
        if abs(speed) < KINEMATIC_SPEED_FLOOR:
            next_state, applied = update_state(state, command, vehicle, dt)
            self._sync_from_kinematic(next_state, applied.steer, vehicle)
            return next_state, applied

        next_state = self._dynamic_step(state, command, vehicle, dt)
        return next_state, command

    def _sync_from_kinematic(self, state, steer, vehicle):
        _, _, yaw_rate, beta = kinematic_derivatives(state, steer, vehicle)
        self.r = finite(yaw_rate)
        self.v_y = finite(state.v) * math.sin(finite(beta))

    def _dynamic_step(self, state, command, vehicle, dt):
        v_x = max(finite(state.v), KINEMATIC_SPEED_FLOOR)
        lateral = np.array([[finite(self.v_y)], [finite(self.r)]], dtype=float)
        a, b = _lateral_matrices(v_x, vehicle)
        nxt = bilinear_step(a, b, lateral, finite(command.steer), dt)
        self.v_y = float(nxt[0, 0])
        self.r = float(nxt[1, 0])

        yaw = finite(state.yaw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x_dot = v_x * cos_yaw - self.v_y * sin_yaw
        y_dot = v_x * sin_yaw + self.v_y * cos_yaw
        next_v = max(
            vehicle.min_speed,
            min(vehicle.max_speed, v_x + finite(command.acceleration) * dt),
        )
        return VehicleState(
            x=finite(state.x) + x_dot * dt,
            y=finite(state.y) + y_dot * dt,
            yaw=pi_to_pi(yaw + self.r * dt),
            v=next_v,
        )
