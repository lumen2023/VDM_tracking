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

from abc import ABC, abstractmethod

from vdm_lab.common.bicycle_model import kinematic_derivatives
from vdm_lab.common.types import ControlCommand, VehicleState
from vdm_lab.common.vehicle import update_state


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
        )


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
