import math

from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid
from vdm_lab.student._controller_utils import (
    command_target_speed,
    remaining_path_distance,
    validated_wheelbase,
)


NAME = "Pure Pursuit Student"


def _finite(value, default):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def _validate_safety_config(config):
    vehicle = config.vehicle
    validated_wheelbase(vehicle)
    values = {
        "dt": config.sim.dt,
        "min_speed": vehicle.min_speed,
        "max_speed": vehicle.max_speed,
        "max_accel": vehicle.max_accel,
        "max_decel": vehicle.max_decel,
        "max_steer": vehicle.max_steer,
        "max_steer_rate": vehicle.max_steer_rate,
        "wheelbase": vehicle.wheelbase,
    }
    checked = {}
    for name, value in values.items():
        try:
            checked[name] = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(
                f"invalid safety configuration: {name} must be numeric"
            ) from exc
        if not math.isfinite(checked[name]):
            raise ValueError(
                f"invalid safety configuration: {name} must be finite"
            )
    if checked["dt"] <= 0.0:
        raise ValueError("invalid safety configuration: dt must be positive")
    if checked["min_speed"] < 0.0:
        raise ValueError("invalid safety configuration: min_speed cannot be negative")
    if checked["max_speed"] <= checked["min_speed"]:
        raise ValueError(
            "invalid safety configuration: max_speed must exceed min_speed"
        )
    if checked["max_accel"] <= 0.0 or checked["max_decel"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: acceleration limits must be positive"
        )
    if not 0.0 < checked["max_steer"] < 0.5 * math.pi:
        raise ValueError(
            "invalid safety configuration: max_steer must be in (0, pi/2)"
        )
    if checked["max_steer_rate"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: max_steer_rate must be positive"
        )
    if checked["wheelbase"] <= 0.0:
        raise ValueError(
            "invalid safety configuration: wheelbase must be positive"
        )


def _bounded_steer(desired, previous, config):
    """Apply both the steering-angle and per-sample steering-rate limits."""
    vehicle = config.vehicle
    max_steer = max(0.0, _finite(vehicle.max_steer, 0.0))
    max_rate = max(0.0, _finite(vehicle.max_steer_rate, 0.0))
    dt = max(0.0, _finite(config.sim.dt, 0.0))
    previous = clamp(_finite(previous, 0.0), -max_steer, max_steer)
    desired = clamp(_finite(desired, 0.0), -max_steer, max_steer)
    max_change = max_rate * dt
    return clamp(desired, previous - max_change, previous + max_change)


def _bounded_acceleration(desired, speed, config, speed_limit=None):
    """Respect acceleration limits without requesting reverse motion next step."""
    vehicle = config.vehicle
    max_accel = max(0.0, _finite(vehicle.max_accel, 0.0))
    max_decel = max(0.0, _finite(vehicle.max_decel, 0.0))
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    if speed_limit is None:
        cruise_limit = max_speed
    else:
        cruise_limit = min(max_speed, max(min_speed, _finite(speed_limit, max_speed)))
    dt = _finite(config.sim.dt, 0.0)
    speed = _finite(speed, math.nan)
    if not math.isfinite(speed):
        return -max_decel
    if speed < min_speed:
        return 0.0

    lower = -max_decel
    upper = max_accel
    if dt > 0.0:
        lower = max(lower, (min_speed - speed) / dt)
        upper = min(upper, (cruise_limit - speed) / dt)
    if lower > upper:
        return -max_decel
    return clamp(_finite(desired, -max_decel), lower, upper)


def control(state, reference, previous_control, config):
    _validate_safety_config(config)
    path = reference.path
    controller = config.controller
    vehicle = config.vehicle

    previous_steer = getattr(previous_control, "steer", 0.0)
    state_values = (state.x, state.y, state.yaw, state.v)
    min_speed = max(0.0, _finite(vehicle.min_speed, 0.0))
    max_speed = max(min_speed, _finite(vehicle.max_speed, min_speed))
    state_speed = _finite(state.v, math.nan)
    if (
        not all(
            math.isfinite(_finite(value, math.nan)) for value in state_values
        )
        or state_speed < min_speed - 1.0e-8
        or state_speed > max_speed + 1.0e-8
    ):
        return ControlCommand(
            acceleration=_bounded_acceleration(-vehicle.max_decel, state.v, config),
            steer=_bounded_steer(0.0, previous_steer, config),
        )
    if len(path.x) == 0 or len(path.y) == 0:
        return ControlCommand(
            acceleration=_bounded_acceleration(-vehicle.max_decel, state.v, config),
            steer=_bounded_steer(0.0, previous_steer, config),
        )

    speed = max(0.0, state_speed)
    base_lookahead = _finite(controller.pp_base_lookahead, math.nan)
    speed_gain = _finite(controller.pp_speed_gain, math.nan)
    waypoint_ds = _finite(config.sim.waypoint_ds, math.nan)
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
        return ControlCommand(
            acceleration=_bounded_acceleration(-vehicle.max_decel, state.v, config),
            steer=_bounded_steer(0.0, previous_steer, config),
        )
    lookahead = base_lookahead + speed_gain * speed
    lookahead = max(lookahead, waypoint_ds, 1.0e-3)

    last_index = min(len(path.x), len(path.y)) - 1
    try:
        target_index = int(reference.nearest_index)
    except (TypeError, ValueError, OverflowError):
        target_index = 0
    target_index = int(clamp(target_index, 0, last_index))
    state_x = _finite(state.x, path.x[target_index])
    state_y = _finite(state.y, path.y[target_index])
    while target_index < last_index:
        distance = math.hypot(
            _finite(path.x[target_index], state_x) - state_x,
            _finite(path.y[target_index], state_y) - state_y,
        )
        if distance >= lookahead:
            break
        target_index += 1

    target_x = _finite(path.x[target_index], math.nan)
    target_y = _finite(path.y[target_index], math.nan)
    if not all(math.isfinite(value) for value in (target_x, target_y)):
        return ControlCommand(
            acceleration=_bounded_acceleration(-vehicle.max_decel, state.v, config),
            steer=_bounded_steer(0.0, previous_steer, config),
        )
    yaw = _finite(state.yaw, 0.0)
    alpha = pi_to_pi(math.atan2(target_y - state_y, target_x - state_x) - yaw)
    desired_steer = math.atan2(
        2.0 * wheelbase * math.sin(alpha),
        lookahead,
    )
    steer = _bounded_steer(desired_steer, previous_steer, config)

    try:
        goal_distance = remaining_path_distance(path, reference.nearest_index)
    except ValueError:
        return ControlCommand(
            acceleration=_bounded_acceleration(-vehicle.max_decel, state.v, config),
            steer=_bounded_steer(0.0, previous_steer, config),
        )
    target_speed = command_target_speed(
        _finite(reference.target_speed, 0.0),
        reference,
        speed,
        vehicle,
    )
    acceleration = speed_pid(target_speed, speed, goal_distance, controller, vehicle)
    acceleration = _bounded_acceleration(
        acceleration, state.v, config, speed_limit=target_speed
    )
    return ControlCommand(acceleration=acceleration, steer=steer)
