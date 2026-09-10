import importlib
import math

import numpy as np

from vdm_lab.common.bicycle_model import normal_acceleration
from vdm_lab.common.basemap import load_basemap
from vdm_lab.common.gpx import generate_gpx_path, resolve_gpx_target_speed
from vdm_lab.common.path import generate_reference_path
from vdm_lab.common.reference import ReferenceTracker, distance_to_goal
from vdm_lab.common.types import ControlCommand, LabConfig, StepRecord, VehicleState
from vdm_lab.common.vehicle import limit_command
from vdm_lab.common.vehicle_backend import KinematicBicycleBackend


ALGORITHM_MODULES = {
    "pp": "pure_pursuit",
    "lqr_kinematic": "lqr_kinematic",
    "lqr_dynamic": "lqr_dynamic",
    "mpc": "mpc",
}


def load_controller(algo, version):
    if algo not in ALGORITHM_MODULES:
        raise ValueError(f"未知算法 {algo}，可选值为: {', '.join(ALGORITHM_MODULES)}")
    if version not in {"solution", "student"}:
        raise ValueError("version 只能是 solution 或 student")
    package = "solutions" if version == "solution" else "student"
    return importlib.import_module(f"vdm_lab.{package}.{ALGORITHM_MODULES[algo]}")


def build_reference_path(config):
    """
    Build one Path regardless of route source.

    Priority:
        config.sim.gpx_file
        -> built-in route_name
    """
    if config.sim.gpx_file:
        target_speed = resolve_gpx_target_speed(
            config.sim.speed_mode,
            config.sim.target_speed,
        )
        return generate_gpx_path(
            config.sim.gpx_file,
            ds=config.sim.waypoint_ds,
            target_speed=target_speed,
            gap_warning_m=config.sim.gpx_gap_warning_m,
            origin_lat=config.sim.coordinate_origin_lat,
            origin_lon=config.sim.coordinate_origin_lon,
        )

    return generate_reference_path(
        route_name=config.sim.route_name,
        speed_mode=config.sim.speed_mode,
        ds=config.sim.waypoint_ds,
        target_speed=config.sim.target_speed,
    )


def _resolve_max_time(config, path):
    max_time = float(config.sim.max_time)

    if not config.sim.gpx_file or not config.sim.auto_extend_gpx_time:
        return max_time

    positive_speed = path.target_speed[path.target_speed > 0.1]
    if len(positive_speed) == 0:
        return max_time

    cruise = max(float(np.max(positive_speed)), 0.5)

    # Nominal driving time + generous allowance for acceleration,
    # steering transients and final stopping.
    estimated = float(path.s[-1]) / cruise + 60.0
    return max(max_time, estimated)


def run_simulation(
    controller_module,
    config=None,
    animate=False,
    gif_path=None,
    vehicle_backend=None,
    path_override=None,
):
    """
    Run one closed-loop tracking experiment.

    New extension points
    --------------------
    path_override:
        Directly inject an already-built Path.

    vehicle_backend:
        Inject another vehicle/simulator backend. If omitted, behavior is
        identical to the repository's original kinematic bicycle simulation.
    """
    config = LabConfig() if config is None else config
    config.sim.animate = animate

    path = path_override if path_override is not None else build_reference_path(config)
    tracker = ReferenceTracker(path)

    state = VehicleState(
        x=float(path.x[0]),
        y=float(path.y[0]),
        yaw=float(path.yaw[0]),
        v=0.0,
    )
    previous_control = ControlCommand(acceleration=0.0, steer=0.0)

    backend = KinematicBicycleBackend() if vehicle_backend is None else vehicle_backend
    state = backend.reset(state, config)

    records = []
    predictions = []
    renderer = None

    if animate:
        from vdm_lab.common.visualization import LiveRenderer

        basemap = load_basemap(path, config.sim)
        renderer = LiveRenderer(
            path,
            config.vehicle,
            title=getattr(controller_module, "NAME", "VDM Lab"),
            gif_path=gif_path,
            show_history_ghosts=config.sim.show_history_ghosts,
            ghost_stride=config.sim.history_ghost_stride,
            ghost_count=config.sim.history_ghost_count,
            view_mode=config.sim.view_mode,
            follow_radius=config.sim.follow_radius,
            basemap=basemap,
            basemap_opacity=config.sim.basemap_opacity,
        )

    time = 0.0
    max_time = _resolve_max_time(config, path)

    while time <= max_time:
        reference = tracker.nearest(state)
        dist_goal = distance_to_goal(state, path)

        command = controller_module.control(
            state,
            reference,
            previous_control,
            config,
        )
        command = limit_command(command, config.vehicle)

        prediction = getattr(command, "prediction", None)
        if prediction is not None:
            predictions.append((time, prediction))

        yaw_rate, beta = backend.dynamics_terms(state, command, config)

        records.append(
            StepRecord(
                time=time,
                x=state.x,
                y=state.y,
                yaw=state.yaw,
                speed=state.v,
                acceleration=command.acceleration,
                steer=command.steer,
                beta=beta,
                yaw_rate=yaw_rate,
                target_index=reference.nearest_index,
                lateral_error=reference.lateral_error,
                heading_error=reference.heading_error,
                curvature=reference.curvature,
                normal_accel=normal_acceleration(state.v, reference.curvature),
                target_speed=reference.target_speed,
            )
        )

        if renderer is not None:
            renderer.draw(state, records, reference, command, prediction)

        if dist_goal < config.sim.stop_distance and state.v < config.sim.stop_speed:
            break

        state, previous_control = backend.step(
            state,
            command,
            config,
            config.sim.dt,
        )
        time += config.sim.dt

        if not math.isfinite(state.x + state.y + state.yaw + state.v):
            raise FloatingPointError("车辆状态出现非有限数，请检查控制器公式或参数。")

    if renderer is not None:
        renderer.finish()

    return path, records, predictions
