import importlib
import math

from vdm_lab.common.bicycle_model import kinematic_derivatives, normal_acceleration
from vdm_lab.common.path import generate_reference_path
from vdm_lab.common.reference import ReferenceTracker, distance_to_goal
from vdm_lab.common.types import ControlCommand, LabConfig, StepRecord, VehicleState
from vdm_lab.common.vehicle import limit_command, update_state


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


def run_simulation(controller_module, config=None, animate=False, gif_path=None):
    config = LabConfig() if config is None else config
    config.sim.animate = animate
    path = generate_reference_path(
        route_name=config.sim.route_name,
        speed_mode=config.sim.speed_mode,
        ds=config.sim.waypoint_ds,
        target_speed=config.sim.target_speed,
    )
    tracker = ReferenceTracker(path)
    state = VehicleState(x=float(path.x[0]), y=float(path.y[0]), yaw=float(path.yaw[0]), v=0.0)
    previous_control = ControlCommand(acceleration=0.0, steer=0.0)

    records = []
    predictions = []
    renderer = None
    if animate:
        from vdm_lab.common.visualization import LiveRenderer

        renderer = LiveRenderer(
            path,
            config.vehicle,
            title=getattr(controller_module, "NAME", "VDM Lab"),
            gif_path=gif_path,
            show_history_ghosts=config.sim.show_history_ghosts,
            ghost_stride=config.sim.history_ghost_stride,
            ghost_count=config.sim.history_ghost_count,
        )

    time = 0.0
    while time <= config.sim.max_time:
        reference = tracker.nearest(state)
        dist_goal = distance_to_goal(state, path)
        command = controller_module.control(state, reference, previous_control, config)
        command = limit_command(command, config.vehicle)

        prediction = getattr(command, "prediction", None)
        if prediction is not None:
            predictions.append((time, prediction))

        _, _, yaw_rate, beta = kinematic_derivatives(state, command.steer, config.vehicle)
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

        state, previous_control = update_state(state, command, config.vehicle, config.sim.dt)
        time += config.sim.dt

        if not math.isfinite(state.x + state.y + state.yaw + state.v):
            raise FloatingPointError("车辆状态出现非有限数，请检查控制器公式或参数。")

    if renderer is not None:
        renderer.finish()
    return path, records, predictions
