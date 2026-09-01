from dataclasses import dataclass, field

import numpy as np


@dataclass
class VehicleConfig:
    wheelbase: float = 2.5
    width: float = 1.8
    front_overhang: float = 0.9
    rear_overhang: float = 0.8
    wheel_track: float = 1.5
    tire_radius: float = 0.32
    tire_width: float = 0.22
    max_steer: float = np.deg2rad(35.0)
    max_steer_rate: float = np.deg2rad(45.0)
    max_accel: float = 2.0
    max_decel: float = 3.5
    max_speed: float = 12.0
    min_speed: float = 0.0
    mass: float = 1140.0
    inertia_z: float = 1436.24
    lf: float = 1.25
    lr: float = 1.25
    cf: float = 155494.663
    cr: float = 155494.663


@dataclass
class SimulationConfig:
    dt: float = 0.1
    max_time: float = 90.0
    target_speed: float | None = None
    speed_mode: str = "low"
    stop_distance: float = 1.5
    stop_speed: float = 0.5
    waypoint_ds: float = 0.5
    route_name: str = "mixed_course"
    animate: bool = False
    show_history_ghosts: bool = True
    history_ghost_stride: int = 12
    history_ghost_count: int = 0


@dataclass
class ControllerConfig:
    kp_speed: float = 1.1
    pp_base_lookahead: float = 3.0
    pp_speed_gain: float = 0.35
    lqr_q: np.ndarray = field(default_factory=lambda: np.diag([2.0, 0.2, 3.0, 0.2]))
    lqr_r: np.ndarray = field(default_factory=lambda: np.diag([1.5]))
    lqr_eps: float = 1.0e-4
    lqr_max_iter: int = 200
    lqr_min_model_speed: float = 0.5
    mpc_horizon: int = 8
    mpc_q: np.ndarray = field(default_factory=lambda: np.diag([2.0, 2.0, 0.6, 1.0]))
    mpc_qf: np.ndarray = field(default_factory=lambda: np.diag([4.0, 4.0, 1.0, 2.0]))
    mpc_r: np.ndarray = field(default_factory=lambda: np.diag([0.2, 0.4]))
    mpc_rd: np.ndarray = field(default_factory=lambda: np.diag([0.2, 0.8]))
    mpc_iter_max: int = 5
    mpc_du_threshold: float = 0.05


@dataclass
class LabConfig:
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    sim: SimulationConfig = field(default_factory=SimulationConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)


@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float
    v: float


@dataclass
class ControlCommand:
    acceleration: float
    steer: float


@dataclass
class Path:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    s: np.ndarray
    target_speed: np.ndarray


@dataclass
class ControllerReference:
    path: Path
    nearest_index: int
    lateral_error: float
    heading_error: float
    curvature: float
    target_speed: float


@dataclass
class StepRecord:
    time: float
    x: float
    y: float
    yaw: float
    speed: float
    acceleration: float
    steer: float
    beta: float
    yaw_rate: float
    target_index: int
    lateral_error: float
    heading_error: float
    curvature: float
    normal_accel: float
    target_speed: float
