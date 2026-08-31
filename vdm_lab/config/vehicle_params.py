import numpy as np

from vdm_lab.common.types import VehicleConfig


# 学生调参入口：所有车辆几何、限幅和动力学参数集中在这里。
# 更换车辆时，优先新增一个参数组，不要在算法文件里直接写死数值。
VEHICLE_PARAMETER_SETS = {
    "student_car": {
        "wheelbase": 2.50,
        "width": 1.80,
        "front_overhang": 0.90,
        "rear_overhang": 0.80,
        "wheel_track": 1.50,
        "tire_radius": 0.32,
        "tire_width": 0.22,
        "max_steer": np.deg2rad(35.0),
        "max_steer_rate": np.deg2rad(45.0),
        "max_accel": 2.0,
        "max_decel": 3.5,
        "max_speed": 12.0,
        "min_speed": 0.0,
        "mass": 1140.0,
        "inertia_z": 1436.24,
        "lf": 1.25,
        "lr": 1.25,
        "cf": 155494.663,
        "cr": 155494.663,
    },
    "compact_ev": {
        "wheelbase": 2.70,
        "width": 1.86,
        "front_overhang": 0.88,
        "rear_overhang": 0.82,
        "wheel_track": 1.58,
        "tire_radius": 0.34,
        "tire_width": 0.24,
        "max_steer": np.deg2rad(32.0),
        "max_steer_rate": np.deg2rad(42.0),
        "max_accel": 2.4,
        "max_decel": 4.0,
        "max_speed": 14.0,
        "min_speed": 0.0,
        "mass": 1650.0,
        "inertia_z": 2450.0,
        "lf": 1.35,
        "lr": 1.35,
        "cf": 150000.0,
        "cr": 152000.0,
    },
}


DEFAULT_VEHICLE_NAME = "student_car"


def available_vehicle_names():
    return tuple(VEHICLE_PARAMETER_SETS.keys())


def make_vehicle_config(name=DEFAULT_VEHICLE_NAME):
    try:
        params = VEHICLE_PARAMETER_SETS[name]
    except KeyError as exc:
        choices = ", ".join(available_vehicle_names())
        raise ValueError(f"未知车辆参数组 {name}，可选值为: {choices}") from exc
    return VehicleConfig(**params)
