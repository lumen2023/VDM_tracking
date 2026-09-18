import math

import numpy as np

from vdm_lab.student._controller_utils import bilinear_discrete, finite, lqr_control
from vdm_lab.student.lqr_kinematic import solve_lqr as _solve_lqr


NAME = "LQR Dynamic Student"

DEFAULT_CURVATURE_PREVIEW_TIME = 0.10


def solve_lqr(A, B, Q, R, eps, max_iter):
    """Keep the dynamic-controller API while sharing the robust DARE solver."""
    return _solve_lqr(A, B, Q, R, eps, max_iter)


def build_dynamic_model(speed, config):
    vehicle = config.vehicle
    controller = config.controller
    dt = finite(config.sim.dt, 0.0)
    minimum_speed = max(1.0e-3, finite(controller.lqr_min_model_speed, 0.5))
    v = max(finite(speed, minimum_speed), minimum_speed)
    mass = finite(vehicle.mass, 0.0)
    inertia_z = finite(vehicle.inertia_z, 0.0)
    lf = finite(vehicle.lf, 0.0)
    lr = finite(vehicle.lr, 0.0)
    cf = finite(vehicle.cf, 0.0)
    cr = finite(vehicle.cr, 0.0)
    if (
        dt <= 0.0
        or mass <= 0.0
        or inertia_z <= 0.0
        or lf <= 0.0
        or lr <= 0.0
        or cf <= 0.0
        or cr <= 0.0
    ):
        raise ValueError("dynamic vehicle parameters and dt must be positive")

    A_c = np.zeros((4, 4))
    A_c[0, 1] = 1.0
    A_c[1, 1] = -(cf + cr) / (mass * v)
    A_c[1, 2] = (cf + cr) / mass
    A_c[1, 3] = (lr * cr - lf * cf) / (mass * v)
    A_c[2, 3] = 1.0
    A_c[3, 1] = (lr * cr - lf * cf) / (inertia_z * v)
    A_c[3, 2] = (lf * cf - lr * cr) / inertia_z
    A_c[3, 3] = -(lf * lf * cf + lr * lr * cr) / (inertia_z * v)

    B_c = np.zeros((4, 1))
    B_c[1, 0] = cf / mass
    B_c[3, 0] = lf * cf / inertia_z

    # Bilinear (Tustin) discretization is stable for the stiff tire dynamics.
    return bilinear_discrete(A_c, B_c, dt)


def dynamic_feedforward(speed, curvature, K, config):
    vehicle = config.vehicle
    v = max(
        finite(speed, 0.0),
        max(1.0e-3, finite(config.controller.lqr_min_model_speed, 0.5)),
    )
    curvature = finite(curvature, 0.0)
    mass = finite(vehicle.mass, 0.0)
    lf = finite(vehicle.lf, 0.0)
    lr = finite(vehicle.lr, 0.0)
    cf = finite(vehicle.cf, 0.0)
    cr = finite(vehicle.cr, 0.0)
    wheelbase = lf + lr
    K = np.asarray(K, dtype=float)
    if (
        mass <= 0.0
        or lf <= 0.0
        or lr <= 0.0
        or cf <= 0.0
        or cr <= 0.0
        or wheelbase <= 0.0
        or K.ndim != 2
        or K.shape[0] < 1
        or K.shape[1] < 3
        or not np.all(np.isfinite(K))
    ):
        raise ValueError("invalid parameters for dynamic steering feedforward")

    # cf/cr are axle stiffnesses in build_dynamic_model, so no per-tire
    # factor of two belongs in the steady-state feedforward equations.
    understeer_gradient = (
        lr * mass / (cf * wheelbase)
        - lf * mass / (cr * wheelbase)
    )
    steady_yaw_error = (
        lr * curvature
        - lf * mass * v * v * curvature / (cr * wheelbase)
    )
    feedforward = (
        wheelbase * curvature
        + understeer_gradient * v * v * curvature
        - K[0, 2] * steady_yaw_error
    )
    if not math.isfinite(float(feedforward)):
        raise FloatingPointError("dynamic steering feedforward is non-finite")
    return float(feedforward)


def control(state, reference, previous_control, config):
    return lqr_control(
        state,
        reference,
        previous_control,
        config,
        build_model=build_dynamic_model,
        solve=solve_lqr,
        feedforward=dynamic_feedforward,
        preview_attr="lqr_dynamic_curvature_preview_time",
        default_preview=DEFAULT_CURVATURE_PREVIEW_TIME,
    )
