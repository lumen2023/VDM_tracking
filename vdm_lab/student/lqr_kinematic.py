import math

import numpy as np

from vdm_lab.student._controller_utils import (
    finite,
    lqr_control,
    matrix_solve,
    validated_wheelbase,
)


NAME = "LQR Kinematic Student"

# The default is deliberately module-local so the shared ControllerConfig and
# any physical-backend configuration remain untouched.  Tests may override it
# on a ControllerConfig instance with ``lqr_kinematic_curvature_preview_time``.
DEFAULT_CURVATURE_PREVIEW_TIME = 0.25


def solve_lqr(A, B, Q, R, eps, max_iter):
    """Solve the discrete algebraic Riccati equation by fixed-point iteration."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)
    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be square")
    n = A.shape[0]
    if B.ndim != 2 or B.shape[0] != n:
        raise ValueError("B has an incompatible shape")
    m = B.shape[1]
    if Q.shape != (n, n) or R.shape != (m, m):
        raise ValueError("Q or R has an incompatible shape")
    if not all(np.all(np.isfinite(item)) for item in (A, B, Q, R)):
        raise FloatingPointError("LQR matrices must be finite")

    tolerance = max(0.0, finite(eps, 0.0))
    iterations = max(1, int(finite(max_iter, 1)))
    P = 0.5 * (Q + Q.T)
    for _ in range(iterations):
        gain = matrix_solve(R + B.T @ P @ B, B.T @ P @ A)
        P_next = A.T @ P @ A - A.T @ P @ B @ gain + Q
        P_next = 0.5 * (P_next + P_next.T)
        if not np.all(np.isfinite(P_next)):
            raise FloatingPointError("Riccati iteration produced a non-finite matrix")
        difference = float(np.max(np.abs(P_next - P)))
        P = P_next
        if difference <= tolerance:
            break

    K = matrix_solve(R + B.T @ P @ B, B.T @ P @ A)
    if not np.all(np.isfinite(K)):
        raise FloatingPointError("LQR gain is non-finite")
    return K


def build_kinematic_model(speed, config):
    dt = finite(config.sim.dt, 0.0)
    wheelbase = validated_wheelbase(config.vehicle)
    speed = max(0.0, finite(speed, 0.0))
    if dt <= 0.0 or wheelbase <= 0.0:
        raise ValueError("dt and wheelbase must be positive")

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = speed
    A[2, 2] = 1.0
    A[2, 3] = dt

    B = np.zeros((4, 1))
    B[3, 0] = speed / wheelbase
    return A, B


def _kinematic_feedforward(_speed, curvature, _gain, config):
    return math.atan(validated_wheelbase(config.vehicle) * curvature)


def control(state, reference, previous_control, config):
    return lqr_control(
        state,
        reference,
        previous_control,
        config,
        build_model=build_kinematic_model,
        solve=solve_lqr,
        feedforward=_kinematic_feedforward,
        preview_attr="lqr_kinematic_curvature_preview_time",
        default_preview=DEFAULT_CURVATURE_PREVIEW_TIME,
    )
