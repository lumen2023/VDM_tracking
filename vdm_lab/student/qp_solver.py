"""Portable condensed convex MPC QP, using SciPy's analytic-gradient SLSQP.

Same physical objective and constraints as the retained CVXPY formulation.
This is an optimization backend, NOT the controller's safe-stop fallback.
Only numpy/scipy are required. For larger horizons prefer CVXPY/OSQP.
"""
from __future__ import annotations
import time
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


def solve_condensed_mpc(z_ref, z_bar, z0, nominal_steer, applied_steer, config):
    from vdm_lab.student import mpc
    start = time.perf_counter()
    N = mpc._horizon(config)
    v, c = config.vehicle, config.controller
    z_ref = np.asarray(z_ref, dtype=float).copy()
    z_bar = np.asarray(z_bar, dtype=float).copy()
    z0 = np.asarray(z0, dtype=float).reshape(-1).copy()
    nominal_steer = mpc._nominal_steer_sequence(nominal_steer, N)
    if z_ref.shape != (4, N+1) or z_bar.shape != z_ref.shape or z0.shape != (4,):
        raise ValueError("MPC states and references have invalid shapes")
    if not all(np.all(np.isfinite(a)) for a in (z_ref, z_bar, z0)):
        raise ValueError("MPC states and references must be finite")
    origin = z0[:2].copy()
    z_ref[:2] -= origin[:, None]; z_bar[:2] -= origin[:, None]; z0[:2] -= origin
    q, qf, r, rd = [mpc._weight_matrix(getattr(c, k), n, k) for k, n in
                    (("mpc_q",4),("mpc_qf",4),("mpc_r",2),("mpc_rd",2))]
    # Interleaved decision w=[a0,d0,a1,d1,...]; z_k = d_k + F_k*w.
    F = [np.zeros((4, 2*N))]; d = [z0]
    H = np.zeros((2*N, 2*N)); g = np.zeros(2*N)
    for k in range(N):
        A, B, C = mpc.linear_model(z_bar[2,k], z_bar[3,k], nominal_steer[k], config)
        Fnext = A @ F[-1]; Fnext[:, 2*k:2*k+2] += B
        F.append(Fnext); d.append(A @ d[-1] + C)
        H[2*k:2*k+2,2*k:2*k+2] += r
        if k < N-1:
            D = np.zeros((2,2*N)); D[:,2*k:2*k+2] = -np.eye(2); D[:,2*k+2:2*k+4] = np.eye(2)
            H += D.T @ rd @ D
    for k in range(N+1):
        Q = qf if k == N else q
        H += F[k].T @ Q @ F[k]
        g += F[k].T @ Q @ (d[k]-z_ref[:,k])
    H = H + H.T; g *= 2.0
    # Speed and steering rate constraints, including rate from applied delta.
    S = np.vstack([F[k][2] for k in range(1,N+1)])
    speed_offset = np.array([d[k][2] for k in range(1,N+1)])
    D = np.zeros((N,2*N))
    for k in range(N):
        D[k,2*k+1] = 1.0
        if k: D[k,2*k-1] = -1.0
    rate = v.max_steer_rate * config.sim.dt
    lower_rate = np.full(N,-rate); upper_rate = np.full(N,rate)
    applied_steer = float(np.clip(applied_steer,-v.max_steer,v.max_steer))
    lower_rate[0] += applied_steer; upper_rate[0] += applied_steer
    M = np.vstack([S,D])
    lb = np.r_[v.min_speed-speed_offset,lower_rate]
    ub = np.r_[v.max_speed-speed_offset,upper_rate]
    lower = np.tile([-v.max_decel,-v.max_steer],N)
    upper = np.tile([v.max_accel,v.max_steer],N)
    a_guess = np.diff(z_bar[2]) / config.sim.dt
    a_guess, steer_guess = mpc._sanitize_controls(a_guess,nominal_steer,z0[2],applied_steer,config)
    w0 = np.column_stack([a_guess,steer_guess]).ravel()
    scale = np.tile([max(v.max_accel,v.max_decel,1.0),max(v.max_steer,1e-3)],N)
    Hs = H * scale[:,None] * scale[None,:]; gs = g * scale; Ms = M * scale[None,:]
    result = minimize(lambda w: 0.5*w @ Hs @ w + gs @ w, w0/scale,
                      jac=lambda w: Hs @ w + gs, method="SLSQP",
                      bounds=Bounds(lower/scale,upper/scale),
                      constraints=[LinearConstraint(Ms,lb,ub)],
                      options={"ftol":1e-9,"maxiter":160,"disp":False})
    w = np.asarray(result.x)*scale
    violation = max(float(np.max(lb-M@w)),float(np.max(M@w-ub)),
                    float(np.max(lower-w)),float(np.max(w-upper)),0.0)
    elapsed = 1000*(time.perf_counter()-start)
    if not result.success or violation > 1e-6 or not np.all(np.isfinite(w)):
        raise mpc.MPCSolverError(f"SciPy QP: {result.message}; violation={violation:.3g}",
                                status=f"scipy_{result.status}",iterations=result.nit,solve_time_ms=elapsed)
    z = np.column_stack([d[k] + F[k]@w for k in range(N+1)])
    z[:2] += origin[:,None]
    u = w.reshape(N,2).T
    return u[0],u[1],z,mpc.MPCSolveInfo("scipy_optimal",result.nit,elapsed)
