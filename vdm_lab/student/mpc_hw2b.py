"""Iterative linear MPC using the SAME CG model as the simulation plant.
State [X,Y,v,psi], input [acceleration,steer]. Cached parametric convex QP;
exact Jacobians and affine residual, actual first-step slew constraint,
asymmetric acceleration bounds, yaw unwrapping and shifted warm starts.
"""
import math
import numpy as np
import cvxpy as cp
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.geometry import pi_to_pi,clamp
from vdm_lab.student.lqr_kinematic import curvature_steer

NAME="CG Linear MPC (student)"
_solver=None
_warm=None
diagnostics={}

def reset():
    global _solver,_warm,diagnostics
    _solver=None
    _warm=None
    diagnostics={"qp_solves":0,"failures":0,"solver_retries":0,"max_constraint_violation":0.0}

def nearest_horizon_reference(state,reference,config):
    p=reference.path
    N=config.controller.mpc_horizon
    # Use actual polyline arc length, not the spline's chord parameter.
    arc=np.r_[0.,np.cumsum(np.hypot(np.diff(p.x),np.diff(p.y)))]
    idx=reference.nearest_index
    s=arc[idx]
    if idx<len(arc)-1:
        h=np.array([p.x[idx+1]-p.x[idx],p.y[idx+1]-p.y[idx]])
        u=np.clip(np.dot(np.array([state.x-p.x[idx],state.y-p.y[idx]]),h)/max(h@h,1e-12),0,1)
        s+=u*(arc[idx+1]-arc[idx])
    unwrapped=np.unwrap(p.yaw)
    z=np.zeros((4,N+1))
    yaw_prev=state.yaw
    for i in range(N+1):
        vr=float(np.interp(s,arc,p.target_speed))
        k=float(np.interp(s,arc,p.curvature))
        delta=curvature_steer(k,config.vehicle)
        beta=math.atan(config.vehicle.lr/config.vehicle.wheelbase*math.tan(delta))
        heading=float(np.interp(s,arc,unwrapped))-beta
        heading=yaw_prev+pi_to_pi(heading-yaw_prev)
        z[:,i]=[np.interp(s,arc,p.x),np.interp(s,arc,p.y),vr,heading]
        yaw_prev=heading
        s=min(arc[-1],s+vr*config.sim.dt)
    return z

def update_kinematic_array(z,acceleration,steer,config):
    x,y,v,yaw=z
    p=config.vehicle
    dt=config.sim.dt
    beta=math.atan(p.lr/p.wheelbase*math.tan(steer))
    return np.array([x+dt*v*math.cos(yaw+beta),y+dt*v*math.sin(yaw+beta),
                     v+dt*acceleration,yaw+dt*v/p.wheelbase*math.tan(steer)*math.cos(beta)])

def linear_model(v,yaw,steer,config):
    p=config.vehicle
    dt=config.sim.dt
    h=p.lr/p.wheelbase
    t=math.tan(steer)
    beta=math.atan(h*t)
    db=h*(1+t*t)/(1+h*h*t*t)
    theta=yaw+beta
    rate=t*math.cos(beta)/p.wheelbase
    dr=((1+t*t)*math.cos(beta)-t*math.sin(beta)*db)/p.wheelbase
    A=np.eye(4)
    A[0,2]=dt*math.cos(theta); A[0,3]=-dt*v*math.sin(theta)
    A[1,2]=dt*math.sin(theta); A[1,3]=dt*v*math.cos(theta)
    A[3,2]=dt*rate
    B=np.zeros((4,2))
    B[0,1]=-dt*v*math.sin(theta)*db
    B[1,1]=dt*v*math.cos(theta)*db
    B[2,0]=dt
    B[3,1]=dt*v*dr
    z=np.array([0.,0.,v,yaw]); u=np.array([0.,steer])
    C=update_kinematic_array(z,*u,config)-A@z-B@u
    return A,B,C

def predict_motion(z0,acceleration,steer,z_ref,config):
    z=np.zeros_like(z_ref); z[:,0]=z0
    for i in range(config.controller.mpc_horizon):
        z[:,i+1]=update_kinematic_array(z[:,i],acceleration[i],steer[i],config)
    return z

class _QP:
    def __init__(self,config):
        N=config.controller.mpc_horizon
        c,p=config.controller,config.vehicle
        self.z=cp.Variable((4,N+1)); self.u=cp.Variable((2,N))
        self.z0=cp.Parameter(4); self.ref=cp.Parameter((4,N+1)); self.last=cp.Parameter(2)
        self.A=[cp.Parameter((4,4)) for _ in range(N)]
        self.B=[cp.Parameter((4,2)) for _ in range(N)]
        self.C=[cp.Parameter(4) for _ in range(N)]
        con=[self.z[:,0]==self.z0,self.z[2,:]>=p.min_speed,self.z[2,:]<=p.max_speed,
             self.u[0,:]>=-p.max_decel,self.u[0,:]<=p.max_accel,cp.abs(self.u[1,:])<=p.max_steer]
        objective=0
        for i in range(N):
            du=self.u[:,i]-(self.last if i==0 else self.u[:,i-1])
            objective+=cp.quad_form(self.z[:,i]-self.ref[:,i],c.mpc_q)
            objective+=cp.quad_form(self.u[:,i],c.mpc_r)+cp.quad_form(du,c.mpc_rd)
            con += [self.z[:,i+1]==self.A[i]@self.z[:,i]+self.B[i]@self.u[:,i]+self.C[i],
                    cp.abs(du[1])<=p.max_steer_rate*config.sim.dt]
        objective+=cp.quad_form(self.z[:,N]-self.ref[:,N],c.mpc_qf)
        self.problem=cp.Problem(cp.Minimize(objective),con)
        self.constraints=con

def solve_linear_mpc(z_ref,z_bar,z0,previous_steer,config,previous_control=None):
    global _solver
    if _solver is None: _solver=_QP(config)
    s=_solver
    s.ref.value=z_ref; s.z0.value=z0
    s.last.value=np.array([previous_control.acceleration,previous_control.steer]) if previous_control is not None else np.array([0.,previous_steer[0]])
    for i in range(config.controller.mpc_horizon):
        A,B,C=linear_model(z_bar[2,i],z_bar[3,i],previous_steer[i],config)
        s.A[i].value=A; s.B[i].value=B; s.C[i].value=C
    try:
        s.problem.solve(solver=cp.OSQP,warm_start=True,verbose=False,eps_abs=1e-5,eps_rel=1e-5,max_iter=20000,polishing=True)
        diagnostics['qp_solves']=diagnostics.get('qp_solves',0)+1
        if s.problem.status not in (cp.OPTIMAL,cp.OPTIMAL_INACCURATE) or max(float(np.max(c.violation())) for c in s.constraints)>2e-3:
            # Retry the identical convex problem, not a different controller.
            diagnostics['solver_retries']=diagnostics.get('solver_retries',0)+1
            s.problem.solve(solver=cp.CLARABEL,verbose=False)
        if s.problem.status not in (cp.OPTIMAL,cp.OPTIMAL_INACCURATE):
            raise RuntimeError('MPC status: '+str(s.problem.status))
        violation=max(float(np.max(c.violation())) for c in s.constraints)
        diagnostics['max_constraint_violation']=max(diagnostics.get('max_constraint_violation',0),violation)
        if violation>2e-3: raise RuntimeError('MPC constraints violated: '+str(violation))
    except Exception:
        diagnostics['failures']=diagnostics.get('failures',0)+1
        raise  # Never silently label a fallback as a successful MPC solve.
    return s.u.value[0].copy(),s.u.value[1].copy(),s.z.value.copy()

def control(state,reference,previous_control,config):
    global _warm
    z_ref=nearest_horizon_reference(state,reference,config)
    z0=np.array([state.x,state.y,state.v,state.yaw])
    N=config.controller.mpc_horizon
    if _warm is None:
        a=np.full(N,previous_control.acceleration); d=np.full(N,previous_control.steer)
    else:
        a=np.r_[_warm[0][1:],_warm[0][-1]]; d=np.r_[_warm[1][1:],_warm[1][-1]]
    for _ in range(config.controller.mpc_iter_max):
        bar=predict_motion(z0,a,d,z_ref,config)
        old=np.r_[a,d]
        a,d,prediction=solve_linear_mpc(z_ref,bar,z0,d,config,previous_control)
        if np.max(np.abs(np.r_[a,d]-old))<config.controller.mpc_du_threshold: break
    _warm=(a,d)
    p=config.vehicle
    delta=clamp(d[0],previous_control.steer-p.max_steer_rate*config.sim.dt,previous_control.steer+p.max_steer_rate*config.sim.dt)
    command=ControlCommand(float(clamp(a[0],-p.max_decel,p.max_accel)),float(clamp(delta,-p.max_steer,p.max_steer)))
    command.prediction=prediction
    return command
