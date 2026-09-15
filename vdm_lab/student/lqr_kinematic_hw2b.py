"""CG LQR with two independent errors [e_y, psi-psi_path+beta_ff].
Derivative errors in the template are not independent kinematic states.
Both A and B are discretized with exact zero-order hold.
"""
import math
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.geometry import clamp, pi_to_pi
from vdm_lab.common.vehicle import speed_pid

NAME = "CG Kinematic LQR (student)"

def solve_lqr(A,B,Q,R,eps=1e-8,max_iter=500):
    P=Q.copy()
    for _ in range(max_iter):
        K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
        new=Q+A.T@P@A-A.T@P@B@K
        new=(new+new.T)/2
        if np.max(np.abs(new-P))<eps:
            P=new
            break
        P=new
    else:
        P=solve_discrete_are(A,B,Q,R)
    return np.linalg.solve(R+B.T@P@B,B.T@P@A)

def curvature_steer(curvature,vehicle):
    k=clamp(curvature,-0.99/vehicle.lr,0.99/vehicle.lr)
    return math.atan(vehicle.wheelbase*k/math.sqrt(1-(vehicle.lr*k)**2))

def build_kinematic_model(speed,config,steer=0.):
    v=max(speed,config.controller.lqr_min_model_speed)
    p=config.vehicle
    h=p.lr/p.wheelbase
    t=math.tan(steer)
    beta=math.atan(h*t)
    db=h*(1+t*t)/(1+h*h*t*t)
    dr=v/p.wheelbase*((1+t*t)*math.cos(beta)-t*math.sin(beta)*db)
    Ac=np.array([[0.,v],[0.,0.]])
    Bc=np.array([[v*db],[dr]])
    A,B,_,_,_=cont2discrete((Ac,Bc,np.eye(2),np.zeros((2,1))),config.sim.dt)
    return A,B

def control(state,reference,previous_control,config):
    p,c=config.vehicle,config.controller
    ff=curvature_steer(reference.curvature,p)
    beta=math.atan(p.lr/p.wheelbase*math.tan(ff))
    A,B=build_kinematic_model(state.v,config,ff)
    K=solve_lqr(A,B,c.lqr_kinematic_q,c.lqr_kinematic_r,c.lqr_eps,c.lqr_max_iter)
    error=np.array([reference.lateral_error,pi_to_pi(reference.heading_error+beta)])
    steer=ff-float((K@error).item())
    goal=math.hypot(state.x-reference.path.x[-1],state.y-reference.path.y[-1])
    return ControlCommand(speed_pid(reference.target_speed,state.v,goal,c,p),clamp(steer,-p.max_steer,p.max_steer))
