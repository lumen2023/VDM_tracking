"""CG pure pursuit: tan(delta)=2L sin(gamma)/(d+2lr cos(gamma)).
Derived by substituting beta=atan(lr*tan(delta)/L) in the pursuit circle.
Unlike the rear-axle formula, this matches the simulator's CG convention.
"""
import math
import numpy as np
from vdm_lab.common.types import ControlCommand
from vdm_lab.common.vehicle import speed_pid
from vdm_lab.common.geometry import clamp, pi_to_pi

NAME = "CG Pure Pursuit (student)"

def target_point(state, reference, lookahead):
    """First forward intersection of path segments and lookahead circle."""
    path = reference.path
    origin = np.array([state.x, state.y])
    for i in range(reference.nearest_index, len(path.x)-1):
        p = np.array([path.x[i],path.y[i]])
        h = np.array([path.x[i+1],path.y[i+1]])-p
        w = p-origin
        a = float(h@h)
        if a < 1e-12: continue
        b,c = 2*float(w@h),float(w@w)-lookahead**2
        disc = b*b-4*a*c
        if disc >= 0:
            for u in sorted(((-b-math.sqrt(disc))/(2*a),(-b+math.sqrt(disc))/(2*a))):
                if 0 <= u <= 1: return p+u*h
    # Continue the terminal tangent for steering while the speed loop stops.
    # A target behind the CG otherwise produces a spurious full-lock command.
    end=np.array([path.x[-1],path.y[-1]])
    return end+lookahead*np.array([math.cos(path.yaw[-1]),math.sin(path.yaw[-1])])

def control(state, reference, previous_control, config):
    v,c = config.vehicle,config.controller
    target = target_point(state,reference,max(0.5,c.pp_base_lookahead+c.pp_speed_gain*state.v))
    dx,dy = target-[state.x,state.y]
    d = max(math.hypot(dx,dy),1e-6)
    gamma = pi_to_pi(math.atan2(dy,dx)-state.yaw)
    steer = math.atan2(2*v.wheelbase*math.sin(gamma),d+2*v.lr*math.cos(gamma))
    goal = math.hypot(state.x-reference.path.x[-1],state.y-reference.path.y[-1])
    return ControlCommand(speed_pid(reference.target_speed,state.v,goal,c,v),clamp(steer,-v.max_steer,v.max_steer))
