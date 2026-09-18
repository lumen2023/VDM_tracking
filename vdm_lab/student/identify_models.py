"""Model reconstruction using synthetic virtual-sensor experiments.

Training and holdout inputs/speeds are disjoint. Plant source remains untouched.
Absolute mass/inertia are treated as known: jointly scaling m, Iz, Cf and Cr
would otherwise leave the lateral equations invariant (non-identifiability).
"""
from __future__ import annotations
from dataclasses import asdict
from pathlib import Path
import math
import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares
from vdm_lab.common.types import LabConfig,VehicleState,ControlCommand
from vdm_lab.common.vehicle_backend import DynamicBicycleBackend,KinematicBicycleBackend
from vdm_lab.common.bicycle_model import tire_slip_angles
from vdm_lab.student.experiments import OUT,write_csv,write_json

SEED=20260917
DT=.02
DURATION=20.0


def steering(t,holdout=False):
    freqs=(.31,.83,1.37) if holdout else (.17,.61,1.03)
    phases=(.7,1.1,-.3) if holdout else (0,.2,-.1)
    return (1-np.exp(-t)) * sum(a*np.sin(2*np.pi*f*t+p) for a,f,p in zip((.025,.016,.01),freqs,phases))


def collect(speed,holdout=False,cf=None,cr=None,substep=.002):
    c=LabConfig();c.sim.dt=DT
    if cf is not None:c.vehicle.cf=cf
    if cr is not None:c.vehicle.cr=cr
    b=DynamicBicycleBackend(substep);state=b.reset(VehicleState(0,0,0,speed),c)
    times=np.arange(int(round(DURATION/DT))+1)*DT
    delta=steering(times[:-1],holdout)
    z=[];poses=[]
    for k,t in enumerate(times):
        z.append((b._v_y,b._r));poses.append((state.x,state.y,state.yaw))
        if k<len(delta):state,_=b.step(state,ControlCommand(0,float(delta[k])),c,DT)
    return dict(speed=speed,time=times,delta=delta,z=np.array(z),pose=np.array(poses),holdout=holdout)


def matrices(speed,cf,cr,c):
    v=c.vehicle;u=float(speed);m,I,Lf,Lr=v.mass,v.inertia_z,v.lf,v.lr
    A=np.array([[-(cf+cr)/(m*u),(-Lf*cf+Lr*cr)/(m*u)-u],
                [(-Lf*cf+Lr*cr)/(I*u),-(Lf*Lf*cf+Lr*Lr*cr)/(I*u)]])
    B=np.array([cf/m,Lf*cf/I])
    augmented=np.zeros((3,3));augmented[:2,:2]=A;augmented[:2,2]=B
    E=expm(augmented*DT)
    return E[:2,:2],E[:2,2]


def rollout_lateral(data,cf,cr,c):
    A,B=matrices(data['speed'],cf,cr,c)
    out=np.zeros_like(data['z'])
    for k,delta in enumerate(data['delta']):out[k+1]=A@out[k]+B*delta
    return out


def kinematic_pose(data,model,L,lr):
    u=data['speed'];delta=data['delta'];tan=np.tan(delta)
    # Compare at the SAME body longitudinal speed u, not CG speed magnitude V.
    # CG model: V=u/cos(beta). Therefore r=u*tan(delta)/L.
    beta=np.arctan(lr/L*tan) if model=='CG kinematic' else np.zeros_like(delta)
    vy=u*np.tan(beta);r=u*tan/L
    poses=np.zeros((len(delta)+1,3))
    for k in range(len(delta)):
        x,y,psi=poses[k];newpsi=psi+r[k]*DT
        if abs(r[k])<1e-9:
            dx=(u*math.cos(psi)-vy[k]*math.sin(psi))*DT
            dy=(u*math.sin(psi)+vy[k]*math.cos(psi))*DT
        else:
            ds=math.sin(newpsi)-math.sin(psi);dc=math.cos(newpsi)-math.cos(psi)
            dx=(u*ds+vy[k]*dc)/r[k];dy=(-u*dc+vy[k]*ds)/r[k]
        poses[k+1]=(x+dx,y+dy,newpsi)
    return np.c_[np.r_[vy,vy[-1]],np.r_[r,r[-1]]],poses


def main(out=OUT):
    out=Path(out)/'identification';out.mkdir(parents=True,exist_ok=True)
    rng=np.random.default_rng(SEED);c=LabConfig();cf0,cr0=c.vehicle.cf,c.vehicle.cr
    train=[collect(v) for v in (6.,9.)];test=[collect(v,True) for v in (4.,10.)]
    noise_sigma=np.array([.005,.0005]) # m/s and rad/s, assumed virtual sensor noise.
    observations=[d['z']+rng.normal(0,noise_sigma,d['z'].shape) for d in train]
    def residual(log_params,obs):
        cf,cr=np.exp(log_params)
        return np.concatenate([((rollout_lateral(d,cf,cr,c)-y)/noise_sigma).ravel() for d,y in zip(train,obs)])
    fit=least_squares(residual,np.log([110000.,190000.]),args=(observations,),
                      bounds=(np.log([30000.,30000.]),np.log([400000.,400000.])),
                      xtol=1e-11,ftol=1e-11,gtol=1e-9)
    cf,cr=np.exp(fit.x)
    covariance=np.linalg.inv(fit.jac.T@fit.jac)*(fit.fun@fit.fun/(len(fit.fun)-2))
    standard=np.sqrt(np.diag(covariance))
    intervals=np.exp(np.c_[fit.x-1.96*standard,fit.x+1.96*standard])
    # Kinematic virtual sensors use the unmodified core dynamics_terms.
    n=600;V=rng.uniform(2,10,n);delta=rng.uniform(-.15,.15,n);backend=KinematicBicycleBackend()
    true=np.array([backend.dynamics_terms(VehicleState(0,0,0,float(v)),ControlCommand(0,float(d)),c) for v,d in zip(V,delta)])
    kin_sigma=np.array([.001,.0005]);observed=true+rng.normal(0,kin_sigma,true.shape)
    def kres(p):
        L,rho=p;beta=np.arctan(rho*np.tan(delta));r=V/L*np.tan(delta)*np.cos(beta)
        return ((np.c_[r,beta]-observed)/kin_sigma).ravel()
    kfit=least_squares(kres,[2.9,.35],bounds=([1.,.05],[4.,.95]),xtol=1e-12,gtol=1e-10,ftol=1e-12)
    L,rho=kfit.x;lr=L*rho
    kcov=np.linalg.inv(kfit.jac.T@kfit.jac)*(kfit.fun@kfit.fun/(len(kfit.fun)-2))
    jac=np.array([[1.,0.],[rho,L]])
    transformed=jac@kcov@jac.T
    kse=np.sqrt(np.diag(transformed));kintervals=np.c_[[L,lr]-1.96*kse,[L,lr]+1.96*kse]
    # Held-out kinematic validation; different samples, no re-fitting.
    Vtest=rng.uniform(1.5,11,400);dtest=rng.uniform(-.18,.18,400)
    ktruth=np.array([backend.dynamics_terms(VehicleState(0,0,0,float(v)),ControlCommand(0,float(d)),c) for v,d in zip(Vtest,dtest)])
    btest=np.arctan(lr/L*np.tan(dtest));ktest=np.c_[Vtest/L*np.tan(dtest)*np.cos(btest),btest]
    params=[]
    for i,(name,truth,est,ci,unit) in enumerate([
        ('L',c.vehicle.wheelbase,L,kintervals[0],'m'),('lr',c.vehicle.lr,lr,kintervals[1],'m'),
        ('Cf',cf0,cf,intervals[0],'N/rad'),('Cr',cr0,cr,intervals[1],'N/rad')]):
        params.append(dict(parameter=name,true_value=truth,estimate=est,ci95_low=ci[0],ci95_high=ci[1],
                           relative_error_pct=100*(est-truth)/truth,unit=unit))
    comparison=[];holdout=[]
    for data in test:
        dyn=collect(data['speed'],True,cf,cr)
        for model in ('Simplified kinematic','CG kinematic','Identified dynamic'):
            z,pose=(dyn['z'],dyn['pose']) if model=='Identified dynamic' else kinematic_pose(data,model,L,lr)
            zerr=z-data['z'];pe=np.linalg.norm(pose[:,:2]-data['pose'][:,:2],axis=1)
            comparison.append(dict(model=model,speed_mps=data['speed'],yaw_rate_rmse_radps=float(np.sqrt(np.mean(zerr[:,1]**2))),
                                   lateral_velocity_rmse_mps=float(np.sqrt(np.mean(zerr[:,0]**2))),
                                   position_rmse_m=float(np.sqrt(np.mean(pe**2))),final_position_error_m=float(pe[-1])))
            for k,t in enumerate(data['time']):
                holdout.append(dict(model=model,speed_mps=data['speed'],time_s=t,yaw_rate_true=data['z'][k,1],yaw_rate_pred=z[k,1],
                                    lateral_velocity_true=data['z'][k,0],lateral_velocity_pred=z[k,0],
                                    x_true=data['pose'][k,0],y_true=data['pose'][k,1],x_pred=pose[k,0],y_pred=pose[k,1],position_error_m=pe[k]))
    all_data=[];max_slip=0.0
    for split,datasets in (('train',train),('test',test)):
        for i,d in enumerate(datasets):
            obs=observations[i] if split=='train' else d['z']
            for k,t in enumerate(d['time']):
                delta_k=d['delta'][min(k,len(d['delta'])-1)]
                af,ar=tire_slip_angles(d['speed'],d['z'][k,0],d['z'][k,1],delta_k,c.vehicle)
                max_slip=max(max_slip,abs(af),abs(ar))
                all_data.append(dict(split=split,speed_mps=d['speed'],time_s=t,steer_rad=delta_k,
                                     lateral_velocity_truth=d['z'][k,0],yaw_rate_truth=d['z'][k,1],
                                     lateral_velocity_observed=obs[k,0],yaw_rate_observed=obs[k,1]))
    # Integration refinement is held input/model, independent of controller dt.
    refinements=[]
    for h in (.004,.002,.001):
        d=collect(10.,True,substep=h);reference=collect(10.,True,substep=.0005)
        refinements.append(dict(substep_s=h,yaw_rate_rmse_radps=float(np.sqrt(np.mean((d['z'][:,1]-reference['z'][:,1])**2))),
                               position_rmse_m=float(np.sqrt(np.mean(np.sum((d['pose'][:,:2]-reference['pose'][:,:2])**2,axis=1))))))
    write_csv(out/'parameters.csv',params);write_csv(out/'holdout_metrics.csv',comparison)
    write_csv(out/'holdout_predictions.csv',holdout);write_csv(out/'excitation_data.csv',all_data)
    write_csv(out/'integration_refinement.csv',refinements)
    write_csv(out/'kinematic_sensors.csv',[dict(speed_mps=v,steer_rad=d,yaw_rate_observed=o[0],beta_observed=o[1],yaw_rate_truth=z[0],beta_truth=z[1])
                                          for v,d,o,z in zip(V,delta,observed,true)])
    write_csv(out/'kinematic_holdout.csv',[dict(speed_mps=v,steer_rad=d,yaw_rate_truth=t[0],beta_truth=t[1],yaw_rate_pred=q[0],beta_pred=q[1])
                                         for v,d,t,q in zip(Vtest,dtest,ktruth,ktest)])
    summary=dict(seed=SEED,dt=DT,duration_s=DURATION,training_speeds_mps=[6,9],test_speeds_mps=[4,10],
                 noise_std_lateral_velocity_mps=.005,noise_std_yaw_rate_radps=.0005,kinematic_noise_std_yaw_rate_radps=.001,
                 kinematic_noise_std_beta_rad=.0005,parameters=params,holdout_metrics=comparison,
                 max_tire_slip_deg=float(np.rad2deg(max_slip)),dynamic_fit_success=bool(fit.success),
                 kinematic_fit_success=bool(kfit.success),dynamic_jacobian_condition=float(np.linalg.cond(fit.jac)),
                 kinematic_holdout_yaw_rmse=float(np.sqrt(np.mean((ktest[:,0]-ktruth[:,0])**2))),
                 kinematic_holdout_beta_rmse=float(np.sqrt(np.mean((ktest[:,1]-ktruth[:,1])**2))),
                 caveat='Synthetic data generated by the supplied models, not real-vehicle validation. 95% intervals are local Gaussian/Jacobian approximations; known mass, inertia and axle distances for dynamic fit.',
                 speed_convention='All open-loop model comparisons use the same longitudinal speed u; CG kinematic V=u/cos(beta).')
    write_json(out/'identification_summary.json',summary)
    print('Reconstructed parameters:',params,flush=True)
    print('Holdout:',comparison,flush=True)
    return summary

if __name__=='__main__':main()
