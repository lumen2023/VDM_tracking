"""Reproducible student-only research harness; never edits shared lab code.

Run from the project root:
    python -m vdm_lab.student.experiments --suite full
Results include every case (including failures), exact configuration, raw
state/input/solver logs, physical arc-length references and derived metrics.
"""
from __future__ import annotations
import argparse
import copy
import csv
from dataclasses import asdict, dataclass
import hashlib
import importlib
import json
import math
import platform
from pathlib import Path as FSPath
import sys
import time
import warnings
import numpy as np
from vdm_lab.common.simulation import build_reference_path, load_controller
from vdm_lab.common.reference import ReferenceTracker
from vdm_lab.common.types import LabConfig, VehicleState, ControlCommand
from vdm_lab.common.vehicle import limit_command
from vdm_lab.common.vehicle_backend import KinematicBicycleBackend, SimplifiedKinematicBackend, DynamicBicycleBackend
from vdm_lab.common.bicycle_model import dynamic_lateral_accel, tire_slip_angles
from vdm_lab.student._controller_utils import geometric_arclength, cg_steady_steer

ROOT = FSPath(__file__).resolve().parents[3]
OUT = FSPath(__file__).resolve().parents[1] / "results"
ALGORITHMS = ("pp", "lqr_kinematic", "lqr_dynamic", "mpc")


def jsonable(obj):
    if isinstance(obj, dict): return {str(k):jsonable(v) for k,v in obj.items()}
    if isinstance(obj, (list,tuple)): return [jsonable(v) for v in obj]
    if isinstance(obj,np.ndarray): return obj.tolist()
    if isinstance(obj,np.generic): return jsonable(obj.item())
    if isinstance(obj,float) and not math.isfinite(obj): return None
    if isinstance(obj,FSPath): return str(obj)
    return obj


def write_json(path, obj):
    path=FSPath(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(jsonable(obj),indent=2,allow_nan=False),encoding="utf-8")


def write_csv(path, rows):
    path=FSPath(path);path.parent.mkdir(parents=True,exist_ok=True)
    if not rows: return
    keys=list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


@dataclass
class Case:
    case_id: str
    group: str
    algo: str="pp"
    route: str="s_curve"
    speed: float=7.0
    plant: str="kinematic"
    dt: float=0.1
    lookahead: float=3.0
    max_steer_deg: float=35.0
    max_steer_rate_deg_s: float=45.0
    stiffness_scale: float=1.0
    controller_stiffness_scale: float=1.0
    mpc_horizon: int=8
    mpc_cg_heading: bool=True
    baseline: bool=False
    measured_dynamic_states: bool=True
    substep: float=0.002
    gpx: bool=False
    max_time: float=90.0


def config_for(case):
    c=LabConfig()
    c.sim.route_name=case.route;c.sim.target_speed=case.speed;c.sim.dt=case.dt
    c.sim.max_time=case.max_time
    c.vehicle.max_steer=np.deg2rad(case.max_steer_deg)
    c.vehicle.max_steer_rate=np.deg2rad(case.max_steer_rate_deg_s)
    c.controller.pp_base_lookahead=case.lookahead
    c.controller.mpc_horizon=case.mpc_horizon
    c.controller.mpc_solver="scipy" # Explicitly records the executed solver.
    c.controller.mpc_cg_heading=case.mpc_cg_heading
    if case.gpx:
        c.sim.gpx_file=str(ROOT/"data/gpx/homework_route_1.gpx")
        c.sim.coordinate_origin_lon=118.8145;c.sim.coordinate_origin_lat=31.8885
        c.sim.waypoint_ds=1.0
    plant_config=copy.deepcopy(c)
    plant_config.vehicle.cf*=case.stiffness_scale;plant_config.vehicle.cr*=case.stiffness_scale
    c.vehicle.cf*=case.controller_stiffness_scale;c.vehicle.cr*=case.controller_stiffness_scale
    return c,plant_config


def prepared_path(config):
    """Keep core route geometry, repair only the injected Path's distance field."""
    path=build_reference_path(config)
    path.s=geometric_arclength(path)
    remaining=path.s[-1]-path.s
    path.target_speed=np.minimum(float(config.sim.target_speed),np.sqrt(1.8*remaining))
    path.target_speed[-1]=0.0
    return path


def local_projection(path,state,index):
    """Local point-to-polyline projection; independent of waypoint spacing."""
    best=None
    for j in range(max(0,index-2),min(len(path.x)-1,index+3)):
        dx=float(path.x[j+1]-path.x[j]);dy=float(path.y[j+1]-path.y[j]);n2=dx*dx+dy*dy
        if n2<=1e-18:continue
        f=float(np.clip(((state.x-path.x[j])*dx+(state.y-path.y[j])*dy)/n2,0,1))
        px=float(path.x[j]+f*dx);py=float(path.y[j]+f*dy)
        ex,ey=state.x-px,state.y-py;d2=ex*ex+ey*ey
        if best is None or d2<best[0]:
            cross=dx*ey-dy*ex
            signed=cross/math.sqrt(n2) # Do not count endpoint longitudinal overshoot as lateral error.
            best=(d2,signed,float(path.s[j]+f*(path.s[j+1]-path.s[j])),px,py,j,f)
    if best is None:
        return 0.0,float(path.s[index]),float(path.x[index]),float(path.y[index]),index,0.0
    return best[1:]


def analyze(case,path,rows,config,termination):
    def a(key):return np.array([r[key] for r in rows],dtype=float)
    t=a("time");V=a("ground_speed");r=a("yaw_rate");beta=a("beta")
    # Kinematic beta changes instantaneously with steering in the ideal plant.
    # This is a sampled course-normal estimate, not a measured accelerometer.
    if case.plant!="dynamic":
        betadot=np.gradient(np.unwrap(beta),t) if len(t)>1 else np.zeros(len(t))
        normal=V*(r+betadot)
        for row, value in zip(rows,normal):row["course_normal_accel"]=float(value)
    error=a("projection_error");traditional=a("lateral_error")
    abs_err=np.abs(error);deltas=a("steer");rates=np.diff(np.r_[0.0,deltas])/case.dt
    ds=np.maximum(np.diff(a("progress_s")),0.0)
    distance_rmse=math.sqrt(float(np.sum(ds*(error[:-1]**2+error[1:]**2)/2)/np.sum(ds))) if np.sum(ds)>0 else float("nan")
    last=rows[-1];goal=math.hypot(last["x"]-path.x[-1],last["y"]-path.y[-1])
    peak=int(np.argmax(abs_err));kmax=int(np.argmax(np.abs(traditional)))
    active=V>=1.0
    metrics=asdict(case)|{
        "reached_goal":termination=="goal", "termination":termination,
        "tracking_pass_1m":bool(termination=="goal" and np.max(abs_err)<=1.0),
        "tracking_gate_definition":"analyst diagnostic: endpoint reached AND maximum absolute segment-normal error <= 1 m; not a course or safety standard",
        "simulated_duration_s":float(t[-1]),"successful_duration_s":float(t[-1]) if termination=="goal" else None,
        "route_length_m":float(path.s[-1]),"completion_fraction":float(a("progress_s")[-1]/path.s[-1]),
        "finish_error_m":goal,"samples":len(rows),
        "rmse_lateral_m":float(np.sqrt(np.mean(error**2))),"distance_rmse_m":distance_rmse,
        "mae_lateral_m":float(np.mean(abs_err)),"p95_lateral_m":float(np.quantile(abs_err,0.95)),"max_lateral_m":float(np.max(abs_err)),
        "core_mean_lateral_m":float(np.mean(np.abs(traditional))),"core_max_lateral_m":float(np.max(np.abs(traditional))),
        "max_error_time_s":float(t[peak]),"max_error_s_m":float(rows[peak]["progress_s"]),
        "max_error_x_m":float(rows[peak]["x"]),"max_error_y_m":float(rows[peak]["y"]),
        "mean_abs_heading_rad":float(np.mean(np.abs(a("heading_error")))),
        "max_speed_mps":float(np.max(a("speed"))),"mean_ground_speed_mps":float(np.mean(V)),
        "max_applied_steer_deg":float(np.rad2deg(np.max(np.abs(deltas)))),
        "steer_limit_setting_deg":case.max_steer_deg,
        "steer_rate_rms_deg_s":float(np.rad2deg(np.sqrt(np.mean(rates**2)))),
        "steer_rate_mean_abs_deg_s":float(np.rad2deg(np.mean(np.abs(rates)))),
        "steer_angle_saturation_pct":float(100*np.mean(np.abs(deltas)>=config.vehicle.max_steer-1e-5)),
        "steer_rate_saturation_pct":float(100*np.mean(np.abs(rates)>=config.vehicle.max_steer_rate-1e-5)),
        "max_reference_accel_mps2":float(np.max(np.abs(a("reference_normal_demand")))),
        "max_course_normal_accel_mps2":float(np.max(np.abs(a("course_normal_accel")[active]))) if np.any(active) else None,
        "max_beta_deg":float(np.rad2deg(np.max(np.abs(beta[active])))) if np.any(active) else None,
        "max_yaw_rate_radps":float(np.max(np.abs(r))),
        "control_median_ms":float(np.median(a("control_wall_ms"))),"control_p95_ms":float(np.quantile(a("control_wall_ms"),0.95)),
        "cold_control_ms":float(a("control_wall_ms")[0]),
        "mpc_fallback_calls":sum(row["controller_status"]=="fallback" for row in rows),
        "mpc_solve_calls":sum(row["solver_status"]=="scipy_optimal" for row in rows),
        "mpc_solver": "scipy_slsqp" if case.algo=="mpc" else "not_applicable",
    }
    if case.plant=="dynamic":
        slip=np.maximum(np.abs(a("alpha_f")),np.abs(a("alpha_r")))
        metrics["max_tire_slip_deg"]=float(np.rad2deg(np.max(slip[active]))) if np.any(active) else None
        metrics["tire_slip_over_5deg_pct"]=float(100*np.mean(slip[active]>np.deg2rad(5))) if np.any(active) else None
    if case.route=="circle" and not case.gpx:
        # Preregistered central 60% of the circular arc; no manual picking.
        arc=np.where(np.isclose(path.curvature,1/12,atol=1e-5))[0]
        s0,s1=path.s[arc[0]],path.s[arc[-1]]
        mask=(a("progress_s")>=s0+0.2*(s1-s0))&(a("progress_s")<=s0+0.8*(s1-s0))&(V>=0.95*case.speed)
        metrics["steady_samples"]=int(mask.sum())
        if mask.sum()>=5:
            vss=V[mask];theory_delta=cg_steady_steer(1/12,config.vehicle)
            vals={"speed_mean_mps":float(vss.mean()),"steer_mean_rad":float(deltas[mask].mean()),
                  "yaw_rate_mean_radps":float(r[mask].mean()),"normal_mean_mps2":float(a("course_normal_accel")[mask].mean()),
                  "lateral_mae_m":float(abs_err[mask].mean()),"lateral_rmse_m":float(np.sqrt(np.mean(error[mask]**2))),
                  "lateral_std_m":float(np.std(error[mask])),"lateral_max_m":float(abs_err[mask].max()),
                  "beta_max_deg":float(np.rad2deg(np.max(np.abs(beta[mask])))),
                  "steer_rate_mean_abs_deg_s":float(np.rad2deg(np.mean(np.abs(rates[mask])))),
                  "theory_steer_rad":theory_delta,"theory_yaw_rate_radps":float((vss/12).mean()),
                  "theory_normal_mps2":float((vss**2/12).mean())}
            for name,real,theory in (("steer",vals["steer_mean_rad"],theory_delta),
                                     ("yaw_rate",vals["yaw_rate_mean_radps"],vals["theory_yaw_rate_radps"]),
                                     ("normal",vals["normal_mean_mps2"],vals["theory_normal_mps2"])):
                vals[name+"_relative_error_pct"]=100*abs(real-theory)/abs(theory)
            metrics.update({"steady_"+k:v for k,v in vals.items()})
    if case.gpx and path.lat is not None:
        j=int(rows[kmax]["target_index"])
        metrics.update(max_core_error_time_s=rows[kmax]["time"],max_error_ref_lat=float(path.lat[j]),
                       max_error_ref_lon=float(path.lon[j]),max_error_ref_index=j,
                       gpx_origin_lon=118.8145,gpx_origin_lat=31.8885,
                       geographic_location_definition="reference waypoint nearest to maximum absolute original lateral error; not surveyed ground truth")
    return jsonable(metrics)


def run_case(case,out=OUT):
    config,plant_config=config_for(case)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        path=prepared_path(config)
        backend={"kinematic":KinematicBicycleBackend,"simplified":SimplifiedKinematicBackend,"dynamic":DynamicBicycleBackend}[case.plant]()
        if case.plant=="dynamic":backend.substep=case.substep
        if case.baseline:
            name={"pp":"pure_pursuit"}.get(case.algo,case.algo)
            controller=importlib.import_module("vdm_lab.student.baseline."+name)
        else:controller=load_controller(case.algo,"student")
        if hasattr(controller,"reset_diagnostics"):controller.reset_diagnostics()
        state=VehicleState(float(path.x[0]),float(path.y[0]),float(path.yaw[0]),0.0)
        state=backend.reset(state,plant_config);tracker=ReferenceTracker(path)
        previous=ControlCommand(0.0,0.0);rows=[];termination="time_limit"
        max_time=max(case.max_time,path.s[-1]/max(case.speed,0.5)+90.0) if case.gpx else case.max_time
        for k in range(int(math.floor(max_time/case.dt))+1):
            ref=tracker.nearest(state)
            if case.plant=="dynamic" and case.measured_dynamic_states:
                state.lateral_velocity=backend._v_y;state.measured_yaw_rate=backend._r
            start=time.perf_counter()
            raw=controller.control(state,ref,previous,config)
            control_ms=(time.perf_counter()-start)*1000
            command=limit_command(raw,plant_config.vehicle)
            r,beta=backend.dynamics_terms(state,command,plant_config)
            ey,progress,px,py,seg,frac=local_projection(path,state,ref.nearest_index)
            vy=backend._v_y if case.plant=="dynamic" else state.v*math.sin(beta)
            V=math.hypot(state.v,vy) if case.plant=="dynamic" else state.v
            af,ar=float("nan"),float("nan");normal=float("nan");body_ay=float("nan")
            if case.plant=="dynamic":
                vydot,_=dynamic_lateral_accel(state.v,vy,r,command.steer,plant_config.vehicle)
                af,ar=tire_slip_angles(state.v,vy,r,command.steer,plant_config.vehicle)
                body_ay=vydot+state.v*r
                normal=V*r+(state.v*vydot-vy*command.acceleration)/max(V,1e-8)
            rows.append(dict(time=k*case.dt,x=state.x,y=state.y,yaw=state.yaw,speed=state.v,
                             ground_speed=V,lateral_velocity=vy,acceleration=command.acceleration,steer=command.steer,
                             beta=beta,yaw_rate=r,target_index=ref.nearest_index,progress_s=progress,
                             lateral_error=ref.lateral_error,projection_error=ey,projection_x=px,projection_y=py,projection_distance=math.hypot(state.x-px,state.y-py),
                             heading_error=ref.heading_error,curvature=ref.curvature,target_speed=ref.target_speed,
                             reference_normal_demand=V*V*ref.curvature,course_normal_accel=normal,body_lateral_accel=body_ay,
                             alpha_f=af,alpha_r=ar,control_wall_ms=control_ms,
                             controller_status=getattr(raw,"controller_status","ok"),
                             solver_status=getattr(raw,"solver_status","not_applicable"),
                             solver_iterations=getattr(raw,"solver_iterations",0),
                             solver_solve_time_ms=getattr(raw,"solver_solve_time_ms",0.0)))
            dist=math.hypot(state.x-path.x[-1],state.y-path.y[-1])
            progress_ok=path.s[-1]-progress<=max(3.0,config.sim.stop_distance*2)
            if dist<config.sim.stop_distance and state.v<config.sim.stop_speed and progress_ok:
                termination="goal";break
            try:state,previous=backend.step(state,command,plant_config,case.dt)
            except FloatingPointError as exc:termination="numerical_failure: "+str(exc);break
            if not all(math.isfinite(v) for v in (state.x,state.y,state.yaw,state.v)):
                termination="nonfinite_state";break
        metrics=analyze(case,path,rows,config,termination)
    folder=FSPath(out)/"runs"/case.case_id;folder.mkdir(parents=True,exist_ok=True)
    write_csv(folder/"trajectory.csv",rows)
    ref_rows=[dict(index=i,s_m=float(path.s[i]),x_m=float(path.x[i]),y_m=float(path.y[i]),
                   yaw_rad=float(path.yaw[i]),curvature_1pm=float(path.curvature[i]),target_speed_mps=float(path.target_speed[i]),
                   lat_deg="" if path.lat is None else float(path.lat[i]),lon_deg="" if path.lon is None else float(path.lon[i])) for i in range(len(path.x))]
    write_csv(folder/"reference_path.csv",ref_rows)
    write_json(folder/"metrics.json",metrics)
    write_json(folder/"config.json",dict(case=asdict(case),controller_config=asdict(config),
              controller_options={"mpc_solver":"scipy","mpc_cg_heading":case.mpc_cg_heading},
              plant_config=asdict(plant_config),physical_path_arclength=True,
              warnings=list(dict.fromkeys(str(w.message) for w in caught))))
    return metrics


def suite_cases():
    cases=[]
    def add(group,**kw):
        case=Case(case_id=f"{len(cases)+1:03d}_{group}_{kw.get('algo','pp')}",group=group,**kw)
        cases.append(case)
    for route,speed in (("double_lane_change",7),("right_angle",5.5),("s_curve",6.5),("mixed_course",7)):
        for algo in ALGORITHMS:add("route_benchmark",route=route,speed=speed,algo=algo)
    for route,group in (("circle","circle_speed"),("s_curve","speed_scurve")):
        for speed in (3,5,7,9,11):
            for algo in ALGORITHMS:add(group,route=route,speed=speed,algo=algo)
    for plant in ("simplified","kinematic","dynamic"):
        for algo in ALGORITHMS:add("plant_comparison",plant=plant,algo=algo)
    for speed in (3,7,11):
        for lookahead in (1,1.5,2,3,4.5,6,8):add("lookahead",lookahead=lookahead,speed=speed)
    for angle in (12,18,25,35):
        for algo in ("pp","lqr_kinematic"):
            add("steer_limit",max_steer_deg=angle,algo=algo,route="right_angle",speed=7)
    for stiffness in (0.5,0.75,1.0,1.25,1.5):
        for matched in (False,True):
            add("stiffness",algo="lqr_dynamic",plant="dynamic",stiffness_scale=stiffness,
                controller_stiffness_scale=stiffness if matched else 1.0)
    for horizon in (4,8,12,16):add("mpc_horizon",algo="mpc",speed=9,mpc_horizon=horizon)
    for dt in (0.2,0.1,0.05,0.025):
        for algo in ("pp","lqr_dynamic"):add("sampling",dt=dt,algo=algo,speed=9)
    for route,speed in (("double_lane_change",7),("right_angle",5.5),("s_curve",6.5)):
        for algo in ("pp","lqr_kinematic","lqr_dynamic"):add("baseline",baseline=True,route=route,speed=speed,algo=algo)
    for flag in (False,True):add("mpc_heading_ablation",algo="mpc",route="circle",speed=7,mpc_cg_heading=flag)
    for flag in (False,True):add("dynamic_feedback_ablation",algo="lqr_dynamic",plant="dynamic",measured_dynamic_states=flag)
    for speed in (4,8):add("gpx",algo="pp",route="homework_gpx",speed=speed,gpx=True)
    # Explicit stress tests: retain inability to complete a geometrically
    # infeasible circle rather than reporting only successful runs.
    for algo in ("pp", "lqr_kinematic", "mpc"):
        add("stress_failure",algo=algo,route="circle",speed=7,max_steer_deg=5,max_time=60)
    for algo in ("lqr_kinematic", "mpc"):
        add("gpx_comparison",algo=algo,route="homework_gpx",speed=8,gpx=True)
    return cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite",choices=["full","smoke"],default="full")
    parser.add_argument("--case",help="run a single manifest case ID")
    parser.add_argument("--out",type=FSPath,default=OUT)
    parser.add_argument("--resume",action="store_true",help="keep completed case folders")
    args=parser.parse_args();cases=suite_cases()
    if args.suite=="smoke":cases=[Case("smoke_"+a,"smoke",algo=a,route="s_curve",speed=7) for a in ALGORITHMS]
    if args.case:cases=[c for c in cases if c.case_id==args.case]
    if not cases:parser.error("no matching cases")
    args.out.mkdir(parents=True,exist_ok=True)
    write_json(args.out/"manifest.json",[asdict(c) for c in cases])
    env=dict(python=sys.version,platform=platform.platform(),numpy=np.__version__,
             seed=20260917,command=" ".join(sys.argv),executed_solver="SciPy SLSQP; CVXPY not required")
    for name in ("scipy","matplotlib","cvxpy"):
        try:env[name]=importlib.import_module(name).__version__
        except ImportError:env[name]="not installed"
    write_json(args.out/"environment.json",env)
    results=[]
    for i,case in enumerate(cases):
        p=args.out/"runs"/case.case_id/"metrics.json"
        start=time.perf_counter()
        metric=json.loads(p.read_text()) if args.resume and p.exists() else run_case(case,args.out)
        results.append(metric)
        print(f"[{i+1}/{len(cases)}] {case.case_id} {case.route} v={case.speed:g} {case.plant}: "
              f"RMSE={metric['rmse_lateral_m']:.3f} m, goal={metric['reached_goal']}, "
              f"fallbacks={metric['mpc_fallback_calls']}, wall={time.perf_counter()-start:.1f}s",flush=True)
        write_json(args.out/"summary.json",results);write_csv(args.out/"summary.csv",results)
    return results

if __name__=="__main__":main()
