"""Publication-ready, single-axis scientific figures from saved experiment logs.

No simulation is silently rerun. PNG (300 dpi) and SVG versions are exported,
with a figure index linking each figure to its data. Matplotlib's default
palette is retained; line styles and markers provide redundant distinctions.
Run: python -m vdm_lab.student.visualizations --results vdm_lab/student/results
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
LABELS={'pp':'Pure Pursuit','lqr_kinematic':'LQR–K','lqr_dynamic':'LQR–D','mpc':'MPC'}
ORDER=list(LABELS)
ROUTES={'double_lane_change':'Double lane change','right_angle':'Right-angle turn','s_curve':'S-curve','mixed_course':'Mixed course'}
STYLES=['-','--','-.',':']
INDEX=[]


def canvas(title,xlabel,ylabel,note='',size=(11.2,6.2)):
    fig=plt.figure(figsize=size)
    ax=fig.add_axes([.105,.19,.855,.65])
    ax.set_title(title,loc='left',fontsize=20,fontweight='bold',pad=23)
    ax.set_xlabel(xlabel,fontsize=15,labelpad=10);ax.set_ylabel(ylabel,fontsize=15,labelpad=10)
    ax.tick_params(labelsize=12)
    ax.spines[['top','right']].set_visible(False)
    ax.grid(True,alpha=.20,linewidth=.7)
    if note:fig.text(.105,.035,note,fontsize=10.5,va='bottom',wrap=True)
    return fig,ax


def legend(ax,ncol=2):
    ax.legend(frameon=False,fontsize=11,ncol=ncol,loc='best')


def save(fig,out,name,caption,data):
    out.mkdir(parents=True,exist_ok=True)
    fig.savefig(out/(name+'.png'),dpi=300,bbox_inches='tight',pad_inches=.15)
    fig.savefig(out/(name+'.svg'),bbox_inches='tight',pad_inches=.15)
    plt.close(fig)
    INDEX.append(dict(figure=name,caption=caption,data=data,png=name+'.png',svg=name+'.svg'))


def main(results=HERE/'results',out=None):
    results=Path(results);out=Path(out) if out else HERE/'deliverables'/'figures'
    summary=results/'summary.csv'
    if not summary.exists():raise FileNotFoundError(f'Run experiments first; missing {summary}')
    d=pd.read_csv(summary)
    def subset(group,**filters):
        z=d[d.group==group].copy()
        for k,v in filters.items():z=z[z[k]==v]
        return z
    def log(row):return pd.read_csv(results/'runs'/row.case_id/'trajectory.csv')
    def ref(row):return pd.read_csv(results/'runs'/row.case_id/'reference_path.csv')
    def export(fig,name,caption,data='results/summary.csv'):save(fig,out,name,caption,data)
    # All four algorithms, identical per-route conditions.
    for route in ROUTES:
        z=subset('route_benchmark',route=route);p=ref(z.iloc[0])
        fig,ax=canvas(ROUTES[route]+': tracked trajectories','East / local x (m)','North / local y (m)',
            f'CG kinematic plant • requested speed {z.speed.iloc[0]:g} m/s • Δt = 0.1 s • same path and initial state')
        for i,a in enumerate(ORDER):
            q=log(z[z.algo==a].iloc[0]);ax.plot(q.x,q.y,STYLES[i],lw=2.1,label=LABELS[a])
        ax.plot(p.x_m,p.y_m,'--',lw=1.4,label='Reference')
        ax.set_aspect('equal',adjustable='datalim');legend(ax,3)
        export(fig,'01_trajectory_'+route,'Path tracking comparison: '+ROUTES[route],','.join(z.case_id))
    z=subset('route_benchmark',route='s_curve');p=ref(z.iloc[0])
    fig,ax=canvas('Transient error is not captured by a single mean','Reference-path progress (m)','Signed lateral error (m)',
        'S-curve • requested 6.5 m/s • segment-normal error; endpoint overshoot is measured separately')
    for i,a in enumerate(ORDER):
        q=log(z[z.algo==a].iloc[0]);ax.plot(q.progress_s,q.projection_error,STYLES[i],lw=2,label=LABELS[a])
    legend(ax);export(fig,'02_error_progress','S-curve signed lateral error against geometric path progress.')
    fig,ax=canvas('Read both the typical error and the tail','Absolute lateral error (m)','Empirical cumulative probability',
        'S-curve • requested 6.5 m/s • complete runs, including acceleration and stopping')
    for i,a in enumerate(ORDER):
        q=log(z[z.algo==a].iloc[0]);e=np.sort(abs(q.projection_error));ax.plot(e,np.arange(1,len(e)+1)/len(e),STYLES[i],lw=2,label=LABELS[a])
    ax.set_ylim(0,1.02);legend(ax);export(fig,'03_absolute_error_cdf','Correct absolute-error empirical CDF (not a signed-error CDF).')
    fig,ax=canvas('MPC leads this fixed-configuration benchmark','Route','Lateral RMSE (m)',
        'Route speeds: 7 / 5.5 / 6.5 / 7 m/s • CG kinematic plant • no per-controller retuning')
    for i,a in enumerate(ORDER):
        vals=[subset('route_benchmark',route=r,algo=a).rmse_lateral_m.iloc[0] for r in ROUTES]
        ax.plot(range(4),vals,STYLES[i],marker='o',lw=2,label=LABELS[a])
    ax.set_xticks(range(4),['Lane change','Right angle','S-curve','Mixed']);ax.set_ylim(bottom=0);legend(ax)
    export(fig,'04_benchmark_accuracy','Route-level RMSE comparison under identical within-route conditions.')
    fig,ax=canvas('Accuracy has a computational cost','Median controller-call time (ms, log scale)','Lateral RMSE (m)',
        'S-curve at 6.5 m/s • same host • Python timing includes controller logic; not a real-time certification')
    for a in ORDER:
        r=z[z.algo==a].iloc[0];ax.scatter(r.control_median_ms,r.rmse_lateral_m,s=100,label=LABELS[a])
        ax.annotate(LABELS[a],(r.control_median_ms,r.rmse_lateral_m),xytext=(7,7),textcoords='offset points',fontsize=12)
    ax.set_xscale('log');ax.set_ylim(bottom=0);export(fig,'05_accuracy_runtime','Error versus measured controller-call runtime.')
    fig,ax=canvas('Smoothness is a separate performance dimension','Controller','Mean absolute steering rate (°/s)',
        'S-curve at 6.5 m/s • finite differences of applied steering • whole-run average')
    v=[z[z.algo==a].steer_rate_mean_abs_deg_s.iloc[0] for a in ORDER]
    ax.bar([LABELS[a] for a in ORDER],v)
    for j,x in enumerate(v):ax.text(j,x+.15,f'{x:.2f}',ha='center',fontsize=12)
    ax.set_ylim(0,max(v)*1.2);export(fig,'06_control_smoothness','Steering smoothness uses a dedicated axis and physical unit.')
    # Primary variable 1: requested speed; do not relabel it as realized speed.
    z=subset('speed_scurve')
    fig,ax=canvas('High requested speed exposes transient sensitivity','Requested cruise speed (m/s)','Lateral RMSE (m)',
        'S-curve • 11 m/s requests reach only 8.77–9.04 m/s • acceleration, braking and actuator limits remain active')
    for i,a in enumerate(ORDER):
        q=z[z.algo==a].sort_values('speed');ax.plot(q.speed,q.rmse_lateral_m,STYLES[i],marker='o',lw=2,label=LABELS[a])
    ax.set_xticks([3,5,7,9,11]);ax.set_ylim(bottom=0);legend(ax)
    export(fig,'07_speed_sensitivity','S-curve requested-speed sweep; transient and achieved-speed qualification included.')
    fig,ax=canvas('Requested speed is not achieved cruise speed','Time (s)','Vehicle speed (m/s)',
        'MPC on the same S-curve • longitudinal bounds and distance-based braking prevent an 11 m/s cruise')
    for speed in [3,7,11]:
        r=z[(z.algo=='mpc')&(z.speed==speed)].iloc[0];q=log(r);ax.plot(q.time,q.speed,lw=2,label=f'Requested {speed} m/s')
    ax.set_ylim(bottom=0);legend(ax);export(fig,'08_achieved_speed','Actual MPC speed histories for three requested speeds.')
    z=subset('circle_speed')
    fig,ax=canvas('Steady circular error need not grow monotonically','Measured mean speed in steady window (m/s)','Steady-window lateral RMSE (m)',
        'R = 12 m • central 60% of the circular arc • samples require V ≥ 95% of the requested speed')
    for i,a in enumerate(ORDER):
        q=z[z.algo==a].sort_values('speed');ax.plot(q.steady_speed_mean_mps,q.steady_lateral_rmse_m,STYLES[i],marker='o',lw=2,label=LABELS[a])
    ax.set_ylim(bottom=0);legend(ax);export(fig,'09_circle_steady_error','Circle steady-state error plotted against actual measured speed.')
    fig,ax=canvas('Measured normal acceleration follows V²/R','Measured mean steady-window speed (m/s)','Mean course-normal acceleration (m/s²)',
        'R = 12 m • points: independently derived from vehicle motion • line: constant-speed theoretical curve')
    for a in ORDER:
        q=z[z.algo==a].sort_values('speed');ax.plot(q.steady_speed_mean_mps,q.steady_normal_mean_mps2,'o',ms=6,label=LABELS[a])
    v=np.linspace(2.8,11,150);ax.plot(v,v*v/12,'--',lw=1.8,label='Theory V² / R')
    ax.set_ylim(bottom=0);legend(ax,3);export(fig,'10_circle_acceleration','Actual course-normal acceleration compared with circular-motion theory; not a self-comparison of logged demand.')
    fig,ax=canvas('Yaw demand grows approximately linearly with speed','Measured mean steady-window speed (m/s)','Mean yaw rate (rad/s)',
        'R = 12 m • steady course: β̇ ≈ 0, so yaw rate r ≈ V/R')
    for a in ORDER:
        q=z[z.algo==a].sort_values('speed');ax.plot(q.steady_speed_mean_mps,q.steady_yaw_rate_mean_radps,'o',ms=6,label=LABELS[a])
    ax.plot(v,v/12,'--',lw=1.8,label='Theory V / R');legend(ax,3);export(fig,'11_circle_yaw_rate','Measured steady yaw rate and V/R reference.')
    fig,ax=canvas('Steady steering is geometric in this neutral model','Measured mean steady-window speed (m/s)','Mean front steering angle (°)',
        'CG kinematic plant • exact δ = atan[Lκ / √(1 − (lᵣκ)²)] • not a demonstration of physical tire saturation')
    for i,a in enumerate(ORDER):
        q=z[z.algo==a].sort_values('speed');ax.plot(q.steady_speed_mean_mps,np.rad2deg(q.steady_steer_mean_rad),STYLES[i],marker='o',lw=2,label=LABELS[a])
    ax.axhline(np.rad2deg(z.steady_theory_steer_rad.iloc[0]),ls='--',lw=1.2,label='Exact CG geometric demand');legend(ax,2)
    export(fig,'12_circle_steering','Steering versus measured speed; no imposed understeer conclusion.')
    # Primary variable 2: PP base lookahead.
    z=subset('lookahead')
    fig,ax=canvas('Longer lookahead increases corner-cutting here','PP base lookahead L₀ (m)','Lateral RMSE (m)',
        'S-curve • actual lookahead also includes the speed term • same controller gains and 35° steering limit')
    for speed in [3,7,11]:
        q=z[z.speed==speed].sort_values('lookahead');ax.plot(q.lookahead,q.rmse_lateral_m,'-o',lw=2,label=f'Requested {speed} m/s')
    ax.set_ylim(bottom=0);legend(ax);export(fig,'13_lookahead_sensitivity','Lookahead × requested-speed factorial experiment, 21 runs.')
    fig,ax=canvas('Lookahead trades spatial accuracy for response shape','Reference-path progress (m)','Signed lateral error (m)',
        'PP • requested 7 m/s • L₀ varied; shorter is not asserted to be universally optimal')
    for L in [1,3,8]:
        q=log(z[(z.speed==7)&(z.lookahead==L)].iloc[0]);ax.plot(q.progress_s,q.projection_error,lw=2,label=f'L₀ = {L:g} m')
    legend(ax);export(fig,'14_lookahead_error_shape','Error histories at three lookahead settings.')
    # Primary variable 3: physical steering-angle bound.
    z=subset('steer_limit')
    fig,ax=canvas('A tighter steering bound can make the turn unreachable','Steering-angle limit (°)','Maximum absolute lateral error (m)',
        'Right-angle route • requested 7 m/s • R = 5 m fillet needs ≈ 27.3° for exact CG-circle tracking')
    for a in ['pp','lqr_kinematic']:
        q=z[z.algo==a].sort_values('max_steer_deg');ax.plot(q.max_steer_deg,q.max_lateral_m,'-o',lw=2,label=LABELS[a])
    ax.set_xticks([12,18,25,35]);ax.set_ylim(bottom=0);legend(ax);export(fig,'15_steering_limit_sensitivity','Steering-limit sweep: physical feasibility and peak tracking error.')
    fig,ax=canvas('Saturation changes the trajectory, not just the metric','East / local x (m)','North / local y (m)',
        'LQR–K • right-angle route • requested 7 m/s • all three runs reach the endpoint despite different path quality')
    for angle in [12,25,35]:
        r=z[(z.algo=='lqr_kinematic')&(z.max_steer_deg==angle)].iloc[0];q=log(r);ax.plot(q.x,q.y,lw=2,label=f'Limit {angle}°')
    p=ref(r);ax.plot(p.x_m,p.y_m,'--',lw=1.5,label='Reference');ax.set_aspect('equal',adjustable='datalim');legend(ax)
    export(fig,'16_steering_limit_trajectories','Steering-limit trajectory comparison; endpoint success does not imply accurate tracking.')
    z=subset('stress_failure')
    fig,ax=canvas('Endpoint arrival is not a tracking-success test','East / local x (m)','North / local y (m)',
        'R = 12 m circle • steering limited to 5° • PP/LQR–K reach the endpoint with > 33 m peak error; MPC times out')
    for i,a in enumerate(['pp','lqr_kinematic','mpc']):
        r=z[z.algo==a].iloc[0];q=log(r);ax.plot(q.x,q.y,STYLES[i],lw=2,label=LABELS[a])
    p=ref(r);ax.plot(p.x_m,p.y_m,'--',lw=1.5,label='Reference');ax.set_aspect('equal',adjustable='datalim');legend(ax)
    export(fig,'17_constraint_failure','Retained severe constraint-limited runs: max-error failures and one timeout.')
    # Extensions: mismatched plant, model parameters, horizon, and time step.
    z=subset('plant_comparison')
    fig,ax=canvas('Controller rankings depend on the simulated plant','Simulation plant','Lateral RMSE (m)',
        'S-curve • requested 7 m/s • controller settings fixed • speed conventions differ slightly between plant interfaces')
    plants=['simplified','kinematic','dynamic']
    for i,a in enumerate(ORDER):
        y=[z[(z.algo==a)&(z.plant==p)].rmse_lateral_m.iloc[0] for p in plants]
        ax.plot(range(3),y,STYLES[i],marker='o',lw=2,label=LABELS[a])
    ax.set_xticks(range(3),['Simplified kinematic','CG kinematic','Dynamic bicycle']);ax.set_ylim(bottom=0);legend(ax)
    export(fig,'18_plant_comparison','Twelve controller–plant combinations, distinct from open-loop model identification.')
    z=subset('stiffness')
    fig,ax=canvas('A matched model does not guarantee better closed-loop error','True front/rear axle stiffness scale','Lateral RMSE (m)',
        'Dynamic plant + LQR–D • requested 7 m/s • fixed Q/R • some tire slip exceeds the 5° validity warning')
    for matched,label in [(False,'Controller stiffness fixed at nominal'),(True,'Controller stiffness matched to plant')]:
        q=z[(z.controller_stiffness_scale==z.stiffness_scale) if matched else (z.controller_stiffness_scale==1)].copy()
        q=q.drop_duplicates('stiffness_scale').sort_values('stiffness_scale')
        ax.plot(q.stiffness_scale,q.rmse_lateral_m,'-o',lw=2,label=label)
    ax.set_ylim(bottom=0);legend(ax,1);export(fig,'19_stiffness_mismatch','Physical stiffness sweep with matched and mismatched controller models; validity caveat explicit.')
    z=subset('mpc_horizon').sort_values('mpc_horizon')
    fig,ax=canvas('Longer prediction is not automatically better','MPC horizon N (steps)','Lateral RMSE (m)',
        'S-curve • requested 9 m/s • Δt = 0.1 s • frozen weights and linearization strategy; 0 solver fallbacks')
    ax.plot(z.mpc_horizon,z.rmse_lateral_m,'-o',lw=2);ax.set_xticks(z.mpc_horizon);ax.set_ylim(bottom=0)
    export(fig,'20_mpc_horizon_accuracy','Horizon sensitivity at fixed sampling time and tuning.')
    fig,ax=canvas('Longer horizons increase measured solution cost','MPC horizon N (steps)','Controller-call time (ms)',
        'Same host and SciPy QP backend • median and 95th percentile across each run • not a worst-case timing guarantee')
    ax.plot(z.mpc_horizon,z.control_median_ms,'-o',lw=2,label='Median');ax.plot(z.mpc_horizon,z.control_p95_ms,'--s',lw=2,label='95th percentile')
    ax.set_xticks(z.mpc_horizon);ax.set_ylim(bottom=0);legend(ax);export(fig,'21_mpc_horizon_runtime','Runtime sensitivity separate from error, avoiding mixed physical units.')
    z=subset('sampling')
    fig,ax=canvas('A sampling-time sweep is a controller change','Controller sampling interval Δt (s)','Lateral RMSE (m)',
        'S-curve • requested 9 m/s • unchanged weights • this is NOT an integration-convergence experiment')
    for a in ['pp','lqr_dynamic']:
        q=z[z.algo==a].sort_values('dt');ax.plot(q.dt,q.rmse_lateral_m,'-o',lw=2,label=LABELS[a])
    ax.set_xscale('log',base=2);ax.set_xticks([.025,.05,.1,.2],['0.025','0.05','0.10','0.20']);ax.set_ylim(bottom=0);legend(ax)
    export(fig,'22_sampling_sensitivity','Changing the controller sample time also changes discrete feedback behavior.')
    base=subset('baseline');fig,ax=canvas('A formula correction is not universal performance tuning','Controller / route','Lateral RMSE (m)',
        'Original versus revised student implementations • identical student-side harness • MPC excluded: original CVXPY path unavailable')
    names=[];old=[];new=[]
    for route in ['double_lane_change','right_angle','s_curve']:
        for a in ['pp','lqr_kinematic','lqr_dynamic']:
            names.append(LABELS[a]+'\n'+{'double_lane_change':'Lane change','right_angle':'Right angle','s_curve':'S-curve'}[route])
            old.append(base[(base.route==route)&(base.algo==a)].rmse_lateral_m.iloc[0])
            new.append(subset('route_benchmark',route=route,algo=a).rmse_lateral_m.iloc[0])
    x=np.arange(9);ax.plot(x,old,'o',ms=8,label='Original');ax.plot(x,new,'x',ms=9,label='Revised')
    for j in range(9):ax.plot([j,j],[old[j],new[j]],lw=1)
    ax.set_xticks(x,names,fontsize=9);ax.set_ylim(bottom=0);legend(ax)
    export(fig,'23_before_after','Paired before/after results; corrections need not lower every benchmark metric.')
    # Identification: independent held-out excitation, synthetic truth explicitly labeled.
    ident=results/'identification';params=pd.read_csv(ident/'parameters.csv');h=pd.read_csv(ident/'holdout_metrics.csv');pr=pd.read_csv(ident/'holdout_predictions.csv')
    fig,ax=canvas('Known synthetic parameters are recovered closely','Parameter','Estimation error relative to truth (%)',
        'Points: estimates • whiskers: local Gaussian/Jacobian 95% intervals • synthetic data, not real-vehicle validation')
    for j,r in params.iterrows():
        y=100*(r.estimate/r.true_value-1);low=100*(r.estimate-r.ci95_low)/r.true_value;high=100*(r.ci95_high-r.estimate)/r.true_value
        ax.errorbar(j,y,yerr=np.array([[low],[high]]),fmt='o',capsize=6,ms=7)
    ax.axhline(0,ls='--',lw=1);ax.set_xticks(range(len(params)),['Wheelbase L','CG-to-rear lᵣ','Front stiffness Cᶠ','Rear stiffness Cʳ'])
    export(fig,'24_identified_parameters','Parameter errors and approximate conditional intervals.','results/identification/parameters.csv')
    # Inspect column names once; this schema is fixed by identify_models.py.
    fig,ax=canvas('The dynamic model captures lateral-velocity transients','Time (s)','Body lateral velocity vᵧ (m/s)',
        'Held-out multisine • longitudinal u = 10 m/s • training used 6 and 9 m/s with disjoint excitation')
    for model in ['Simplified kinematic','CG kinematic','Identified dynamic']:
        q=pr[(pr.model==model)&(pr.speed_mps==10)];ax.plot(q.time_s,q.lateral_velocity_pred,lw=1.8,label=model)
    ax.plot(q.time_s,q.lateral_velocity_true,'--',lw=1.2,label='Synthetic truth');legend(ax,2)
    export(fig,'25_model_lateral_velocity','Held-out lateral-velocity comparison.','results/identification/holdout_predictions.csv')
    fig,ax=canvas('Kinematic yaw response misses the dynamic phase lag','Time (s)','Yaw rate r (rad/s)',
        'Held-out u = 10 m/s • both kinematic models have r = u tan(δ)/L under matched longitudinal speed')
    for model,label in [('CG kinematic','Both kinematic models'),('Identified dynamic','Identified dynamic')]:
        q=pr[(pr.model==model)&(pr.speed_mps==10)];ax.plot(q.time_s,q.yaw_rate_pred,lw=1.8,label=label)
    ax.plot(q.time_s,q.yaw_rate_true,'--',lw=1.3,label='Synthetic truth');legend(ax,1)
    export(fig,'26_model_yaw_rate','Held-out yaw response without duplicating identical kinematic predictions.','results/identification/holdout_predictions.csv')
    fig,ax=canvas('Model quality depends on the metric and operating point','Held-out longitudinal speed u (m/s)','Position RMSE (m, log scale)',
        '20 s open-loop predictions • dynamic model fitted only on training runs • low error is expected for matched synthetic structure')
    for model in ['Simplified kinematic','CG kinematic','Identified dynamic']:
        q=h[h.model==model].sort_values('speed_mps');ax.plot(q.speed_mps,q.position_rmse_m,'-o',lw=2,label=model)
    ax.set_yscale('log');ax.set_xticks([4,10]);legend(ax,1)
    export(fig,'27_model_holdout_error','Held-out position RMSE; CG kinematics is not uniformly better than the simplified model.','results/identification/holdout_metrics.csv')
    refine=pd.read_csv(ident/'integration_refinement.csv')
    fig,ax=canvas('Integrator refinement is tested separately','Internal RK4 substep (ms)','Yaw-rate RMSE versus 0.5 ms reference (rad/s)',
        'Fixed input, controller-free open-loop test • 10 m/s • approximately fourth-order error reduction')
    ax.plot(refine.substep_s*1000,refine.yaw_rate_rmse_radps,'-o',lw=2)
    ax.set_xscale('log',base=2);ax.set_yscale('log');ax.set_xticks([1,2,4],['1','2','4'])
    export(fig,'28_integration_refinement','Numerical integration refinement, not confounded with controller discretization.','results/identification/integration_refinement.csv')
    # GPX: source data only, no fabricated map or independently planned route.
    z=subset('gpx');r=z[z.speed==8].iloc[0];p=ref(r);q=log(r)
    fig,ax=canvas('Supplied GPX: useful replay, limited geometric evidence','Local east offset (m)','Local north offset (m)',
        'Provided homework_route_1.gpx • requested 8 m/s • longest raw GPX gap ≈ 620 m; interpolation cannot recover road geometry')
    ax.plot(q.x,q.y,lw=1.7,label='PP trajectory');ax.plot(p.x_m,p.y_m,'--',lw=1,label='GPX-derived reference')
    k=int(abs(q.lateral_error).argmax());ax.scatter(q.x.iloc[k],q.y.iloc[k],marker='x',s=100,label='Maximum absolute core lateral error')
    ax.set_aspect('equal',adjustable='datalim');legend(ax,1)
    export(fig,'29_gpx_replay','Local-coordinate replay of the supplied, not independently planned, GPX route.',r.case_id)
    fig,ax=canvas('A small whole-route RMSE can hide a localized peak','Reference-path progress (m)','Absolute lateral error (m)',
        'Supplied 3.91 km GPX • PP • full-resolution log retained • no claim of surveyed road accuracy')
    for _,r in z.iterrows():
        q=log(r);ax.plot(q.progress_s,abs(q.projection_error),lw=1.2,label=f'Requested {r.speed:g} m/s')
    ax.set_ylim(bottom=0);legend(ax);export(fig,'30_gpx_error','GPX absolute-error histories and local peaks.')
    fig,ax=canvas('Tracking error can peak after a curvature change','Reference-path progress (m)','Reference curvature (1/m)',
        'S-curve at 6.5 m/s • vertical markers locate each controller’s maximum absolute segment-normal error')
    z=subset('route_benchmark',route='s_curve');p=ref(z.iloc[0]);ax.plot(p.s_m,p.curvature_1pm,lw=2,label='Reference curvature')
    for a in ORDER:
        r=z[z.algo==a].iloc[0];ax.axvline(r.max_error_s_m,ls='--',lw=1.5,label=LABELS[a]+' peak error')
    legend(ax,3);export(fig,'31_peak_error_location','Location of maximum tracking errors relative to reference curvature; causal attribution remains qualified.')
    # Supplied offline OSM context; the same coordinate origin is used by both.
    import lzma
    from matplotlib.collections import LineCollection
    from vdm_lab.common.gpx import latlon_to_local_xy
    gj=HERE.parents[2]/'data'/'planet_118.792,31.875_118.837,31.902.osm.geojson.xz'
    if gj.exists():
        with lzma.open(gj,'rt',encoding='utf-8') as f:geo=json.load(f)
        segments=[]
        for feature in geo['features']:
            geometry=feature.get('geometry') or {};kind=geometry.get('type');coords=geometry.get('coordinates',[])
            lines=[coords] if kind=='LineString' else [ring for poly in coords for ring in poly] if kind=='MultiPolygon' else []
            for line in lines:
                pts=np.asarray(line)
                if len(pts)<2:continue
                xx,yy=latlon_to_local_xy(pts[:,1],pts[:,0],origin_lat=31.8885,origin_lon=118.8145)
                segments.append(np.column_stack([xx,yy]))
        r=subset('gpx',speed=8).iloc[0];q=log(r);p=ref(r)
        fig,ax=canvas('A shared coordinate origin aligns route and offline map','Local east offset (m)','Local north offset (m)',
            'Provided GeoJSON and GPX • origin (118.8145° E, 31.8885° N) • © OpenStreetMap contributors • sparse GPX ≠ surveyed road path')
        ax.add_collection(LineCollection(segments,linewidths=.45,alpha=.25))
        ax.plot(p.x_m,p.y_m,'--',lw=1.5,label='GPX-derived reference');ax.plot(q.x,q.y,lw=2,label='PP, requested 8 m/s')
        ax.set_xlim(p.x_m.min()-150,p.x_m.max()+150);ax.set_ylim(p.y_m.min()-150,p.y_m.max()+150)
        ax.set_aspect('equal',adjustable='box');legend(ax,1)
        export(fig,'32_gpx_offline_map','GPX/GeoJSON alignment with common origin; map attribution retained.',str(gj.name)+';134_gpx_pp')
    z=d[(d.gpx==True)&(d.speed==8)]
    fig,ax=canvas('The GPX replay reverses the ideal-route ranking','Controller','Lateral RMSE (m)',
        'Provided 3.91 km route • requested 8 m/s • LQR–K times out at 579.0 s; failed run retained')
    names=['pp','lqr_kinematic','mpc'];vals=[z[z.algo==a].rmse_lateral_m.iloc[0] for a in names]
    ax.bar([LABELS[a] for a in names],vals)
    for j,val in enumerate(vals):ax.text(j,val+.06,f'{val:.3f} m',ha='center',fontsize=13)
    ax.set_ylim(0,max(vals)*1.2)
    export(fig,'33_gpx_algorithm_comparison','All three required algorithms on the same supplied GPX at 8 m/s; not all reach the goal.')
    (out/'figure_index.json').write_text(json.dumps(INDEX,indent=2),encoding='utf-8')
    pd.DataFrame(INDEX).to_csv(out/'figure_index.csv',index=False)
    print(f'Exported {len(INDEX)} figures as 300-dpi PNG + editable SVG to {out}')
    return out

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,default=HERE/'results')
    parser.add_argument('--out',type=Path)
    args=parser.parse_args();main(args.results,args.out)
