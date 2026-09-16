"""Recompute campus comparison, peak diagnostics, plots and integrity checks from logs."""
import csv
import json
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from campus_route import ROOT, ORIGIN, R, to_lonlat

CASES = ['pp_baseline', 'lqr_baseline', 'mpc_baseline', 'pp_slower']
LABELS = ['PP', 'Kinematic LQR', 'MPC', 'PP (2 m/s cap)']
COLORS = ['#d55e00', '#0072b2', '#009e73', '#9467bd']
DEST = ROOT/'campus_results'


def read_csv(path):
    return np.genfromtxt(path, delimiter=',', names=True)


def write_csv(name, rows):
    with (DEST/name).open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def map_lines(ax):
    geo = json.loads((ROOT/'data/campus_basemap.geojson').read_text(encoding='utf-8'))
    for f in geo['features']:
        points = np.array(f['geometry']['coordinates'])
        x = np.deg2rad(points[:,0]-ORIGIN[0])*R*np.cos(np.deg2rad(ORIGIN[1]))
        y = np.deg2rad(points[:,1]-ORIGIN[1])*R
        ax.plot(x, y, color='#b8c6cd', lw=.55, alpha=.8, zorder=0)


def base_map(ax, plan):
    map_lines(ax)
    ax.plot(plan['x_m'], plan['y_m'], color='#333333', lw=1.3, ls='--', label='Smooth reference')
    ax.set(xlabel='East from map origin [m]', ylabel='North from map origin [m]')
    ax.set_xlim(plan['x_m'].min()-40, plan['x_m'].max()+40)
    ax.set_ylim(plan['y_m'].min()-40, plan['y_m'].max()+40)
    ax.set_aspect('equal'); ax.grid(alpha=.16)


def save(fig, name):
    fig.savefig(DEST/name, dpi=180, bbox_inches='tight')
    plt.close(fig)


def main():
    DEST.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10, 'axes.spines.top':False, 'axes.spines.right':False})
    plan = read_csv(ROOT/'data/planned_path.csv')
    data=[]; refs=[]; configs=[]; rows=[]; peaks=[]
    mask = abs(plan['curvature_1pm']) > .015
    groups=np.split(np.flatnonzero(mask), np.flatnonzero(np.diff(np.flatnonzero(mask))>1)+1)
    turns=[]
    for indices in groups:
        if len(indices)<2: continue
        angle=np.rad2deg(np.trapz(plan['curvature_1pm'][indices], plan['s_m'][indices]))
        if abs(angle)>30:
            turns.append((int(indices[0]),int(indices[-1])))
    assert len(turns)==3, turns
    for case,label in zip(CASES,LABELS):
        directory=ROOT/'outputs/campus'/case
        t=read_csv(directory/'trajectory.csv'); ref=read_csv(directory/'reference_path.csv')
        m=json.loads((directory/'metrics.json').read_text()); c=json.loads((directory/'config.json').read_text())
        assert c['plan_sha256']==hashlib.sha256((ROOT/'data/planned_path.csv').read_bytes()).hexdigest(), 'Logs belong to a different route'
        data.append(t); refs.append(ref); configs.append(c)
        idx=t['target_index'].astype(int)
        assert np.all((idx>=0)&(idx<len(ref))) and np.all(np.diff(idx)>=0)
        error=(t['x']-ref['x_m'][idx])*(-np.sin(ref['yaw_rad'][idx]))+(t['y']-ref['y_m'][idx])*np.cos(ref['yaw_rad'][idx])
        assert np.allclose(error,t['lateral_error'],atol=1e-10)
        assert np.isclose(np.mean(abs(error)),m['mean_lateral_error_m'])
        assert np.isclose(np.max(abs(error)),m['max_lateral_error_m'])
        assert np.all(np.isfinite(t.view(float).reshape(len(t),-1)))
        duration=float(t['time'][-1]); reached=m['reached_goal']
        j=int(np.argmax(abs(error))); peak=t[j]; ri=idx[j]
        lon,lat=to_lonlat(peak['x'],peak['y'])
        turn_number=int(np.argmin([abs(ref['s_m'][ri]-(plan['s_m'][a]+plan['s_m'][b])/2) for a,b in turns]))
        a,b=turns[turn_number]
        curvature_peak_index=a+int(np.argmax(abs(plan['curvature_1pm'][a:b+1])))
        phase='before' if ref['s_m'][ri]<plan['s_m'][a] else ('after' if ref['s_m'][ri]>plan['s_m'][b] else 'within')
        peakrow=dict(algorithm=label,case=case,time_s=float(peak['time']),x_m=float(peak['x']),y_m=float(peak['y']),
            route_s_m=float(ref['s_m'][ri]),speed_mps=float(peak['speed']),target_speed_mps=float(peak['target_speed']),
            curvature_1pm=float(peak['curvature']),steer_rad=float(peak['steer']),lateral_error_m=float(peak['lateral_error']),
            vehicle_lon_deg=float(lon),vehicle_lat_deg=float(lat),reference_lon_deg=float(ref['lon_deg'][ri]),
            reference_lat_deg=float(ref['lat_deg'][ri]),nearest_major_turn=turn_number+1,phase_relative_to_turn=phase,
            reference_index=int(ri),turn_curvature_peak_s_m=float(plan['s_m'][curvature_peak_index]),
            station_offset_from_turn_curvature_peak_m=float(ref['s_m'][ri]-plan['s_m'][curvature_peak_index]))
        peaks.append(peakrow)
        row=dict(algorithm=label,case=case,reached_goal=reached,
            arrival_time_s=duration if reached else '',simulation_duration_s=duration,
            mean_lateral_error_m=m['mean_lateral_error_m'],max_lateral_error_m=m['max_lateral_error_m'],
            finish_error_m=m['finish_error_m'],route_length_m=float(ref['s_m'][-1]),
            mean_progress_speed_mps=float(ref['s_m'][-1]/duration) if reached else '',
            last_reference_s_m=float(ref['s_m'][idx[-1]]),
            mean_abs_steer_rate_radps=float(np.mean(abs(np.diff(t['steer']))/np.diff(t['time']))),
            steering_saturation_fraction=float(np.mean(abs(t['steer'])>=c['config']['vehicle']['max_steer']-1e-8)))
        rows.append(row)
    for ref in refs[1:3]:
        for name in refs[0].dtype.names:
            assert np.allclose(ref[name],refs[0][name],equal_nan=True), name
    for c in configs[1:3]:
        assert c['config']==configs[0]['config']
        assert c['baseline_sha256']==configs[0]['baseline_sha256']
    for name in ('x_m','y_m','yaw_rad','curvature_1pm','s_m'):
        assert np.array_equal(refs[0][name],refs[3][name])
    c0=json.loads(json.dumps(configs[0]['config']));c3=json.loads(json.dumps(configs[3]['config']))
    c0['sim'].pop('target_speed');c3['sim'].pop('target_speed');assert c0==c3
    # GPX and canonical plan are the same geometry in the declared projection.
    gpx=ET.parse(ROOT/'data/gpx/campus_route.gpx')
    ll=np.array([(float(n.attrib['lon']),float(n.attrib['lat'])) for n in gpx.iter() if n.tag.endswith('trkpt')])
    assert len(ll)==len(plan)
    projected=(ll-np.array(ORIGIN))*np.array([R*np.cos(np.deg2rad(ORIGIN[1])),R])*np.pi/180
    gpx_error=float(np.max(np.hypot(projected[:,0]-plan['x_m'],projected[:,1]-plan['y_m'])))
    assert gpx_error<1e-4
    step=np.hypot(np.diff(plan['x_m']),np.diff(plan['y_m']))
    assert np.max(abs(step-np.diff(plan['s_m'])))<.002
    v=configs[0]['config']['vehicle']; max_k=np.tan(v['max_steer'])/np.sqrt(v['wheelbase']**2+(v['lr']*np.tan(v['max_steer']))**2)
    infeasible=abs(plan['curvature_1pm'])>max_k
    adjustment=json.loads((ROOT/'data/campus_route_metadata.json').read_text())
    assert not infeasible.any()
    assert adjustment['dense_check_max_curvature_1pm'] < max_k
    validation=dict(passed=True,baseline_same_reference=True,baseline_same_config=True,
        slowdown_geometry_unchanged=True,slowdown_only_cruise_cap_changed=True,
        lateral_error_recomputed=True,gpx_plan_max_difference_m=gpx_error,
        maximum_kinematically_feasible_cg_curvature_1pm=float(max_k),
        reference_max_curvature_1pm=float(max(abs(plan['curvature_1pm']))),
        reference_length_exceeding_steering_curvature_m=float(np.sum(np.diff(plan['s_m'])*infeasible[:-1])),
        dense_curvature_limit_passed=bool(adjustment['dense_check_max_curvature_1pm']<max_k),
        dense_max_geometric_steer_deg=adjustment['max_geometric_steer_deg'],
        all_cases_reached_goal=all(row['reached_goal'] for row in rows),
        turns=[dict(name=f'Turn {i+1}',s_start_m=float(plan['s_m'][a]),s_end_m=float(plan['s_m'][b])) for i,(a,b) in enumerate(turns)])
    (DEST/'validation.json').write_text(json.dumps(validation,indent=2),encoding='utf-8')
    write_csv('campus_results.csv',rows[:3]);write_csv('campus_max_error.csv',peaks)
    write_csv('campus_improvement.csv',[rows[0],rows[3]])
    fig,axes=plt.subplots(1,2,figsize=(12,7),gridspec_kw={'width_ratios':[1.05,1]})
    ax=axes[0];base_map(ax,plan)
    for i,(a,b) in enumerate(turns):
        j=(a+b)//2;ax.scatter(plan['x_m'][j],plan['y_m'][j],s=25,color='#555555');ax.annotate(f'Turn {i+1}',(plan['x_m'][j],plan['y_m'][j]),xytext=(9,7),textcoords='offset points')
    for j,label in [(0,'Start'),(-1,'Destination')]:
        ax.scatter(plan['x_m'][j],plan['y_m'][j],s=40,color='#7b3294');ax.annotate(label,(plan['x_m'][j],plan['y_m'][j]),xytext=(6,8),textcoords='offset points')
    ax.set_title('Campus smooth planning route')
    ax=axes[1];ax.plot(plan['s_m'],plan['target_speed_mps'],color=COLORS[1]);ax.set(xlabel='Route station [m]',ylabel='Shared reference speed [m/s]',title='3 m/s cap with curvature and acceleration limits');ax.grid(alpha=.2)
    twin=ax.twinx();twin.plot(plan['s_m'],plan['curvature_1pm'],color='#999999',alpha=.7);twin.set_ylabel('Reference curvature [1/m]',color='#777777')
    fig.text(.04,.02,'Background: OpenStreetMap contributors (ODbL). Labels anonymized; route coordinates retained.',fontsize=8)
    fig.tight_layout(rect=(0,.04,1,1));save(fig,'campus_route.png')
    fig,axes=plt.subplots(1,2,figsize=(13,7));base_map(axes[0],plan)
    for t,ref,label,color in zip(data[:3],refs[:3],LABELS,COLORS):
        axes[0].plot(t['x'],t['y'],lw=1,color=color,label=label)
        axes[1].plot(ref['s_m'][t['target_index'].astype(int)],t['lateral_error'],lw=1,color=color,label=label)
    axes[0].set_title('Three unchanged student controllers');axes[0].legend(fontsize=8)
    axes[1].set(xlabel='Reference station [m]',ylabel='Signed lateral error [m]',title='Tracking error along the route');axes[1].grid(alpha=.2);axes[1].legend()
    fig.tight_layout();save(fig,'campus_tracking.png')
    fig,axes=plt.subplots(1,3,figsize=(16,5.5))
    for ax,t,p,label,color in zip(axes,data,peaks,LABELS,COLORS):
        base_map(ax,plan);ax.plot(t['x'],t['y'],color=color,lw=1.6,label=label)
        ax.scatter(p['x_m'],p['y_m'],marker='X',s=80,color=color,zorder=5,label='Max |lateral error|')
        ax.set_xlim(p['x_m']-13,p['x_m']+13);ax.set_ylim(p['y_m']-13,p['y_m']+13)
        ax.set_title(f"{label}: {abs(p['lateral_error_m']):.3f} m\nt={p['time_s']:.1f} s, s={p['route_s_m']:.1f} m")
        ax.text(.02,.02,f"v={p['speed_mps']:.2f} m/s\nk={p['curvature_1pm']:.3f} 1/m\nsteer={np.rad2deg(p['steer_rad']):.1f} deg",transform=ax.transAxes,fontsize=9,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
        ax.legend(fontsize=8,loc='upper right')
    fig.tight_layout();save(fig,'campus_max_error.png')
    fig,axes=plt.subplots(1,2,figsize=(12,4.5))
    for i in (0,3):
        t,ref=data[i],refs[i];s=ref['s_m'][t['target_index'].astype(int)]
        axes[0].plot(s,t['lateral_error'],color=COLORS[i],label=LABELS[i]);axes[1].plot(s,t['speed'],color=COLORS[i],label=LABELS[i])
    for ax in axes:ax.set_xlabel('Reference station [m]');ax.grid(alpha=.2);ax.legend()
    axes[0].set(ylabel='Signed lateral error [m]',title='PP slowdown: error comparison');axes[1].set(ylabel='Actual speed [m/s]',title='Only cruise cap changed: 3 to 2 m/s')
    fig.tight_layout();save(fig,'campus_improvement.png')
    before=read_csv(ROOT/'data/route_before_adjustment.csv')
    fig,axes=plt.subplots(1,4,figsize=(18,5))
    for i,(ax,join) in enumerate(zip(axes,adjustment['joins'])):
        center=(join['old_station_start_m']+join['old_station_end_m'])/2
        j=int(np.argmin(abs(before['s_m']-center)));x,y=before['x_m'][j],before['y_m'][j]
        map_lines(ax)
        ax.plot(before['x_m'],before['y_m'],color='#888888',ls='--',label='Before')
        ax.plot(plan['x_m'],plan['y_m'],color='#0072b2',lw=2,label='Revised')
        ax.set(xlim=(x-17,x+17),ylim=(y-17,y+17),xlabel='East [m]',ylabel='North [m]',title=['Turn 1','Short transitions','Turn 2','Turn 3'][i])
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle(f"Local widening: minimum radius {adjustment['minimum_radius_m']:.2f} m; max geometric steer {adjustment['max_geometric_steer_deg']:.1f} deg < 35 deg")
    fig.tight_layout();save(fig,'campus_route_adjustment.png')
    report=['# Campus route tracking — Wang Yihan','',
      'This section uses the group student controllers and unchanged kinematic vehicle backend. The existing campus route has locally widened turns and is used directly as a smooth planning standard; no original corner polyline is used as a reference.','',
      '## Settings','',
      f"Route length: {plan['s_m'][-1]:.3f} m; {len(plan)} samples, 0.2 m spacing (short final interval). Map origin: (118.8145, 31.8885), WGS84 local spherical projection. The road sequence and endpoints are retained; local turn widening reduces the previous 885.417 m length to the value above.",
      '', 'Vehicle: student_car, wheelbase 2.5 m, lf=lr=1.25 m, maximum steering 35 deg; dt=0.1 s; initial speed 0; time limit 900 s. All three runs share the exact same geometry, speed profile, vehicle and controller defaults.',
      '', 'The cruise cap is 3 m/s. The common profile limits reference v²|k| to 0.7 m/s², uses forward/backward passes at 0.7/0.8 m/s², starts at a 0.5 m/s target and ends at zero. Actual tracking speed can differ from this target. MPC retains its original horizon and constraints.',
      '', '![Campus route and shared speed profile](campus_route.png)',
      '', '## Baseline results','',
      '| Algorithm | Reached | Arrival time (s) | Mean absolute lateral error (m) | Max absolute lateral error (m) | Finish distance (m) |',
      '|---|---|---:|---:|---:|---:|']
    for r in rows[:3]:report.append(f"| {r['algorithm']} | {r['reached_goal']} | {r['arrival_time_s']:.1f} | {r['mean_lateral_error_m']:.5f} | {r['max_lateral_error_m']:.5f} | {r['finish_error_m']:.5f} |")
    report+=['', '![Three-algorithm tracking comparison](campus_tracking.png)',
      '', 'Arrival means endpoint distance <1.5 m and speed <0.5 m/s, as in the group framework; finish distance is not lateral error. Failed runs would have a blank arrival time and a separate simulation duration. These are simulated travel times, not measured campus commute times.',
      '', '## Maximum deviation','',
      '| Algorithm | Time (s) | Reference station (m) | Signed error (m) | Actual speed (m/s) | Reference curvature (1/m) | Steer (deg) | Turn / phase |',
      '|---|---:|---:|---:|---:|---:|---:|---|']
    for p in peaks[:3]:report.append(f"| {p['algorithm']} | {p['time_s']:.1f} | {p['route_s_m']:.2f} | {p['lateral_error_m']:.5f} | {p['speed_mps']:.3f} | {p['curvature_1pm']:.4f} | {np.rad2deg(p['steer_rad']):.2f} | {p['nearest_major_turn']} / {p['phase_relative_to_turn']} |")
    report+=['', '![Maximum deviation locations](campus_max_error.png)',
      '', 'Peaks are selected from the same log row by argmax(abs(lateral_error)). Station/curvature refer to the controller-selected reference point. campus_max_error.csv records both vehicle coordinates and reference-point coordinates, explicitly distinguished. Turn intervals use connected |curvature|>0.015 1/m regions with integrated heading change >30 degrees.',
      '', 'Peak diagnostics: ' + '; '.join(f"{p['algorithm']} peaks {p['phase_relative_to_turn']} Turn {p['nearest_major_turn']}, {p['station_offset_from_turn_curvature_peak_m']:+.1f} m relative to its curvature maximum, steering {np.rad2deg(p['steer_rad']):.1f} degrees" for p in peaks[:3]) + '. The signed station offset is a spatial comparison, not a measured controller delay. Actual speed need not equal the local reference speed.',
      '', '## Feasibility and interpretation','',
      f"The revised maximum reference curvature is {validation['reference_max_curvature_1pm']:.4f} 1/m. The CG bicycle steering limit is tan(delta_max)/sqrt(L² + lr² tan²(delta_max)) = {max_k:.4f} 1/m. No reference segment exceeds this limit. Dense checking gives a minimum radius of {adjustment['minimum_radius_m']:.2f} m and maximum geometric steer of {adjustment['max_geometric_steer_deg']:.2f} degrees, below 35 degrees. Local quintic joins preserve position, tangent and second derivative at their boundaries. Three principal turns and a pair of short high-curvature transitions were widened; the start and destination did not move. The maximum same-parameter shift from the old smooth route is {adjustment['maximum_same_parameter_displacement_m']:.2f} m. This establishes steering-geometric feasibility, not collision-free road access.",
      '', 'Nonzero tracking errors remain possible despite a geometrically feasible reference, because preview behavior, finite sampling and transient speed/steering response still matter. Different peak positions do not by themselves establish a causal control delay. The full-route mean is strongly diluted by long straight sections; inspect peak and local trajectories as well. normal_accel in the group log is v² times reference curvature, not independently measured actual normal acceleration.',
      '', '![Local route adjustment](campus_route_adjustment.png)',
      '', 'The route change does not improve every tracking metric: PP maximum error changes from 1.376 to 0.774 m, MPC from 0.405 to 0.326 m, while LQR changes from 0.535 to 0.556 m. LQR and MPC full-route mean errors also increase relative to the previous route. This is a route-feasibility correction, not controller tuning. Curvature-based speed targets change with the route, so this before/after comparison is not an isolated curvature-only experiment. See campus_route_change.csv for the original and revised metrics.',
      '', '## One-variable improvement trial','']
    r0,r1=rows[0],rows[3]
    report += [f"Only the PP cruise cap changes from 3 to 2 m/s; the same curvature/acceleration planning rule, geometry, controller and vehicle remain. Mean error changes from {r0['mean_lateral_error_m']:.5f} to {r1['mean_lateral_error_m']:.5f} m ({100*(1-r1['mean_lateral_error_m']/r0['mean_lateral_error_m']):.1f}% lower); maximum error from {r0['max_lateral_error_m']:.5f} to {r1['max_lateral_error_m']:.5f} m ({100*(1-r1['max_lateral_error_m']/r0['max_lateral_error_m']):.1f}% lower). Arrival time increases from {r0['arrival_time_s']:.1f} to {r1['arrival_time_s']:.1f} s. The revised route already satisfies the geometric steering limit; this trial evaluates remaining tracking transients. In the tightest curve, curvature-based speed limiting already dominates, so reducing cruise speed need not proportionally reduce the peak.",
      '', '![PP improvement trial](campus_improvement.png)',
      '', '## Reproduction and limitations','',
      'See [readme_wyh.md](../readme_wyh.md) for exact commands and merge instructions. Per-run config.json records every default, source hashes and output path. campus_results/validation.json checks shared settings, geometry, coordinate conversion and independently recomputed lateral-error metrics.',
      '', 'The route is planned, not a measured GNSS track. No pedestrians, obstacles, road rights, signals, collision checking, tire-force saturation or real campus speed restrictions are simulated. Map lines are background only. Generic labels remove building names but do not anonymize the GPX coordinates. Map data: © OpenStreetMap contributors, ODbL; https://www.openstreetmap.org/copyright.']
    (DEST/'Campus_Analysis_EN.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    print(json.dumps(dict(results=rows,peaks=peaks,validation=validation),indent=2))


if __name__=='__main__':
    main()
