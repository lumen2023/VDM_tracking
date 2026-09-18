"""Build the audit, requirement traceability and technical report from saved data."""
from __future__ import annotations
import json
from pathlib import Path
import pandas as pd

HERE=Path(__file__).resolve().parent

AUDIT_ROWS=[
('README / teaching guide','Documentation mismatch','The four student controller TODO families already contain implementations; root text still calls them unfinished templates.','Record actual completion and errata here; preserve the supplied README.','AUDIT.md; original source inspection'),
('PP target geometry','Corrected','Steering divided by nominal lookahead even when the selected point has a different chord length.','Use the actual vehicle-to-target chord; guard zero distance.','pure_pursuit.py; exact-chord regression test'),
('LQR Riccati solve','Corrected','A finite iterate could be returned after exhausting the iteration budget without convergence.','Validate symmetric PSD Q and positive-definite R; use a residual-checked SciPy DARE fallback.','lqr_kinematic.py; forced one-iteration and closed-loop stability tests'),
('LQR–K model scope','Retained, qualified','The educational four-state realization is a simplified no-CG-slip approximation, not the full CG plant.','Keep the intended baseline model; explicitly compare it on three plant types.','lqr_kinematic.py; plant_comparison runs'),
('LQR–D state feedback','Extended','The shared state interface omits lateral velocity and yaw rate; original control uses kinematic proxies.','Accept optional measured lateral_velocity / measured_yaw_rate; retain compatible proxy behavior for the root CLI.','lqr_dynamic.py; paired dynamic_feedback_ablation'),
('LQR–D tire convention','Already correct','Cf and Cr are axle stiffnesses; supplied feedforward already omits an erroneous per-tire factor of two.','Preserve that correct implementation; explain the convention and neutral-steer default.','lqr_dynamic.py; existing feedforward regression test'),
('MPC plant equations / Jacobians','Already correct','The supplied nonlinear predictor and linearization already include CG sideslip consistently.','Preserve formulas and finite-difference regression coverage; do not claim them as new fixes.','mpc.py; original Jacobian/predictor tests'),
('MPC yaw reference','Corrected','Path yaw is course direction, while the state yaw is vehicle body heading.','Set body reference psi = chi - beta; unwrap to the nearest current-heading branch.','mpc.py; body-heading and branch-cut tests'),
('MPC reference sampling','Corrected','Discrete waypoint jumps and nonphysical path s distort horizon preview.','Project locally and interpolate continuously in sampled geometric arc length.','mpc.py; geometric-arclength tests'),
('MPC dependency / reporting','Extended','CVXPY is unavailable in this environment; silently substituting a controller would invalidate comparisons.','Add an explicit SciPy condensed-QP backend with bounds, rate limits and solve-status checks; retain CVXPY path.','qp_solver.py; full MPC QP cross-check against independent OSQP'),
('Core path distance field','Student-side mitigation','Spline parameter s is based on original waypoint chords, not the resulting spline’s physical arc length.','Recompute the injected Path.s and distance-based braking in the student research harness; no shared-file edit.','experiments.py; prepared_path'),
('Core acceleration log','Student-side mitigation','Core normal_accel is v² times reference curvature, a demand signal rather than measured trajectory normal acceleration.','Log reference demand separately; derive actual course-normal acceleration from plant motion.','experiments.py; trajectory.csv schema'),
('Lateral-error evaluation','Corrected in new evaluator','Waypoint error and endpoint Euclidean overshoot can conflate cross-track and along-track displacement.','Use signed local segment-normal error; retain original error and separate point-to-segment distance.','experiments.py; endpoint projection regression test'),
('Completion and failed runs','Student-side mitigation','Endpoint arrival is insufficient to judge tracking; core thresholds are hard-coded.','Use configured thresholds plus a progress guard; also publish a labeled 1 m peak-error diagnostic and keep failures.','experiments.py; stress_failure and GPX comparisons'),
('Comparison plotting','Replaced','Hard-coded missing timestamp folders, mixed-unit comparisons, signed-error CDF and misleading error-vector geometry.','Provide CLI-driven, single-axis figures from saved logs; absolute-error CDF, units, captions, PNG/SVG and source index.','visualizations.py; plot_comparison.py'),
('Experiment coverage','Completed except provenance task','The submission did not supply a reproducible sensitivity / identification evidence package.','Add 139 closed-loop cases, synthetic model reconstruction, held-out comparisons, tests, raw logs and deck.','results/; REPORT.md; deliverables/'),
('Independent personal GPX','Outstanding user-specific input','A supplied homework GPX does not prove independent dormitory-to-classroom route planning.','Replay the supplied GPX honestly; preserve privacy and request student-owned route provenance before claiming this item.','REPORT.md, GPX section'),
]


def md_table(headers,rows):
    return '| '+' | '.join(headers)+' |\n| '+' | '.join(['---']*len(headers))+' |\n'+'\n'.join('| '+' | '.join(str(c).replace('|','/').replace('\n',' ') for c in r)+' |' for r in rows)


def main():
    d=pd.read_csv(HERE/'results/summary.csv');ident=json.loads((HERE/'results/identification/identification_summary.json').read_text())
    proof=json.loads((HERE/'results/validation/core_integrity.json').read_text());checks=json.loads((HERE/'results/validation/trajectory_checks.json').read_text())
    labels={'pp':'PP','lqr_kinematic':'LQR–K','lqr_dynamic':'LQR–D','mpc':'MPC'}
    benchmark=d[d.group=='route_benchmark']
    bench=md_table(['Route (requested m/s)','Controller','Goal','MAE (m)','RMSE (m)','Peak |e| (m)','Mean |δ̇| (°/s)'],
        [[f'{r.route} ({r.speed:g})',labels[r.algo],r.reached_goal,f'{r.mae_lateral_m:.4f}',f'{r.rmse_lateral_m:.4f}',f'{r.max_lateral_m:.4f}',f'{r.steer_rate_mean_abs_deg_s:.2f}'] for _,r in benchmark.iterrows()])
    speed_table=md_table(['Requested speed (m/s)','PP RMSE (m)','LQR–K RMSE (m)','LQR–D RMSE (m)','MPC RMSE (m)'],
        [[v,*[f'{d[(d.group=="speed_scurve")&(d.algo==a)&(d.speed==v)].rmse_lateral_m.iloc[0]:.4f}' for a in labels]] for v in [3,5,7,9,11]])
    params=md_table(['Parameter','Truth','Estimate','Approximate 95% interval','Relative error'],
        [[p['parameter']+' ('+p['unit']+')',f"{p['true_value']:.6g}",f"{p['estimate']:.8g}",f"[{p['ci95_low']:.8g}, {p['ci95_high']:.8g}]",f"{p['relative_error_pct']:+.4f}%"] for p in ident['parameters']])
    holdout=md_table(['Longitudinal u (m/s)','Model','Yaw-rate RMSE (rad/s)','Lateral-velocity RMSE (m/s)','Position RMSE (m)'],
        [[r['speed_mps'],r['model'],f"{r['yaw_rate_rmse_radps']:.6g}",f"{r['lateral_velocity_rmse_mps']:.6g}",f"{r['position_rmse_m']:.6g}"] for r in ident['holdout_metrics']])
    gpx=md_table(['Controller / requested speed','Goal','Successful duration (s)','Observed end time (s)','RMSE (m)','Peak |e| (m)','Maximum core-error reference location (lat, lon)'],
        [[f'{labels[r.algo]} / {r.speed:g} m/s',r.reached_goal,'—' if pd.isna(r.successful_duration_s) else f'{r.successful_duration_s:.1f}',f'{r.simulated_duration_s:.1f}',f'{r.rmse_lateral_m:.4f}',f'{r.max_lateral_m:.4f}',f'({r.max_error_ref_lat:.6f}, {r.max_error_ref_lon:.6f})'] for _,r in d[d.gpx].iterrows()])
    design=md_table(['Study group','Runs','Purpose'],[[g,len(q),{'route_benchmark':'Four routes × four controllers','circle_speed':'Five speeds × four controllers on R = 12 m circle','speed_scurve':'Five speeds × four controllers on transient S-curve','plant_comparison':'Three plants × four controllers','lookahead':'Seven PP base lookaheads × three speeds','steer_limit':'Four steering bounds × PP / LQR–K','stiffness':'Five physical stiffness scales; matched and nominal controller models','mpc_horizon':'Four MPC horizons','sampling':'Four sampling intervals × PP / LQR–D','baseline':'Original vs revised paired comparisons','mpc_heading_ablation':'Course/body-heading reference ablation','dynamic_feedback_ablation':'Measured vs proxy dynamic states','gpx':'Supplied GPX at 4 and 8 m/s with PP','stress_failure':'Severely infeasible circle steering limit','gpx_comparison':'LQR–K and MPC on same GPX at 8 m/s'}[g]] for g,q in d.groupby('group',sort=False)])
    audit=md_table(['Area','Status','Finding','Fix / treatment','Evidence'],AUDIT_ROWS)
    todo=md_table(['Required TODO family','Original archive status','Revised verification'],[
        ['Pure Pursuit: lookahead, target, alpha, steering','Implemented','Actual chord formula and zero-distance handling tested'],
        ['Kinematic LQR: model, Riccati, feedback, feedforward','Implemented','Weight validation, converged fallback and stability tested'],
        ['Dynamic LQR: minimum speed, A/B, discretization, feedforward','Implemented','Axle-stiffness convention tested; measured-state extension and ablation'],
        ['MPC: horizon reference, predictor, linearization, QP, receding horizon','Implemented','Heading / sampling corrections; actual MPC QP cross-checked with OSQP'],
    ])
    trace=md_table(['Requirement','Evidence','Status'],[
        ['Why tracking becomes harder with speed','REPORT §§3,7; 20 circle + 20 S-curve runs; acceleration / yaw / steering plots','Completed with non-monotonic qualification'],
        ['Three key variables','Speed, PP base lookahead, steering-angle limit; 69 dedicated cases across these groups','Completed'],
        ['Reconstruct and compare models','Synthetic L / lr / Cf / Cr identification; disjoint holdouts; 12 closed-loop plant comparisons','Completed for synthetic data, not real vehicles'],
        ['At least three routes and nine controller runs','Four routes × PP / LQR–K / LQR–D / MPC = 16 runs','Completed'],
        ['Circle at 3, 5, 7 m/s with PP / LQR–K / MPC','Expanded to five requested speeds and four controllers; explicit steady window','Completed'],
        ['Vary lf, lr, or max_steer','max_steer = 12 / 18 / 25 / 35° for PP and LQR–K','Completed'],
        ['GPX replay, duration and maximum-error location','Four supplied-route runs; same-origin offline map; failed LQR–K duration marked invalid','Completed for supplied GPX'],
        ['Independently planned dormitory-to-classroom GPX','No student-owned route provenance supplied','Outstanding; not fabricated'],
        ['Keep non-student core code unchanged','SHA-256 verification of 56 original non-student files; no added non-student files','Completed'],
    ])
    (HERE/'AUDIT.md').write_text('# Audit and requirement traceability\n\n## TODO completion\n\n'+todo+'\n\nLiteral implementation gaps and scientific validity are different questions. The uploaded controllers were not empty templates. Two `pass` statements are exception fall-throughs, not unimplemented control formulas. Root documentation is preserved as requested.\n\n## Findings and fixes\n\n'+audit+'\n\n## Requirement coverage\n\n'+trace+'\n',encoding='utf-8')
    report=r'''# VDM autonomous driving: modeling, tracking and evidence

## Executive findings

This is a student-only revision of the supplied second tracking experiment. All four controller TODO families were already implemented in the archive. The work therefore corrects numerical, geometric, reference-frame and evaluation issues rather than claiming to have filled empty templates. The original files outside `vdm_lab/student` remain byte-for-byte unchanged: **56 SHA-256 checks pass**, and no files were added outside that folder.

The evidence package contains **139 deterministic closed-loop simulations**, **49,403 logged samples**, **36 figures in PNG and SVG**, noisy synthetic parameter reconstruction, disjoint holdout experiments and **22 unit tests: 21 pass, one CVXPY-specific test is skipped** because that optional dependency is unavailable here. The executed MPC backend is a genuine condensed convex QP solved through SciPy SLSQP; it is not a substitute feedback controller. Its full QP is independently cross-checked with OSQP through an optional CasADi bridge. Across the recorded study, **9,537 MPC control calls report successful SciPy solves and zero safe-stop fallback calls**. This is not a count of all inner sequential-linearization QPs.

MPC has the lowest mean and peak lateral error on all four idealized benchmark routes at the fixed tested settings. That result does not generalize automatically: on the supplied sparse GPX, PP achieves lower RMSE and lower peak error than MPC, and LQR–K does not reach the goal. Likewise, circle steady-state error is not monotonically increasing with speed, even though normal-acceleration demand grows quadratically. These negative and non-monotonic results are retained.

Only **137 of 139** runs meet the endpoint goal condition. A separate analyst diagnostic requiring both endpoint arrival and peak lateral error ≤ 1 m is met by **97 of 139**; this deliberately broad stress-study fraction is not a deployment success rate or a course grading threshold. Infeasible low-steering-limit runs can reach the endpoint while missing the path by more than 33 m.

The only user-specific course item not claimed complete is an **independently planned dormitory-to-classroom GPX**. The provided homework GPX is replayed and analyzed, but it is not relabeled as the student’s own route.

## 1. Scope, reproducibility and audit

`AUDIT.md` provides a line-item finding/fix table and a requirement-to-evidence mapping. Revised controller interfaces remain compatible with the course CLI. New orchestration, evaluation, reconstruction, plots, tests and deliverables live only inside `student`. The root README, shared dynamics, configuration dataclasses, paths and simulator are untouched.

The student research harness intentionally differs from the original runner in three documented evaluation details. It injects physical sampled arc length into the existing path object and recomputes distance-based braking; it uses configuration values rather than hard-coded goal thresholds and adds a progress guard; and it logs vehicle-motion quantities separately from reference demands. Therefore these research metrics should not be mixed silently with old root-runner metrics. Original controller implementations are compared against revised ones under the **same student harness**, isolating controller-code differences as far as the fixed configuration allows. Original MPC is not given a fabricated before/after score because its CVXPY path could not be executed in this environment.

The tested environment is Python 3.13.5, NumPy 2.3.5, SciPy 1.17.0 and Matplotlib 3.10.8. The root environment instructions pin NumPy below 2 and remain untouched. `requirements_student.txt` supplies the alternative student-side dependencies. Timing results are descriptive measurements on this host, not worst-case real-time guarantees. Noise appears only in the synthetic identification training observations; the closed-loop benchmark states are ideal simulator states, without localization noise or latency.

## 2. Frames and kinematic reconstruction

Let the center of gravity be C, the front and rear axle distances be l_f and l_r, and L = l_f + l_r. Body heading ψ is the longitudinal-axis orientation. Course direction χ is the velocity orientation, with χ = ψ + β. Front steering is δ; the rear axle does not steer. The CG kinematic plant’s V is **speed magnitude**, not necessarily body-longitudinal velocity u. The dynamic plant uses u and lateral velocity v_y, so V = sqrt(u² + v_y²). [R1, R5]

For pure rolling with rear steering zero, the rear axle turning radius satisfies R_r = L / tan δ. The CG geometry gives tan β = l_r / R_r, hence

$$\beta=\arctan\!\left(\frac{l_r}{L}\tan\delta\right).$$

Because R_r = R cos β and r = V/R, the yaw rate and world-frame motion follow:

$$\dot X=V\cos(\psi+\beta),\quad \dot Y=V\sin(\psi+\beta),\quad r=\dot\psi=\frac{V}{L}\tan\delta\cos\beta,\quad \dot V=a.$$

These equations correspond to `state.x`, `state.y`, `state.yaw`, `state.v`, `steer`, `beta` and `yaw_rate`; see the unchanged `common/bicycle_model.py` and `common/vehicle_backend.py`. The simplified plant instead sets β = 0 and uses r = V tan δ/L. It is a useful low-complexity approximation, but it is not identical to a CG-centered no-slip model. Model selection should consider operating conditions and discretization, not just the label “dynamic.” [R5]

For a constant-curvature CG circle, β is constant and κ = r/V. Eliminating β yields the exact geometric inverse

$$\delta_{ss}=\arctan\!\left[\frac{L\kappa}{\sqrt{1-(l_r\kappa)^2}}\right],\quad |l_r\kappa|<1.$$

The familiar atan(Lκ) is a small-sideslip approximation. A 12 m circle with L = 2.5 m and l_r = 1.25 m needs about 11.83° exact steady steering; a 5 m fillet needs about 27.31°. A steering bound that cannot supply the required curvature cannot be repaired merely by a better tracking controller.

Pure Pursuit constructs a circular arc through a lookahead target. With target bearing α and actual target chord d, its geometric law is κ_cmd = 2 sin α/d and δ_cmd = atan(L κ_cmd). [R2] The revised implementation uses the **actual** d, not just the nominal lookahead L_0 + 0.35 V, because discrete point selection and path endings generally make these distances unequal. PP’s retained steering law still represents its simplified geometric controller; correcting the chord does not turn it into a full CG or tire-dynamic controller.

## 3. Why speed generally makes tracking harder

For a fixed path, curvature converts speed into yaw and lateral acceleration demands:

$$r_{ref}\simeq V\kappa,\qquad a_{n,ref}=V^2\kappa.$$

On a 12 m circle, the theoretical demand rises from 0.75 m/s² at 3 m/s to 4.083 m/s² at 7 m/s and 10.083 m/s² at 11 m/s. These are reference-demand calculations, not proof that every run achieves the requested speed or that a real tire can supply the acceleration. A real friction envelope would require approximately V²|κ| ≤ μg; μ is not identified or enforced by this lab model. [R1]

Several mechanisms explain the increased difficulty. For small errors, cross-track error grows approximately as ė_y ≈ V(e_ψ + β), so a fixed angular mismatch produces greater lateral motion at greater speed. A delay τ also advances the vehicle by Vτ before correction. A spatial curvature transition is traversed faster: with the small-slip steering approximation,

$$\dot\delta_{ref}\simeq\frac{L V\,d\kappa/ds}{1+(L\kappa)^2}.$$

Thus entry/exit and S-curve transitions challenge steering-rate limits even if steady circular steering is speed-independent. At sample interval Δt = 0.1 s, nominal forward travel per update increases from 0.3 m at 3 m/s to 1.1 m at 11 m/s. Speed-dependent tire dynamics and model mismatch add transient phase lag; actuator angle, rate and longitudinal limits can prevent the ideal reference from being realized.

The qualification is essential: **greater physical demand does not imply monotonic error growth in every ideal simulation**. Steady curvature has dκ/ds = 0. The kinematic plant has neither finite tire friction nor tire relaxation; scheduled feedback gains and preview can compensate over some speeds. The supplied dynamic parameters are symmetric, l_f = l_r and C_f = C_r, giving zero linear understeer gradient. It would be incorrect to explain every observed trend by increasing understeer in this default car.

## 4. Dynamic bicycle model and LQR

With positive axle cornering stiffnesses and small slip angles,

$$\alpha_f\simeq\delta-\frac{v_y+l_f r}{u},\quad \alpha_r\simeq-\frac{v_y-l_r r}{u},\quad F_{yf}=C_f\alpha_f,\quad F_{yr}=C_r\alpha_r.$$

Force and moment balance give m(v̇_y + ur) = F_yf + F_yr and I_z ṙ = l_f F_yf − l_r F_yr. Consequently, for z = [v_y, r]ᵀ,

$$\dot z=\begin{bmatrix}
-\frac{C_f+C_r}{mu}&\frac{-l_fC_f+l_rC_r}{mu}-u\\
\frac{-l_fC_f+l_rC_r}{I_z u}&-\frac{l_f^2C_f+l_r^2C_r}{I_z u}
\end{bmatrix}z+\begin{bmatrix}C_f/m\\l_fC_f/I_z\end{bmatrix}\delta.$$

The source uses **axle**, not per-tire, stiffness. Adding another factor of two would create an inconsistency. The 1/u terms also make the model ill-conditioned at low speed; clamping the model speed to 0.5 m/s regularizes numerical control but does not make the dynamic tire assumptions valid at a standstill. The dynamic plant is integrated with RK4 and a 2 ms internal substep.

For the dynamic tracking controller, x_e = [e_y, ė_y, e_ψ, ė_ψ]ᵀ. The homogeneous straight-reference matrix is

$$A_c=\begin{bmatrix}
0&1&0&0\\
0&-\frac{C_f+C_r}{mu}&\frac{C_f+C_r}{m}&\frac{l_rC_r-l_fC_f}{mu}\\
0&0&0&1\\
0&\frac{l_rC_r-l_fC_f}{I_z u}&\frac{l_fC_f-l_rC_r}{I_z}&-\frac{l_f^2C_f+l_r^2C_r}{I_z u}
\end{bmatrix},\quad B_c=\begin{bmatrix}0\\C_f/m\\0\\l_f C_f/I_z\end{bmatrix}.$$

Curvature enters as a forcing term handled by feedforward rather than shown in this homogeneous matrix. Tustin discretization is A_d = (I − Δt A_c/2)⁻¹(I + Δt A_c/2), B_d = (I − Δt A_c/2)⁻¹ Δt B_c. The kinematic LQR retains the educational realization A_d = [[1,Δt,0,0],[0,0,V,0],[0,0,1,Δt],[0,0,0,0]] and B_d = [0,0,0,V/L]ᵀ. It resets derivative states from small-angle relations; it is not simply an Euler discretization of the full CG model.

LQR minimizes Σ(x_eᵀQx_e + δ_fbᵀRδ_fb). Its DARE and gain are [R3]

$$P=Q+A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA,\qquad K=(R+B^TPB)^{-1}B^TPA,$$

with δ = −Kx_e + δ_ff. A finite unconverged iterate is not an acceptable solution certificate; the revision validates weights, checks convergence, and uses a residual-checked QZ-based DARE solver if needed. Closed-loop spectral radius is checked by the controllers. The default weights remain Q = diag(2,0.2,3,0.2), R = 1.5.

At steady curvature, the dynamic open-loop feedforward has δ_ss = Lκ + K_u u²κ, where K_u = m(l_r/C_f − l_f/C_r)/L. The source additionally compensates feedback acting on the nonzero steady body/course-heading difference. For this symmetric car K_u = 0. Optional measured v_y and r now supply ė_y = v_y cos e_ψ + u sin e_ψ and ė_ψ ≈ r − uκ. The root simulator still lacks those state fields and therefore uses the compatible proxy branch; the measured-state research results must not be attributed to the unchanged root interface.

## 5. MPC derivation and solver verification

The MPC state is z = [X,Y,V,ψ]ᵀ and input is w = [a,δ]ᵀ. The nonlinear Euler predictor follows the CG equations from §2. Around nominal (z̄,w̄), its affine model is z_{k+1} = A_k z_k + B_k w_k + c_k, with A_k = ∂f_d/∂z, B_k = ∂f_d/∂w and c_k = f_d(z̄,w̄) − A_k z̄ − B_k w̄. The steering derivative uses

$$\frac{d\beta}{d\delta}=\frac{(l_r/L)\sec^2\delta}{1+(l_r/L)^2\tan^2\delta}.$$

For example, B_{X,δ} = −Δt V sin(ψ+β) β′ and B_{Y,δ} = Δt V cos(ψ+β) β′. The yaw derivative is B_{ψ,δ} = Δt V[sec²δ cosβ − tanδ sinβ β′]/L. The original predictor and these Jacobians were already consistent; tests retain finite-difference checks.

The reference is interpolated along geometric path distance and uses **ψ_ref = χ_ref − β_ref**. Correcting this frame distinction is different from changing the predictor. The ideal steady β_ref is clipped to actuator-feasible curvature before inversion; this does not make an impossible path feasible. On variable curvature, this remains a practical local steady reference rather than an exact dynamically feasible trajectory.

The finite-horizon cost is

$$J=\sum_{k=0}^{N-1}\left(\|z_k-z_k^{ref}\|_Q^2+\|w_k\|_R^2\right)+\|z_N-z_N^{ref}\|_{Q_f}^2+\sum_{k=0}^{N-2}\|w_{k+1}-w_k\|_{R_d}^2.$$

Speed, acceleration, steering angle and steering-rate limits are enforced, including the first change from previously applied steering. There is **no hard lateral-error corridor, collision avoidance or tire-friction constraint**. Repeated optimization applies only the first control, then updates the state and horizon. [R4]

The portable solver eliminates all states: z_k = d_k + F_k W, where W stacks 2N controls. Substitution yields a convex quadratic objective and linear inequalities. Local x/y translation and input scaling reduce avoidable numerical conditioning problems. SLSQP receives analytic gradients and rejects unsuccessful, nonfinite or constraint-violating results (primal tolerance 10⁻⁶). The original CVXPY implementation remains available, but all study cases explicitly select SciPy; the optional CVXPY-dependent test is not reported as passed. An independent full-problem OSQP cross-check agrees within the test tolerance of 5×10⁻⁵ for controls and predicted states.

## 6. Experimental design and evaluation

Default vehicle parameters are L = 2.5 m, l_f = l_r = 1.25 m, m = 1140 kg, I_z = 1436.24 kg·m², C_f = C_r = 155,494.663 N/rad. Unless varied, Δt = 0.1 s, steering limit is 35°, rate limit 45°/s, acceleration bounds are −3.5 to +2.0 m/s², and speed bounds are 0 to 12 m/s. All closed-loop runs start on the path at zero speed. The speed reference includes distance-based braking V_ref(s) = min(V_target, sqrt(1.8·remaining_distance)); consequently a short route may never reach the requested cruise speed.

__DESIGN__

The primary error is the signed component normal to the locally selected polyline segment. Its whole-run MAE, RMSE, 95th percentile and maximum absolute value are reported. Original tangent/waypoint error is retained separately. Euclidean endpoint distance and completion fraction are separate, so stopping overshoot is not mislabeled as lateral deviation. A distance-weighted RMSE is also saved; time-averaged and distance-averaged metrics need not agree when speed varies. The reference tracker still chooses a local ordered neighborhood, which can be ambiguous on self-intersections or very large excursions; these are not globally map-matched road errors.

For circular steady-state results, the analysis uses the central 60% of the known circular arc and samples with V ≥ 0.95 V_target, requiring at least five samples. It does not hand-pick a visually favorable interval. Theoretical acceleration is averaged as mean(V²/R), not computed from the square of the mean when speed varies.

The original core normal-acceleration column is a **reference demand**, V²κ_ref. A separate vehicle-motion estimate is used for validation: a_n = V(r + β̇). For the kinematic plant, β̇ is estimated by centered finite differences of unwrapped β, so rapidly changing steering can make the sampled estimate sensitive to Δt. For the dynamic plant, β = atan2(v_y,u) gives a_n = V r + (u v̇_y − v_y u̇)/V. Body-lateral acceleration v̇_y + ur is also recorded and is not silently equated with course-normal acceleration.

## 7. Results: controllers and the three key variables

### 7.1 Fixed-configuration multi-route comparison

__BENCH__

MPC ranks first in both MAE and maximum error on these four benchmark routes, not merely in RMSE. On the S-curve at 6.5 m/s, RMSE is 0.4417 m for PP, 0.2915 m for LQR–K, 0.0808 m for LQR–D and 0.0226 m for MPC. Median controller-call times are approximately 0.033, 1.693, 1.605 and 7.386 ms respectively. The lower error therefore has a measured computational cost. LQR–K also has the largest mean absolute steering rate in this particular run (22.92°/s); optimal quadratic feedback is not synonymous with the smoothest applied steering under saturation.

The maximum-error location plot overlays each controller’s peak-error progress on the reference curvature profile. Error peaks may be displaced from curvature extrema, consistent with preview, finite response and transient lag. This plot alone cannot identify a single causal mechanism because feedback, longitudinal motion and constraints interact.

![Benchmark accuracy](deliverables/figures/04_benchmark_accuracy.png)

### 7.2 Variable 1 — requested speed

__SPEED__

At 7 m/s requested speed, S-curve MPC RMSE is 0.0252 m; at 11 m/s it is 0.0495 m. LQR–D changes from 0.0741 to 0.5338 m, while LQR–K changes from 0.2899 to 0.5288 m. However, **the 11 m/s requests achieve maxima of only 8.77–9.04 m/s**. These runs compare complete constrained maneuvers under requested-speed changes; they are not steady 11 m/s tests. The high-request transients are harder even without realizing the nominal speed throughout the route.

The circle gives the necessary counterexample to a blanket monotonic claim. MPC steady RMSE is 0.0136, 0.0111, 0.0031, 0.0123 and 0.0263 m across the five speed settings. PP and LQR–K retain approximately 0.4 m bias over much of the sweep. Their steady error is influenced by controller/CG geometry and preview, not by a simulated friction limit. Vehicle-derived normal acceleration agrees with V²/R: the MPC relative mismatch remains below 0.13% in the specified windows. At the 11 m/s request, the MPC window’s mean achieved speed is about 10.623 m/s.

![Speed sensitivity](deliverables/figures/07_speed_sensitivity.png)

### 7.3 Variable 2 — PP base lookahead

A 7×3 sweep uses L_0 = 1, 1.5, 2, 3, 4.5, 6 and 8 m at requested speeds 3, 7 and 11 m/s. At 7 m/s, RMSE increases from 0.2365 m at L_0 = 1 m to 0.4461 m at 3 m and 1.1970 m at 8 m. The associated peak error increases from 0.6324 to 2.9921 m. Longer preview cuts across the S-curve in this setting; its smoothing benefit is not sufficient to offset the geometric error.

The experiment does **not** establish that the smallest lookahead is universally optimal. Localization noise, steering delay, other curvatures and speeds outside the tested range could reverse the trade-off. The code also adds a speed-dependent lookahead term, so L_0 is not the whole physical preview distance.

![Lookahead sensitivity](deliverables/figures/13_lookahead_sensitivity.png)

### 7.4 Variable 3 — steering-angle limit

On the right-angle route at requested 7 m/s, reducing the bound to 12° gives peak errors of 3.495 m for PP and 5.156 m for LQR–K. At 35°, these are 1.129 and 0.479 m. A 5 m fillet requires about 27.31° for exact steady CG tracking. Constraint-limited paths must therefore be slowed, smoothed or declared infeasible rather than “solved” by unconstrained gain changes alone; slowing can reduce dynamic/rate demand but cannot change the minimum geometric turning radius at a fixed angle bound.

The trend is not uniformly monotonic: LQR–K’s 25° peak is 0.289 m, lower than its 35° result. Saturation changes the nonlinear closed-loop response and can suppress an overshoot for a specific maneuver; this is a local observation, not evidence that 25° exactly tracks the 5 m circle. In an additional 5°-limited 12 m circle stress test, PP and LQR–K reach the endpoint with peak deviations above 33 m, whereas MPC times out with peak error 11.38 m. All three fail the separate 1 m tracking-quality diagnostic.

![Steering-limit sensitivity](deliverables/figures/15_steering_limit_sensitivity.png)

## 8. Extensions: mismatch, horizon, sampling and corrections

Controller–plant comparison separates the design model from the simulated physical model. On the simplified plant, LQR–K RMSE is 0.1477 m versus MPC 0.2149 m. On the CG plant, MPC is 0.0252 m versus LQR–K 0.2899 m. On the dynamic plant, MPC is 0.0350 m and measured-state LQR–D is 0.0462 m. Changing the plant changes the ranking. These closed-loop comparisons retain the interface’s slightly different speed conventions; the open-loop reconstruction experiment below performs the explicit u-to-V conversion.

Scaling both true axle stiffnesses from 0.5 to 1.5 of nominal and comparing matched with fixed-nominal LQR–D models reveals no universal matched-model advantage. At half stiffness, matched-controller RMSE is 0.1801 m versus 0.0954 m with nominal controller stiffness. This is compatible with interactions among fixed Q/R, feedforward, discretization and saturation; it is not evidence that incorrect physical parameters are generally preferable. Peak tire slips in this aggressive closed-loop sweep reach 5.8–8.7°, outside a conservative 5° small-slip warning region, so quantitative tire-force realism is limited.

At requested 9 m/s on the S-curve, MPC horizons N = 4,8,12,16 give RMSE 0.0539, 0.0445, 0.1282 and 0.2588 m. Median call time grows from 5.38 to 43.78 ms. A longer horizon with unchanged weights, local reference-speed heuristics and linearization is not guaranteed to improve accuracy. These are frozen-tuning sensitivity results, not separately optimized controllers.

Changing controller Δt from 0.2 to 0.025 s is also not a pure numerical-convergence test: it changes the discrete feedback, preview and number of updates with the same per-step weights. The LQR–D result deteriorates at the smallest tested interval. To distinguish this from integration error, a separate open-loop fixed-input RK4 substep study compares 4,2,1 ms against 0.5 ms. Yaw-rate errors fall from 3.01×10⁻⁸ to 1.77×10⁻⁹ to 1.01×10⁻¹⁰ rad/s, approximately fourth-order reduction in this benign test.

The original-versus-revised paired plot confirms that mathematical corrections do not guarantee lower error in every scenario. The measured-state LQR–D ablation, however, reduces dynamic-plant S-curve RMSE from 0.0589 to 0.0462 m (about 21.5%). The MPC heading-only ablation is modest on the easy circle, so the entire benchmark gain must not be attributed to that one change.

## 9. Reconstructing models from synthetic experimental data

Two complementary reconstructions are attempted. For geometry, 600 virtual-sensor observations use V in [2,10] m/s and δ in [−0.15,0.15] rad. Gaussian observation noise has standard deviations 0.001 rad/s in yaw rate and 0.0005 rad in β. Bounded nonlinear least squares fits L and ρ = l_r/L jointly using the exact kinematic equations; l_r = ρL. Another 400 independently drawn inputs form the holdout, including a slightly expanded speed/steering range. All samples, truth, observations and holdout predictions are saved.

For tire parameters, the unchanged dynamic plant generates two 20 s training runs at longitudinal u = 6 and 9 m/s with a tapered multisine steering input. Noise standard deviations are 0.005 m/s for v_y and 0.0005 rad/s for r. C_f and C_r are positive log-parameters. The optimizer minimizes whitened full-rollout residuals using exact zero-order-hold discretization of the two-state lateral model. The 4 and 10 m/s holdouts use different frequencies and phases, with no refitting. Maximum tire slip across these identification excitations is only 0.641°, much smaller than in the aggressive closed-loop stiffness sweep.

__PARAMS__

The intervals are **local Gaussian/Jacobian approximations conditional on the assumed model and known quantities**. They are not confidence bounds on an actual vehicle. Dynamic fitting assumes m, I_z, l_f and l_r known. Simultaneously scaling m, I_z, C_f and C_r by the same factor leaves the lateral equations unchanged, so those four absolute parameters cannot all be identified from these lateral signals alone. The dynamic residual Jacobian condition number is about 3.20 in this particular excited experiment.

Open-loop model comparisons use the **same longitudinal u**. For CG kinematics, V = u/cosβ, giving r = u tanδ/L and v_y = u tanβ. Simplified and CG kinematic yaw predictions are therefore identical under this fair speed convention, although their position and lateral-velocity predictions differ. The learned L and l_r are used in the kinematic predictions; the learned stiffnesses are used in the dynamic prediction.

__HOLDOUT__

The dynamic fit is excellent because the synthetic truth shares its structure; this is a successful implementation/identifiability demonstration, not external validation. At 10 m/s, CG kinematic position RMSE (0.02473 m) is slightly worse than simplified kinematic (0.02147 m), despite its much better lateral-velocity prediction. Integrating different phase errors can produce cancellation, so one metric must not be used to rank models universally. At 4 m/s the CG position advantage is clear. No claim is made about nonlinear tire saturation, load transfer, road banking, aerodynamic effects, sensor delay or real localization noise. [R5]

![Model reconstruction](deliverables/figures/24_identified_parameters.png)

## 10. GPX route, map alignment and provenance

The supplied `homework_route_1.gpx` generates a 3,912.23 m local reference. All geographic data and the offline OSM GeoJSON use origin (longitude 118.8145°, latitude 31.8885°). The local approximation is x ≈ R_E cosφ_0(λ−λ_0), y ≈ R_E(φ−φ_0), with angles in radians; +x is east and +y is north. The overlay retains OpenStreetMap attribution. [R6]

__GPX__

The maximum-error coordinates are the **reference waypoint nearest the maximum absolute original/core lateral error**, not surveyed vehicle coordinates. This definition is kept distinct from the new segment-normal peak location. For PP at 8 m/s, the core-error maximum occurs at 93.8 s near the supplied reference location (31.887951°, 118.800779°). A failed run’s 579.0 s termination time is not described as successful travel duration.

PP at 8 m/s has RMSE 0.1173 m, better than MPC’s 0.2225 m on this route. LQR–K times out with 3.0486 m RMSE. These results challenge any claim that the ideal-route winner must win on arbitrary GPX geometry. The original data contain a maximum adjacent-point gap of about **619.9 m**. Resampling inserts interpolated points but cannot reconstruct missing bends or certify lane centerlines; sparse corners, curvature estimates and low-cost preview references can strongly affect comparisons. The plot shows coordinate alignment, not proof of road adherence or safety.

This route is provided course data, **not independently planned student dormitory/classroom data**. That provenance-specific requirement remains open. Exact real personal endpoints should be removed or generalized before public presentation; the deck uses local coordinates rather than presenting a new claimed personal itinerary.

![Offline map overlay](deliverables/figures/32_gpx_offline_map.png)

## 11. Limitations, defensible conclusions and next experiments

The study establishes implementation correctness for tested formulas and scenarios, sensitivity under controlled parameter changes, and successful recovery of known synthetic parameters. It does not establish deployable autonomous driving. There are no obstacles, traffic rules, perception failures, collision checks, tire-friction saturation or validated real vehicle observations. Some local linear-tire closed-loop tests exceed their own small-slip comfort region. Deterministic cases have no invented confidence intervals; identification intervals alone follow the explicitly stated noise/model assumptions.

The strongest conclusions are: speed increases motion and acceleration demand but not necessarily steady simulation error; preview and actuator feasibility can dominate a controller label; matching the state reference point and speed definition matters; endpoint completion and optimizer success are not tracking-quality guarantees; and model evaluation must separate open-loop prediction, closed-loop control and numerical integration.

A meaningful next extension would use a provenance-verified dense route, a curvature/rate-feasible speed profile, realistic localization delay/noise and a nonlinear tire model, then retune all controllers using the same training/validation separation. That would test whether the observed rankings survive outside the idealized assumptions. No such future experiment is represented as already completed.

## 12. Reproduction and evidence index

From the repository root, install `vdm_lab/student/requirements_student.txt`, then run `python -m vdm_lab.student.reproduce`. This executes the 139-case suite, synthetic identification, all figures, validation and report generation. `--resume` is intended only for an interrupted run with unchanged simulation code and configurations. For one case use `python -m vdm_lab.student.experiments --case 048_speed_scurve_mpc --out vdm_lab/student/results_one_case` so the main summary is not replaced by a one-case manifest.

`results/summary.csv` and `.json` contain all case metrics; `manifest.json` records factors. Each case directory holds the exact controller/plant configuration, warnings, raw trajectory, reference path and derived metrics. `results/identification` contains training sensors, holdout inputs and predictions, fitted parameters and numerical-refinement data. `results/validation` contains test transcripts, actuator checks and original-core SHA-256 proof. `deliverables/figures/figure_index.csv` maps 36 figures to their sources. `build_presentation.js` plus `package.json` regenerates the editable PowerPoint from those figures and results; `README_REFINED.md` contains commands.

### Sources

[R1] Supplied course lecture, Chin-An Tan, *Vehicle Dynamics and Mobility: Bicycle Model*, `VehicleDynamicsMobility_01_BicycleModel.pdf`, especially pp. 8–13; and supplied task documents `vdm_lab/tasks/README.md` and `gpx_geojson_task.md`. These are course-source assumptions, supplemented by the independently computed derivations in this report.

[R2] R. Craig Coulter (1992), *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01. https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm

[R3] SciPy documentation, `scipy.linalg.solve_discrete_are`, DARE definition and QZ-based solution. https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html (accessed 17 September 2026; installed SciPy used here is 1.17.0).

[R4] OSQP documentation, *Model predictive control (MPC)*, constrained quadratic formulation, receding-horizon execution and solver-status checking. https://osqp.org/docs/examples/mpc.html (accessed 17 September 2026).

[R5] J. Kong, M. Pfeiffer, G. Schildbach and F. Borrelli (2015), *Kinematic and dynamic vehicle models for autonomous driving control design*, IEEE Intelligent Vehicles Symposium, DOI 10.1109/IVS.2015.7225830. https://ieeexplore.ieee.org/document/7225830/

[R6] OpenStreetMap contributors, supplied offline map data; copyright and attribution information. https://www.openstreetmap.org/copyright

All numerical results, unless explicitly labeled theoretical, come from the executed student-side logs included with this delivery. No external paper’s experimental numbers are substituted for the lab results.
'''
    for k,v in {'__DESIGN__':design,'__BENCH__':bench,'__SPEED__':speed_table,'__PARAMS__':params,'__HOLDOUT__':holdout,'__GPX__':gpx}.items():report=report.replace(k,v)
    (HERE/'REPORT.md').write_text(report,encoding='utf-8')
    readme='''# Refined VDM submission — start here

## Main deliverables

- `deliverables/VDM_Tracking_Study.pptx`: editable 16:9 presentation with derivations, numerical results, caveats, source notes and appendices.
- `deliverables/VDM_Tracking_Study.pdf`: presentation PDF when exported.
- `deliverables/figures/`: 36 individually captioned figures in 300 dpi PNG and SVG; `figure_index.csv` maps them to source runs.
- `REPORT.md`: full technical report and numerical tables. `AUDIT.md`: TODO status, fixes and requirement coverage.
- `results/`: 139 case logs/configurations, model reconstruction, aggregate CSV/JSON and verification evidence.

All project changes and additions are inside `vdm_lab/student`. The supplied non-student files are unchanged; see `results/validation/core_integrity.json`.

## Run from the original project root

```bash
python -m pip install -r vdm_lab/student/requirements_student.txt
python -m vdm_lab.student.reproduce
```

For already supplied results, render only the figures:

```bash
python -m vdm_lab.student.plot_comparison
```

Run tests and inspect unchanged-core / logged-bound checks:

```bash
python -m vdm_lab.student.validate_delivery
```

Run one case without overwriting the master summary:

```bash
python -m vdm_lab.student.experiments --case 048_speed_scurve_mpc --out vdm_lab/student/results_one_case
```

Use `--resume` only to resume interrupted simulations without simulation-code/configuration changes. Cached results are not automatically recomputed when controller source changes. `reanalyze.py` recalculates metrics from saved trajectories; it is not a resimulation.

## Rebuild the PowerPoint

After Python results and figures exist, install Node.js and run in `vdm_lab/student`:

```bash
npm install
npm run build
```

PptxGenJS 4.0.0 and MathJax 3.2.2 are declared in `package.json`. The deck is generated from saved data and contains editable text/tables and vector equation artwork. The plotted curves are supplied separately as SVG/PNG. Open the PPTX in PowerPoint or LibreOffice to export a PDF; the delivered PDF is already provided for easy review. No external fonts are bundled.

## Solver and model caveats

All experiments explicitly use the new SciPy condensed-QP MPC backend; no CVXPY result is claimed. The optional original CVXPY-dependent test is skipped here; the real MPC QP is independently checked against OSQP via optional CasADi. Default module-level MPC selection is `auto`: use CVXPY if installed, otherwise SciPy. Exact cross-machine trajectories can vary slightly with solver/library versions. Timings are host-dependent.

The root CLI remains compatible but does not automatically use the student harness's repaired arc length, added telemetry or measured dynamic states. Therefore root-runner and research-harness numbers are not interchangeable. Root `normal_accel` is reference demand, not independently measured vehicle acceleration.

The GPX is supplied course data, not an independently planned student dormitory-to-classroom route. That provenance-specific requirement remains outstanding. No real-vehicle validation or road-safety certification is implied.
'''
    (HERE/'README_REFINED.md').write_text(readme,encoding='utf-8')
    (HERE/'deliverables').mkdir(exist_ok=True)
    payload={'summary':d.where(pd.notnull(d),None).to_dict('records'),'identification':ident,'audit':[list(x) for x in AUDIT_ROWS],
             'core_file_count':len(proof['expected']),'checks':checks}
    # Use summary.json to avoid pandas NaN values in optional metric columns.
    payload['summary']=json.loads((HERE/'results/summary.json').read_text())
    (HERE/'deliverables/presentation_data.json').write_text(json.dumps(payload,indent=2,allow_nan=False),encoding='utf-8')
    print(f'Wrote report ({len(report.split()):,} words), audit, README and presentation data.')
if __name__=='__main__':main()
