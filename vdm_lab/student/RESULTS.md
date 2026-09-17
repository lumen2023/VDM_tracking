# Student lab results (TODO follow-up)

This note is the student-side write-up for the items that can be finished **without** editing `run_experiment.py`, `vdm_lab/common/`, `vdm_lab/config/`, GPX data, or RealCar. It is not the official course PDF.

## 1. Setup and reporting rules

- **Controllers:** student implementations (`vdm_lab.student.*`), **not** `--version solution`. Student PP / kinematic LQR already apply a curvature-preview cruise cap; student MPC applies a reachable speed cap. Those hardenings differ from the instructor solutions.
- **Vehicle:** `student_car` via `make_vehicle_config("student_car")` (`L=\ell_f+\ell_r=2.5\,\mathrm{m}`, \(\delta_{\max}=35^\circ\), \(\dot\delta_{\max}=45^\circ/\mathrm{s}\)).
- **Plant:** default `KinematicBicycleBackend`. The new `DynamicBicycleBackend` is injected only when a student script passes `vehicle_backend=...`. `lqr_dynamic` still designs a dynamic *controller*; it does not switch the plant by itself.
- **Integrator / log:** `run_simulation()` in `vdm_lab/common/simulation.py`, `dt=0.1\,\mathrm{s}`. Logs use the existing `save_records` / `save_summary` / `save_gif` helpers.
- **`normal_accel`:** \(\,v^2\kappa_{\mathrm{ref}}\). It is **not** IMU lateral acceleration and is **not** computed from executed curvature.
- **No traffic lights, pedestrians, or right-of-way.** `duration_s` is simulator time to the stop condition (or `max_time`), not commute time.
- **Public text:** the GPX is the existing `data/gpx/homework_route_1.gpx`. No dormitory/classroom names or street numbers.

### How to rerun

From the repository root, with the `vdm-lab` environment:

```bash
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.run_all
```

Individual blocks:

```bash
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.circle_speed
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.lookahead_oat
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.plant_compare
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.gpx_nav
/home/zoe/anaconda3/envs/vdm-lab/bin/python -m vdm_lab.student.experiments.task1_extra
```

Lookahead is swept by setting `config.controller.pp_base_lookahead` in the student runner (the root CLI was not modified). Circle GIFs are only generated for PP low and PP high.

**This batch:** `outputs/20260917_212117_student/` (55 closed-loop runs). Index: `outputs/20260917_212117_student/index.json`. Table metrics match the earlier `outputs/20260917_200917_student/` batch.

## 2. Skipped on purpose

| Item | Why |
|---|---|
| RealCar / ROS / `catkin_make` | No ROS on this machine; on-car tree is out of scope. |
| BRouter re-export of a new campus GPX | No new track; reuse `data/gpx/homework_route_1.gpx`. Origin set in the script to `(lon, lat) = (118.8145, 31.8885)`. |
| `--plant` / `--pp-base-lookahead` on `run_experiment.py` | Would edit the root CLI. |
| Official course PDF | Outside the student-script scope. |

GPX import warned that the existing file is sparse (max raw gap **619.9 m**, median 27.5 m). Linear resampling cannot invent missing road curvature.

## 3. Member C — dynamic bicycle plant

File: `vdm_lab/student/dynamic_bicycle_backend.py`. Tests: `vdm_lab/student/tests/test_dynamic_backend.py`.

- Extra states \(v_y,r\) live on the backend; `VehicleState` stays \(x,y,\psi,v\).
- Linear tires, axle stiffness as in `lqr_dynamic.build_dynamic_model`: \(\alpha_f=(v_y+\ell_f r)/v_x-\delta_f\), \(\alpha_r=(v_y-\ell_r r)/v_x\), \(F_y=-C\alpha\).
- \(v_x<0.5\,\mathrm{m/s}\) uses the shared kinematic step (the floor written in TODO.md).
- `dynamics_terms` returns internal \(r\) and \(\beta=\mathrm{atan2}(v_y,v_x)\), not geometric \(\beta(\delta)\).
- Commands are still passed through `limit_command`. Lateral \((v_y,r)\) is bilinear-discretized (stiff tires, \(dt=0.1\,\mathrm{s}\)).

Limits (keep these in the slides): linear tires, no friction circle, \(dt=0.1\,\mathrm{s}\), kinematic floor at \(0.5\,\mathrm{m/s}\).

### Same PP, `circle`, medium — kinematic vs dynamic

Command: `python -m vdm_lab.student.experiments.plant_compare`

| Plant | `reached_goal` | mean \(\|e_y\|\) / m | max \(\|e_y\|\) / m | max \(\|\beta\|\) / rad | max \(\|r\|\) / rad s\(^{-1}\) | dir |
|---|---|---:|---:|---:|---:|---|
| kinematic (default) | True | 0.279 | 0.487 | 0.122 | 0.488 | `outputs/20260917_212117_student/plant_pp_circle_medium_kinematic` |
| `DynamicBicycleBackend` | True | 0.249 | 0.468 | 0.118 | 0.515 | `outputs/20260917_212117_student/plant_pp_circle_medium_dynamic` |

On this mild circle the two plants stay close. Dynamic \(\beta\) is internal sideslip, so it is **not** the same signal as kinematic \(\beta(\delta)\). PP is still a kinematic controller: the comparison is plant mismatch, not “dynamic LQR vs dynamic plant”.

## 4. Member A — circle × speed (9 runs)

Command: `python -m vdm_lab.student.experiments.circle_speed`

Circle cruise speeds from `routes.py`: low / medium / high = **3 / 5 / 7 m/s**. Radius \(R=12\,\mathrm{m}\). Steady samples keep \(|\kappa-1/12|<0.005\), then the first/last 15% of that arc is dropped (entry/exit). Theory uses the **steady-segment mean speed** \(\bar v\):

\[
\delta_{\mathrm{th}}=\arctan(L/12),\quad
r_{\mathrm{th}}=\bar v/12,\quad
a_{n,\mathrm{th}}=\bar v^2/12.
\]

\(\delta_{\mathrm{th}}=0.2054\,\mathrm{rad}\) for \(L=2.5\,\mathrm{m}\). Relative error is \(|q_{\mathrm{sim}}-q_{\mathrm{th}}|/|q_{\mathrm{th}}|\). Logged \(a_n\) is \(v^2\kappa_{\mathrm{ref}}\), so \(\varepsilon(a_n)\approx 0\) only checks that substitution, not tracking.

GIFs (minimum set): low `outputs/20260917_212117_student/circle_pp_low/animation.gif`; high `outputs/20260917_212117_student/circle_pp_high/animation.gif`. All other circle runs have `summary.png` only.

| algo | speed | \(\bar v\) | \(\bar\delta\) | \(\varepsilon_\delta\) | \(\bar r\) | \(\varepsilon_r\) | \(\bar a_n\) | \(\varepsilon_{a_n}\) | entry \(\max\|e_y\|\) | steady MAE | `reached_goal` | dir |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| PP | low | 3.000 | 0.2140 | 4.20% | 0.2593 | 3.73% | 0.750 | 0.00% | 0.100 | 0.438 | True | `.../circle_pp_low` |
| PP | medium | 5.000 | 0.2152 | 4.77% | 0.4345 | 4.30% | 2.083 | 0.00% | 0.121 | 0.465 | True | `.../circle_pp_medium` |
| PP | high | 6.994 | 0.2151 | 4.74% | 0.6077 | 4.27% | 4.077 | 0.00% | 0.158 | 0.481 | True | `.../circle_pp_high` |
| LQR kin | low | 3.000 | 0.2119 | 3.19% | 0.2569 | 2.77% | 0.750 | 0.00% | 0.036 | 0.342 | True | `.../circle_lqr_kinematic_low` |
| LQR kin | medium | 5.000 | 0.2134 | 3.90% | 0.4310 | 3.44% | 2.083 | 0.00% | 0.052 | 0.372 | True | `.../circle_lqr_kinematic_medium` |
| LQR kin | high | 6.994 | 0.2133 | 3.84% | 0.6030 | 3.46% | 4.077 | 0.00% | 0.065 | 0.433 | True | `.../circle_lqr_kinematic_high` |
| MPC | low | 3.000 | 0.2075 | 1.01% | 0.2513 | 0.52% | 0.750 | 0.00% | 0.012 | 0.060 | True | `.../circle_mpc_low` |
| MPC | medium | 5.000 | 0.2089 | 1.72% | 0.4218 | 1.22% | 2.083 | 0.00% | 0.019 | 0.136 | True | `.../circle_mpc_medium` |
| MPC | high | 6.993 | 0.2092 | 1.87% | 0.5915 | 1.50% | 4.075 | 0.00% | 0.008 | 0.103 | True | `.../circle_mpc_high` |

Directories are under `outputs/20260917_212117_student/`.

**Entry vs steady.** For PP/LQR the *steady* MAE is larger than the *entry* peak: most of the offset is a constant lag on the circle (lookahead / model), not the straight-to-arc transient. MPC keeps both small.

**High speed (7 m/s).** No run hit \(\delta_{\max}\) (`steer_saturated=False`). All three reached the 7 m/s cruise and `reached_goal=True`. Demeaned-steer sign changes exist (PP 67, LQR 61, MPC 35 on the trimmed arc) but the amplitude is small (steer std 0.008 / 0.042 / 0.049 rad). That is chatter around a bias, not a saturated hunt. PP high \(J_\delta=0.084\,\mathrm{rad/s}\).

Student MPC printed a few `user_limit` solver warnings on the circle; the logged command stayed finite and the run still finished.

## 5. Member B — lookahead OAT

Command: `python -m vdm_lab.student.experiments.lookahead_oat`

Grid: \(v\in\{3,5,7\}\,\mathrm{m/s}\), \(L_d\in\{1.5,3.0,6.0\}\,\mathrm{m}\), routes `{circle, s_curve, right_angle}` (27), plus `s_curve` **high** (8 m/s) × \(L_d=1.5\). \(J_\delta=\mathrm{mean}(|\Delta\delta|/\Delta t)\).

### `circle`

| \(v\) | \(L_d\) | `reached_goal` | mean \(\|e_y\|\) | max \(\|e_y\|\) | \(J_\delta\) | dir |
|---:|---:|---|---:|---:|---:|---|
| 3 | 1.5 | True | 0.190 | 0.292 | 0.270 | `.../lookahead_circle_v3p0_ld1p5` |
| 3 | 3.0 | True | 0.287 | 0.446 | 0.132 | `.../lookahead_circle_v3p0_ld3p0` |
| 3 | 6.0 | True | 0.497 | 0.877 | 0.062 | `.../lookahead_circle_v3p0_ld6p0` |
| 5 | 1.5 | True | 0.199 | 0.351 | 0.035 | `.../lookahead_circle_v5p0_ld1p5` |
| 5 | 3.0 | True | 0.279 | 0.487 | 0.030 | `.../lookahead_circle_v5p0_ld3p0` |
| 5 | 6.0 | True | 0.466 | 0.944 | 0.027 | `.../lookahead_circle_v5p0_ld6p0` |
| 7 | 1.5 | True | 0.191 | 0.365 | 0.128 | `.../lookahead_circle_v7p0_ld1p5` |
| 7 | 3.0 | True | 0.259 | 0.523 | 0.084 | `.../lookahead_circle_v7p0_ld3p0` |
| 7 | 6.0 | True | 0.426 | 1.029 | 0.052 | `.../lookahead_circle_v7p0_ld6p0` |

### `s_curve`

| \(v\) | \(L_d\) | `reached_goal` | mean \(\|e_y\|\) | max \(\|e_y\|\) | \(J_\delta\) | dir |
|---:|---:|---|---:|---:|---:|---|
| 3 | 1.5 | True | 0.213 | 0.605 | 0.296 | `.../lookahead_s_curve_v3p0_ld1p5` |
| 3 | 3.0 | True | 0.330 | 0.950 | 0.170 | `.../lookahead_s_curve_v3p0_ld3p0` |
| 3 | 6.0 | True | 0.680 | 1.894 | 0.101 | `.../lookahead_s_curve_v3p0_ld6p0` |
| 5 | 1.5 | True | 0.220 | 0.684 | 0.193 | `.../lookahead_s_curve_v5p0_ld1p5` |
| 5 | 3.0 | True | 0.333 | 1.053 | 0.140 | `.../lookahead_s_curve_v5p0_ld3p0` |
| 5 | 6.0 | True | 0.674 | 2.078 | 0.104 | `.../lookahead_s_curve_v5p0_ld6p0` |
| 7 | 1.5 | True | 0.222 | 0.795 | 0.223 | `.../lookahead_s_curve_v7p0_ld1p5` |
| 7 | 3.0 | True | 0.347 | 1.232 | 0.166 | `.../lookahead_s_curve_v7p0_ld3p0` |
| 7 | 6.0 | True | 0.675 | 2.285 | 0.115 | `.../lookahead_s_curve_v7p0_ld6p0` |
| high (8) | 1.5 | True | 0.228 | 0.838 | 0.227 | `.../lookahead_s_curve_high_ld1p5` |

### `right_angle`

| \(v\) | \(L_d\) | `reached_goal` | mean \(\|e_y\|\) | max \(\|e_y\|\) | \(J_\delta\) | dir |
|---:|---:|---|---:|---:|---:|---|
| 3 | 1.5 | True | 0.105 | 0.653 | 0.121 | `.../lookahead_right_angle_v3p0_ld1p5` |
| 3 | 3.0 | True | 0.153 | 0.990 | 0.083 | `.../lookahead_right_angle_v3p0_ld3p0` |
| 3 | 6.0 | True | 0.331 | 1.874 | 0.048 | `.../lookahead_right_angle_v3p0_ld6p0` |
| 5 | 1.5 | True | 0.093 | 0.716 | 0.082 | `.../lookahead_right_angle_v5p0_ld1p5` |
| 5 | 3.0 | True | 0.138 | 1.095 | 0.070 | `.../lookahead_right_angle_v5p0_ld3p0` |
| 5 | 6.0 | True | 0.320 | 1.993 | 0.053 | `.../lookahead_right_angle_v5p0_ld6p0` |
| 7 | 1.5 | True | 0.081 | 0.782 | 0.088 | `.../lookahead_right_angle_v7p0_ld1p5` |
| 7 | 3.0 | True | 0.125 | 1.193 | 0.075 | `.../lookahead_right_angle_v7p0_ld3p0` |
| 7 | 6.0 | True | 0.298 | 2.103 | 0.057 | `.../lookahead_right_angle_v7p0_ld6p0` |

### Tuning notes (from this grid, not slogans)

- Every cell `reached_goal=True`. The binding accuracy term is **lookahead**, not speed: \(L_d: 1.5\to 6.0\) roughly **doubles to triples** max \(\|e_y\|\) on corners; \(v: 3\to 7\) is a smaller bump at fixed \(L_d\).
- Short \(L_d\) raises \(J_\delta\) (more steering activity). On these lab routes that did not prevent finishing, including **high × \(L_d=1.5\)**.
- Textbook “raise lookahead with speed” is a damping heuristic. Here it was **not** required for completion; it **hurt** tracking. Use a longer \(L_d\) only if actuator rate / chatter is the limit.
- **Do not only raise speed on a sharp corner.** `right_angle` at 7 m/s with \(L_d=6\) has max \(\|e_y\|=2.10\,\mathrm{m}\) vs \(0.78\,\mathrm{m}\) at \(L_d=1.5\). Slow down or shorten the look-ahead (or both).

## 6. Member D — existing GPX, three algorithms + retest

Command: `python -m vdm_lab.student.experiments.gpx_nav`

Same file `data/gpx/homework_route_1.gpx`, same student controllers, local origin `(118.8145, 31.8885)`. Baseline cruise **8 m/s**. Path length after resample ≈ **3914 m**. Time is simulator time.

| algo | \(v_{\mathrm{cmd}}\) | `reached_goal` | time / s | mean \(\|e_y\|\) | max \(\|e_y\|\) | peak \(x,y\) / m | peak lat, lon | \(\kappa\) at peak | \(\delta\) at peak | dir |
|---|---:|---|---:|---:|---:|---|---|---:|---:|---|
| PP | 8 | True | 516.8 | 0.035 | 1.428 | (−791.41, 591.86) | 31.893830, 118.806128 | 0 | −0.611 | `.../gpx_pp_v8` |
| LQR kin | 8 | **False** | 549.2 | 1.714 | 5.871 | (−798.68, 153.70) | 31.889877, 118.805988 | ≈0 | +0.611 | `.../gpx_lqr_kinematic_v8` |
| MPC | 8 | True | 500.4 | 0.029 | 1.466 | (−832.55, 590.22) | 31.893789, 118.805691 | 0 | −0.598 | `.../gpx_mpc_v8` |
| LQR kin (retest) | **5** | **True** | 813.4 | 0.051 | 1.989 | (−833.09, 590.69) | 31.893788, 118.805686 | 0 | −0.611 | `.../gpx_lqr_kinematic_v5` |

LQR at 8 m/s hit the time budget still moving at 8 m/s (`finish_error=462.6\,\mathrm{m}`, last tracked arc \(s\approx 3384.5/3914\,\mathrm{m}\)). Steer sat on 13% of samples. Student curvature-preview **was not enough** to finish this sparse GPX at 8 m/s.

**Improvement that did work:** explicit slowdown 8 → 5 m/s. LQR then `reached_goal=True`. Peak \(\|e_y\|\) drops from 5.87 m to 1.99 m. That is a speed change, not a new GPX and not a claim about real walking time.

Peak \(\kappa\approx 0\) on the successful runs is consistent with **piecewise-linear GPX**: curvature spikes at vertices and is ~0 on long chords. The 619.9 m gap warning matters; max \(\|e_y\|\) is not a survey-grade “worst city corner”.

Do not rank PP vs MPC “travel time” as commuting: no lights, no traffic, and MPC’s 500 s vs PP’s 517 s is open-loop tracking on a sparse polyline.

## 7. Course task 1 extras

Command: `python -m vdm_lab.student.experiments.task1_extra`

Medium speed on three built-in routes × PP / kinematic LQR / MPC (9), plus student **dynamic LQR** on `s_curve` and `mixed_course`, plus one **in-script** geometry change \(\ell_f=1.60,\ell_r=0.90\) (wheelbase kept at 2.50 m; `vehicle_params.py` untouched).

Lag: compare arc length of \(\max\|e_y\|\) with arc length of \(\max|\kappa|\) on `reference_path.csv`. Positive `lag_m` means the error peak is **after** the curvature peak.

| run | `reached_goal` | mean \(\|e_y\|\) | max \(\|e_y\|\) | lag / m | after peak? | dir |
|---|---|---:|---:|---:|---|---|
| PP `s_curve` | True | 0.339 | 1.171 | +15.0 | yes | `.../task1_pp_s_curve_medium` |
| LQR kin `s_curve` | True | 0.211 | 0.529 | +21.0 | yes | `.../task1_lqr_kinematic_s_curve_medium` |
| MPC `s_curve` | True | 0.054 | 0.207 | +28.5 | yes | `.../task1_mpc_s_curve_medium` |
| PP `mixed_course` | True | 0.261 | 0.768 | +1.5 | yes | `.../task1_pp_mixed_course_medium` |
| LQR kin `mixed_course` | True | 0.184 | 0.451 | +3.0 | yes | `.../task1_lqr_kinematic_mixed_course_medium` |
| MPC `mixed_course` | True | 0.053 | 0.227 | +8.5 | yes | `.../task1_mpc_mixed_course_medium` |
| PP `right_angle` | True | 0.133 | 1.106 | +4.5 | yes | `.../task1_pp_right_angle_medium` |
| LQR kin `right_angle` | True | 0.096 | 0.647 | +6.5 | yes | `.../task1_lqr_kinematic_right_angle_medium` |
| MPC `right_angle` | True | 0.017 | 0.164 | +6.0 | yes | `.../task1_mpc_right_angle_medium` |
| LQR dyn `s_curve` | True | 0.059 | 0.184 | +5.0 | yes | `.../task1_lqr_dynamic_s_curve_medium` |
| LQR dyn `mixed_course` | True | 0.041 | 0.106 | +6.5 | yes | `.../task1_lqr_dynamic_mixed_course_medium` |
| PP mixed, \(\ell_f/\ell_r\) shift | True | 0.193 | 0.639 | 0.0 | at peak | `.../task1_pp_mixed_course_medium_lf_shift` |

On these three routes the **error peak is after the curvature peak** (lookahead / steering-rate delay), except the shifted-geometry PP run which landed on the peak. Dynamic LQR (still on the **kinematic** plant) reduced max \(\|e_y\|\) vs kinematic LQR here; that is a controller comparison, not a plant comparison.

Shifting mass distribution \(\ell_f,\ell_r\) while keeping \(L\) changed PP mixed-course max \(\|e_y\|\) from 0.768 m to 0.639 m. One cell, not a full vehicle study.

## 8. What changed vs the earlier ISSUES list

- Student MPC on this batch **did not overshoot** the 3/5/7 m/s circle targets (`max_speed` matches the cruise). That is a student-controller difference vs the instructor-MPC overspeed noted in `ISSUES.md`; do not mix the two tables.
- Student kinematic LQR on the **same** homework GPX at 8 m/s still **fails to finish** (same qualitative failure as the earlier 8 m/s LQR log: time-out, large \(e_y\), steer at the stop). Preview capping was not a sufficient fix. **5 m/s retest succeeds.**

## 9. Limits (short)

- Kinematic default plant; dynamic backend is optional and linear-tire only.
- `dt=0.1\,\mathrm{s}`; GPX vertices are sparse.
- No environment constraints; `reached_goal` is a simulator stop test.
- Student ≠ solution. Numbers in this file are student closed-loop logs under `outputs/20260917_212117_student/`.
