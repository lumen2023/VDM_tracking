# Vehicle Dynamics & Motion Control — Path Tracking Project

**Merged report for Problems 1, 3 and 4**

This repository (`VDM_tracking`) is the merged submission of a two-member group project built on
the course simulation platform `vdm_lab`. Each member worked on a separate clone and a separate
set of problems; this document is the single English report covering all of the merged work, and
every numerical result produced by either member is stored under [`outputs/`](outputs/).

| Member | Problems | Original clone | Results in this repo |
| --- | --- | --- | --- |
| A | **Problem 1** (why tracking gets harder with speed) and **Problem 4** (dorm → classroom navigation) | `VDM_tracking` | [`outputs/analysis_circle_speed/`](outputs/analysis_circle_speed/), [`outputs/analysis_pp_corner_cut/`](outputs/analysis_pp_corner_cut/), [`outputs/analysis_steer_rate/`](outputs/analysis_steer_rate/), [`outputs/2006*` run logs](outputs/), [`outputs/dorm_*`](outputs/) |
| B | **Problem 3** (plan and track a dorm → classroom route from GPX) | `VDM_tracking1` | [`outputs/problem3/`](outputs/problem3/) |

> **Merge note.** All artifacts of member B were copied verbatim from `VDM_tracking1/outputs/`
> into [`outputs/problem3/`](outputs/problem3/). The per-run trajectory/animation dumps of member B
> are not present in that clone and therefore could not be transferred; only the aggregated
> results, statistics tables, figures and command log exist. See
> [§9 Artifact index](#9-artifact-index) and [`outputs/problem3/README.md`](outputs/problem3/README.md).
>
> The repository-root `README.md` — the course's bilingual deployment and run manual — was replaced
> by this report and is preserved **verbatim** as [`docs/SETUP.md`](docs/SETUP.md) (only its
> `vdm_lab/...` links were rewritten to `../vdm_lab/...` so they still resolve from `docs/`).
>
> The two clones also diverged in code: member A's `vdm_lab` adds geographic-route speed profiles
> (`speed_profile`, `curvature_smooth_m`, `lateral_accel_limit`) and inverse GPX projection
> (`local_xy_to_latlon`), while member B's clone keeps the plain student controllers. The code was
> **not** merged — the merged repository keeps member A's `vdm_lab` tree, and member B's results
> are the archived outputs of their own tree. This is called out wherever it matters.

---

## 1. Simulation platform and vehicle

Both members used the same platform defaults, which makes the two result sets comparable.

| Item | Value |
| --- | --- |
| Integrator | explicit Euler, `dt = 0.1 s` |
| Plant model | kinematic bicycle (`ψ̇ = v cos β tan δ_f / L`), both members |
| Vehicle | `student_car`: `L = 2.50 m`, `l_f = l_r = 1.25 m` |
| Steering limit | `max_steer = ±35° = ±0.6109 rad` |
| Declared steering rate limit | `max_steer_rate = 45°/s = 0.7854 rad/s` |
| Longitudinal limits | `a_max = +2 m/s²`, `a_min = −3.5 m/s²`, `v_max = 12 m/s` |
| Inertia / tyre data (controller only) | `m = 1140 kg`, `I_z = 1436.24 kg·m²`, `C_f = C_r = 155494.663` |
| Controllers | Pure Pursuit, kinematic LQR, dynamic LQR (member B only), linear MPC |

Controller parameters were left at their defaults in all reported runs:

- **PP**: `L_d = L_0 + 0.35 v` (`L_0 = 3 m` in the built-in-route controller), speed P-gain 1.1
- **Kinematic LQR**: `Q = diag(2, 0.2, 3, 0.2)`, `R = [1.5]`, Riccati tolerance `1e−4`, max 200 iterations
- **MPC**: horizon 8 steps (0.8 s), `Q = diag(2,2,0.6,1)`, `Q_f = diag(4,4,1,2)`,
  `R = diag(0.2,0.4)`, `R_d = diag(0.2,0.8)`, ≤5 iterations, QP solved with OSQP

Two properties of the platform matter repeatedly in the findings below and are documented in
[§2.7](#27-platform-defects-identified-in-problem-1):

1. `vdm_lab/common/vehicle.py::limit_command()` clamps the **magnitude** of the steering angle but
   never applies `max_steer_rate`. A controller can therefore command a physically impossible
   steering jump.
2. `vdm_lab/common/vehicle.py::speed_pid()` throttles near the goal using the **Euclidean distance**
   to the path end, not the remaining arc length. On closed or looping routes this fires in the
   middle of the route.

### 1.1 Modelling background

The kinematic bicycle model with the centre of gravity `C` and wheelbase `L = l_f + l_r` gives,
with rear-axle no-slip (`w = l_r r`) and the front axle rolling along the front wheel:

```text
β    = atan( (l_r / L) · tan δ_f )        side-slip angle of the velocity vector
ψ̇    = v · cos β · tan δ_f / L            yaw rate  (logged as yaw_rate)
ẋ    = v · cos(ψ + β)
ẏ    = v · sin(ψ + β)
```

Note that `β` here is a **geometric** consequence of the no-slip constraints, not a tyre slip
angle. On a constant-radius arc with curvature `κ = 1/R`:

```text
δ_f ≈ atan(L κ)          (small-β approximation)
r   ≈ v κ                yaw rate proportional to speed
a_n = v² κ               normal (lateral) acceleration proportional to speed²
```

The `v²` in the last relation is the physical core of Problem 1.

The dynamic bicycle model additionally carries lateral tyre forces and yaw inertia, and is used
**only inside the dynamic-LQR controller**:

```text
F_yf = C_f ( δ_f − (v_y + l_f r)/u )
F_yr = −C_r ( (v_y − l_r r)/u )
m (v̇_y + u r) = F_yf + F_yr
I_z ṙ          = l_f F_yf − l_r F_yr
```

Because the **plant** is always the kinematic model, a good score from the dynamic LQR does not
validate a full dynamic vehicle model; it validates the controller design against this plant.

---

## 2. Problem 1 — why path tracking gets harder as speed rises

**Setup.** Fixed-curvature circle, `R = 12.0 m`, `κ = 1/12 m⁻¹`, three nominal speed steps
`3.0 / 5.0 / 7.0 m/s` (low / medium / high). Controllers: PP, kinematic LQR, MPC — all student
implementations in `vdm_lab/student/`. Nine closed-loop runs, all nine reaching the goal.
Reproduce with:

```bash
for algo in pp lqr_kinematic mpc; do
  for speed in low medium high; do
    python run_experiment.py --algo "$algo" --version student \
      --route circle --speed-mode "$speed" --save-log --save-fig
  done
done
```

### 2.1 Summary of the answer

| # | Cause | Quantitative relation | Measured evidence |
| --- | --- | --- | --- |
| 1 | Lateral acceleration demand grows with the **square** of speed (physical) | `a_n = v²κ` | demand 0.75 → 4.083 m/s² from 3 → 7 m/s (×5.44); measured relative error ≤ 3.42 % |
| 2 | PP's steady-state **corner cut** is proportional to the look-ahead distance, which itself grows linearly with speed (**dominant cause**) | `e_ss ≈ 0.93 · L_f · β` | 13-point controlled sweep, `R² = 0.99881`; `L_f` 4.05 → 5.12 m |
| 3 | Distance covered per control step grows, so correction opportunities fall as `1/v` | `v·dt`: 0.3 m → 0.7 m | consistent with the data but **not isolated by this experiment** — see [§2.4](#24-cause-3--fewer-corrections-per-metre-not-isolated) |

**One intuition is refuted by the data: it is not "not enough steering".** The steady-state
steering angle `δ ≈ atan(Lκ)` is **independent of speed**; measured values across the three steps
are 0.2126 / 0.2146 / 0.2149 rad against a theoretical 0.2054 rad. At high speed the largest PP
steering command is 0.2293 rad — only **38 %** of the ±0.6109 rad limit. Speed does not change *how
much steering is needed*; it changes how fast the resulting lateral drift accumulates, and how far
ahead the controller is looking.

| Speed step | v [m/s] | theory `a_n = v²κ` | PP measured | LQR k measured | MPC measured | max rel. error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| low | 3.000 | 0.75000 | 0.74999 | 0.74999 | 0.75923 | 0.00 % |
| medium | 4.79 | 1.915 | 1.92319 | 1.91787 | 1.99494 | 0.44 % |
| high | 6.07 | 3.071 | 3.17191 | 3.16509 | 4.02728 | 3.42 % |

The peak logged normal acceleration at the high step is **4.082 / 4.082 / 4.066 m/s²**
(PP / LQR k / MPC) against a theoretical `7.0²/12 = 4.0833` — all three reach **≥ 99.6 %** of the
demand. The residual difference is a **statistics artefact, not a model error**: theory is evaluated
with the steady-arc *mean* speed, and `v²` is convex, so `mean(v²) ≥ mean(v)²`, and the gap widens
with in-segment speed ripple.

*Figures*: [`fig01_steer_vs_v.png`](docs/figures/fig01_steer_vs_v.png) (steering independent of
speed), [`fig02_yawrate_vs_v.png`](docs/figures/fig02_yawrate_vs_v.png) (yaw rate linear in speed),
[`fig03_an_vs_v.png`](docs/figures/fig03_an_vs_v.png) (normal acceleration quadratic in speed).

### 2.2 Dominant cause — the PP corner cut is `L_f · β`

**It is a steady geometric offset, not drift.** At 3 m/s PP already tracks with a 0.4369 m lateral
offset while using only 0.2054 rad of steering — far from saturation. A least-squares circle fit of
the steady-arc trajectory shows the fitted centre sits within 1.5 cm of `(16, 12)` for all three
speed steps, and `lateral offset ≈ R − r` holds to the **millimetre** (max 2.77 mm). The same
conclusion holds for the other two controllers:

| Controller | cut `R − r` [m] (low / medium / high) | max deviation from mean lateral error |
| --- | --- | ---: |
| PP | 0.4357 / 0.4708 / 0.4817 | 2.77 mm |
| LQR kinematic | 0.3243 / 0.3780 / 0.3187 | 10.52 mm † |
| MPC | 0.2168 / 0.2514 / 0.2988 | 1.16 mm |

† The LQR residual is itself evidence of its high-speed oscillation: its trajectory is no longer a
smooth circle.

**Controlled sweeps.** With `v = 3.0 m/s`, `l_f = l_r = 1.25`, `ds = 0.05` fixed, only one variable
was changed per sweep.

*Sweep A — base look-ahead `L_0` only.* The corner cut grows linearly with `L_f`
([`fig09_cut_vs_lookahead.png`](docs/figures/fig09_cut_vs_lookahead.png)).

*Sweep B — `l_f/l_r` split only, wheelbase `L` unchanged (the decisive sweep).*

| `(l_f, l_r)` [m] | steady `β` [rad] | corner cut [m] |
| --- | ---: | ---: |
| (2.5, 0.0) | **0.00000** | **−0.0423** ← `β = 0`, the cut vanishes |
| (2.0, 0.5) | 0.04217 | +0.1253 |
| (1.25, 1.25) | 0.10716 | +0.3774 |
| (0.5, 2.0) | 0.17440 | +0.6288 |
| (0.0, 2.5) | 0.22057 | +0.7968 |

Driving `β` from 0 to 0.22 rad by redistributing mass alone moves the cut from −0.04 m to +0.80 m.
[`fig10_cut_vs_beta.png`](docs/figures/fig10_cut_vs_beta.png).

*Sweep C — reference resampling spacing `ds` only.* A secondary contribution, not the main term
([`fig12_cut_vs_ds.png`](docs/figures/fig12_cut_vs_ds.png)).

**Regression** over the 13 points of sweeps A and B at `ds = 0.05`:

```text
corner_cut ≈ 0.9302 · (L_f · β) − 0.0280 m          R² = 0.99881,  max residual 0.0227 m
```

[`fig11_cut_regression.png`](docs/figures/fig11_cut_regression.png). The coefficient `0.9302 ≈ 1`
means that, on a constant-curvature path, **the PP steady-state corner cut equals `L_f · β`**.

**Why `L_f · β`.** PP's steering law `δ = atan(2L sin α / L_f)` is derived from a **no-side-slip
single-track model**: it assumes the vehicle travels along the *tangent of the chord*. The real
vehicle travels along `ψ + β`. PP believes that steering by `α` reaches the look-ahead point, while
the vehicle actually moves along `α + β`, so **every control cycle it turns slightly too far
inwards**, and the look-ahead distance amplifies the resulting offset until it settles as a
parallel shift of the whole arc:

```text
v ↑  →  L_f = L_0 + 0.35 v ↑  →  corner_cut ≈ 0.93 · L_f · β ↑  →  steady lateral error ↑
```

`β` itself barely changes with speed (PP: 0.1176 / 0.1229 / 0.1162 rad across the three steps), so
the speed growth of the cut is carried almost entirely by `L_f`: from 3.0 to 6.07 m/s, `L_f` grows
from 4.05 to 5.12 m (+26.5 %) while the measured cut grows from 0.4369 to 0.4790 m (+9.6 %) — less
than 26.5 %, because closed-loop feedback compensates part of it at high speed.

> **A retracted formula.** An earlier `e_ss ≈ L_f²κ/8` hypothesis predicted cuts of
> 0.171 / 0.235 / 0.309 m against measured 0.437 / 0.472 / 0.479 m (1.7–2.6× under-estimate) and,
> decisively, predicted no dependence on `β` — refuted by the `β = 0` control point in sweep B.

**This mechanism does not apply to LQR or MPC.** Neither controller contains `L_f`; both model
curvature explicitly (LQR: `δ = −K e + L κ`; MPC: curvature-referenced deviation in the cost). The
ranking of the cuts follows exactly how explicitly each controller models curvature:

| Controller | How curvature is modelled | cut at low speed [m] | steady error at high speed [m] |
| --- | --- | ---: | ---: |
| MPC | inside the optimisation cost | 0.2168 | **0.2987** |
| LQR kinematic | as a feed-forward term | 0.3243 | 0.3292 |
| PP | not at all — feedback only | 0.4357 | 0.4790 |

⚠ **Do not rank by relative growth.** Low → high relative growth is PP +9.6 %, LQR k −1.7 %,
MPC +37.8 %. MPC grows the most yet remains the most accurate throughout (its 0.2987 m is still
below PP's low-speed 0.4357 m). Tracking quality must be judged on **absolute** error.

### 2.3 Error growth must be read on the steady arc, not the whole run

| Controller | steady-arc **max** error [m] (low → high) | **whole-run mean** error [m] (low → high) |
| --- | --- | --- |
| PP | 0.4462 → 0.5046 → **0.5251** ↑ | 0.2859 → 0.2878 → **0.2750** ↓ |
| LQR kinematic | 0.3419 → 0.4130 → **0.4761** ↑ | 0.2195 → 0.2387 → **0.1900** ↓ |
| MPC | 0.2260 → 0.2541 → **0.3475** ↑ | 0.1442 → 0.1566 → **0.1679** ↑ |

**The whole-run mean lateral error *falls* with speed for PP and LQR — this does not mean high
speed is more accurate.** The denominator changes: at higher speed a larger share of the sampled
points lies on the straight entry/exit segments where the error is identically zero (PP: 215 of 372
samples on the steady arc at low speed = 58 %, but 89 of 208 = 43 % at high speed). The zero-error
samples dilute the average. **Tracking difficulty must be judged on the steady-arc statistics and
on the maximum error.**

**The error is a steady plateau, not a transient spike.** Whole-run max minus steady-arc max is
exactly zero in **eight of the nine runs** (transient contribution 0 %); the only exception is MPC
at high speed (0.0090 m, 2.6 %). See
[`fig17_pp_high_lateral_error.png`](docs/figures/fig17_pp_high_lateral_error.png) and
[`fig18_mpc_high_lateral_error.png`](docs/figures/fig18_mpc_high_lateral_error.png): the error rises
quickly on arc entry and then stays flat.

**Why the LQR high-speed mean looks non-monotonic.** Its steady-arc mean error is
0.3267 → 0.3791 → 0.3292 m; the high step is *lower* than the medium step. The controller did not
become more accurate — it switched from "smooth tracking with a fixed bias" to "oscillation about
the mean": the steady error standard deviation rises from 0.0196 to 0.0701 m (×3.6) while the
**maximum error still rises monotonically** (0.3419 → 0.4130 → 0.4761). This is why both mean and
maximum must be reported together.

### 2.4 Cause 3 — fewer corrections per metre, not isolated

With `dt = 0.1 s` fixed, a vehicle at 3 m/s advances 0.3 m per step and at 7 m/s advances 0.7 m, so
the number of control decisions available through a given bend falls as `1/v`. This mechanism is
plausible and consistent with the observed monotonic loss of accuracy, **but this experiment did not
isolate it**: the diagnostic work in [§2.5](#25-refuted-explanations) confirmed that no run exhibited
genuine closed-loop instability. Demonstrating this effect separately requires a new experiment
(e.g. fixing the distance advanced per step and varying only the controller update rate), which was
not performed.

### 2.5 Refuted explanations

**① "There is not enough steering at high speed."** Refuted — see [§2.1](#21-summary-of-the-answer).

**② "Error accumulates monotonically with speed."** Refuted — the whole-run mean error *decreases*
for PP (0.2859 → 0.2750 m); see the sampling-ratio trap in [§2.3](#23-error-growth-must-be-read-on-the-steady-arc-not-the-whole-run).

**③ "The LQR loop is unstable at high speed, so gain scheduling is needed."** Refuted by three
independent checks. The kinematic LQR at the high step shows a **step-to-step equal-amplitude
oscillation** of the steering angle at ±0.61 rad (`J_δ = 5.4927`, 10.9× the medium step; `β`
oscillating to 0.33677 rad = 19.3°; instantaneous yaw rate up to 1.850 rad/s), which initially
looked like a stability problem:

| Hypothesis | Check | Result |
| --- | --- | --- |
| Fixed `K`, linearised model unstable | `eig(A − BK)` for `v = 3…8 m/s` | **Refuted**: `max\|λ\|` *falls* from 0.844 (v = 3) to 0.706 (v = 7); poles stay well inside the unit circle |
| Algebraic loop through the previous steering command | loop gain `K₃ · v / L` | **Refuted**: only 0.655 at v = 7; the critical speed is ≈ 10.7 m/s |
| Reference-resampling aliasing (0.7 m per step > `ds = 0.5 m`) | sweep `ds` from 0.5 down to 0.05 | **Refuted**: `J_δ` remains 3.4–5.6; the oscillation does not disappear |

The real clue is in the raw log: **58 of 91 arc samples have a steering sign flip**, and the largest
step-to-step `|Δsteer|` reaches **0.9 rad**, i.e. ≈ **9 rad/s** — **11× the declared 0.7854 rad/s
limit**. `vdm_lab/common/vehicle.py` clamps only the angle magnitude and never applies
`max_steer_rate`. Wrapping the controller in a true rate limiter and re-running
(`scripts/verify_steer_rate_limit.py`) gives:

| Speed step | `J_δ` (steady arc) raw → rate-limited | `max\|steer\|` [rad] | steady `σ(e)` [m] | steady `max\|e\|` [m] |
| --- | --- | --- | --- | --- |
| medium | 0.5035 → **0.1087** | 0.3514 → 0.2909 | 0.0196 → 0.0183 | 0.4130 → 0.4063 |
| high | 5.4927 → **0.4702** | 0.6109 → **0.3142** (no longer saturated) | 0.0701 → 0.0425 | 0.4761 → 0.4845 |

At the high step `J_δ` drops to **1/11.7** of its raw value, the steering no longer pins against the
limit, and accuracy is essentially unchanged.
[`fig13_steer_raw_vs_limited.png`](docs/figures/fig13_steer_raw_vs_limited.png) shows the raw signal
(red) oscillating at ±0.61 rad against the rate-limited signal (blue) settling at ≈ 0.21 rad —
exactly the theoretical `atan(Lκ) = 0.2054 rad`.

**Conclusion: the LQR high-speed "oscillation" is an artefact of a platform that does not enforce
the steering rate limit.** The controller commands a physically impossible jump every step, and
that jump is fed back into the next step's solution through `previous_control.steer` inside
`e_yaw_dot`, closing a step-to-step limit cycle. Once the actuator rate is constrained, the loop is
cut. This does **not** mean the LQR has no genuine high-speed issue — its accuracy does degrade with
speed (see §2.2 above) — but the *oscillation* is a platform
artefact, and the fixed-gain cross-speed margin is sufficient: **no gain scheduling is required.**

### 2.6 Three-controller comparison at the high speed step

| Metric (high step) | PP | LQR kinematic | MPC |
| --- | ---: | ---: | ---: |
| steady lateral error [m] | 0.4790 | 0.3292 | **0.2987** |
| steady max error [m] | 0.5251 | 0.4761 | **0.3475** |
| whole-run max error [m] | 0.5251 | 0.4761 | **0.3475** |
| steady speed [m/s] (target 7.0) | 6.070 ‡ | 6.060 ‡ | **6.952** |
| `max\|Δsteer\|` [rad] | **0.0328** | 1.2217 § | 0.1735 § |
| steps violating the rate limit (0.0785 rad/step) | **0 / 207 (0 %)** | 126 / 211 (59.7 %) § | 87 / 195 (44.6 %) § |
| reached the goal | yes | yes | yes |

‡ see platform defect 2 in [§2.7](#27-platform-defects-identified-in-problem-1). § see refuted
explanation ③ above.

**Accuracy: MPC > kinematic LQR > PP**, with the same ordering at all three speed steps and the
largest gap at high speed (MPC 38 % better than PP).

**Smoothness must be judged by "does the per-step jump exceed the actuator rate limit", not by
`J_δ`**, because `J_δ` is contaminated by refuted explanation ③
([`fig16_steer_rate_violations.png`](docs/figures/fig16_steer_rate_violations.png)):

- **PP satisfies the rate constraint completely (0 % violations).** Its steering is a pure
  geometric function of a continuously moving look-ahead point, so it has no oscillation source.
  The price is accuracy.
- **MPC violates it 44.6 % of the time**, with a small amplitude (0.1735 rad). The QP enforces
  `|u[1,t+1] − u[1,t]| ≤ rate·dt` only **inside the horizon** and never constrains `u[1,0]` against
  the previously executed command. This is a fixable implementation detail.
- **Kinematic LQR violates it 59.7 % of the time** with the largest amplitude (1.2217 rad) — the
  artefact of ③.

> This report covers the `circle` route only. It makes **no claim** about whether the same ordering
> holds on `double_lane_change` / `right_angle` / `s_curve` / `mixed_course`. Member B's
> independent built-in-route benchmark is reported in [§3.4](#34-supporting-benchmark-built-in-routes-task-1).

### 2.7 Platform defects identified in Problem 1

| # | Location | Problem | Impact | Suggested fix |
| --- | --- | --- | --- | --- |
| 1 | `vdm_lab/common/vehicle.py::limit_command()` | clamps steering **magnitude** only; `max_steer_rate` is never applied | controllers may command ≈ 9 rad/s steering jumps (limit 0.7854 rad/s); the LQR high-speed artefact; all smoothness metrics are distorted | add `clamp(steer, prev ± max_steer_rate · dt)` |
| 2 | `vdm_lab/common/vehicle.py::speed_pid()` | goal-proximity throttling uses the **Euclidean straight-line** distance instead of the remaining arc length | on a closed route such as `circle`, the vehicle is wrongly judged "near the goal" mid-arc and decelerates unexpectedly | use `s_total − s_vehicle` |

**Evidence for defect 2** (high-step steady speeds of only 6.070 / 6.060 m/s against a 7.0 target):

```python
if distance_to_goal < 14.0:
    target_speed = min(target_speed, (2.0 * 0.9 * distance_to_goal) ** 0.5)
```

`circle` is a closed loop, so the vehicle passes near the path end while circulating: 23 of PP's 89
high-step steady samples lie within 14 m of the goal (closest 8.52 m), triggering the limit
`v ≤ √(1.8 × 8.52) = 3.92 m/s`; the measured minimum is 4.247 m/s. The low step is unaffected
because its 3.0 m/s target is already below the limit. MPC tracks the reference speed directly
inside its QP and never calls `speed_pid`, so it is unaffected (6.952 m/s).
[`fig15_lqrk_high_speed.png`](docs/figures/fig15_lqrk_high_speed.png) shows the LQR speed dropping
from 6.8 to 4.2 and climbing back to 7.0 between t ≈ 5 and 8 s, exactly where the vehicle passes the
goal.

**Neither defect changes the main speed–difficulty conclusions** (the `a_n = v²κ` validation, the
`L_f·β` cut mechanism and the accuracy ranking), because those rest on steady-state geometric
quantities. They do, however, invalidate direct quotation of the high-step `J_δ` and steady speed.

### 2.8 Engineering recommendations from Problem 1

- **Do not increase the look-ahead distance blindly with speed.** Each extra metre of `L_f` adds
  ≈ `0.93·β` m of steady cut. Beyond `L_f = L_0 + k·v`, a curvature feed-forward or a steering
  bias compensation is needed — the measured accuracy advantage of LQR and MPC comes precisely from
  modelling curvature explicitly.
- **Goal-proximity throttling must use remaining arc length**, otherwise closed-loop or
  out-and-back routes decelerate in the middle.
- **The simulator must apply `max_steer_rate` at the actuator layer**; without it, no smoothness
  metric computed from any controller is trustworthy — as the LQR result here demonstrates.

---

## 3. Problem 3 — planning and tracking a dorm → classroom route from GPX (member B)

Member B's work is archived in [`outputs/problem3/`](outputs/problem3/). The original run produced
**31 experiments, 30 of which completed their route**; failed or aborted runs are retained rather
than deleted. The machine-readable record is
[`all_results.json`](outputs/problem3/all_results.json) (structured),
[`all_results.csv`](outputs/problem3/all_results.csv) (one row per run, ~80 columns),
[`circle_statistics.csv`](outputs/problem3/circle_statistics.csv) and
[`gpx_statistics.csv`](outputs/problem3/gpx_statistics.csv); every command is logged in
[`commands.md`](outputs/problem3/commands.md). The long-form Chinese analysis written by
member B at the time is preserved as [`outputs/problem3/report.md`](outputs/problem3/report.md)
(Chinese), with the summary in
[`outputs/problem3/README.md`](outputs/problem3/README.md).

| Block | Runs | Reached goal | Notes |
| --- | ---: | ---: | --- |
| Task 1: built-in routes | 16 | 16 | 4 controllers × 4 routes |
| Task 2: circle speed | 9 | 9 | 3 controllers × 3 speed steps |
| Task 3: GPX route | 3 | 2 | PP and kinematic LQR completed; MPC stopped early |
| GPX improvement | 1 | 1 | PP target speed 8 → 4 m/s |
| Vehicle parameter | 2 | 2 | `max_steer` 35° → 20° |

Environment for this run: Python 3.13.7, NumPy 2.5.1, SciPy 1.18.0, Matplotlib 3.11.0,
CVXPY 1.9.2 with OSQP. Note that the installed NumPy exceeds the `< 2.0` bound declared in
`requirements.txt`; the simulation was run in the existing environment without relaxing the
declared constraint.

### 3.1 Route preparation

`data/gpx/homework_route_1.gpx` was used, exported from BRouter. Start and end are anonymised as
A and B.

> ⚠ **This is not a verified personal dorm → classroom route.** No personal dormitory or classroom
> coordinates were obtained and no claim is made that this route was self-planned by the group.
> The "personal route" submission requirement therefore **remains open**. The file contains precise
> coordinates and is suitable for local use only; location privacy must be handled separately
> before any public submission.

| Property | Value |
| --- | --- |
| Reference length | 3914.126 m |
| Raw GPX points | 58 |
| Resampled points (`ds = 1 m`) | 3916 |
| Max raw point spacing | 619.924 m |
| Median raw point spacing | 27.501 m |
| Longitude range | 118.799037 – 118.818331 |
| Latitude range | 31.886255 – 31.894316 |
| Fully inside the task-map boundary | yes |
| Map origin (shared by GPX and GeoJSON) | (118.8145, 31.8885) |

The very large maximum point spacing is an important limitation: linear resampling can only insert
points, it cannot recover the true road geometry between two distant GPX fixes, and it is therefore
a key caveat when interpreting the maximum-deviation locations below.

### 3.2 Task 3 results at the 8 m/s target

| Controller | Reached | Route length [m] | Effective time [s] | Mean progress speed [m/s] | Mean \|e\| [m] | Max \|e\| [m] | Peak time [s] |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Kinematic LQR | yes | 3914.1259 | 507.8 | 7.7080 | 0.4541 | 2.7181 | 162.5 |
| MPC | **no** | 3914.1259 | — | — | 0.0043 † | 0.9114 † | 3.2 |
| PP | yes | 3914.1259 | 491.6 | 7.9620 | 0.0272 | 1.9507 | 156.9 |

† MPC statistics cover only the first 732 m it completed and **must not be ranked against the
complete runs**.

| Controller | Simulated duration [s] | Final reference progress [m] | Final x [m] | Final y [m] | Goal distance [m] |
| --- | ---: | ---: | ---: | ---: | ---: |
| Kinematic LQR | 507.8 | 3914.1259 | 363.0469 | 41.6882 | 0.9466 |
| MPC | 96.0 | 732.0000 | −1299.1324 | −57.3553 | 1664.1810 |
| PP | 491.6 | 3914.1259 | 363.0179 | 41.6962 | 0.9181 |

**MPC solver failure.** MPC aborted with `RuntimeError` and solver status `user_limit` at
simulation time 96.1 s, reference progress 732.0 m, vehicle position (−1298.861, −58.079) m. The
`trajectory.csv` for that run only contains samples up to the last successful control solve, i.e.
one `dt` earlier than the failed call; `failure.json` stores the failing status and the full
exception. `user_limit` means the solver stopped on a resource/iteration limit — this alone does
**not** prove the optimisation problem is mathematically infeasible.

**Maximum-deviation locations.**

| Controller | Peak time [s] | Peak progress [m] | Vehicle x [m] | Vehicle y [m] | κ at peak [1/m] | v [m/s] | δ [rad] | signed e [m] | saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Kinematic LQR | 162.5 | 1248.0 | −788.7077 | −25.7141 | −0.0000 | 8.0000 | 0.6109 | −2.7181 | 0.7966 |
| MPC † | 3.2 | 10.0 | −1451.8153 | 646.5136 | −0.5160 | 6.1806 | −0.6109 | −0.9114 | 0.0042 |
| PP | 156.9 | 1246.0 | −793.2153 | −28.0853 | 0.0000 | 8.0000 | 0.4086 | 1.9507 | 0.0000 |
| PP (4 m/s rerun) | 943.0 | 3787.0 | 235.3524 | 31.2307 | −0.1110 | 4.0000 | −0.4857 | −1.7210 | 0.0001 |

† MPC's peak lies in the first sharp bend near the start (s = 10 m), where steering hit the −35°
limit; it never reached the 1246–1248 m region where PP and LQR peaked. The three peaks are
therefore **not at the same place**, and MPC's smaller peak must not be read as a better full task.
The failure location (≈ 732 m) and the maximum-error location are also separate events.

For kinematic LQR and PP at 8 m/s the maximum deviations both occur in the region **after** a sharp
bend, at s = 1248 m and 1246 m, while the reference curvature near s = 1244 m is about 0.710 1/m and
the curvature at the peak instant is already ≈ 0. This supports the "error lags the sharp bend"
interpretation rather than "maximum error coincides with maximum curvature". Kinematic LQR's error
is additionally accompanied by strong steering oscillation. These are timing correlations with a
model-based explanation; actuator delay was not measured separately.

Figures: [`peak_map.png`](outputs/problem3/README.md) images were produced per run but the per-run
directories of member B's clone were not retained — only the aggregate comparison figures
[`task1_comparison.png`](outputs/problem3/task1_comparison.png) and
[`circle_comparison.png`](outputs/problem3/circle_comparison.png) survive.

### 3.3 Speed-reduction improvement on the GPX route

Only the PP target speed was changed, from 8 to 4 m/s; route, vehicle and controller parameters
were unchanged.

| Controller | Target speed [m/s] | Reached | Effective time [s] | Mean \|e\| [m] | Max \|e\| [m] | Peak progress [m] |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| PP | 8 | yes | 491.6 | 0.0272 | 1.9507 | 1246.0 |
| PP | 4 | yes | 976.4 | 0.0209 | 1.7210 | 3787.0 |

Peak error fell from 1.9507 to 1.7210 m (−11.8 %) and mean error by −23.1 %, at the cost of nearly
doubling the completion time (491.6 → 976.4 s). The maximum-deviation location **moved** to another
sharp bend around s = 3787 m (curvature ≈ −0.639 1/m at s = 3785 m, a very acute polyline corner).
Lower speed increases the number of correction opportunities, but the look-ahead distance shrinks
with speed too, so the peak is not guaranteed to fall monotonically. Moreover, a finite wheelbase
and a finite maximum steering angle still bound the minimum turning radius, so reducing speed cannot
be expected to remove geometric untrackability entirely.

**Model scope.** The offline road data is background only: the model includes no traffic signals,
pedestrians, right-of-way, real speed limits, junction waiting or road-grade-induced longitudinal
dynamics. The "effective time" is therefore the tracking completion time of this model, not a real
commuting time — and the planner's own estimated time in the GPX comments is not a measured
commuting time either.

### 3.4 Supporting benchmark: built-in routes (task 1)

These 16 runs (4 controllers × 4 routes) are part of member B's batch and are archived with the
Problem 3 results. Each route was compared with the algorithm as the only changed variable.
Cross-route comparisons are **not** fair: the same speed step means different numbers on different
routes (double lane change and mixed course 7 m/s, right angle 5.5 m/s, S-curve 6.5 m/s), so only
within-route rankings are reported.

![Built-in route controller comparison](outputs/problem3/task1_comparison.png)

| Route | Controller | Reached | mean \|e\| [m] | max \|e\| [m] | max \|δ\| [rad] | max \|a_n\| [m/s²] | max \|r\| [rad/s] | `J_δ` [rad/s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| double_lane_change | dynamic LQR | yes | 0.0827 | 0.1919 | 0.4328 | 9.0606 | 1.2203 | 0.2126 |
| double_lane_change | kinematic LQR | yes | 0.1904 | 0.6584 | 0.6109 | 9.2139 | 1.8485 | 4.3656 |
| double_lane_change | MPC | yes | 0.1849 | 0.5036 | 0.4787 | 8.7656 | 1.3659 | 0.5218 |
| double_lane_change | PP | yes | 0.3143 | 0.9447 | 0.3355 | 9.2065 | 0.9428 | 0.1568 |
| mixed_course | dynamic LQR | yes | 0.0706 | 0.1518 | 0.3344 | 7.0526 | 0.9585 | 0.1296 |
| mixed_course | kinematic LQR | yes | 0.1568 | 0.4770 | 0.6109 | 7.0526 | 1.8504 | 5.3884 |
| mixed_course | MPC | yes | 0.1527 | 0.3606 | 0.3725 | 6.6991 | 1.0448 | 0.4917 |
| mixed_course | PP | yes | 0.2555 | 0.7676 | 0.2552 | 6.9933 | 0.7244 | 0.1201 |
| right_angle | dynamic LQR | yes | 0.0566 | 0.3083 | 0.5228 | 5.9868 | 1.2119 | 0.0927 |
| right_angle | kinematic LQR | yes | 0.0989 | 0.7186 | 0.6109 | 5.9791 | 1.4418 | 0.6268 |
| right_angle | MPC | yes | 0.0667 | 0.4881 | 0.6109 | 5.5359 | 1.3615 | 0.1326 |
| right_angle | PP | yes | 0.1352 | 1.1064 | 0.4238 | 5.9703 | 0.9567 | 0.0767 |
| s_curve | dynamic LQR | yes | 0.0922 | 0.2148 | 0.5598 | 11.3297 | 1.5542 | 0.2323 |
| s_curve | kinematic LQR | yes | 0.2164 | 0.5469 | 0.6109 | 11.1877 | 1.7177 | 1.4576 |
| s_curve | MPC | yes | 0.1848 | 0.4891 | 0.6109 | 10.3271 | 1.6280 | 0.4480 |
| s_curve | PP | yes | 0.3332 | 1.1710 | 0.5871 | 11.1857 | 0.9468 | 0.2918 |

Route-averaged:

| Controller | mean lateral error over 4 routes [m] | mean peak error over 4 routes [m] |
| --- | ---: | ---: |
| PP | 0.260 | 1.000 |
| Kinematic LQR | 0.166 | 0.598 |
| Dynamic LQR | 0.076 | 0.215 |
| MPC | 0.147 | 0.459 |

- Dynamic LQR had the lowest mean lateral error on **all four** routes.
- Among the three mandatory controllers, MPC had the lowest mean error on all four routes.
- PP is simple and comparatively smooth-steering but cuts corners in sharp and repeated bends.
- MPC exploits a 0.8 s preview and penalises control variation, but its behaviour depends on the
  model, horizon and weights.
- The plant is still the kinematic backend; dynamic LQR's advantage shows that its control design
  works in this simulation setup, **not** that a dynamic vehicle model has been validated.

**Peak-location diagnostics.**

| Route | Controller | Peak time [s] | Peak progress [m] | κ at peak [1/m] | max \|κ\| on route [1/m] | steering saturation fraction |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| double_lane_change | dynamic LQR | 6.3 | 30.0 | −0.0003 | 0.2133 | 0.0000 |
| double_lane_change | kinematic LQR | 5.7 | 26.5 | −0.0922 | 0.2133 | 0.1929 |
| double_lane_change | MPC | 5.3 | 24.0 | −0.1598 | 0.2133 | 0.0000 |
| double_lane_change | PP | 6.9 | 36.0 | 0.1854 | 0.2133 | 0.0000 |
| mixed_course | dynamic LQR | 11.1 | 63.5 | 0.0517 | 0.1439 | 0.0000 |
| mixed_course | kinematic LQR | 10.2 | 57.5 | 0.1040 | 0.1439 | 0.2663 |
| mixed_course | MPC | 10.4 | 59.5 | 0.0785 | 0.1439 | 0.0000 |
| mixed_course | PP | 9.8 | 56.0 | 0.1275 | 0.1439 | 0.0000 |
| right_angle | dynamic LQR | 6.3 | 26.35 | 0.0000 | 0.2000 | 0.0000 |
| right_angle | kinematic LQR | 5.3 | 20.9971 | 0.2000 | 0.2000 | 0.0348 |
| right_angle | MPC | 5.3 | 20.9971 | 0.2000 | 0.2000 | 0.0087 |
| right_angle | PP | 4.8 | 18.4981 | 0.2000 | 0.2000 | 0.0000 |
| s_curve | dynamic LQR | 7.2 | 34.5 | −0.0149 | 0.2705 | 0.0000 |
| s_curve | kinematic LQR | 6.3 | 29.5 | 0.0876 | 0.2705 | 0.0400 |
| s_curve | MPC | 6.0 | 27.5 | 0.1938 | 0.2705 | 0.0068 |
| s_curve | PP | 7.9 | 41.0 | −0.2650 | 0.2705 | 0.0000 |

The peak instant is not necessarily the maximum-curvature point: the dynamic LQR's right-angle peak
occurs on the straight exit segment (κ = 0), and lane-change errors also accumulate near curvature
sign reversals. A single curvature value is not sufficient to establish a causal control-lag
explanation.

**Vehicle-parameter experiment.** On `right_angle` at 5.5 m/s (medium), `max_steer` was reduced from
35° to 20° for PP and kinematic LQR, keeping the baselines on the same route. A 5 m radius needs
`atan(2.5/5) = 26.565°`, so 20° is geometrically below the requirement.

| Controller | Steer limit [°] | Reached | mean \|e\| [m] | max \|e\| [m] | max \|β\| [rad] | max \|r\| [rad/s] | saturation fraction |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Kinematic LQR | 35 | yes | 0.0989 | 0.7186 | 0.3368 | 1.4418 | 0.0348 |
| PP | 35 | yes | 0.1352 | 1.1064 | 0.2219 | 0.9567 | 0.0000 |
| Kinematic LQR | 20 | yes | 0.1547 | 1.0933 | 0.1800 | 0.7860 | 0.1795 |
| PP | 20 | yes | 0.1259 | 1.0403 | 0.1800 | 0.7826 | 0.0789 |

Tightening the limit lowers the upper bounds on side-slip angle and yaw rate — cornering capability
genuinely shrinks — but the error does not degrade by a uniform factor, because the original
controllers also cut corners and oscillate. Kinematic LQR mean error rose 0.0989 → 0.1547 m and peak
error 0.7186 → 1.0933 m, while PP's actually fell slightly (0.1352 → 0.1259 m mean; 1.1064 →
1.0403 m peak). Both variants still reached the goal, which shows that **the arrival criterion
cannot substitute for cornering accuracy**.

### 3.5 Supporting benchmark: circle speed (task 2) — independent replication of Problem 1

Member B ran the same circle benchmark (`R = 12 m`, three speed steps, PP / kinematic LQR / MPC),
giving an independent replication of the Problem 1 experiment.

![Circle route comparison](outputs/problem3/circle_comparison.png)

| Controller | Speed step | Reached | whole-run mean \|e\| [m] | whole-run max \|e\| [m] | steady mean δ [rad] | steady mean r [rad/s] | steady mean a_n [m/s²] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Kinematic LQR | high | yes | 0.1900 | 0.4761 | 0.1968 | 0.5164 | 3.1818 |
| Kinematic LQR | low | yes | 0.2195 | 0.3419 | 0.2115 | 0.2567 | 0.7500 |
| Kinematic LQR | medium | yes | 0.2387 | 0.4130 | 0.2129 | 0.4125 | 1.9250 |
| MPC | high | yes | 0.1679 | 0.3475 | 0.2114 | 0.5950 | 4.0288 |
| MPC | low | yes | 0.1442 | 0.2260 | 0.2102 | 0.2561 | 0.7590 |
| MPC | medium | yes | 0.1566 | 0.2541 | 0.2108 | 0.4162 | 1.9922 |
| PP | high | yes | 0.2750 | 0.5251 | 0.2151 | 0.5289 | 3.1893 |
| PP | low | yes | 0.2859 | 0.4462 | 0.2140 | 0.2592 | 0.7500 |
| PP | medium | yes | 0.2878 | 0.5046 | 0.2154 | 0.4173 | 1.9261 |

Deviation from the nominal steady-state theory:

| Controller | Step | theory δ [rad] | δ rel. error [%] | theory r [rad/s] | r rel. error [%] | theory a_n [m/s²] | a_n rel. error [%] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Kinematic LQR | high | 0.2054 | 4.1962 | 0.5833 | 11.4805 | 4.0833 | 22.0779 |
| Kinematic LQR | medium | 0.2054 | 3.6328 | 0.4167 | 1.0111 | 2.0833 | 7.6005 |
| Kinematic LQR | low | 0.2054 | 2.9496 | 0.2500 | 2.6642 | 0.7500 | 0.0001 |
| MPC | high | 0.2054 | 2.9124 | 0.5833 | 2.0056 | 4.0833 | 1.3355 |
| MPC | medium | 0.2054 | 2.6545 | 0.4167 | 0.1085 | 2.0833 | 4.3739 |
| MPC | low | 0.2054 | 2.3287 | 0.2500 | 2.4563 | 0.7500 | 1.2025 |
| PP | high | 0.2054 | 4.7344 | 0.5833 | 9.3249 | 4.0833 | 21.8954 |
| PP | medium | 0.2054 | 4.8763 | 0.4167 | 0.1567 | 2.0833 | 7.5481 |
| PP | low | 0.2054 | 4.1698 | 0.2500 | 3.6963 | 0.7500 | 0.0002 |

Relative error is `|simulated mean − theory| / |theory| × 100 %`. The steering approximation neglects
`β`, whereas the plant retains `cos β`, so the residual contains both the geometric approximation
and the control steady-state bias. For varying-speed segments the *mean* normal acceleration must be
compared with `mean(v²)/R`, not `mean(v)²/R`:

| Controller | Step | mean(v)/R [rad/s] | mean(v²)/R [m/s²] | max residual of `a_n = v²κ` |
| --- | --- | --- | --- | --- |
| Kinematic LQR | high | 0.5064 | 3.1818 | 0.0000 |
| Kinematic LQR | medium | 0.3996 | 1.9250 | 0.0000 |
| Kinematic LQR | low | 0.2500 | 0.7500 | 0.0000 |
| MPC | high | 0.5794 | 4.0288 | 0.0000 |
| MPC | medium | 0.4075 | 1.9922 | 0.0000 |
| MPC | low | 0.2515 | 0.7590 | 0.0000 |
| PP | high | 0.5073 | 3.1893 | 0.0000 |
| PP | medium | 0.3998 | 1.9261 | 0.0000 |
| PP | low | 0.2500 | 0.7500 | 0.0000 |

The near-zero residual is a **consistency check of the logging formula itself** (`a_n = v²κ` is the
very expression that was logged) and cannot independently validate tyre grip. There is no friction
circle or tyre-saturation model in this platform, so the experiment can expose discretisation and
model mismatch but cannot prove that a real vehicle could sustain that lateral acceleration.

Steady-arc statistics (samples with `|κ − 1/12| < 0.005`, first and last 10 m of the arc removed,
i.e. reference progress ≈ 26.000–81.393 m; the same spatial window for all controllers; varying-speed
and oscillating samples retained, signed-error standard deviation with `ddof = 0`, `J_δ` from
adjacent 0.1 s samples):

| Controller | Step | samples | mean v [m/s] | v std | signed mean e [m] | mean \|e\| [m] | max \|e\| [m] | e std [m] | max \|β\| [rad] | `J_δ` [rad/s] |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Kinematic LQR | high | 89 | 6.0767 | 1.1202 | 0.3291 | 0.3291 | 0.4761 | 0.0710 | 0.3368 | 5.4402 |
| Kinematic LQR | medium | 111 | 4.7954 | 0.3232 | 0.3796 | 0.3796 | 0.4130 | 0.0203 | 0.1785 | 0.4999 |
| Kinematic LQR | low | 178 | 3.0000 | 0.0000 | 0.3312 | 0.3312 | 0.3419 | 0.0068 | 0.1364 | 0.9655 |
| MPC | high | 77 | 6.9531 | 0.0218 | 0.2982 | 0.2982 | 0.3385 | 0.0130 | 0.1563 | 1.2725 |
| MPC | medium | 110 | 4.8894 | 0.0069 | 0.2519 | 0.2519 | 0.2530 | 0.0005 | 0.1068 | 0.0001 |
| MPC | low | 179 | 3.0179 | 0.0186 | 0.2190 | 0.2190 | 0.2260 | 0.0049 | 0.1370 | 0.7725 |
| PP | high | 86 | 6.0877 | 1.1008 | 0.4779 | 0.4779 | 0.5251 | 0.0113 | 0.1162 | 0.1111 |
| PP | medium | 110 | 4.7973 | 0.3149 | 0.4711 | 0.4711 | 0.5046 | 0.0165 | 0.1230 | 0.0314 |
| PP | low | 177 | 3.0000 | 0.0000 | 0.4383 | 0.4383 | 0.4462 | 0.0036 | 0.1176 | 0.1939 |

Transient / oscillation summary:

| Controller | Step | peak in last 10 m before arc [m] | peak mid-arc [m] | peak in first 10 m after arc [m] | whole-run saturation | max steering rate [rad/s] |
| --- | --- | --- | --- | --- | --- | --- |
| Kinematic LQR | high | 0.3766 | 0.4761 | 0.3525 | 0.2264 | 12.2173 |
| Kinematic LQR | medium | 0.3895 | 0.4130 | 0.4084 | 0.0000 | 3.8225 |
| Kinematic LQR | low | 0.3123 | 0.3419 | 0.3413 | 0.0000 | 3.0416 |
| MPC | high | 0.3110 | 0.3385 | 0.3475 | 0.0000 | 1.7347 |
| MPC | medium | 0.2541 | 0.2530 | 0.2533 | 0.0000 | 0.2626 |
| MPC | low | 0.2214 | 0.2260 | 0.2260 | 0.0000 | 1.0888 |
| PP | high | 0.5235 | 0.5251 | 0.4828 | 0.0000 | 0.3281 |
| PP | medium | 0.5037 | 0.5046 | 0.4755 | 0.0000 | 0.3149 |
| PP | low | 0.4387 | 0.4462 | 0.4404 | 0.0000 | 0.2637 |

Member B's reading of these numbers agrees with Problem 1: with a fixed `dt` the nominal distance
per step grows from 0.3 to 0.7 m, so corrections in a sharp bend become harder; PP's look-ahead
grows with speed, which smooths the command but increases the corner cut; the LQR's error-rate term
and previous-step steering produce strong discrete feedback whose oscillation can keep the mean
small; MPC's constraint and control-variation penalties reduce some abruptness but do not remove
model mismatch.

**One confounder is worth repeating** (this is platform defect 2 of Problem 1): `speed_pid`
throttles within 14 m of the goal measured as a straight-line distance, so on the closed circle PP
and LQR can decelerate mid-arc. Hence the medium/high steady speeds are below the nominal step and
the nominal speed must not be treated as truly constant. MPC optimises speed inside its QP and never
calls this PID, so it is unaffected — that difference is part of the current algorithm
implementation.

At the high step, the smallest steady-arc mean absolute error is MPC's (0.2982 m) while the smallest
`J_δ` is PP's (0.1111 rad/s): **accuracy and smoothness must be ranked separately**. Kinematic LQR's
whole-run mean error at the high step is *below* its medium-step value, yet its saturation fraction
is ≈ 22.6 % and its arc `J_δ` ≈ 5.44 rad/s with visible sign oscillation, so "high speed is easier"
is not a valid conclusion. PP's arc error (≈ 0.478 m) is clearly larger than MPC's (≈ 0.298 m), while
PP's `J_δ` (≈ 0.111 rad/s) is smaller than MPC's (≈ 1.272 rad/s). The error growth from low to high
speed is not monotonic in every metric: the physical demand grows as `v²`, but the control error also
depends on speed regulation, look-ahead, sampling and model mismatch.

---

## 4. Problem 4 — dorm (Juyuan Building 8) → Teaching Building 1 navigation (member A)

A 1450.6 m route with 2903 reference points at 0.5 m spacing, 7 corners, run fully offline. Full
analysis in Chinese: [`docs/问题4_报告.md`](docs/问题4_报告.md); figures in
[`docs/figures/q4_*`](docs/figures/) and [`outputs/dorm_figures/`](outputs/dorm_figures/); raw
experiment tables in [`outputs/dorm_experiments.md`](outputs/dorm_experiments.md) /
[`.json`](outputs/dorm_experiments.json) and
[`outputs/dorm_lookahead_sweep.csv`](outputs/dorm_lookahead_sweep.csv).

### 4.1 Map construction

**Data source:** the repository's offline OSM slice
`data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz` (1.3 km × 3 km, covering the whole
campus), decompressed with the standard-library `lzma` module — no network access and no new
dependency.

**Graph building:** each OSM way is split into edges at its node coordinates, and nodes shared by
several ways (junctions) are merged into a single vertex, giving an undirected weighted graph:

- 1745 nodes, 1989 edges, 146.7 km of total length
- **number of connected components = 1**, so the campus road network is fully connected and any two
  points are reachable

**Why a graph rather than connecting the GPX points directly:** connecting start and end directly
yields a straight line through buildings. Only graph routing keeps the path on real roads.

**Landmark snapping:** "Juyuan Building 8" and "Teaching Building 1" are OSM polygons; their
centroids were projected onto the nearest road-graph node.

| Landmark | centroid → network distance | snapped node |
| --- | --- | --- |
| Juyuan Building 8 | 44.4 m | 227 |
| Teaching Building 1 | 74.5 m | 1109 |

The offset is the true distance from a building-polygon centroid to the road centreline (buildings
have depth), not a localisation error.

**Routing:** Dijkstra over 54 network nodes, giving a raw polyline of 1463.5 m.

![Road graph and routing result](docs/figures/q4_fig1_road_graph.png)

### 4.2 Reference path generation

**The problem: OSM polylines have sharp corners at junctions.** Two roads share a single node at a
junction, so the polyline turns through nearly 90° there. Differentiating the resampled polyline
directly gives **|κ| = 1.60 1/m, i.e. a 0.62 m turning radius**, whereas the vehicle limit from
`tan(δ_max)/L = tan 35° / 2.50` is **0.280 1/m (R = 3.57 m)**. The raw reference demands
**5.7× the vehicle's capability** — this is not a tracking-accuracy issue, the vehicle simply cannot
follow it and the controller saturates throughout. It was the largest pitfall in this task and the
real reason a runnable algorithm can still produce very poor results.

**Two-step fix:**

1. **Corner fillets** — insert a circular arc tangent to both edges at every vertex turning more
   than 20°; all 7 corners were processed.
2. **Curvature smoothing** — a 12 m Gaussian filter on the differentiated curvature.

After processing, **`κ_max = 0.117 1/m` (R = 8.58 m)**, inside the vehicle envelope.

![Curvature comparison](docs/figures/q4_fig3_curvature.png)

**Attribution — smoothing, not filleting, does the work** (with 12 m smoothing active in both rows):

| Processing | `κ_max` [1/m] | turning radius [m] |
| --- | --- | --- |
| raw polyline, no smoothing | 1.6049 | 0.62 |
| raw polyline + smoothing | 0.1355 | 7.38 |
| fillet + smoothing | 0.1166 | 8.58 |

Smoothing accounts for 92 % of the improvement, filleting only 14 %: smoothing averages the vertex
spike over a 12 m window, while a fillet merely replaces that spike with a real arc — and an 8 m
radius arc has curvature 0.125 1/m, the same order as the smoothed result. Filleting is still worth
doing because it makes the reference path **physically realisable** as geometry (true arcs), not just
a flattened polyline.

**The A/B comparison also exposed a hidden bug:** curvature smoothing was applied only when
`speed_profile == "curvature"`; the `constant` profile differentiated the raw path. As a result
"fillet + constant speed" reported a 25.4 m/s² lateral acceleration while "raw + constant speed"
reported only 17.8 m/s² — the optimised path looked *worse*, which is clearly wrong. Fix:
**curvature is a property of the path, not of the speed profile**, so it must be smoothed
unconditionally. Curvature feeds both the lateral-acceleration metric and the LQR feed-forward term,
so letting different profiles use different curvature destroys the A/B comparison.

**Corner inventory.**

| # | progress [m] | turn angle [°] | fillet radius [m] |
| --- | --- | --- | --- |
| 1 | 15.0 | −24.3 | 8.0 |
| 6 | 62.5 | −59.2 | 7.7 |
| 11 | 347.5 | +89.5 | 8.0 |
| 13 | 460.0 | −89.4 | 8.0 |
| **16** | **589.0** | **−87.2** | **2.4** |
| 50 | 1387.4 | −90.4 | 5.4 |
| 51 | 1396.9 | +89.3 | 5.0 |

**Corner #16 is the hardest point on the route**: 87° of turn in only 2.4 m of available space, the
only genuinely tight bend. Every lateral-error peak in the experiments below falls at
s ≈ 586–593 m, i.e. here.

### 4.3 Speed profile

Curvature-based limit `v = √(a_lat_max / κ)` with `a_lat_max = 2.5 m/s²`: straights run at the target
speed, and corner #16 (`κ = 0.417 1/m`) is limited to **4.47 m/s**. Two feasibility sweeps are then
applied — a **backward** braking pass from the 0 m/s terminal speed using
`v_i² ≤ v_{i+1}² + 2 a_brake Δs`, and a **forward** acceleration pass using
`v_i² ≤ v_{i-1}² + 2 a_accel Δs`.

**A pitfall here:** both passes must be written as **point-by-point recursions**, not as array
expressions. Written as `np.sqrt(speed[1:]**2 + 2*a*ds)` the right-hand side uses the *unlimited*
speed array, so `min()` changes nothing except the last point — and the vehicle then brakes only in
the final 0.12 m. After the fix the 8 → 0 deceleration ramp is about 15 m long and the peak lateral
acceleration is exactly 2.500 m/s².

![Speed profile](docs/figures/q4_fig4_speed_profile.png)
![Lateral acceleration](docs/figures/q4_fig5_lateral_accel.png)

### 4.4 Pure Pursuit implementation

From the current position, find the nearest point on the reference path and step forward
`L_d = L_f + k·v` to the look-ahead point, then

```text
δ = atan2(2 · L · sin α / L_d)
```

with wheelbase `L = 2.50 m`, speed-adaptive look-ahead (`k = 0.35`, so the vehicle looks further
ahead at speed and the gain `2L/L_d` does not become large enough to cause steering chatter), and `α`
the angle between the heading and the line to the look-ahead point. The implementation lives in
`vdm_lab/student/` behind the same `Controller` interface as the provided LQR and MPC, so the
controllers are interchangeable.

![Trajectories of the three controllers](docs/figures/q4_fig6_algorithms_map.png)

### 4.5 Experiment matrix and findings

Only one independent variable was changed per group. All runs used
`--map-origin 118.8145 31.8885`, aligned with the basemap origin.

| Group | Experiment | Reached | mean error [m] | peak error [m] | terminal error [m] | peak `a_n` [m/s²] | max front steer [°] |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A | repo baseline (unsmoothed) | yes | 0.055 | 1.964 | 0.897 | **17.827** | 27.230 |
| A | raw + smoothing + constant speed | yes | 0.055 | 1.964 | 0.897 | 7.970 | 27.230 |
| A | raw + smoothing + curvature limit | yes | 0.060 | 1.781 | 1.270 | **4.535** | 28.741 |
| A | fillet + constant speed | yes | 0.059 | 1.822 | 0.923 | 7.300 | 23.673 |
| A | fillet + curvature limit | yes | 0.065 | 1.712 | 1.228 | **4.338** | 25.555 |
| A | LQR unsmoothed | yes | 0.313 | 2.924 | 1.014 | **91.040** | 35.000 |
| A | LQR smoothed | yes | 0.393 | 2.532 | 1.031 | 8.614 | 35.000 |
| B | PP curvature limit @ 3 m/s | yes | 0.046 | 1.417 | 0.682 | 1.049 | 31.528 |
| B | PP curvature limit @ 5 m/s | yes | 0.051 | 1.593 | 0.920 | 2.811 | 28.438 |
| B | PP curvature limit @ 7 m/s | yes | 0.060 | 1.643 | 1.166 | 3.908 | 26.502 |
| C | PP @ 8 m/s | yes | 0.065 | 1.712 | 1.228 | 4.338 | 25.555 |
| C | LQR @ 8 m/s | yes | 0.303 | 0.632 | 1.250 | 4.352 | 35.000 |
| C | MPC @ 8 m/s | **no** | 0.033 | 0.936 | 1.625 | 3.463 | 35.000 |
| C | PP @ 5 m/s | yes | 0.051 | 1.593 | 0.920 | 2.811 | 28.438 |
| C | LQR @ 5 m/s | yes | 0.036 | 0.521 | 0.918 | 2.801 | 35.000 |
| C | MPC @ 5 m/s | yes | 0.029 | 0.870 | 0.728 | 2.639 | 35.000 |

**Finding 1 — curvature smoothing is the single most important step.** The LQR row is the clearest:
an unsmoothed reference produced a **peak lateral acceleration of 91.04 m/s²**, dropping to
8.61 m/s² after smoothing — a factor of **10.6**. Test-vehicle tyres cannot supply that
acceleration; 91 m/s² is purely the controller flailing at a bend that physically does not exist.

**Finding 2 — curvature-based speed limiting brings lateral acceleration down to the design value.**
For raw + smoothing, switching from constant speed to curvature limiting cut the peak from
7.970 to 4.535 m/s²; for the fillet pair, from 7.300 to 4.338 m/s². The cost is a larger terminal
error (0.897 → 1.228 m) because the vehicle is slower overall yet enters the final deceleration
segment later, so it coasts further past the goal. At ≈ 1 m against a 1450 m route, this is
negligible.

**Finding 3 — higher speed means larger error, quantitatively confirming Problem 1.** With the
curvature limit active, PP was run at 3 / 5 / 7 m/s changing only the target speed:

| Speed [m/s] | mean error [m] | peak error [m] | terminal error [m] | peak `a_n` [m/s²] |
| --- | --- | --- | --- | --- |
| 3 | 0.046 | 1.417 | 0.682 | 1.049 |
| 5 | 0.051 | 1.593 | 0.920 | 2.811 |
| 7 | 0.060 | 1.643 | 1.166 | 3.908 |

Both error metrics increase monotonically, matching the Problem 1 theory. The mechanism is
`a_n = v²κ`: through the same bend, going from 3 to 7 m/s multiplies the lateral-acceleration demand
by 4.8, with measured 1.049 → 3.908 (3.7× — the difference is because at 3 m/s the vehicle never
reaches the curvature-limited speed).

![Error vs speed](docs/figures/q4_fig9_error_vs_speed.png)
![Lateral acceleration vs speed](docs/figures/q4_fig10_accel_vs_speed.png)

**Finding 4 — controller trade-offs at 5 m/s.**

| | mean error [m] | peak error [m] | front steer [°] | note |
| --- | --- | --- | --- | --- |
| PP @ 5 m/s | 0.051 | 1.593 | 28.4 | ~20 lines of code, largest peak error |
| LQR @ 5 m/s | **0.036** | **0.521** | 35.0 | smallest peak error, but steering permanently saturated |
| MPC @ 5 m/s | **0.029** | 0.870 | 35.0 | best mean error, largest computational cost |

**PP** watches a single look-ahead point and therefore cuts the inside of bends (peak 1.593 m vs
0.521 m for LQR and 0.870 m for MPC); its value is simplicity and interpretability.
**LQR** has the smallest peak error, but its maximum front-wheel angle is *exactly* 35.000° in every
single run — it runs on the physical limit throughout, because it has no steering constraint and
falls back on saturation. Its steady-state error is genuinely small, but it has no margin left under
disturbance. **MPC** has the best mean error (0.029 m) because it writes the next 0.8 s
(8 steps × 0.1 s) of tracking error into its cost and steers into bends early; the price is solving a
QP every control cycle.

**Finding 5 — the look-ahead distance is a genuine trade-off, confirmed experimentally.** Sweeping
`L_f ∈ {1, 2, 3, 5, 8, 12} m` × speed `{3, 5, 8} m/s` gave 18 closed-loop runs. The peak error grows
monotonically with `L_f` (at 8 m/s: 1.057 m at `L_f = 1` up to 4.117 m at `L_f = 12`), while steering
chatter falls monotonically with `L_f` (at 3 m/s: steering-rate P95 from 23.8 to 1.8 °/s, and
steering reversals from 463 at `L_f = 1` to 1 at `L_f = 12`). Mechanism: a small `L_f` gives a large
gain `2L/L_f`, tracking the line tightly but chattering; a large `L_f` is smooth but cuts the bend
radius earlier, producing a systematic offset to the inside. **All 18 peak errors fall at
s ≈ 586–593 m** — again corner #16, whose 2.4 m radius sets the quality ceiling of the whole route.

![Error vs look-ahead](docs/figures/q4_fig11_error_vs_lookahead.png)
![Steering rate vs look-ahead](docs/figures/q4_fig12_steerrate_vs_lookahead.png)

**Finding 6 — an explainable anomaly: MPC @ 8 m/s did not reach the goal.** It is the only "no" in
the matrix. The terminal error was 1.625 m against a 1.5 m threshold — short by 0.125 m. MPC arrived
at the goal at 2.36 m/s and coasted a further 1.625 m before stopping, while the stop criterion
requires **both** distance < 1.5 m **and** speed < 0.5 m/s simultaneously, and its speed had not yet
decayed when it passed the goal. The deeper cause is an interface asymmetry: **PP and LQR actively
throttle within 14 m of the goal** (`sqrt(2·0.9·d)`, provided by `speed_pid`), whereas MPC's
acceleration comes from its own QP with no such term and its speed tracking lags the reference
profile by 1.0–2.4 m/s, so it cannot brake in time. The provided `mpc.py` was **not modified** (it is
course-supplied); the finding is recorded because it shows MPC needs an explicit position-dependent
terminal constraint near the goal, otherwise its speed loop cannot follow the deceleration segment.

A related observation in `mpc.py`: `cp.abs(u[0,:]) <= vehicle.max_accel` also clamps *deceleration*
to 2.0 m/s², making the following `u[0,:] >= -vehicle.max_decel` (−3.5) dead code. Consequently a
planned deceleration designed for −3.5 m/s² cannot actually be executed — which is why the default
planning deceleration of `curvature_speed_profile` is `0.5 × 3.5 = 1.75 m/s²`, leaving margin for the
tightest controller constraint.

---

## 5. Cross-problem synthesis

### 5.1 The same physics appears in three independent settings

| Setting | Evidence for `a_n = v²κ` / speed-dependent difficulty |
| --- | --- |
| Problem 1, circle `R = 12 m`, 3 → 7 m/s | demand 0.75 → 4.083 m/s² (×5.44); measured a_n ≈ 99.6 % of theory |
| Problem 4, 1450 m city route, PP at 3 → 7 m/s | peak `a_n` 1.049 → 3.908 m/s² (×3.7); mean error 0.046 → 0.060 m, peak error 1.417 → 1.643 m, both monotone |
| Problem 3, same city route at 8 vs 4 m/s | peak error 1.9507 → 1.7210 m (−11.8 %), mean error −23.1 %, time ×1.99 |

The two members independently reproduced the Problem 1 circle benchmark as part of Problem 3's
batch, and the trajectories agree exactly on the primary metrics (whole-run mean/max lateral error
for all nine runs). The *steady-arc* aggregates differ in the third decimal (e.g. kinematic LQR at
the high step: steady mean steering 0.19164 vs 0.1968 rad; steady signed mean error 0.3292 vs
0.3291 m) because the two members used slightly different steady-segment windows. **Consequence:
steady-segment statistics are only comparable when the window definition is reported with the
table** — both members' tables state their criteria.

### 5.2 Independent discovery of the same two platform defects

Both members hit the same platform limits from different directions:

- **No `max_steer_rate` enforcement** (`limit_command`). Member A proved it causes the LQR
  high-speed artefact by re-running with an external rate limiter (Problem 1, [§2.5](#25-refuted-explanations));
  member B measured 44.6 % rate-limit violations for MPC and 59.7 % for the LQR on the same circle
  (`all_results.csv`, columns `max_steer_rate_radps`, `rate_over_45deg_fraction`).
- **Goal-proximity throttling by straight-line distance** (`speed_pid`). Member A showed it
  wrongly slows the vehicle mid-arc on closed routes; member B hit the mirror image of the same
  weakness from the MPC side, where the missing terminal braking term let MPC overshoot the goal.

### 5.3 Algorithm ranking depends on the metric *and* the terrain

| Scenario | Best mean error | Best peak error | Comment |
| --- | --- | --- | --- |
| Problem 1, circle (sustained curvature) | MPC (0.2987 m) | MPC (0.3475 m) | PP worst on both |
| Problem 3, built-in routes | dynamic LQR > MPC > LQR k > PP | dynamic LQR > MPC > LQR k > PP | PP worst on all four |
| Problem 3, 3914 m GPX route | PP (0.0272 m) | PP (1.9507 m) | LQR k mean 0.4541 m with 79.7 % steering saturation |
| Problem 4, 1450 m city route | MPC (0.029 m) | LQR (0.521 m) | PP peak 1.593 m, LQR steering always saturated |

The reversal between the circle (PP worst) and the long geographic routes (PP best on mean error) is
explained by what those routes contain: the circle is **continuously curved**, which maximises PP's
`L_f·β` corner-cut bias, while the long GPX and city routes are dominated by straights where every
controller tracks well, and PP's 0 % rate-limit violations keep it free of the oscillatory penalties
that hurt kinematic LQR (79.7 % saturation, `J_δ` ≈ 5.44 rad/s on the circle). This is an
observation from the measured data, consistent with the mechanisms above; it has not been tested on
a controlled pair of routes with matched curvature statistics.

### 5.4 MPC is the accuracy leader but its solver is the reliability risk

Across all three problems MPC produced the smallest circle error and the best mean error on both
long routes, and dynamic LQR (member B) was the overall accuracy leader on the built-in routes. The
reliability cost is visible twice:

- Problem 3: MPC aborted with solver status `user_limit` at 732 m of a 3914 m route.
- Problem 4: MPC @ 8 m/s failed the stop criterion by 0.125 m because of the missing terminal
  braking term; the same run shows `abs(u[0]) ≤ 2.0` clamping deceleration below the declared
  `−3.5 m/s²` limit.

Both are consistent with "MPC's advantage in short, well-posed problems becomes an engineering
burden on long routes and near the terminal state".

### 5.5 Honest scope: what the merged work does *not* show

- **The plant is always the kinematic bicycle model.** The dynamic LQR's advantage validates a
  controller design against this plant, not a full dynamic vehicle model. Tyre slip angles, load
  transfer and friction limits are never exercised.
- **`a_n = v²κ` in the logs is the logging formula itself**; its near-zero residual is a
  consistency check, not a grip validation. There is no friction circle in the platform.
- **`lqr_dynamic` was not implemented or tested by member A** (it was an optional advanced item in
  member A's problem set); member B's `lqr_dynamic` results exist only for the built-in routes.
- **Problem 1 only covers the `circle` route**; no claim is made there about other routes.
- **Problem 1 cause 3 was not isolated** by experiment.
- **The two platform defects were identified and worked around, not fixed.** All Problem 1 data was
  collected on the unfixed platform; the rate-limit comparison is an external wrapper that does not
  modify `vdm_lab/common/vehicle.py`.
- **MPC's steering-rate constraint is incomplete** (only intra-horizon), causing 44.6 % violations.
- **`ds` was not separated from the steady-state values** in Problem 1: the reported table uses the
  default `ds = 0.5 m`, which carries ≈ 0.06 m of discretisation contribution, while the
  `R² = 0.99881` fit uses `ds = 0.05 m`.
- **Problem 3's route is a repository sample GPX, not a personal dorm → classroom route**, and the
  exact coordinates raise a location-privacy issue that must be handled before public submission.
- **Problem 3's maximum point spacing is 619.924 m**: linear resampling of a sparse GPX cannot
  recover real road geometry, which limits how far the maximum-deviation explanation can be pushed.
- **Problem 4's offline map carries no traffic semantics** (signals, pedestrians, priority, real
  speed limits, junction waits, road grade), so its completion times are model times, not commute
  times.

---

## 6. Reproducibility

### 6.1 Environment

The two members ran in slightly different environments, which is acceptable because the plant and
controllers are deterministic and the shared benchmark reproduced exactly.

| Item | Member A (Problems 1, 4) | Member B (Problem 3) |
| --- | --- | --- |
| Python | 3.10 (`vdm_lab` bytecode cache) | 3.13.7 |
| NumPy / SciPy / Matplotlib | as installed for the analyses below | 2.5.1 / 1.18.0 / 3.11.0 |
| QP solver | OSQP | CVXPY 1.9.2 + OSQP |
| `requirements.txt` note | — | installed NumPy exceeded the declared `< 2.0` bound; the environment was used as-is |

### 6.2 Problem 1 and Problem 4 commands

```bash
# Problem 1: 9 circle runs (3 controllers x 3 speed steps)
python run_experiment.py --algo <pp|lqr_kinematic|mpc> --version student \
  --route circle --speed-mode <low|medium|high> --save-log --save-fig

python scripts/analyze_circle_speed.py     # tables 1-4 and figures 1-8
python scripts/analyze_pp_corner_cut.py    # corner-cut sweeps A/B/C, figures 9-12
python scripts/plot_run_panels.py          # split combined panels into single plots
python scripts/verify_steer_rate_limit.py  # rate-limit before/after, figures 13-14
python scripts/make_report_figures.py      # assemble docs/figures/ (fig01..fig18)
```

Problem 4:

```bash
python scripts/build_dorm_route.py        # OSM graph, Dijkstra, fillet + smoothing, speed profile
python scripts/run_dorm_experiments.py    # groups A/B/C and the look-ahead sweep
python scripts/sweep_lookahead.py         # 18-run look-ahead sweep
python scripts/analyze_dorm_task.py       # aggregate into outputs/dorm_*
```

### 6.3 Problem 3 commands

The exact per-run command for all 31 experiments is logged in
[`outputs/problem3/commands.md`](outputs/problem3/commands.md). Member B ran a batch driver
(`scripts/complete_course_tasks.py`) that writes an `output_dir` per case and reuses an existing
`result.json`; the equivalent single-run entry point is:

```powershell
python run_experiment.py --algo <pp|lqr_kinematic|lqr_dynamic|mpc> --version student \
  --vehicle student_car --route <route> --speed-mode <low|medium|high> --save-log --save-fig
python scripts/complete_course_tasks.py --gpx data/gpx/homework_route_1.gpx --tasks task3 improvement `
  --output reports/course_tasks/outputs/course_tasks_gpx
python scripts/build_course_report.py
```

> ⚠ Those driver scripts belong to member B's clone and are **not present** in this merged
> repository (see the merge note at the top). Their full command log is preserved in
> [`outputs/problem3/commands.md`](outputs/problem3/commands.md), including the per-case
> `--only <case_id>` invocations and the equivalent main-entry commands.

---

## 7. Conclusions

1. **Speed makes path tracking harder through three mechanisms**: the quadratic lateral-acceleration
   demand `a_n = v²κ` (physical, measured to within 3.42 %), PP's look-ahead-amplified corner cut
   `≈ 0.93·L_f·β` (geometric, `R² = 0.99881`, and the dominant term for PP), and the `1/v` loss of
   correction opportunities per metre (plausible, not isolated). It is **not** caused by a lack of
   steering authority: the required steady steering angle is speed-independent and uses at most
   38 % of the limit.
2. **The platform, not the LQR, produced the high-speed steering oscillation.** The steering-rate
   limit is never enforced, so the controller commands ≈ 9 rad/s against a declared 0.7854 rad/s
   limit; adding a real rate limiter cuts `J_δ` to 1/11.7 with no accuracy loss. Fixed-gain LQR has
   sufficient cross-speed margin — **no gain scheduling is needed.**
3. **On geographic routes, the reference path is the dominant error source.** Raw OSM polylines
   demand `κ = 1.60 1/m` against a `0.280 1/m` vehicle limit (5.7×); 12 m curvature smoothing
   (92 % of the benefit) plus corner fillets bring this to `0.117 1/m` and reduce the LQR peak
   lateral acceleration from 91.04 to 8.61 m/s². Corner #16 at s = 589 m (87° in 2.4 m) sets the
   error ceiling for every run in Problem 4.
4. **Controller choice depends on the metric and the terrain.** MPC is the accuracy leader on the
   circle and gives the best mean error on both long routes; dynamic LQR dominates the built-in
   routes; PP is last on sustained curvature but first on the long GPX route's mean error, and it is
   the only controller with 0 % steering-rate violations. LQR achieves small errors but runs
   steering at exactly 35.000° in every geographic run, i.e. with no margin.
5. **MPC's price is reliability, not accuracy.** A `user_limit` solver abort ended one long GPX run,
   and the absence of a terminal braking term made MPC miss the Problem 4 stop window by 0.125 m.
6. **Curvature-based speed limiting is the practical lever.** It caps lateral acceleration at the
   design value (7.97 → 4.54 m/s²) for ≈ 0.3 m of extra terminal error, and reducing speed further
   (8 → 4 m/s on the GPX route) buys −11.8 % peak and −23.1 % mean error for roughly double the
   travel time — a tracking-accuracy-versus-efficiency trade chosen by the application, not a
   free improvement.

---

## 8. Open items

- [x] All runs, arrival states and failures retained rather than pruned.
- [x] Per-run command, configuration and output path logged ([`outputs/problem3/commands.md`](outputs/problem3/commands.md)).
- [x] Circle experiments use a single spatial window across controllers.
- [x] GPX file and basemap share the same map origin.
- [x] Maximum deviations identified by absolute lateral error, with signed values retained.
- [x] All figures carry titles, axis labels, units and legends.
- [ ] **Replace the sample GPX with a genuinely self-planned dorm → classroom route** (Problem 3).
- [ ] **Anonymise precise coordinates before any public submission** (Problem 3).
- [ ] Fix `limit_command()` to enforce `max_steer_rate` at the actuator layer, and re-derive all
      smoothness metrics.
- [ ] Fix `speed_pid()` goal throttling to use remaining arc length instead of straight-line distance.
- [ ] Add an explicit terminal position/speed constraint to MPC so its speed loop follows the
      reference deceleration profile.
- [ ] Complete MPC's steering-rate constraint to include `u[1,0]` against the previously executed
      command.
- [ ] Unify the corner-cut attribution between the two clones' `vdm_lab` trees, and align the
      dynamic-model stiffness sign convention and discretisation before re-validating the
      controllers on a dynamic vehicle backend.

---

## 9. Artifact index

> The Problem 1 and Problem 4 slide decks (`docs/PPT/*.pptx`) and their talk scripts were originally
> written in Chinese and have been translated into English in place — slide text, speaker notes,
> talk script body and both appendices. The Chinese originals have been removed. Emphasis was
> preserved during translation: within a multi-run paragraph the English was placed back into the
> original runs, so the highlighted numbers still highlight the same numbers, and the smaller-font
> subscript in `v = √(alat,max / κ)` is unchanged.

### Problem 1 (member A — circle speed / tracking difficulty)

| Artifact | Path |
| --- | --- |
| Full Chinese analysis | [`docs/问题1_速度与跟踪难度分析.md`](docs/问题1_速度与跟踪难度分析.md) |
| Extended attribution backup | [`docs/问题1_分析细节_完整版备份.md`](docs/问题1_分析细节_完整版备份.md) |
| Short talk script (English) | [`docs/PPT/problem1_talk_script_short.md`](docs/PPT/problem1_talk_script_short.md) |
| Slide deck (English, with speaker notes) | [`docs/PPT/problem1_speed_and_path_tracking_difficulty.pptx`](docs/PPT/problem1_speed_and_path_tracking_difficulty.pptx) |
| 9 raw run logs (`trajectory.csv`, `reference_path.csv`, `metrics.json`, `summary.png`) | [`outputs/20260912_1518xx_*/`](outputs/) |
| Tables 1–4 + CSVs | [`outputs/analysis_circle_speed/`](outputs/analysis_circle_speed/) |
| Corner-cut sweeps A/B/C + CSVs | [`outputs/analysis_pp_corner_cut/`](outputs/analysis_pp_corner_cut/) |
| Rate-limit comparison | [`outputs/analysis_steer_rate/rate_limit_compare.{md,csv}`](outputs/analysis_steer_rate/) |
| Split single plots per run | [`outputs/panels/`](outputs/panels/) |
| 18 report figures (`fig01`–`fig18`) | [`docs/figures/`](docs/figures/) |

### Problem 3 (member B — GPX route)

Everything member B produced is under [`outputs/problem3/`](outputs/problem3/):

| Artifact | Path |
| --- | --- |
| Structured results, 31 runs | [`outputs/problem3/all_results.json`](outputs/problem3/all_results.json) |
| Flat results table (~80 columns per run) | [`outputs/problem3/all_results.csv`](outputs/problem3/all_results.csv) |
| Circle steady-state statistics vs theory | [`outputs/problem3/circle_statistics.csv`](outputs/problem3/circle_statistics.csv) |
| GPX timing / progress / max-deviation locations | [`outputs/problem3/gpx_statistics.csv`](outputs/problem3/gpx_statistics.csv) |
| Built-in route comparison figure | [`outputs/problem3/task1_comparison.png`](outputs/problem3/task1_comparison.png) |
| Circle comparison figure | [`outputs/problem3/circle_comparison.png`](outputs/problem3/circle_comparison.png) |
| Per-run command log | [`outputs/problem3/commands.md`](outputs/problem3/commands.md) |
| Full analysis (Chinese) | [`outputs/problem3/report.md`](outputs/problem3/report.md) |
| Summary (Chinese) | [`outputs/problem3/README.md`](outputs/problem3/README.md) |
| Slide deck (English) | [`docs/PPT/task3_新版.pptx`](docs/PPT/task3_新版.pptx) |

> Per-run trajectories, animations and intermediate logs from member B's batch were deliberately
> dropped when that submission directory was slimmed down; the aggregated numbers are complete in
> the files above, and the per-run directories can be regenerated from `commands.md`. The
> per-run links inside `outputs/problem3/report.md` (e.g. `outputs/course_tasks/...peak_map.png`)
> therefore do not resolve in this merged repository — the two aggregate figures above do.

### Problem 4 (member A — dorm → classroom navigation)

| Artifact | Path |
| --- | --- |
| Full Chinese report | [`docs/问题4_报告.md`](docs/问题4_报告.md) |
| Short talk script (English) | [`docs/PPT/problem4_talk_script_short.md`](docs/PPT/problem4_talk_script_short.md) |
| Slide deck (English, with speaker notes) | [`docs/PPT/problem4_dorm_to_classroom_navigation.pptx`](docs/PPT/problem4_dorm_to_classroom_navigation.pptx) |
| 16-run experiment tables | [`outputs/dorm_experiments.md`](outputs/dorm_experiments.md), [`outputs/dorm_experiments.json`](outputs/dorm_experiments.json) |
| 18-run look-ahead sweep | [`outputs/dorm_lookahead_sweep.csv`](outputs/dorm_lookahead_sweep.csv) |
| Figures (full resolution set) | [`outputs/dorm_figures/`](outputs/dorm_figures/) |
| Report figures `q4_fig1`–`q4_fig12` + chase-cam GIF | [`docs/figures/q4_*`](docs/figures/) |
| Raw route and run logs | [`outputs/20260912_17*`](outputs/) |
| Route inputs | [`data/gpx/dorm_to_classroom.gpx`](data/gpx/dorm_to_classroom.gpx), [`data/gpx/dorm_to_classroom_raw.gpx`](data/gpx/dorm_to_classroom_raw.gpx) |
| Offline OSM slice | [`data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz`](data/) |

### Shared

| Artifact | Path |
| --- | --- |
| Deployment and run manual (the original repository-root README, bilingual) | [`docs/SETUP.md`](docs/SETUP.md) |
| Course task specification | [`vdm_lab/tasks/README.md`](vdm_lab/tasks/README.md) |
| GPX / GeoJSON task note | [`vdm_lab/tasks/gpx_geojson_task.md`](vdm_lab/tasks/gpx_geojson_task.md) |
| Platform extension notes | [`vdm_lab/GPX_EXTENSION_README.md`](vdm_lab/GPX_EXTENSION_README.md) |
| Course teaching guide | [`TEACHING_GUIDE.md`](TEACHING_GUIDE.md) |
| Lecture slides on the bicycle model | [`VehicleDynamicsMobility_01_BicycleModel.pdf`](VehicleDynamicsMobility_01_BicycleModel.pdf) |
