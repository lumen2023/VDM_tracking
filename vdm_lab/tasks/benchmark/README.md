
# Multi-Route Benchmark and PP Parameter Sensitivity

This section summarizes the controller benchmark completed on the current `VDM_tracking` framework. The study compares **Pure Pursuit (PP)**, **Kinematic LQR**, and **Linear MPC** on three representative routes, then evaluates the sensitivity of PP to the look-ahead distance.

## 1. Objectives

The experiments are designed to answer two questions:

1. How do PP, Kinematic LQR, and MPC behave on paths with different geometric characteristics?
2. How does the PP look-ahead distance affect tracking accuracy, steering activity, and response lag?

All comparisons use the same vehicle, speed mode, route, and simulation settings. Only the controller or the tested parameter is changed.

## 2. Experimental Setup

### Controllers

- Pure Pursuit (`pp`)
- Kinematic LQR (`lqr_kinematic`)
- Linear MPC (`mpc`)

### Vehicle and speed

- Vehicle: `student_car`
- Speed mode: `medium`
- Version: `student`

### Benchmark routes

| Route                  | Main feature                           | Main observation                                |
| ---------------------- | -------------------------------------- | ----------------------------------------------- |
| `double_lane_change` | rapid curvature-direction changes      | return speed, overshoot, steering oscillation   |
| `right_angle`        | sharp corner                           | peak error, corner cutting, steering saturation |
| `s_curve`            | continuous positive/negative curvature | phase lag, oscillation, steering smoothness     |

## 3. Multi-Route Benchmark

All 9 controller-route combinations reached the goal.

| Route              | Controller    | Mean lateral error / m | Max lateral error / m | J_delta |
| ------------------ | ------------- | ---------------------: | --------------------: | ------: |
| double_lane_change | PP            |                 0.3143 |                0.9447 |  0.1568 |
| double_lane_change | Kinematic LQR |                 0.1904 |                0.6584 |  4.3656 |
| double_lane_change | MPC           |                 0.1849 |                0.5036 |  0.5218 |
| right_angle        | PP            |                 0.1352 |                1.1064 |  0.0767 |
| right_angle        | Kinematic LQR |                 0.0989 |                0.7186 |  0.6268 |
| right_angle        | MPC           |                 0.0667 |                0.4881 |  0.1326 |
| s_curve            | PP            |                 0.3332 |                1.1710 |  0.2918 |
| s_curve            | Kinematic LQR |                 0.2164 |                0.5469 |  1.4576 |
| s_curve            | MPC           |                 0.1848 |                0.4891 |  0.4480 |

### Overall trend

In the current benchmark:

- **MPC produced the lowest mean and maximum lateral errors on all three routes.**
- **Kinematic LQR ranked second in tracking accuracy, but showed the largest steering-rate activity.**
- **PP had the largest tracking errors but the lowest `J_delta`, indicating the smoothest steering according to this metric.**

Therefore, the experiments show a clear trade-off between **tracking accuracy** and **steering smoothness** rather than one controller being best in every metric.

## 4. Route-Specific Observations

### 4.1 Double Lane Change

PP showed the largest overshoot, with lateral-error peaks of approximately `+0.94 m` and `-0.85 m`.

Kinematic LQR returned toward the reference rapidly, but its steering command exhibited strong high-frequency oscillation. Its `J_delta = 4.3656`, much larger than PP and MPC.

MPC achieved a similar fast recovery while maintaining substantially smaller tracking-error peaks and lower steering activity than Kinematic LQR.

**Main observation:** PP shows the largest overshoot; Kinematic LQR reacts aggressively but oscillates; MPC provides the smallest peak error with a more stable response.

### 4.2 Right Angle

The maximum tracking errors occurred around the sharp corner for all three controllers.

| Controller    | Max lateral error / m | Steering behavior                            |
| ------------- | --------------------: | -------------------------------------------- |
| PP            |                1.1064 | no steering saturation; clear corner cutting |
| Kinematic LQR |                0.7186 | reaches the 35 deg steering limit            |
| MPC           |                0.4881 | reaches the 35 deg steering limit            |

The trajectory comparison shows that PP cuts furthest inside the corner. Kinematic LQR and MPC use the available steering authority more aggressively, while MPC achieves the smallest peak error.

**Main observation:** Sharp curvature creates the largest error near the corner. In the detailed results, LQR and MPC reach the steering limit, while PP avoids saturation but sacrifices path-following accuracy through corner cutting.

### 4.3 S-Curve

The S-curve highlights the differences in phase lag and steering oscillation.

- PP shows the clearest phase lag and the largest error peaks, approximately `±1.17 m`.
- Kinematic LQR shows sustained high-frequency steering oscillation.
- MPC maintains smaller error peaks, approximately `±0.49 m`.

The steering-rate metric is:

```text
PP   : 0.2918
LQR  : 1.4576
MPC  : 0.4480
```

**Main observation:** PP is relatively smooth but lags behind the changing curvature, while Kinematic LQR reacts aggressively and oscillates. MPC gives the lowest tracking error among the three in this experiment.

## 5. PP Look-Ahead Sensitivity

The PP base look-ahead distance was varied on the `s_curve` route while keeping all other conditions unchanged.

### Fixed setup

```text
route       = s_curve
speed_mode  = medium
vehicle     = student_car
algorithm   = pp
```

### Tested values

```text
pp_base_lookahead = 1.5 m
pp_base_lookahead = 3.0 m
pp_base_lookahead = 5.0 m
```

### Results

| Look-ahead / m | Mean lateral error / m | Max lateral error / m | Max steer / rad | J_delta |
| -------------: | ---------------------: | --------------------: | --------------: | ------: |
|            1.5 |                 0.2211 |                0.7671 |          0.4279 |  0.2324 |
|            3.0 |                 0.3332 |                1.1710 |          0.5871 |  0.2918 |
|            5.0 |                 0.5461 |                1.8542 |          0.3382 |  0.1688 |

### Interpretation

**1.5 m look-ahead**

- Lowest mean and maximum lateral errors.
- Faster response to curvature changes.
- No significant high-frequency oscillation was observed in this experiment.
- Steering remained below saturation.

**3.0 m look-ahead**

- Intermediate tracking accuracy.
- Largest maximum steering demand.
- Largest `J_delta` among the three tested values.

**5.0 m look-ahead**

- Lowest steering-rate activity.
- Largest lateral tracking errors.
- Clear corner cutting and phase lag.

### Overall trend

For the tested S-curve case:

```text
larger look-ahead
    -> smoother steering
    -> larger phase lag
    -> larger tracking error

smaller look-ahead
    -> faster response
    -> smaller tracking error
    -> more active steering
```

The `1.5 m` value performed best in tracking accuracy in this specific experiment, but this should not be treated as a universal optimum for all routes and speeds.

## 6. Reproducing the Benchmark

Example commands:

```powershell
python run_experiment.py --algo pp --version student --route double_lane_change --speed-mode medium --vehicle student_car --save-log --save-fig
python run_experiment.py --algo lqr_kinematic --version student --route double_lane_change --speed-mode medium --vehicle student_car --save-log --save-fig
python run_experiment.py --algo mpc --version student --route double_lane_change --speed-mode medium --vehicle student_car --save-log --save-fig
```

Repeat the same three controllers for:

```text
right_angle
s_curve
```

The benchmark summary can then be generated with the analysis scripts in:

```text
analysis/
```

Recommended retained outputs:

```text
algorithm_comparison.csv
parameter_comparison.csv
analysis/figs/
```

Raw timestamped runs under `outputs/` are reproducible experiment artifacts and do not need to be committed to Git.

## 7. Key Figures

Recommended figures for the repository:

```text
analysis/figs/error_double_lane_change.png
analysis/figs/error_right_angle.png
analysis/figs/error_s_curve.png

analysis/figs/steer_double_lane_change.png
analysis/figs/steer_right_angle.png
analysis/figs/steer_s_curve.png

analysis/figs/traj_double_lane_change.png
analysis/figs/traj_right_angle.png
analysis/figs/traj_s_curve.png

analysis/figs/lookahead_sensitivity.png
analysis/figs/lookahead_error_curves.png
```

For the root project README, only one or two representative figures are recommended. The full set can remain in this benchmark section.

## 8. Main Conclusions

The current medium-speed experiments show that path geometry strongly affects controller behavior.

- MPC consistently achieved the smallest lateral tracking errors on the three tested routes.
- Kinematic LQR improved tracking accuracy relative to PP, but showed strong steering oscillation in the more demanding transient cases.
- PP produced smoother steering according to `J_delta`, but showed larger phase lag, overshoot, and corner cutting.
- Increasing PP look-ahead distance reduced steering activity but increased tracking error and response lag.
- Controller evaluation should therefore consider both **tracking accuracy** and **control smoothness**, rather than relying on a single metric.

These conclusions apply to the current `student_car`, `medium` speed mode, tested routes, and controller parameters.
