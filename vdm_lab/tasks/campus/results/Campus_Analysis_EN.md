# Campus route tracking — Wang Yihan

This section uses the group student controllers and unchanged kinematic vehicle backend. The existing campus route has locally widened turns and is used directly as a smooth planning standard; no original corner polyline is used as a reference.

## Settings

Route length: 879.539 m; 4399 samples, 0.2 m spacing (short final interval). Map origin: (118.8145, 31.8885), WGS84 local spherical projection. The road sequence and endpoints are retained; local turn widening reduces the previous 885.417 m length to the value above.

Vehicle: student_car, wheelbase 2.5 m, lf=lr=1.25 m, maximum steering 35 deg; dt=0.1 s; initial speed 0; time limit 900 s. All three runs share the exact same geometry, speed profile, vehicle and controller defaults.

The cruise cap is 3 m/s. The common profile limits reference v²|k| to 0.7 m/s², uses forward/backward passes at 0.7/0.8 m/s², starts at a 0.5 m/s target and ends at zero. Actual tracking speed can differ from this target. MPC retains its original horizon and constraints.

![Campus route and shared speed profile](campus_route.png)

## Baseline results

| Algorithm | Reached | Arrival time (s) | Mean absolute lateral error (m) | Max absolute lateral error (m) | Finish distance (m) |
|---|---|---:|---:|---:|---:|
| PP | True | 299.7 | 0.03071 | 0.77417 | 0.52964 |
| Kinematic LQR | True | 300.2 | 0.02496 | 0.55586 | 0.63035 |
| MPC | True | 299.5 | 0.01462 | 0.32623 | 0.00685 |

![Three-algorithm tracking comparison](campus_tracking.png)

Arrival means endpoint distance <1.5 m and speed <0.5 m/s, as in the group framework; finish distance is not lateral error. Failed runs would have a blank arrival time and a separate simulation duration. These are simulated travel times, not measured campus commute times.

## Maximum deviation

| Algorithm | Time (s) | Reference station (m) | Signed error (m) | Actual speed (m/s) | Reference curvature (1/m) | Steer (deg) | Turn / phase |
|---|---:|---:|---:|---:|---:|---:|---|
| PP | 93.9 | 270.80 | -0.77417 | 2.176 | -0.1828 | -21.38 | 1 / within |
| Kinematic LQR | 94.8 | 272.60 | -0.55586 | 2.100 | -0.1409 | -20.47 | 1 / within |
| MPC | 93.2 | 270.20 | -0.32623 | 2.142 | -0.1880 | -21.63 | 1 / within |

![Maximum deviation locations](campus_max_error.png)

Peaks are selected from the same log row by argmax(abs(lateral_error)). Station/curvature refer to the controller-selected reference point. campus_max_error.csv records both vehicle coordinates and reference-point coordinates, explicitly distinguished. Turn intervals use connected |curvature|>0.015 1/m regions with integrated heading change >30 degrees.

Peak diagnostics: PP peaks within Turn 1, +0.8 m relative to its curvature maximum, steering -21.4 degrees; Kinematic LQR peaks within Turn 1, +2.6 m relative to its curvature maximum, steering -20.5 degrees; MPC peaks within Turn 1, +0.2 m relative to its curvature maximum, steering -21.6 degrees. The signed station offset is a spatial comparison, not a measured controller delay. Actual speed need not equal the local reference speed.

## Feasibility and interpretation

The revised maximum reference curvature is 0.1889 1/m. The CG bicycle steering limit is tan(delta_max)/sqrt(L² + lr² tan²(delta_max)) = 0.2644 1/m. No reference segment exceeds this limit. Dense checking gives a minimum radius of 5.29 m and maximum geometric steer of 25.92 degrees, below 35 degrees. Local quintic joins preserve position, tangent and second derivative at their boundaries. Three principal turns and a pair of short high-curvature transitions were widened; the start and destination did not move. The maximum same-parameter shift from the old smooth route is 2.26 m. This establishes steering-geometric feasibility, not collision-free road access.

Nonzero tracking errors remain possible despite a geometrically feasible reference, because preview behavior, finite sampling and transient speed/steering response still matter. Different peak positions do not by themselves establish a causal control delay. The full-route mean is strongly diluted by long straight sections; inspect peak and local trajectories as well. normal_accel in the group log is v² times reference curvature, not independently measured actual normal acceleration.

![Local route adjustment](campus_route_adjustment.png)

The route change does not improve every tracking metric: PP maximum error changes from 1.376 to 0.774 m, MPC from 0.405 to 0.326 m, while LQR changes from 0.535 to 0.556 m. LQR and MPC full-route mean errors also increase relative to the previous route. This is a route-feasibility correction, not controller tuning. Curvature-based speed targets change with the route, so this before/after comparison is not an isolated curvature-only experiment. See campus_route_change.csv for the original and revised metrics.

## One-variable improvement trial

Only the PP cruise cap changes from 3 to 2 m/s; the same curvature/acceleration planning rule, geometry, controller and vehicle remain. Mean error changes from 0.03071 to 0.02483 m (19.2% lower); maximum error from 0.77417 to 0.73853 m (4.6% lower). Arrival time increases from 299.7 to 441.8 s. The revised route already satisfies the geometric steering limit; this trial evaluates remaining tracking transients. In the tightest curve, curvature-based speed limiting already dominates, so reducing cruise speed need not proportionally reduce the peak.

![PP improvement trial](campus_improvement.png)

## Reproduction and limitations

See [readme_wyh.md](../readme_wyh.md) for exact commands and merge instructions. Per-run config.json records every default, source hashes and output path. campus_results/validation.json checks shared settings, geometry, coordinate conversion and independently recomputed lateral-error metrics.

The route is planned, not a measured GNSS track. No pedestrians, obstacles, road rights, signals, collision checking, tire-force saturation or real campus speed restrictions are simulated. Map lines are background only. Generic labels remove building names but do not anonymize the GPX coordinates. Map data: © OpenStreetMap contributors, ODbL; https://www.openstreetmap.org/copyright.
