# Bicycle Models and Path-Tracking Control: Mathematical Derivations and Code Mapping

This document provides the theoretical preparation for the assignment. It distinguishes three objects that should never be conflated:

1. the continuous-time vehicle models introduced in the course;
2. the approximate models used internally by each controller; and
3. the quantities actually written to `trajectory.csv`.

That distinction matters. The default simulated plant includes the geometric sideslip angle `beta`, whereas the PP and MPC prediction models omit it. Likewise, the logged `normal_accel` is computed from the **reference-path curvature**; it is not a lateral-acceleration measurement.

Two controller implementations coexist in this repository:

- **Instructor solutions** in `vdm_lab/solutions/` preserve the compact reference implementation.
- **Hardened student controllers** in `vdm_lab/student/` implement the assignment formulas and add finite-value checks, configuration validation, steering-angle and steering-rate limiting, forward-only acceleration bounds, and fail-stop behavior.

Unless a paragraph explicitly says otherwise, the mathematical plant equations refer to the shared simulator, and the controller equations refer to the common nominal algorithm. Version-specific behavior is identified where it changes the implemented command.

> **Physical-vehicle boundary:** These models and controllers are teaching software, not a validated drive-by-wire stack. The repository still has no concrete real-vehicle backend, deterministic control deadline, actuator calibration, localization-age check, communication watchdog, independent emergency stop, obstacle handling, or safety supervisor. Do not connect its scalar commands directly to physical actuators. Resolve the deployment issues in Section 9 and validate progressively with actuators disabled, then at very low speed on a closed test site under an approved safety process.

## 1. Symbols, units, and sign conventions

### 1.1 Vehicle and motion variables

| Symbol | Meaning | SI unit | Main code variable |
| --- | --- | --- | --- |
| \(x,y\) | Global position of the vehicle reference point | m | `VehicleState.x`, `VehicleState.y` |
| \(\psi\) | Body-heading angle measured from global \(+x\) | rad | `VehicleState.yaw` |
| \(v\) | Speed magnitude at the vehicle reference point | m/s | `VehicleState.v` |
| \(\delta_f\) | Mean front road-wheel angle; \(\delta_r=0\) | rad | `ControlCommand.steer` |
| \(\beta\) | Geometric velocity angle relative to the body axis | rad | `beta` |
| \(r=\dot\psi\) | Body yaw rate | rad/s | `yaw_rate` |
| \(l_f,l_r\) | Distances from point C to the front and rear axles | m | `vehicle.lf`, `vehicle.lr` |
| \(L=l_f+l_r\) | Wheelbase | m | `vehicle.wheelbase` |
| \(\kappa\) | Signed curvature | 1/m | `Path.curvature` |
| \(\rho=1/\kappa\) | Signed curvature radius; magnitude \(1/|\kappa|\) | m | Derived from `curvature` |
| \(a_n\) | Signed normal acceleration | m/s² | `normal_accel` |
| \(m\) | Vehicle mass | kg | `vehicle.mass` |
| \(I_z\) | Yaw moment of inertia | kg·m² | `vehicle.inertia_z` |
| \(C_f,C_r\) | Front and rear cornering stiffness | N/rad | `vehicle.cf`, `vehicle.cr` |

The repository uses a right-handed planar frame. For built-in routes, \(+x\) and \(+y\) are plot coordinates. For GPX routes they are local East and North. Positive angles are counterclockwise. Therefore:

- positive front-wheel angle means a left turn;
- positive yaw rate means counterclockwise rotation;
- positive curvature means a left-hand curve; and
- positive lateral error means that the vehicle lies to the left of the reference path in the direction of travel.

Angular errors are wrapped to \([-\pi,\pi]\) by `pi_to_pi` or the equivalent \(\operatorname{atan2}(\sin\theta,\cos\theta)\). Every command and logged angle uses radians.

### 1.2 Default parameters

The `student_car` configuration in `vdm_lab/config/vehicle_params.py` is

\[
L=2.50\ \mathrm{m},\quad l_f=l_r=1.25\ \mathrm{m},\quad
m=1140\ \mathrm{kg},\quad I_z=1436.24\ \mathrm{kg\,m^2},
\]

\[
C_f=C_r=155494.663\ \mathrm{N/rad},\quad
|\delta_f|\leq35^\circ=0.610865\ \mathrm{rad},
\]

\[
|\dot\delta_f|\leq45^\circ/\mathrm{s}=0.785398\ \mathrm{rad/s},\quad
a\in[-3.5,2.0]\ \mathrm{m/s^2},\quad v\in[0,12]\ \mathrm{m/s}.
\]

The default sample period is \(\Delta t=0.1\ \mathrm{s}\). When the steering-rate limit is enforced, the largest allowed steering change in one sample is

\[
\Delta\delta_{\max}=\dot\delta_{\max}\Delta t
=0.078540\ \mathrm{rad}=4.5^\circ.
\]

The configuration stores both `wheelbase` and `lf + lr`. Both supplied vehicle sets satisfy `wheelbase == lf + lr`, but shared code does not enforce the equality. Keep them consistent when changing vehicle parameters: most controllers use `wheelbase`, while `bicycle_model.py` uses `lf + lr`.

Every hardened student controller validates the following configuration values before computing a command:

\[
\Delta t>0,\qquad
0\leq v_{\min}<v_{\max},\qquad
a_{\max}>0,\qquad a_{\mathrm{decel,max}}>0,
\]

\[
0<\delta_{\max}<\frac{\pi}{2},\qquad
\dot\delta_{\max}>0,\qquad L>0,
\]

and requires each value to be numeric and finite. Invalid safety configuration raises `ValueError` rather than being silently converted into a command.

## 2. Kinematic bicycle model

![Points A, B, and C and the instantaneous center O in the kinematic bicycle model](../KMLM.png)

### 2.1 Assumptions

The kinematic bicycle model combines the left and right wheels on each axle into one equivalent wheel. It assumes:

- planar motion with no roll or pitch;
- pure rolling, so velocity normal to each wheel is zero;
- no rear steering, \(\delta_r=0\);
- a rigid vehicle with constant \(L=l_f+l_r\); and
- no tire-force or friction-limit calculation.

In `KMLM.png`, A, B, and C denote the front-axle center, rear-axle center, and an intermediate reference point, respectively. O is the instantaneous center of rotation. The equations in `front_steer_slip_angle()` treat C as the center-of-gravity/reference point.

### 2.2 Derivation of \(\beta\) from the no-slip constraints

In body coordinates, the velocity at C is

\[
\mathbf v_C^b=
\begin{bmatrix}v\cos\beta\\v\sin\beta\end{bmatrix}
=\begin{bmatrix}u\\v_y\end{bmatrix},
\qquad r=\dot\psi.
\]

Rigid-body planar kinematics gives the rear- and front-axle velocities

\[
\mathbf v_B^b=
\begin{bmatrix}u\\v_y-l_r r\end{bmatrix},\qquad
\mathbf v_A^b=
\begin{bmatrix}u\\v_y+l_f r\end{bmatrix}.
\]

The rear wheel points along the body axis. Pure rolling requires zero lateral velocity at B:

\[
v_y-l_r r=0
\quad\Longrightarrow\quad
r=\frac{v\sin\beta}{l_r}.
\tag{1}
\]

The front wheel is rotated by \(\delta_f\). Its no-slip condition requires the velocity at A to be parallel to the front wheel:

\[
-u\sin\delta_f+(v_y+l_f r)\cos\delta_f=0.
\]

Hence

\[
\tan\delta_f
=\frac{v_y+l_f r}{u}
=\frac{v\sin\beta(1+l_f/l_r)}{v\cos\beta}
=\frac{L}{l_r}\tan\beta.
\]

Therefore,

\[
\boxed{\beta=\arctan\!\left(\frac{l_r}{L}\tan\delta_f\right)}.
\tag{2}
\]

The implementation uses `atan2(lr * tan(steer), L)`. Because \(L>0\) and steering is limited to less than \(\pi/2\), it agrees with Equation (2) while retaining a well-defined quadrant convention.

The same result follows from the instantaneous-center geometry. The signed rear-axle radius is

\[
R_r=\frac{L}{\tan\delta_f}.
\]

The magnitude of the front-axle radius is

\[
|R_f|=\sqrt{R_r^2+L^2}
=\frac{L}{|\sin\delta_f|},
\]

and the radius of point C is

\[
R_C=\sqrt{R_r^2+l_r^2}.
\]

Because velocity is perpendicular to the radius,

\[
\tan\beta=\frac{l_r}{R_r}
=\frac{l_r}{L}\tan\delta_f.
\]

This \(\beta\) is a **rigid-body geometric velocity angle**. It is different from the front and rear tire slip angles \(\alpha_f,\alpha_r\) used in the dynamic bicycle model.

### 2.3 Yaw rate and global position derivatives

Substituting Equation (2) into Equation (1) gives

\[
\frac{v\sin\beta}{l_r}
=\frac{v}{L}\tan\delta_f\cos\beta.
\]

Thus,

\[
\boxed{\dot\psi=r=\frac{v}{L}\tan\delta_f\cos\beta}.
\tag{3}
\]

The velocity at C points in the global direction \(\psi+\beta\), so

\[
\boxed{\dot x=v\cos(\psi+\beta)},\qquad
\boxed{\dot y=v\sin(\psi+\beta)}.
\tag{4}
\]

These equations map directly to `front_steer_slip_angle()`, `yaw_rate_from_steer()`, and `kinematic_derivatives()` in `vdm_lab/common/bicycle_model.py`.

The default backend advances the state with forward Euler integration:

\[
x_{k+1}=x_k+\dot x_k\Delta t,\quad
y_{k+1}=y_k+\dot y_k\Delta t,
\]

\[
\psi_{k+1}=\operatorname{wrap}(\psi_k+r_k\Delta t),\quad
v_{k+1}=\operatorname{clip}(v_k+a_k\Delta t,v_{\min},v_{\max}).
\tag{5}
\]

Before integration, `limit_command()` clips acceleration and steering angle to the configured magnitude limits. It does not enforce steering rate; the hardened student controllers apply that limit before the command reaches the backend.

### 2.4 Exact steering-curvature relationship

At steady steering, \(\beta\) is constant and the curvature of the C-point trajectory is yaw rate divided by speed:

\[
\boxed{\kappa_C=\frac{r}{v}
=\frac{\tan\delta_f\cos\beta}{L}}.
\tag{6}
\]

Substitute Equation (2) and define \(q=\tan\delta_f\):

\[
\kappa_C=\frac{q}{\sqrt{L^2+l_r^2q^2}}.
\tag{7}
\]

To make point C follow a specified curvature \(\kappa\), with \(|l_r\kappa|<1\), inversion gives

\[
\boxed{\delta_f=
\arctan\!\left(
\frac{L\kappa}{\sqrt{1-l_r^2\kappa^2}}
\right)}.
\tag{8}
\]

The course approximation assumes small geometric sideslip:

\[
\cos\beta\approx1
\quad\Longrightarrow\quad
\boxed{\delta_f\approx\arctan(L\kappa)}.
\tag{9}
\]

With the additional small-angle approximation \(\tan\delta_f\approx\delta_f\),

\[
\boxed{\delta_f\approx L\kappa}.
\tag{10}
\]

Equation (9) is the assignment formula for circle comparisons. Equation (10) is the raw curvature feedforward used by the instructor kinematic-LQR solution. The hardened student kinematic LQR uses Equation (9). These expressions are close at small curvature but are not identical.

## 3. Circular motion, curvature, and normal acceleration

![Tangential and normal components of circular motion](../exp_cm.png)

### 3.1 Fundamental relationships

For a circle of signed radius \(\rho\), arc length is \(s=\rho\theta\). Therefore,

\[
v=\dot s=\rho\dot\theta=\rho\omega,
\qquad
\boxed{\kappa=\frac{1}{\rho}},
\qquad
\boxed{\omega=v\kappa}.
\tag{11}
\]

Let \(\mathbf e_t\) be the unit tangent and \(\mathbf e_n\) the left unit normal. For the counterclockwise circle in `exp_cm.png`, the left normal points toward the center. The acceleration of a general planar curve is

\[
\mathbf a=\dot v\,\mathbf e_t+v^2\kappa\,\mathbf e_n.
\tag{12}
\]

Its signed normal component is

\[
\boxed{a_n=\frac{v^2}{\rho}=v^2\kappa}.
\tag{13}
\]

For a right-hand curve, \(\kappa<0\); \(\mathbf e_n\) remains the left normal and the signed value in Equation (13) is negative, so the physical acceleration points right. `metrics.json` takes the absolute value when reporting its maximum.

For any parameter \(q\), the signed curvature of a planar path \((x(q),y(q))\) is

\[
\boxed{\kappa=
\frac{x'(q)y''(q)-y'(q)x''(q)}
{\left(x'(q)^2+y'(q)^2\right)^{3/2}}}.
\]

`path.py` applies this expression to the spline derivatives and floors the denominator at `1e-8`.

At steady state, \(\dot\beta=0\), so the velocity direction and body rotate at the same angular speed:

\[
r=\dot\psi=v\kappa.
\tag{14}
\]

When steering changes, the velocity-direction angle is \(\psi+\beta\), with angular rate \(r+\dot\beta\). The instantaneous trajectory-normal acceleration is then not completely described by \(vr\) alone.

### 3.2 Theoretical values for the 12 m circle

The circular segment of the built-in `circle` route has

\[
R=12\ \mathrm{m},\qquad
\kappa=\frac1{12}=0.0833333\ \mathrm{m^{-1}}.
\]

For `student_car`, the relevant steering values are:

| Meaning | Formula | Value |
| --- | --- | ---: |
| Course small-sideslip steering estimate | \(\arctan(L/R)\) | \(0.205395\ \mathrm{rad}=11.7683^\circ\) |
| Steering that gives point C an exact 12 m radius in the shared plant | Equation (8) | \(0.206487\ \mathrm{rad}=11.8309^\circ\) |
| Raw instructor kinematic-LQR feedforward | \(L/R\) | \(0.208333\ \mathrm{rad}=11.9366^\circ\) |

For the exact C-point geometry,

\[
\beta=\arcsin(l_r/R)
=0.104356\ \mathrm{rad}=5.9793^\circ.
\]

Use \(\arctan(L/R)\) as the assignment's theoretical steering value. The other two values explain model and implementation differences. The final controller command also includes feedback and rate limiting, so it need not equal any raw feedforward value.

The nominal circle-speed results are:

| Speed mode | \(v\) / m/s | \(r=v/R\) / rad/s | \(a_n=v^2/R\) / m/s² | \(a_n/g\) |
| --- | ---: | ---: | ---: | ---: |
| low | 3.0 | 0.250000 | 0.750000 | 0.0765 |
| medium | 5.0 | 0.416667 | 2.083333 | 0.2124 |
| high | 7.0 | 0.583333 | 4.083333 | 0.4164 |

Here \(g=9.80665\ \mathrm{m/s^2}\). Raising speed from 3 m/s to 7 m/s multiplies required yaw rate by \(7/3\approx2.33\), but multiplies required normal acceleration by \((7/3)^2\approx5.44\).

### 3.3 Logged value versus physical vehicle value

`simulation.py` records

\[
\texttt{normal\_accel}=v^2\kappa_{\mathrm{ref}},
\tag{15}
\]

where \(\kappa_{\mathrm{ref}}\) is the curvature of the current nearest reference point. This value is not computed from the executed steering, a second derivative of the actual trajectory, tire forces, or an IMU.

For constant steering, the geometric normal acceleration of the default kinematic plant would instead be

\[
a_{n,\mathrm{vehicle}}
=v^2\frac{\tan\delta_f\cos\beta}{L}.
\tag{16}
\]

Using `normal_accel` to verify Equation (13) primarily verifies substitution of actual speed and reference curvature. It does not by itself prove that the vehicle followed the same curvature. The report must also examine `yaw_rate`, `steer`, and lateral error.

## 4. Path coordinates, tracking errors, and kinematic LQR

### 4.1 Error definitions in the code

Let the nearest reference point be

\[
\mathbf p_r=[x_r,y_r]^T.
\]

For reference heading \(\psi_r\), define the tangent and left-normal unit vectors

\[
\mathbf t_r=
\begin{bmatrix}\cos\psi_r\\\sin\psi_r\end{bmatrix},\qquad
\mathbf n_r=
\begin{bmatrix}-\sin\psi_r\\\cos\psi_r\end{bmatrix}.
\]

`ReferenceTracker` defines

\[
\boxed{e_y=(\mathbf p-\mathbf p_r)^T\mathbf n_r},
\qquad
\boxed{e_\psi=\operatorname{wrap}(\psi-\psi_r)}.
\tag{17}
\]

Thus \(e_y>0\) places the vehicle left of the path, and \(e_\psi>0\) means the body heading is more counterclockwise than the reference heading.

In exact Frenet coordinates for the C-point model,

\[
\dot e_y=v\sin(e_\psi+\beta),
\tag{18}
\]

\[
\dot s=\frac{v\cos(e_\psi+\beta)}
{1-\kappa_r e_y},
\tag{19}
\]

\[
\dot e_\psi=r-\kappa_r\dot s.
\tag{20}
\]

Both LQR implementations use the simpler estimates below. They set the model speed to

\[
\bar v=\max(v,0.5\ \mathrm{m/s})
\]

under the default configuration and compute

\[
\boxed{\dot e_y^{\mathrm{code}}
=\bar v\sin e_\psi},
\tag{21}
\]

\[
\boxed{\dot e_\psi^{\mathrm{code}}
=\frac{\bar v}{L}\tan\delta_{f,k-1}
-\bar v\kappa_r}.
\tag{22}
\]

Equation (21) omits \(\beta\). Equation (22) also omits \(\cos\beta\), the denominator in Equation (19), and the actual along-path speed. `previous_control.steer` is the steering returned by the backend on the preceding simulation step.

### 4.2 Discrete kinematic-LQR error model

The controller state is

\[
\boldsymbol\xi=
\begin{bmatrix}
e_y&\dot e_y&e_\psi&\dot e_\psi
\end{bmatrix}^T.
\tag{23}
\]

`build_kinematic_model()` constructs

\[
A_k=
\begin{bmatrix}
1&\Delta t&0&0\\
0&0&\bar v&0\\
0&0&1&\Delta t\\
0&0&0&0
\end{bmatrix},\qquad
B_k=
\begin{bmatrix}
0\\0\\0\\\bar v/L
\end{bmatrix}.
\tag{24}
\]

Therefore,

\[
\boldsymbol\xi_{k+1}
=A_k\boldsymbol\xi_k+B_k\delta_{\mathrm{fb},k}.
\tag{25}
\]

Row by row,

\[
e_{y,k+1}=e_{y,k}+\Delta t\dot e_{y,k},\quad
\dot e_{y,k+1}=\bar v e_{\psi,k},
\]

\[
e_{\psi,k+1}=e_{\psi,k}
+\Delta t\dot e_{\psi,k},\quad
\dot e_{\psi,k+1}
=\frac{\bar v}{L}\delta_{\mathrm{fb},k}.
\]

This is a common teaching-oriented mixed discrete error model, not a complete Euler discretization of four continuous differential equations. Match the code exactly: `A[1,2] = speed` and `B[3,0] = speed / wheelbase` do not receive another factor of \(\Delta t\).

### 4.3 Riccati iteration and feedback

The infinite-horizon discrete LQR minimizes

\[
J=\sum_{k=0}^{\infty}
\left(
\boldsymbol\xi_k^TQ\boldsymbol\xi_k
+\delta_{\mathrm{fb},k}^TR\delta_{\mathrm{fb},k}
\right).
\tag{26}
\]

Starting from \(P_0=Q\), the code iterates

\[
P_{i+1}=A^TP_iA
-A^TP_iB(R+B^TP_iB)^+B^TP_iA+Q,
\tag{27}
\]

until the maximum elementwise change falls below `lqr_eps = 1e-4` or 200 iterations are reached. The superscript \(+\) denotes a Moore-Penrose pseudoinverse. The gain is

\[
K=(R+B^TPB)^+B^TPA.
\tag{28}
\]

The instructor solution evaluates these expressions with `pinv`. The hardened student solver uses a direct solve when the matrix condition number is below \(10^{12}\), then falls back to an SVD pseudoinverse. It symmetrizes \(P\), rejects non-finite matrices, and checks that

\[
\rho(A-BK)<1
\]

before accepting a lateral-control command.

The default weights are

\[
Q=\operatorname{diag}(2.0,0.2,3.0,0.2),
\qquad R=[1.5].
\]

Because \(A\) and \(B\) depend on speed, the controller recomputes \(K\) at every control step.

### 4.4 Feedforward and applied steering

The instructor solution computes

\[
\delta_{f,\mathrm{desired}}^{\mathrm{solution}}
=-K\boldsymbol\xi+L\kappa_r.
\]

The hardened student controller uses the course steering-curvature relationship:

\[
\delta_{f,\mathrm{desired}}^{\mathrm{student}}
=-K\boldsymbol\xi+\arctan(L\kappa_r).
\tag{29}
\]

The instructor solution clips only steering magnitude. The hardened student controller additionally applies

\[
\delta_{f,k}
=\operatorname{clip}\!\left(
\operatorname{clip}(
\delta_{f,\mathrm{desired}},-\delta_{\max},\delta_{\max}),
\delta_{f,k-1}-\dot\delta_{\max}\Delta t,
\delta_{f,k-1}+\dot\delta_{\max}\Delta t
\right).
\]

A non-finite state, reference, gain, or unstable closed-loop matrix causes the hardened controller to request steering toward zero at the allowed rate and to set the longitudinal target to zero. Invalid safety configuration is treated separately as a fatal `ValueError`.

## 5. Linear dynamic bicycle model and dynamic LQR

### 5.1 Linear tire model

Let \(v_x\approx v>0\) be approximately constant longitudinal speed, \(v_y\) the lateral velocity at the center of gravity, and \(r=\dot\psi\). Under the small-angle linear-tire assumption,

\[
\alpha_f\approx
\delta_f-\frac{v_y+l_f r}{v},\qquad
\alpha_r\approx-\frac{v_y-l_r r}{v},
\tag{30}
\]

\[
F_{yf}=C_f\alpha_f,\qquad
F_{yr}=C_r\alpha_r.
\tag{31}
\]

The lateral and yaw equations are

\[
m(\dot v_y+vr)=F_{yf}+F_{yr},
\tag{32}
\]

\[
I_z\dot r=l_fF_{yf}-l_rF_{yr}.
\tag{33}
\]

For small path-tracking errors,

\[
\dot e_y\approx v_y+ve_\psi,\qquad
\dot e_\psi=r-v\kappa_r.
\tag{34}
\]

For locally constant \(v\) and \(\kappa_r\),

\[
v_y=\dot e_y-ve_\psi,\qquad
r=\dot e_\psi+v\kappa_r.
\tag{35}
\]

Substituting Equation (35) into Equations (32)-(33), assigning curvature-dependent affine terms to feedforward, and retaining the homogeneous error terms gives the controller model.

### 5.2 Continuous state-space matrices

Dynamic LQR uses

\[
\dot{\boldsymbol\xi}
=A_c\boldsymbol\xi+B_c\delta_f,
\tag{36}
\]

with

\[
A_c=
\begin{bmatrix}
0&1&0&0\\
0&-\dfrac{C_f+C_r}{mv}
&\dfrac{C_f+C_r}{m}
&\dfrac{l_rC_r-l_fC_f}{mv}\\
0&0&0&1\\
0&\dfrac{l_rC_r-l_fC_f}{I_zv}
&\dfrac{l_fC_f-l_rC_r}{I_z}
&-\dfrac{l_f^2C_f+l_r^2C_r}{I_zv}
\end{bmatrix},
\tag{37}
\]

\[
B_c=
\begin{bmatrix}
0\\C_f/m\\0\\l_fC_f/I_z
\end{bmatrix}.
\tag{38}
\]

For example, the second row is

\[
\ddot e_y=
-\frac{C_f+C_r}{mv}\dot e_y
+\frac{C_f+C_r}{m}e_\psi
+\frac{l_rC_r-l_fC_f}{mv}\dot e_\psi
+\frac{C_f}{m}\delta_f,
\tag{39}
\]

after omitting its curvature-dependent affine term.

Because the matrices contain \(1/v\), both versions use the model-speed floor

\[
v\leftarrow\max(v,0.5\ \mathrm{m/s})
\tag{40}
\]

with the default controller configuration. This prevents numerical singularity; it does not make the dynamic model physically accurate near standstill. The hardened implementation additionally requires positive finite \(m,I_z,l_f,l_r,C_f,C_r\).

### 5.3 Version-specific discretization

Both versions use the trapezoidal/bilinear state transition

\[
\boxed{
A_d=(I-\tfrac{\Delta t}{2}A_c)^{-1}
\left(I+\tfrac{\Delta t}{2}A_c\right)}.
\tag{41}
\]

The implementation uses a pseudoinverse in the instructor solution and a condition-aware linear solve with pseudoinverse fallback in the hardened student controller.

Their input discretizations differ:

\[
\boxed{
B_d^{\mathrm{solution}}=B_c\Delta t,
\qquad
B_d^{\mathrm{student}}
=(I-\tfrac{\Delta t}{2}A_c)^{-1}B_c\Delta t}.
\tag{42}
\]

The student pair is the consistent trapezoidal discretization. The instructor solution combines a trapezoidal \(A_d\) with an Euler-style \(B_d\); reproduce that distinction when comparing the two versions.

Both controllers construct their actual error state using Equations (21)-(22). They do not directly measure \(v_y\) or the dynamic model's yaw-rate state.

### 5.4 Dynamic curvature feedforward

The code defines

\[
K_v^{\mathrm{code}}
=\frac{l_rm}{2C_fL}
-\frac{l_fm}{2C_rL},
\tag{43}
\]

and a quantity called `steady_yaw_error` or `yaw_steady_error`,

\[
e_{\psi,ss}^{\mathrm{code}}
=l_r\kappa_r
-\frac{l_fmv^2\kappa_r}{2C_rL}.
\tag{44}
\]

Writing the third-state gain as \(K_\psi=K[0,2]\),

\[
\boxed{
\delta_{\mathrm{ff,dyn}}
=L\kappa_r
+K_v^{\mathrm{code}}v^2\kappa_r
-K_\psi e_{\psi,ss}^{\mathrm{code}}}.
\tag{45}
\]

The desired command is

\[
\boxed{
\delta_{f,\mathrm{desired}}
=-K\boldsymbol\xi+\delta_{\mathrm{ff,dyn}}}.
\tag{46}
\]

The default vehicle is front-rear symmetric, so \(K_v^{\mathrm{code}}=0\). Equation (44) and its gain compensation remain speed dependent.

The instructor solution applies only an angle clip. The hardened student dynamic LQR validates its model, checks \(\rho(A-BK)<1\), and applies the same per-sample steering-rate limit and fail-stop behavior as the hardened kinematic LQR.

The default `run_simulation()` still uses `KinematicBicycleBackend` for dynamic LQR. Mass, inertia, and cornering stiffness affect controller design only; the simulated plant state is still advanced by the kinematic equations in Section 2. The current experiment therefore does not validate the predictive accuracy of a dynamic vehicle plant.

## 6. Pure Pursuit geometry

### 6.1 Lookahead-circle derivation

Classical Pure Pursuit uses a local frame at the tracking point, with the vehicle facing local \(+x\). A lookahead point has coordinates

\[
(x_t,y_t)=(L_d\cos\alpha,L_d\sin\alpha),
\]

where \(L_d\) is lookahead distance and \(\alpha\) is the target bearing relative to body heading. Assume the connecting trajectory is a circle tangent to the current vehicle direction, with center \((0,R)\). The target lies on that circle:

\[
x_t^2+(y_t-R)^2=R^2.
\]

Substitution gives

\[
L_d^2-2RL_d\sin\alpha=0.
\]

Therefore, the requested curvature is

\[
\boxed{\kappa_{pp}=\frac1R
=\frac{2\sin\alpha}{L_d}}.
\tag{47}
\]

Combining it with the simplified bicycle relationship
\(\tan\delta_f=L\kappa\) gives

\[
\boxed{
\delta_{f,\mathrm{desired}}
=\operatorname{atan2}(2L\sin\alpha,L_d)}.
\tag{48}
\]

### 6.2 Current implementation

Lookahead grows with speed:

\[
\boxed{
L_d=k_0+k_vv=3.0+0.35v\ \mathrm{m}}.
\tag{49}
\]

The units of `pp_speed_gain = 0.35` are seconds. Starting at the nearest reference point, the controller selects the first forward path sample whose Euclidean distance from the vehicle is at least \(L_d\). It computes

\[
\alpha=\operatorname{wrap}\!\left[
\operatorname{atan2}(y_t-y,x_t-x)-\psi
\right].
\tag{50}
\]

Path discretization normally makes the selected point slightly farther away than \(L_d\), but Equation (48) still uses the configured \(L_d\) in its denominator. Classical PP also normally tracks the rear-axle center, whereas the shared propagation model's \(\beta\) geometry corresponds to point C. Both facts make this implementation approximate relative to the ideal derivation.

The instructor solution clips steering angle only. The hardened student PP applies both angle and per-sample rate limits, validates finite path/state/controller data, and rate-centers steering while commanding a stop when runtime inputs are invalid.

A larger \(L_d\) generally gives smoother steering but more curve lag. A smaller \(L_d\) reacts faster but can oscillate and amplify path discretization or localization noise.

### 6.3 Longitudinal control for PP and LQR

PP and both LQR controllers share `speed_pid()`. Let \(d_g\) be straight-line distance to the final path point and \(v_r\) the path target speed. For \(d_g<14\) m,

\[
v_{\mathrm{cmd}}
=\min\left(v_r,\sqrt{2(0.9)d_g}\right).
\]

For \(d_g<1\) m, \(v_{\mathrm{cmd}}=0\). The nominal acceleration command is

\[
a_{\mathrm{cmd}}
=\operatorname{clip}
\left(1.1(v_{\mathrm{cmd}}-v),
-a_{\mathrm{decel,max}},a_{\max}\right).
\]

The square-root term follows the constant-deceleration stopping relation \(v^2=2ad\) with planned deceleration \(0.9\ \mathrm{m/s^2}\).

The hardened student controllers apply an additional one-step speed bound:

\[
a_k\in
\left[
\max\left(-a_{\mathrm{decel,max}},
\frac{v_{\min}-v_k}{\Delta t}\right),
\min\left(a_{\max},
\frac{v_{\max}-v_k}{\Delta t}\right)
\right].
\]

A finite measured speed below \(v_{\min}\), including a negative speed, is treated as an invalid forward-only state and returns **neutral longitudinal acceleration \(a=0\)**. It never requests positive propulsion. A non-finite speed produces bounded braking. This is a controller fail-stop convention, not a substitute for gear, rollback, or signed-velocity handling in a vehicle supervisor.

MPC does not call `speed_pid()`; it optimizes acceleration directly.

## 7. Linear MPC model, linearization, objective, and constraints

### 7.1 Nonlinear prediction model

The MPC state and input are

\[
\mathbf z=
\begin{bmatrix}x&y&v&\psi\end{bmatrix}^T,\qquad
\mathbf u=
\begin{bmatrix}a&\delta_f\end{bmatrix}^T.
\tag{51}
\]

`update_kinematic_array()` uses

\[
x_{k+1}=x_k+v_k\cos\psi_k\Delta t,
\]

\[
y_{k+1}=y_k+v_k\sin\psi_k\Delta t,
\]

\[
v_{k+1}=v_k+a_k\Delta t,
\]

\[
\psi_{k+1}
=\psi_k+\frac{v_k}{L}\tan\delta_{f,k}\Delta t.
\tag{52}
\]

This simplified model omits \(\beta\) and differs from the default plant in Equations (3)-(4). The hardened implementation checks finite state data, applies asymmetric acceleration bounds, clips speed to the forward-only interval, and clips steering away from the \(\tan\delta\) singularity.

### 7.2 First-order affine linearization

Let the nominal prediction point be
\((\bar v_t,\bar\psi_t,\bar\delta_t)\). A first-order Taylor expansion of Equation (52), expressed with absolute states and controls, is

\[
\boxed{
\mathbf z_{t+1}
=A_t\mathbf z_t+B_t\mathbf u_t+C_t},
\tag{53}
\]

where

\[
A_t=
\begin{bmatrix}
1&0&\Delta t\cos\bar\psi_t
&-\Delta t\bar v_t\sin\bar\psi_t\\
0&1&\Delta t\sin\bar\psi_t
& \Delta t\bar v_t\cos\bar\psi_t\\
0&0&1&0\\
0&0&\dfrac{\Delta t\tan\bar\delta_t}{L}&1
\end{bmatrix},
\tag{54}
\]

\[
B_t=
\begin{bmatrix}
0&0\\
0&0\\
\Delta t&0\\
0&\dfrac{\Delta t\bar v_t}
{L\cos^2\bar\delta_t}
\end{bmatrix},
\tag{55}
\]

\[
C_t=
\begin{bmatrix}
\Delta t\bar v_t\sin\bar\psi_t\,\bar\psi_t\\
-\Delta t\bar v_t\cos\bar\psi_t\,\bar\psi_t\\
0\\
-\dfrac{\Delta t\bar v_t\bar\delta_t}
{L\cos^2\bar\delta_t}
\end{bmatrix}.
\tag{56}
\]

The affine vector \(C_t\) is necessary because the optimization variables are absolute values rather than deviations from the nominal trajectory. Substituting
\((\bar{\mathbf z}_t,\bar{\mathbf u}_t)\) into Equation (53) exactly reproduces Equation (52) at the linearization point. \(A_t\) and \(B_t\) are the state and input Jacobians.

### 7.3 Horizon reference

The default horizon is \(N=8\), giving nominal lookahead time

\[
T=N\Delta t=0.8\ \mathrm{s}.
\]

`nearest_horizon_reference()` constructs \(N+1\) states

\[
\mathbf z_t^{ref}
=\begin{bmatrix}
x_t^{ref}&y_t^{ref}&v_t^{ref}&\psi_t^{ref}
\end{bmatrix}^T.
\]

The instructor solution increments preview distance by

\[
\Delta s_{\mathrm{preview}}^{\mathrm{solution}}
=\max\left(
v_{\mathrm{current}},
0.5\max(v_{\mathrm{ref}},1.0)
\right)\Delta t
\tag{57}
\]

and converts cumulative distance to a sample offset using
`round(distance / waypoint_ds)`.

The hardened student controller uses

\[
\Delta s_{\mathrm{preview}}^{\mathrm{student}}
=\max\left(0,v_{\mathrm{current}},0.5v_{\mathrm{ref}}\right)
\Delta t.
\]

When a finite monotonic `path.s` array is available, it selects the next index with `searchsorted` on physical arc length. Otherwise it falls back to the sample-spacing calculation. Reference speed is clipped to \([v_{\min},v_{\max}]\).

Both versions unwrap each reference yaw relative to the preceding yaw, preventing an artificial \(2\pi\) jump when a circle crosses \(\pi/-\pi\).

### 7.4 Quadratic objective

Each QP minimizes

\[
\begin{aligned}
J={}&
\sum_{t=0}^{N-1}
(\mathbf z_t^{ref}-\mathbf z_t)^TQ
(\mathbf z_t^{ref}-\mathbf z_t)\\
&+\sum_{t=0}^{N-1}\mathbf u_t^TR\mathbf u_t\\
&+\sum_{t=0}^{N-2}
(\mathbf u_{t+1}-\mathbf u_t)^TR_d
(\mathbf u_{t+1}-\mathbf u_t)\\
&+(\mathbf z_N^{ref}-\mathbf z_N)^TQ_f
(\mathbf z_N^{ref}-\mathbf z_N).
\end{aligned}
\tag{58}
\]

Default weights are

\[
Q=\operatorname{diag}(2,2,0.6,1),\quad
Q_f=\operatorname{diag}(4,4,1,2),
\]

\[
R=\operatorname{diag}(0.2,0.4),\quad
R_d=\operatorname{diag}(0.2,0.8).
\]

The terms penalize state-tracking error, control magnitude, control changes, and terminal error. Because state and input components have different physical units, these weights also provide implicit numerical scaling.

The hardened student controller requires each weight matrix to be finite, symmetric after numerical symmetrization, correctly sized, and positive semidefinite. It also requires `mpc_horizon` to be a positive integer.

### 7.5 Constraints

Both versions impose the affine dynamics, steering magnitude, and speed bounds:

\[
\mathbf z_0=\mathbf z_{\mathrm{current}},\qquad
\mathbf z_{t+1}=A_t\mathbf z_t+B_t\mathbf u_t+C_t,
\]

\[
v_{\min}\leq v_t\leq v_{\max},
\tag{59}
\]

\[
|\delta_{f,t}|\leq\delta_{\max}.
\tag{60}
\]

The instructor solution expresses acceleration as

\[
|a_t|\leq a_{\max},\qquad
a_t\geq-a_{\mathrm{decel,max}}.
\tag{61}
\]

With the defaults, this makes its effective QP interval
\([-2.0,2.0]\ \mathrm{m/s^2}\), so `max_decel = 3.5` is shadowed by the absolute-value constraint.

The hardened student MPC uses the intended asymmetric interval

\[
\boxed{
-a_{\mathrm{decel,max}}
\leq a_t\leq a_{\max}}
\]

and enforces the speed bounds on predicted states \(t=1,\ldots,N\) after separately validating the measured initial state.

For steering rate, the instructor solution constrains only adjacent future controls:

\[
|\delta_{f,t+1}-\delta_{f,t}|
\leq\dot\delta_{\max}\Delta t,
\qquad t=0,\ldots,N-2.
\tag{62}
\]

It does not constrain the first optimized steering value against the previously applied command.

The hardened student MPC adds the missing first-move constraint

\[
\boxed{
|\delta_{f,0}-\delta_{f,\mathrm{applied}}|
\leq\dot\delta_{\max}\Delta t}
\]

and retains Equation (62). It then sanitizes the returned sequence step by step against acceleration, speed, steering-angle, and steering-rate limits. The other hardened student controllers apply the same first-command angle/rate bound outside an optimizer.

Neither MPC version constrains lateral acceleration, yaw rate, tire friction, road boundaries, or obstacles.

### 7.6 Sequential linearization and receding-horizon control

A normal control call performs the following sequence:

1. Fill initial length-\(N\) control sequences from the preceding applied command.
2. Predict a nominal trajectory \(\bar{\mathbf z}\) with Equation (52).
3. Build Equation (53) along \(\bar{\mathbf z}\) and the nominal steering sequence.
4. Solve Equations (58)-(62) with OSQP.
5. Compare the new and old acceleration and steering sequences; stop when their maximum element change is at most `0.05`, or after at most five iterations.
6. Execute only the first optimized control and solve again at the next sample.

This is receding-horizon control. The common `0.05` threshold is applied to both m/s² and radians; it is a numerical convergence threshold, not a physical limit.

The hardened student implementation catches solver failure, unusable solver status, non-finite output, malformed reference data, and invalid runtime state. It returns bounded braking where signed speed is valid, moves steering toward zero at the configured rate, and supplies a finite fallback prediction. A finite speed below \(v_{\min}\), including negative speed, returns neutral acceleration as described in Section 6.3. Missing `cvxpy` is deliberately re-raised as an installation/preflight failure.

The controller has no wall-clock deadline. A verification run on the medium-speed S-curve measured 140 commands with 67.3 ms mean computation, 116.9 ms p95, 255.5 ms maximum/first-call latency, and 12 commands exceeding the 100 ms nominal sample period. This timing is environment-specific, but it demonstrates that the present MPC implementation is not deterministic enough for physical deployment.

## 8. Code and CSV variable mapping

### 8.1 Main source locations

| Content | Implementation |
| --- | --- |
| \(\beta,\dot x,\dot y,\dot\psi,a_n\) functions | `vdm_lab/common/bicycle_model.py` |
| Command magnitude clipping and Euler state update | `vdm_lab/common/vehicle.py` |
| Nearest point, lateral error, and heading error | `vdm_lab/common/reference.py` |
| Spline curvature \((x'y''-y'x'')/(x'^2+y'^2)^{3/2}\) | `vdm_lab/common/path.py` |
| Vehicle, simulation, and controller parameters | `vdm_lab/common/types.py`, `vdm_lab/config/vehicle_params.py` |
| Instructor reference controllers | `vdm_lab/solutions/*.py` |
| Hardened PP | `vdm_lab/student/pure_pursuit.py` |
| Hardened kinematic LQR | `vdm_lab/student/lqr_kinematic.py` |
| Hardened dynamic LQR | `vdm_lab/student/lqr_dynamic.py` |
| Hardened MPC | `vdm_lab/student/mpc.py` |
| Per-step record construction | `vdm_lab/common/simulation.py` |
| CSV and aggregate metric output | `vdm_lab/common/logging.py` |

### 8.2 `trajectory.csv`

The columns come directly from `StepRecord`:

| CSV column | Mathematical meaning | Unit | Detail |
| --- | --- | --- | --- |
| `time` | \(t_k\) | s | Current record time |
| `x`, `y` | \(x_k,y_k\) | m | Current vehicle-reference-point position |
| `yaw` | \(\psi_k\) | rad | Body heading wrapped to \([-\pi,\pi]\) |
| `speed` | \(v_k\) | m/s | Current state speed |
| `acceleration` | \(a_k\) | m/s² | Current magnitude-limited control command |
| `steer` | \(\delta_{f,k}\) | rad | Current magnitude-limited road-wheel command |
| `beta` | \(\beta_k\) | rad | Equation (2), evaluated from current `steer` |
| `yaw_rate` | \(r_k=\dot\psi_k\) | rad/s | Equation (3), evaluated from current `speed` and `steer` |
| `target_index` | Nearest-point index | — | Nearest reference sample, not PP's internal lookahead target |
| `lateral_error` | \(e_{y,k}\) | m | Equation (17), with its left/right sign |
| `heading_error` | \(e_{\psi,k}\) | rad | Equation (17), wrapped to \([-\pi,\pi]\) |
| `curvature` | \(\kappa_{r,k}\) | 1/m | Nearest reference curvature, not estimated actual-trajectory curvature |
| `normal_accel` | \(v_k^2\kappa_{r,k}\) | m/s² | Equation (15), a reference-path quantity |
| `target_speed` | \(v_{r,k}\) | m/s | Target speed at the nearest reference point |

The record sequence is: read current state, compute and limit the current command, compute `beta` and `yaw_rate`, write the record, then advance the plant with that command. Thus `steer`, `beta`, and `yaw_rate` on one row are time-aligned in the default backend.

Although the hardened controllers apply a rate limit, `trajectory.csv` does not contain a separate commanded-versus-actuator-feedback distinction. In the simulator, the logged steering is the command accepted by the kinematic backend.

### 8.3 `reference_path.csv`

| CSV column | Meaning | Unit |
| --- | --- | --- |
| `index` | Path-sample index | — |
| `s_m` | Accumulated reference arc length | m |
| `x_m`, `y_m` | Local reference coordinates | m |
| `yaw_rad` | Reference heading \(\psi_r\) | rad |
| `curvature_1pm` | Reference curvature \(\kappa_r\) | 1/m |
| `target_speed_mps` | Reference speed | m/s |
| `lat_deg`, `lon_deg` | GPX latitude and longitude; blank for built-in routes | deg |
| `elevation_m` | GPX elevation; blank when unavailable | m |

## 9. Assumptions, implementation differences, and deployment implications

### 9.1 Model and data interpretation

1. **The vehicle reference point is inconsistent across modules.** The \(l_f,l_r,\beta\) equations in `bicycle_model.py` require `state.x, state.y` to represent point C. In `visualization.py`, however, the same point is drawn at the rear axle and the front axle is drawn at `state + wheelbase`. Classical PP and the simplified MPC model also more naturally use a rear-axle point. Before physical integration, define whether state position means rear-axle center, center of gravity, or GNSS antenna, then apply the appropriate rigid-body transform.

2. **Controller models differ from the shared simulated plant.** The plant uses Equations (2)-(4), including \(\beta\). PP, LQR error-rate approximations, and MPC omit some or all of \(\beta\). The mismatch grows with steering angle and with any reference-point offset.

3. **Dynamic LQR does not run against a dynamic plant.** The default backend remains kinematic. The dynamic parameters \(m,I_z,C_f,C_r\) affect only controller matrices and feedforward.

4. **Cornering-stiffness convention is not fully consistent.** Equations (37)-(38) use \(C_f,C_r\) directly as axle stiffnesses. The factors `2 * cf` and `2 * cr` in Equations (43)-(44) match a common per-tire-stiffness convention. If one physical definition is intended, the matrix and feedforward contain a factor-of-two inconsistency that must be resolved after vehicle identification.

5. **Dynamic discretization differs by version.** The instructor solution uses the hybrid pair in Equation (42); the hardened student controller uses the consistent trapezoidal input matrix. Results from the two versions therefore need not match exactly.

6. **`normal_accel` is a reference quantity.** It uses reference curvature rather than executed-vehicle curvature. It imposes no tire-friction or comfort limit. Physical validation requires IMU lateral acceleration together with speed, yaw-rate, and steering feedback.

### 9.2 Controller hardening and its limits

7. **Steering-rate protection is version-specific.** The instructor PP/LQR controllers and shared backend clip angle only. Instructor MPC constrains future in-horizon differences but omits the first change from the applied command. All hardened student controllers enforce both angle and first-command rate limits; student MPC enforces the limit throughout its horizon as well. This hardening can increase tracking error on abrupt paths because it removes physically implausible steering jumps that make the compact solutions appear more accurate.

8. **The instructor MPC acceleration constraint is narrower than its configuration implies.** Its effective default interval is \([-2,2]\ \mathrm{m/s^2}\). The hardened student MPC uses \([-3.5,2]\ \mathrm{m/s^2}\) and post-validates predicted speed.

9. **Hardened safety configuration fails closed at initialization.** Non-numeric, non-finite, non-positive, or contradictory safety limits raise `ValueError` before control. Runtime state/reference/model faults instead produce a finite stop-oriented command. This distinction prevents a corrupt safety envelope from being silently treated as a normal recoverable fault.

10. **Negative speed is outside the forward-only controller contract.** The simulator clamps speed to \(v\geq0\), but a real sensor or adapter can report negative speed during rollback, reverse gear, or a sign error. Hardened controllers return neutral \(a=0\) for finite \(v<v_{\min}\); they do not request positive forward torque. A hardware supervisor must independently handle gear state, rollback braking, and signed velocity.

11. **Fail-stop commands are still software requests.** Rate-centering steering and bounded braking only help if an actuator layer receives the command on time, interprets its sign correctly, and reports actual execution. They are not a substitute for an independent watchdog or emergency stop.

### 9.3 Physical-vehicle blockers

12. **No real-vehicle backend exists.** `run_experiment.py` does not inject an external backend. `ExternalVehicleBackend.step()` remains `NotImplementedError`. There is no state acquisition, actuator conversion, applied-command feedback, stale-data detection, or drive-enable handshake. The current program therefore cannot command a real car without additional, safety-reviewed integration code outside the student controller directory.

13. **The command units are abstract vehicle-level quantities.** `steer` is front road-wheel angle, not steering-wheel angle. `acceleration` is desired longitudinal acceleration, not throttle or brake percentage. A physical backend needs steering ratio, sign, zero offset, saturation, actual-angle feedback, and calibrated longitudinal closed-loop control.

14. **MPC has no deterministic deadline or stale-command fallback.** A mathematical fallback after solver failure does not address a solve that simply completes after the 100 ms command period. The real-time layer must enforce deadlines and decide what command remains valid when a deadline is missed.

15. **No environment-level safety constraints exist.** The controllers do not enforce lanes, road boundaries, obstacle clearance, collision avoidance, traffic rules, or route feasibility. `reached_goal=True` is only a simulator completion condition.

Before enabling physical actuators, an independent supervisor must at minimum provide localization validity and age checks, coordinate-frame validation, speed-dependent steering and lateral-acceleration limits, command sequence numbers and timestamps, a communication watchdog, deadline handling, manual takeover, a hardware emergency stop, and road/obstacle constraints. Simulation success cannot replace these controls.

## 10. Recommended calculation procedure for the circle experiment

Select candidate circular records from `trajectory.csv` using

\[
|\texttt{curvature}-1/12|<0.005.
\]

Remove the entry and exit transients. For each algorithm and speed, calculate

\[
\overline{\delta_f},\quad
\overline r,\quad
\overline a_n,\quad
\overline{|e_y|},\quad
\max|e_y|,\quad
\operatorname{std}(e_y),
\]

and steering activity

\[
J_\delta=\operatorname{mean}\left(
\frac{|\delta_{f,k}-\delta_{f,k-1}|}{\Delta t}
\right).
\]

Use the actual mean speed \(\bar v\) over the selected steady segment rather than only the named speed mode:

\[
\delta_{\mathrm{theory}}=\arctan(L/12),\quad
r_{\mathrm{theory}}=\bar v/12,\quad
a_{n,\mathrm{theory}}=\bar v^2/12.
\]

For any quantity \(q\), report relative error as

\[
\epsilon_{\mathrm{rel}}
=\frac{|q_{\mathrm{sim}}-q_{\mathrm{theory}}|}
{|q_{\mathrm{theory}}|}\times100\%.
\]

Interpret separately:

- the entry transient;
- steady tracking in the middle of the circle;
- steering recovery at circle exit;
- steering-rate saturation in the hardened controllers;
- the distinction between reference-derived `normal_accel` and actual vehicle response; and
- model differences between the chosen controller version and the shared kinematic plant.

