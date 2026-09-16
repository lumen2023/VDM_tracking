# VDM Path Tracking Simulation

本仓库用于车辆动力学与运动控制课程的路径跟踪仿真实验。当前阶段围绕运动学自行车模型与路径跟踪控制展开，逐步实现并比较 Pure Pursuit（PP）、LQR 和 MPC，并进一步研究速度、车辆参数、控制器参数及车辆模型对跟踪性能的影响。

---

## 1. Pure Pursuit（PP）

### 1.1 实验目标

本阶段完成以下工作：

1. 理解运动学自行车模型与路径跟踪闭环；
2. 完成 `vdm_lab/student/pure_pursuit.py` 中 PP 控制器的核心实现；
3. 使用 `solution` 版本验证学生版 PP 的正确性；
4. 在半径固定的 `circle` 路线上进行 low / medium / high 三档速度实验；
5. 结合理论公式与仿真数据分析“为什么车辆速度越高，路径跟踪通常越困难”。

PP 的核心思想是：车辆不直接追踪当前位置附近的最近路径点，而是在参考路径前方选取一个**前视目标点**，并通过几何关系计算前轮转角，使车辆不断朝该目标点行驶。

---

### 1.2 PP 算法实现

学生版 PP 位于：

```text
vdm_lab/student/pure_pursuit.py
```

本次实现主要完成四个步骤。

#### Step 1：计算前视距离

前视距离随车速增大：

\[
L_f = L_0 + k_v v
\]

代码实现：

```python
lookahead = controller.pp_base_lookahead + controller.pp_speed_gain * state.v
```

其中：

- `L0 = pp_base_lookahead`：基础前视距离；
- `kv = pp_speed_gain`：速度增益；
- `v = state.v`：车辆当前速度。

速度越高，车辆观察的路径点越远，通常可以获得更平滑的转向响应；但前视距离过大也可能导致切弯和较大的横向偏差。

#### Step 2：搜索前视目标点

从当前最近路径点 `reference.nearest_index` 开始沿参考路径向前搜索，直到候选点与车辆的欧氏距离不小于 `lookahead`：

```python
target_index = reference.nearest_index

while target_index < len(path.x) - 1:
    distance = math.hypot(
        path.x[target_index] - state.x,
        path.y[target_index] - state.y,
    )

    if distance >= lookahead:
        break

    target_index += 1
```

这样可以避免车辆反复追踪已经驶过的路径点。

#### Step 3：计算目标点相对车辆的方向角

目标点相对于车辆当前航向的夹角为：

\[
\alpha =
\operatorname{atan2}(y_t-y,\;x_t-x)-\psi
\]

代码中通过 `pi_to_pi()` 将角度归一化到 \([-\pi,\pi]\)：

```python
alpha = pi_to_pi(
    math.atan2(
        target_y - state.y,
        target_x - state.x,
    ) - state.yaw
)
```

当：

- \(\alpha > 0\)：目标点位于车辆左侧；
- \(\alpha < 0\)：目标点位于车辆右侧；
- \(\alpha \approx 0\)：目标点基本位于正前方。

#### Step 4：计算前轮转角

PP 的几何转向关系为：

\[
\delta_f =
\operatorname{atan2}
\left(
2L\sin\alpha,\;L_f
\right)
\]

其中 \(L\) 为车辆轴距。

代码实现：

```python
steer = math.atan2(
    2.0 * vehicle.wheelbase * math.sin(alpha),
    lookahead,
)
```

最终控制器返回：

```python
return ControlCommand(
    acceleration=acceleration,
    steer=steer,
)
```

其中纵向加速度由项目现有的速度比例控制器 `speed_pid()` 计算，PP 主要负责横向转向控制。

---

### 1.3 Student PP 与参考实现验证

为了确认学生版 PP 实现正确，分别在 `double_lane_change` 和 `circle` 低速工况下运行 `solution` 与 `student` 版本。

运行示例：

```powershell
python run_experiment.py --algo pp --version student --route double_lane_change --speed-mode low --save-log --save-fig
python run_experiment.py --algo pp --version student --route circle --speed-mode low --save-log --save-fig
```

结果如下。

| Route              | Version  | Reached Goal | Mean Lateral Error / m | Max Lateral Error / m |
| ------------------ | -------- | ------------ | ---------------------: | --------------------: |
| double_lane_change | solution | True         |                  0.313 |                 0.756 |
| double_lane_change | student  | True         |                  0.313 |                 0.756 |
| circle             | solution | True         |                  0.286 |                 0.446 |
| circle             | student  | True         |                  0.286 |                 0.446 |

学生版与参考实现的主要指标一致，因此可以确认本组实现的 PP 核心逻辑正确。

---

### 1.4 圆形路径速度实验

#### 1.4.1 实验设置

为了单独研究速度变化对 PP 跟踪性能的影响，固定：

- 控制器：Pure Pursuit；
- 车辆：`student_car`；
- 路线：`circle`；
- 圆弧半径：\(R = 12\,m\)；
- 车辆轴距：\(L = l_f + l_r = 2.5\,m\)；
- 其他控制器与仿真参数保持不变。

只改变目标速度：

| Speed Mode | Target Speed |
| ---------- | -----------: |
| low        |      3.0 m/s |
| medium     |      5.0 m/s |
| high       |      7.0 m/s |

运行命令：

```powershell
python run_experiment.py --algo pp --version student --route circle --speed-mode low --save-log --save-fig
python run_experiment.py --algo pp --version student --route circle --speed-mode medium --save-log --save-fig
python run_experiment.py --algo pp --version student --route circle --speed-mode high --save-log --save-fig
```

本次实验对应输出目录：

```text
outputs/20260915_080400_pp_circle_low
outputs/20260915_080444_pp_circle_medium
outputs/20260915_080458_pp_circle_high
```

#### 1.4.2 全程指标

由各实验的 `metrics.json` 得到：

| Metric                          |    Low | Medium |   High |
| ------------------------------- | -----: | -----: | -----: |
| Target speed / m/s              |    3.0 |    5.0 |    7.0 |
| Mean lateral error / m          |  0.286 |  0.288 |  0.275 |
| Max lateral error / m           |  0.446 |  0.505 |  0.525 |
| Mean heading error / rad        | 0.0646 | 0.0582 | 0.0505 |
| Max steer / rad                 |  0.232 |  0.242 |  0.229 |
| Max normal acceleration / m/s² |  0.750 |  2.083 |  4.082 |
| Max side-slip\(\beta\) / rad    |  0.118 |  0.123 |  0.116 |
| Max yaw rate / rad/s            |  0.282 |  0.490 |  0.636 |
| Reached goal                    |   True |   True |   True |

可以看到，最大横向误差随速度增加：

\[
0.446 \rightarrow 0.505 \rightarrow 0.525\,m
\]

从 low 到 high，最大横向误差增加约 **17.7%**。

全程平均横向误差并未单调增加，因此不能简单用“平均误差越大”概括高速跟踪困难。后续需要进一步分析稳态圆弧段。

---

### 1.5 圆弧稳态分析

任务中圆形路径的理论曲率为：

\[
\kappa = \frac{1}{R}
       = \frac{1}{12}
       \approx 0.08333\,m^{-1}
\]

稳态分析使用 `trajectory.csv` 中满足：

```text
abs(curvature - 1/12) < 0.005
```

的记录作为圆弧候选段，并去除候选段首尾各 10% 的记录，以降低进入和驶出圆弧时瞬态过程的影响。

分析脚本：

```text
analyze_circle.py
```

汇总结果：

```text
circle_speed_analysis.csv
```

#### 1.5.1 稳态实验结果

| Metric                              |    Low |   Medium |   High |
| ----------------------------------- | -----: | -------: | -----: |
| Target speed / m/s                  |  3.000 |    5.000 |  7.000 |
| Mean actual speed / m/s             |  3.000 |    4.794 |  6.071 |
| Mean steer / rad                    | 0.2136 |   0.2146 | 0.2148 |
| Mean yaw rate / rad/s               | 0.2588 |   0.4155 | 0.5268 |
| Mean normal acceleration / m/s²    | 0.7500 |   1.9232 | 3.1694 |
| Mean lateral error / m              | 0.4379 |   0.4724 | 0.4804 |
| Max lateral error / m               | 0.4462 |   0.5046 | 0.5251 |
| Lateral error std / m               | 0.0037 |   0.0169 | 0.0144 |
| Max\(                               |  \beta | \) / rad | 0.1176 |
| Mean steer rate\(J_\delta\) / rad/s | 0.1941 |   0.0340 | 0.1091 |

稳态平均横向误差：

\[
0.438 \rightarrow 0.472 \rightarrow 0.480\,m
\]

从 low 到 high 增加约 **9.7%**。

同时，medium / high 的横向误差标准差明显高于 low，说明速度提高后圆弧跟踪误差的波动也更明显。

---

### 1.6 理论值与仿真结果对比

#### 1.6.1 稳态前轮转角

对于小侧偏、近似稳态的运动学自行车模型：

\[
\delta_f \approx \arctan(L\kappa)
\]

代入：

\[
L=2.5\,m,\qquad
\kappa=\frac{1}{12}\,m^{-1}
\]

得到：

\[
\delta_f
\approx
\arctan\left(\frac{2.5}{12}\right)
\approx
0.2054\,rad
\approx
11.77^\circ
\]

三档速度的仿真平均转角分别为：

| Speed  | Simulation / rad | Theory / rad | Relative Error |
| ------ | ---------------: | -----------: | -------------: |
| low    |           0.2136 |       0.2054 |          4.01% |
| medium |           0.2146 |       0.2054 |          4.48% |
| high   |           0.2148 |       0.2054 |          4.57% |

三档转角几乎不随速度变化，说明固定圆弧下所需的几何稳态转角主要由**车辆轴距和路径曲率**决定，而不是直接由车速决定。

#### 1.6.2 横摆角速度

稳态近似关系：

\[
\dot{\psi} \approx v\kappa
\]

使用目标速度时的理论值：

| Speed  | Target\(v\) / m/s | Theory yaw rate / rad/s | Simulation / rad/s |
| ------ | ----------------: | ----------------------: | -----------------: |
| low    |               3.0 |                  0.2500 |             0.2588 |
| medium |               5.0 |                  0.4167 |             0.4155 |
| high   |               7.0 |                  0.5833 |             0.5268 |

需要注意，medium 和 high 工况的圆弧实际平均速度分别只有约 `4.794 m/s` 和 `6.071 m/s`，并未完全达到目标速度。

若使用实际速度计算理论 yaw rate，则与仿真结果的相对误差约保持在 3.5%–4.1%。这说明 `vκ` 是较好的稳态近似，但实际仿真使用的是完整运动学自行车模型：

\[
\dot\psi =
\frac{v}{L}
\tan(\delta_f)
\cos(\beta)
\]

因此存在一定差异。

#### 1.6.3 法向加速度

圆周运动关系：

\[
a_n=v^2\kappa
\]

使用目标速度得到：

| Speed  | Theory\(a_n\) / m/s² | Simulation Mean / m/s² |
| ------ | --------------------: | ----------------------: |
| low    |                0.7500 |                  0.7500 |
| medium |                2.0833 |                  1.9232 |
| high   |                4.0833 |                  3.1694 |

从 3 m/s 提高到 7 m/s，速度约提高到原来的：

\[
\frac{7}{3}\approx2.33
\]

而理论法向加速度需求提高到：

\[
\left(\frac{7}{3}\right)^2\approx5.44
\]

倍。

medium 和 high 的仿真均值低于以目标速度计算的理论值，主要是因为圆弧稳态区间内车辆实际平均速度没有完全达到目标速度。

需要说明的是，本项目中的 `normal_accel` 本身就是根据：

```python
normal_accel = speed * speed * curvature
```

计算得到，因此使用同一时刻实际速度和曲率重新计算时会得到相同结果。这里的比较主要用于验证代码公式与课程理论的一致性，而不是独立的真实车辆物理验证。

---

### 1.7 为什么高速路径跟踪更困难？

结合理论和本次 PP 实验，可以得到以下结论。

#### 1. 横向动态需求随速度快速增长

在路径曲率固定时：

\[
a_n=v^2\kappa
\]

因此速度提高后，横向加速度需求按速度平方增长。高速车辆需要更快建立横向运动状态，对车辆横向响应提出更高要求。

#### 2. 横摆响应随速度提高

近似有：

\[
\dot{\psi}\approx v\kappa
\]

因此相同曲率下，高速车辆需要更高的横摆响应速度。

#### 3. 单个采样周期内车辆前进距离增加

仿真采样时间 `dt` 固定时：

\[
\Delta s \approx v\,dt
\]

速度越高，每个控制周期内车辆前进越远，因此控制器可用于发现并修正偏差的时间更短。

#### 4. 高速下最大误差和稳态误差均有所增加

本实验中：

- 最大横向误差：`0.446 m → 0.525 m`；
- 稳态平均横向误差：`0.438 m → 0.480 m`；
- medium / high 的稳态误差波动明显高于 low。

因此，高速工况下跟踪性能更容易受到瞬态响应、控制延迟及车辆约束的影响。

#### 5. 高速困难并不意味着需要显著更大的稳态转角

三档速度的稳态平均转角都约为：

\[
0.214\,rad
\]

说明对于相同半径圆弧，稳态几何转角基本不变。

高速跟踪困难更主要来自：

- 更高的横摆响应需求；
- 更高的横向加速度需求；
- 更短的误差修正时间；
- 控制器和车辆执行器的动态限制。

---

### 1.8 PP 阶段结论

本阶段完成了学生版 Pure Pursuit 控制器，并通过 `solution` 与 `student` 对比确认实现正确。

PP 的优点包括：

- 算法结构简单；
- 几何意义清晰；
- 计算量低；
- 易于与不同参考路径和车辆模型结合。

同时，PP 的性能对前视距离较敏感。随着速度提高，前视距离随速度自适应增加，可以改善控制平滑性，但也可能增加切弯和路径偏差。

圆形路径实验表明：

1. 固定曲率下，稳态转角随速度变化较小；
2. 速度提高后，横摆和横向加速度需求明显提高；
3. 稳态及最大横向误差均有一定增加；
4. 高速跟踪困难主要体现为动态响应要求和误差修正时间的增加，而不是单纯需要更大的方向盘转角。

这部分结果将作为后续 **LQR 与 MPC 对比实验**的 PP 基准。

---

### 1.9 当前 PP 相关文件

```text
vdm_lab/student/pure_pursuit.py
analyze_circle.py
circle_speed_analysis.csv
```

结果图：

| Low                             | Medium                             | High                             |
| ------------------------------- | ---------------------------------- | -------------------------------- |
| ![](docs\figures\pp\pp_low.png) | ![](docs\figures\pp\pp_medium.png) | ![](docs\figures\pp\pp_high.png) |

---

## 2. Kinematic LQR

### 2.1 算法目标

在完成 Pure Pursuit 后，本阶段实现运动学 LQR（Linear Quadratic Regulator）路径跟踪控制器。

与 PP 通过前视目标点进行几何跟踪不同，LQR 直接基于车辆相对于参考路径的误差状态进行反馈控制。本文使用的误差状态为：

\[
x_e =
\begin{bmatrix}
e_y \\
\dot e_y \\
e_\psi \\
\dot e_\psi
\end{bmatrix}
\]

其中：

- \(e_y\)：横向误差；
- \(\dot e_y\)：横向误差变化率；
- \(e_\psi\)：航向误差；
- \(\dot e_\psi\)：航向误差变化率。

控制目标是同时减小路径跟踪误差与控制输入代价。

---

### 2.2 运动学误差模型

LQR 使用离散状态空间模型：

\[
x_{k+1}=Ax_k+Bu_k
\]

其中控制输入 \(u_k\) 为前轮转角。

学生版中建立的运动学误差模型为：

```python
A[0, 0] = 1.0
A[0, 1] = dt

A[1, 2] = speed

A[2, 2] = 1.0
A[2, 3] = dt

B[3, 0] = speed / wheelbase
```

其中近似关系：

\[
\dot e_y \approx v e_\psi
\]

说明相同的航向误差在更高车速下会更快转化为横向位置误差，这也是高速路径跟踪难度增加的一个重要原因。

误差状态计算为：

```python
e_y = reference.lateral_error

e_y_dot = speed * math.sin(
    reference.heading_error
)

e_yaw = reference.heading_error

e_yaw_dot = (
    speed / vehicle.wheelbase
    * math.tan(previous_control.steer)
    - speed * reference.curvature
)

error_state = np.array([
    [e_y],
    [e_y_dot],
    [e_yaw],
    [e_yaw_dot],
])
```

其中：

\[
\dot e_\psi
===========

\dot\psi_}
----------

\dot\psi_{\text{reference}}
\]

并使用：

\[
\dot\psi_{\text{vehicle}}
\approx
\frac{v}{L}\tan\delta
\]

和：

\[
\dot\psi_{\text{reference}}
\approx
v\kappa
\]

构造航向误差变化率。

---

### 2.3 LQR 最优反馈

LQR 最小化代价函数：

\[
J=
\sum
\left(
x^TQx+u^TRu
\right)
\]

其中：

- \(Q\) 决定控制器对状态误差的重视程度；
- \(R\) 决定控制器对转向输入大小的惩罚程度。

通过离散 Riccati 迭代求得矩阵 \(P\)，然后计算反馈增益：

\[
K=
(R+B^TPB)^{-1}B^TPA
\]

反馈控制为：

\[
\delta_{fb}=-Kx
\]

代码中使用：

```python
feedback = float(
    -(K @ error_state)[0, 0]
)
```

---

### 2.4 曲率前馈

如果仅使用：

\[
\delta=-Kx
\]

当车辆恰好位于路径中心且航向误差为零时，反馈项也为零。对于曲线路径，这会导致车辆没有提前建立所需转角。

因此加入曲率前馈：

\[
\delta_{ff}=L\kappa
\]

代码：

```python
feedforward = (
    vehicle.wheelbase
    * reference.curvature
)
```

最终控制律：

\[
\boxed{
\delta=-Kx+L\kappa
}
\]

最终转角还受车辆最大转角约束。

---

### 2.5 Student 与 Solution 验证

使用 `double_lane_change` 低速工况验证学生版实现。

```powershell
python run_experiment.py --algo lqr_kinematic --version student --route double_lane_change --speed-mode low --save-log --save-fig
python run_experiment.py --algo lqr_kinematic --version solution --route double_lane_change --speed-mode low --save-log --save-fig
```

实验结果：

| Metric                 | Student | Solution |
| ---------------------- | ------: | -------: |
| Steps                  |     191 |      191 |
| Reached goal           |    True |     True |
| Mean lateral error / m |   0.221 |    0.221 |
| Max lateral error / m  |   0.562 |    0.562 |
| Finish error / m       |   0.784 |    0.784 |

Student 与 Solution 的结果一致，因此运动学 LQR 实现验证通过。

---

### 2.6 Circle 三档速度实验

固定：

- 算法：Kinematic LQR；
- 车辆：`student_car`；
- 路线：`circle`；
- 圆弧半径：\(R=12\,m\)；
- 其他控制与车辆参数不变。

仅改变目标速度：

```powershell
python run_experiment.py --algo lqr_kinematic --version student --route circle --speed-mode low --save-log --save-fig
python run_experiment.py --algo lqr_kinematic --version student --route circle --speed-mode medium --save-log --save-fig
python run_experiment.py --algo lqr_kinematic --version student --route circle --speed-mode high --save-log --save-fig
```

对应输出目录：

```text
outputs/20260915_113151_lqr_kinematic_circle_low
outputs/20260915_113156_lqr_kinematic_circle_medium
outputs/20260915_113201_lqr_kinematic_circle_high
```

全程统计：

| Metric                 |   Low | Medium |  High |
| ---------------------- | ----: | -----: | ----: |
| Target speed / m/s     |   3.0 |    5.0 |   7.0 |
| Mean lateral error / m | 0.219 |  0.239 | 0.190 |
| Max lateral error / m  | 0.342 |  0.413 | 0.476 |
| Reached goal           |  True |   True |  True |

最大横向误差随速度提高：

\[
0.342\rightarrow0.413\rightarrow0.476\,m
\]

说明高速度下的峰值路径偏差更加明显。

---

### 2.7 LQR 稳态圆弧分析

使用与 PP 相同的筛选条件：

```text
abs(curvature - 1/12) < 0.005
```

并去除候选圆弧段首尾各 10% 的过渡记录。

结果如下：

| Metric                              |    Low |   Medium |   High |
| ----------------------------------- | -----: | -------: | -----: |
| Target speed / m/s                  |  3.000 |    5.000 |  7.000 |
| Mean actual speed / m/s             |  3.000 |    4.787 |  6.060 |
| Mean steer / rad                    | 0.2104 |   0.2115 | 0.1997 |
| Theory steer / rad                  | 0.2054 |   0.2054 | 0.2054 |
| Steer relative error                |  2.45% |    2.96% |  2.77% |
| Mean yaw rate / rad/s               | 0.2554 |   0.4092 | 0.5224 |
| Mean normal acceleration / m/s²    | 0.7500 |   1.9179 | 3.1622 |
| Mean lateral error / m              | 0.3294 |   0.3791 | 0.3255 |
| Max lateral error / m               | 0.3419 |   0.4130 | 0.4761 |
| Lateral error std / m               | 0.0099 |   0.0196 | 0.0708 |
| Max\(                               |  \beta | \) / rad | 0.1364 |
| Mean steer rate\(J_\delta\) / rad/s | 0.9726 |   0.5035 | 5.6422 |

从结果可见：

1. 三档速度下平均转角仍接近理论值 \(0.2054\,rad\)；
2. 高速时最大横向误差继续增大；
3. high 工况的横向误差标准差明显升高；
4. high 工况的最大 \(|\beta|\) 和转角变化率显著增大；
5. 因此 LQR 虽然能够保持较好的路径精度，但高速时控制动作明显更激烈。

特别是：

\[
J_\delta:
0.9726,\ 0.5035,\ 5.6422\,rad/s
\]

high 工况的平均转角变化率远高于 low / medium，说明高速时控制器为了压制误差进行了更加快速的转向修正。

---

### 2.8 PP 与 Kinematic LQR 对比

#### 2.8.1 全程横向误差

| Speed  | PP Mean / m |    LQR Mean / m | PP Max / m |     LQR Max / m |
| ------ | ----------: | --------------: | ---------: | --------------: |
| low    |       0.286 | **0.219** |      0.446 | **0.342** |
| medium |       0.288 | **0.239** |      0.505 | **0.413** |
| high   |       0.275 | **0.190** |      0.525 | **0.476** |

在三档速度下，LQR 的平均误差和最大误差均低于 PP。

全程平均误差相对 PP 大约降低：

- low：23.4%；
- medium：17.0%；
- high：30.9%。

---

#### 2.8.2 稳态圆弧横向误差

| Speed  | PP Mean / m |     LQR Mean / m | Improvement |
| ------ | ----------: | ---------------: | ----------: |
| low    |      0.4379 | **0.3294** |       24.8% |
| medium |      0.4724 | **0.3791** |       19.8% |
| high   |      0.4804 | **0.3255** |       32.3% |

LQR 在三档速度下均降低了稳态圆弧平均横向误差。

但是更高的精度伴随着更激烈的控制动作。

---

#### 2.8.3 控制平滑性比较

| Speed  | PP\(J_\delta\) / rad/s | LQR\(J_\delta\) / rad/s |
| ------ | ---------------------: | ----------------------: |
| low    |                 0.1941 |                  0.9726 |
| medium |                 0.0340 |                  0.5035 |
| high   |                 0.1091 |        **5.6422** |

特别是在 high 工况：

\[
\frac{J_{\delta,LQR}}{J_{\delta,PP}}
\approx
51.7
\]

说明 LQR 为获得更高跟踪精度，进行了远比 PP 更频繁或更剧烈的转向修正。

因此不能仅依据横向误差判断控制器性能，还需要同时评价：

- 跟踪精度；
- 控制平滑性；
- 侧偏响应；
- 是否出现转角饱和；
- 高速稳定性。

---

#### 2.8.4 侧偏角比较

| Speed | PP max \(|\beta|\) / rad | LQR max \(|\beta|\) / rad |
| --- | ---: | ---: |
| low | 0.1176 | 0.1364 |
| medium | 0.1230 | 0.1785 |
| high | 0.1162 | **0.3368** |

high 工况下 LQR 的最大侧偏角约为 PP 的 2.9 倍。

需要注意，本阶段使用的仍然是运动学自行车模型，因此该结果主要反映当前模型中的几何侧偏量和控制激烈程度。轮胎侧向力饱和等真实高速动力学效应需要在后续动力学模型中进一步分析。

---

### 2.9 LQR 阶段结论

本阶段完成了运动学 LQR 的：

- 状态空间误差模型；
- Riccati 迭代；
- 最优反馈增益计算；
- 曲率前馈；
- 学生版与参考版验证；
- circle 低、中、高三档速度实验；
- PP 与 LQR 初步对比。

实验显示：

1. 在本次 `double_lane_change` 和 `circle` 工况中，LQR 的横向跟踪误差均低于 PP；
2. LQR 的平均稳态转角与圆形路径理论值接近；
3. 随速度提高，LQR 的最大横向误差仍明显增加；
4. high 工况下横向误差标准差、侧偏角和转角变化率显著增加；
5. LQR 表现出“更高跟踪精度，但可能更激进”的控制特征；
6. 因此后续与 MPC 比较时，需要同时考虑误差和控制平滑性，而不能只比较单一的平均横向误差。

下一阶段将在相同路线和速度条件下实现并测试 **MPC**，形成 PP / LQR / MPC 的统一对比。

---

### 2.10 当前 LQR 相关文件

```text
vdm_lab/student/lqr_kinematic.py
analyze_circle_lqr.py
circle_speed_analysis_lqr.csv
```

结果图：

| Low                                     | Medium                                     | High                                     |
| --------------------------------------- | ------------------------------------------ | ---------------------------------------- |
| ![](docs\figures\kinematic\lqr_low.png) | ![](docs\figures\kinematic\lqr_medium.png) | ![](docs\figures\kinematic\lqr_high.png) |

---

## 3. Linear MPC

### 3.1 算法目标

在完成 Pure Pursuit 和 Kinematic LQR 后，本阶段实现 Linear Model Predictive Control（MPC）。

与前两种方法不同，MPC 不只计算当前一步控制，而是在每一个仿真时刻预测未来一段时间内的车辆状态，并同时优化一串未来控制输入：

\[
u_0,u_1,\ldots,u_{T-1}
\]

其中控制输入为：

\[
u=\begin{bmatrix}a\\\delta\end{bmatrix}
\]

MPC 通过最小化未来预测时域内的跟踪误差、控制输入大小和控制变化，同时满足车辆速度、加速度、转角与转角变化率约束，得到当前时刻最合适的控制命令。

最后只执行最优控制序列中的第一步：

\[
\boxed{a_0,\delta_0}
\]

下一仿真周期再根据新的车辆状态重新预测和优化，这就是滚动时域控制（Receding Horizon Control）。

---

### 3.2 预测参考轨迹

MPC 状态定义为：

\[
z=\begin{bmatrix}x\\y\\v\\\psi\end{bmatrix}
\]

对于预测时域 \(T\)，构造：

\[
z_{ref}\in\mathbb{R}^{4\times(T+1)}
\]

即未来 \(T+1\) 个参考状态。

参考点从当前最近路径点开始，根据预计行驶距离：

\[
\Delta s\approx v\Delta t
\]

向前选择对应路径点。

对于圆形路径，还需要对参考航向进行连续化处理。由于 \(+\pi\) 与 \(-\pi\) 实际表示相邻方向，如果直接相减可能产生接近 \(2\pi\) 的伪误差，因此使用 `pi_to_pi()` 保证预测时域内 yaw 连续。

---

### 3.3 线性化车辆模型

MPC 内部预测基于运动学自行车模型：

\[
x_{k+1}=x_k+v_k\cos\psi_k\Delta t
\]

\[
y_{k+1}=y_k+v_k\sin\psi_k\Delta t
\]

\[
v_{k+1}=v_k+a_k\Delta t
\]

\[
\psi_{k+1}=\psi_k+\frac{v_k}{L}\tan\delta_k\Delta t
\]

由于模型中含有 \(\sin\)、\(\cos\) 和 \(\tan\) 等非线性项，因此在当前预测轨迹附近进行一阶线性化：

\[
\boxed{z_{k+1}=Az_k+Bu_k+C}
\]

其中 \(A\)、\(B\)、\(C\) 随预测速度、yaw 和 steer 更新。因此本项目中的 Linear MPC 实际流程是：先预测未来状态，再沿预测轨迹局部线性化，然后求解线性约束下的二次规划问题。

---

### 3.4 MPC 目标函数

MPC 的代价函数由四部分组成：

\[
J=
\sum_{t=0}^{T-1}
\left[
(z_t-z_t^{ref})^TQ(z_t-z_t^{ref})
+u_t^TRu_t
\right]
\]

再加控制变化惩罚：

\[
\sum_{t=0}^{T-2}
(u_{t+1}-u_t)^TR_d(u_{t+1}-u_t)
\]

以及终端状态代价：

\[
(z_T-z_T^{ref})^TQ_f(z_T-z_T^{ref})
\]

各矩阵的作用为：

- \(Q\)：惩罚预测状态与参考状态之间的误差；
- \(R\)：惩罚过大的加速度和转角；
- \(R_d\)：惩罚相邻控制输入变化过快；
- \(Q_f\)：保证预测时域末端仍接近参考状态。

这使 MPC 能够显式平衡：

\[
\boxed{\text{跟踪精度}\quad\text{vs}\quad\text{控制平滑性}}
\]

---

### 3.5 车辆约束

优化过程中加入车辆物理限制：

\[
v_{min}\le v\le v_{max}
\]

\[
-a_{decel,max}\le a\le a_{max}
\]

\[
|\delta|\le\delta_{max}
\]

以及转角变化率约束：

\[
|\delta_{t+1}-\delta_t|
\le
\dot\delta_{max}\Delta t
\]

与 PP/LQR 计算后再进行 `clamp()` 不同，MPC 在求解阶段就知道车辆控制边界，因此优化结果本身已经考虑车辆的可执行能力。

该二次规划问题使用 OSQP 求解。

---

### 3.6 Iterative MPC 与滚动时域

由于线性模型 \(A,B,C\) 依赖未来预测状态，而未来状态又依赖控制输入，因此本实现采用迭代方式：

```text
上一时刻控制作为初始猜测
        ↓
predict_motion()
        ↓
得到预测轨迹 z_bar
        ↓
沿 z_bar 建立 A、B、C
        ↓
solve_linear_mpc()
        ↓
得到新的控制序列
        ↓
若控制变化仍较大，则再次预测与求解
        ↓
收敛或达到最大迭代次数
        ↓
只执行 a[0]、steer[0]
```

因此每一个仿真时刻都会重新利用最新车辆状态进行优化。

---

### 3.7 Student 与 Solution 验证

使用 `double_lane_change` 低速工况验证 Student MPC：

```powershell
python run_experiment.py --algo mpc --version student --route double_lane_change --speed-mode low --save-log --save-fig
python run_experiment.py --algo mpc --version solution --route double_lane_change --speed-mode low --save-log --save-fig
```

结果：

| Metric                 | Student | Solution |
| ---------------------- | ------: | -------: |
| Steps                  |     187 |      187 |
| Reached goal           |    True |     True |
| Mean lateral error / m |   0.177 |    0.177 |
| Max lateral error / m  |   0.450 |    0.450 |
| Finish error / m       |   0.088 |    0.088 |

Student 与 Solution 结果完全一致，因此 MPC 实现验证通过。

在同一 `double_lane_change + low` 工况下：

| Algorithm     | Mean lateral error / m | Max lateral error / m | Finish error / m |
| ------------- | ---------------------: | --------------------: | ---------------: |
| PP            |                  0.313 |                 0.756 |            0.879 |
| Kinematic LQR |                  0.221 |                 0.562 |            0.784 |
| MPC           |        **0.177** |       **0.450** |  **0.088** |

在这个单一工况下，MPC 的横向误差最低，但该结果不能直接推广到所有路线与参数设置。

---

### 3.8 Circle 三档速度实验

运行：

```powershell
python run_experiment.py --algo mpc --version student --route circle --speed-mode low --save-log --save-fig
python run_experiment.py --algo mpc --version student --route circle --speed-mode medium --save-log --save-fig
python run_experiment.py --algo mpc --version student --route circle --speed-mode high --save-log --save-fig
```

输出目录：

```text
outputs/20260915_120823_mpc_circle_low
outputs/20260915_120946_mpc_circle_medium
outputs/20260915_121034_mpc_circle_high
```

全程统计：

| Metric                 |   Low | Medium |  High |
| ---------------------- | ----: | -----: | ----: |
| Target speed / m/s     |   3.0 |    5.0 |   7.0 |
| Mean lateral error / m | 0.144 |  0.157 | 0.168 |
| Max lateral error / m  | 0.226 |  0.254 | 0.348 |
| Finish error / m       | 0.031 |  0.041 | 0.029 |
| Reached goal           |  True |   True |  True |

MPC 的平均和最大横向误差均随速度增加：

\[
0.144\rightarrow0.157\rightarrow0.168\,m
\]

\[
0.226\rightarrow0.254\rightarrow0.348\,m
\]

因此在 MPC 实验中，“速度升高后跟踪难度增加”的趋势非常清楚。

---

### 3.9 MPC 稳态圆弧分析

与 PP/LQR 使用相同筛选方法：

```text
abs(curvature - 1/12) < 0.005
```

并去除候选圆弧段首尾各 10% 的过渡记录。

| Metric                              |              Low |           Medium |             High |
| ----------------------------------- | ---------------: | ---------------: | ---------------: |
| Target speed / m/s                  |            3.000 |            5.000 |            7.000 |
| Mean actual speed / m/s             |            3.019 |            4.893 |            6.935 |
| Mean steer / rad                    |           0.2097 |           0.2107 |           0.2101 |
| Theory steer / rad                  |           0.2054 |           0.2054 |           0.2054 |
| Steer error                         |            2.07% |            2.57% |            2.30% |
| Mean yaw rate / rad/s               |           0.2555 |           0.4162 |           0.5900 |
| Mean normal acceleration / m/s²    |           0.7593 |           1.9949 |           4.0085 |
| Mean lateral error / m              | **0.2188** | **0.2520** | **0.3001** |
| Max lateral error / m               | **0.2260** | **0.2541** | **0.3468** |
| Lateral error std / m               |           0.0049 |          0.00065 |           0.0155 |
| Max\(                               |            \beta |         \) / rad |           0.1370 |
| Mean steer rate\(J_\delta\) / rad/s |           0.7728 |          0.00056 |           1.2451 |

稳态平均横向误差：

\[
0.2188\rightarrow0.2520\rightarrow0.3001\,m
\]

从 low 到 high 增加约 37.2%。

同时 high 工况实际平均速度达到：

\[
6.935\,m/s
\]

已经非常接近目标 \(7\,m/s\)。相应平均法向加速度：

\[
4.0085\,m/s^2
\]

也接近目标速度理论值：

\[
\frac{7^2}{12}=4.0833\,m/s^2
\]

需要注意，日志中的 `normal_accel` 本身由速度和路径曲率计算，因此使用同一实际速度与曲率得到的理论值属于内部一致性检查，而不是独立的车辆动力学验证。

---

### 3.10 三算法 Circle 精度比较

#### 全程平均与最大横向误差

| Speed  | PP Mean | LQR Mean |        MPC Mean | PP Max | LQR Max |         MPC Max |
| ------ | ------: | -------: | --------------: | -----: | ------: | --------------: |
| low    |   0.286 |    0.219 | **0.144** |  0.446 |   0.342 | **0.226** |
| medium |   0.288 |    0.239 | **0.157** |  0.505 |   0.413 | **0.254** |
| high   |   0.275 |    0.190 | **0.168** |  0.525 |   0.476 | **0.348** |

在当前 circle 实验中，三档速度的误差排序均为：

\[
\boxed{\text{MPC}<\text{LQR}<\text{PP}}
\]

这里只表示当前实验中横向误差的大小，不表示算法在所有场景下的普遍优劣。

#### 稳态圆弧平均横向误差

| Speed  | PP / m | LQR / m |          MPC / m |
| ------ | -----: | ------: | ---------------: |
| low    | 0.4379 |  0.3294 | **0.2188** |
| medium | 0.4724 |  0.3791 | **0.2520** |
| high   | 0.4804 |  0.3255 | **0.3001** |

MPC 相比 PP 的稳态平均误差约降低：

- low：50.0%；
- medium：46.6%；
- high：37.5%。

MPC 相比 LQR 约降低：

- low：33.6%；
- medium：33.5%；
- high：7.8%。

high 工况下 MPC 仍具有最低平均误差，但相对于 LQR 的优势已经明显缩小。

---

### 3.11 三算法控制平滑性比较

使用稳态圆弧中的平均转角变化率 \(J_\delta\)：

| Speed  |       PP / rad/s | LQR / rad/s |       MPC / rad/s |
| ------ | ---------------: | ----------: | ----------------: |
| low    | **0.1941** |      0.9726 |            0.7728 |
| medium |           0.0340 |      0.5035 | **0.00056** |
| high   | **0.1091** |      5.6422 |            1.2451 |

high 工况下：

\[
J_{\delta,LQR}=5.6422
\]

而：

\[
J_{\delta,MPC}=1.2451
\]

MPC 比 LQR 低约 77.9%，说明在当前高速圆弧工况下，MPC 在获得更低跟踪误差的同时，也明显抑制了方向盘快速变化。

但 PP high 的 \(J_\delta\) 仍最低，因此不能简单把“控制最平滑”也归给 MPC。当前实验更适合总结为：

```text
PP  ：控制最简单、较平滑，但误差较大
LQR ：误差较小，但高速下控制可能非常激进
MPC ：误差最低，并明显改善 LQR 的高速控制平滑性
```

---

### 3.12 侧偏响应比较

high 工况最大 \(|\beta|\)：

| Algorithm | max \(|\beta|\) / rad |
| --- | ---: |
| PP | 0.1162 |
| LQR | **0.3368** |
| MPC | 0.1628 |

LQR high 的侧偏响应明显最大；MPC 高于 PP，但远低于 LQR。

由于当前主仿真植物仍是运动学自行车模型，这里的 \(\beta\) 主要反映模型中的几何侧偏和转向激烈程度。轮胎侧向力、摩擦极限以及高速失稳等真实动力学效应，需要在后续动力学模型实验中进一步分析。

---

### 3.13 为什么速度升高后跟踪更困难

Circle 实验给出了比较清楚的理论与实验对应关系。

固定曲率：

\[
\kappa=\frac{1}{R}
\]

理论稳态转角近似：

\[
\delta\approx\arctan(L\kappa)
\]

因此在固定半径下，稳态转角基本不随速度变化。三种算法的实验也都显示平均稳态转角约为 \(0.20\sim0.21\,rad\)。

但横摆角速度需求：

\[
\dot\psi\approx v\kappa
\]

随速度线性增加，而法向加速度：

\[
a_n=v^2\kappa
\]

随速度平方增加。

此外，对固定仿真步长 \(\Delta t\)：

\[
\Delta s\approx v\Delta t
\]

速度越高，同样一个控制周期内车辆前进距离越大，因此每米路径上的可用修正次数减少，误差更容易在控制器下一次修正前继续累积。

当前运动学模型还没有显式模拟轮胎侧向力饱和。因此“高速时轮胎更容易达到摩擦极限”属于后续动力学模型需要验证的现象，而不能由当前运动学实验直接证明。

---

### 3.14 MPC 阶段结论

本阶段完成了 Linear MPC 的：

- 预测时域参考轨迹构造；
- 非线性运动学模型局部线性化；
- 状态误差、控制输入、控制变化和终端状态代价；
- 速度、加速度、转角和转角变化率约束；
- OSQP 二次规划求解；
- iterative linear MPC；
- receding horizon 控制；
- Student / Solution 一致性验证；
- circle 三档速度实验；
- PP / LQR / MPC 精度与平滑性初步比较。

当前实验表明：MPC 在 `double_lane_change + low` 以及 circle 三档速度实验中都取得了最低的横向误差；在 high circle 工况中，其转角变化率明显低于 LQR，说明预测优化和控制变化惩罚能够改善高速控制激烈程度。但 PP 在部分工况下仍具有更小的转角变化率，因此三种算法之间存在跟踪精度、控制平滑性和计算复杂度之间的权衡。

下一阶段需要在 `right_angle`、`s_curve` 等更多路线和统一条件下继续比较三种算法，避免仅依据 circle 和单一路况得出过度泛化结论。

---

### 3.15 当前 MPC 相关文件

```text
vdm_lab/student/mpc.py
analyze_circle_mpc.py
circle_speed_analysis_mpc.csv
```

结果图：

| Low                               | Medium                               | High                               |
| --------------------------------- | ------------------------------------ | ---------------------------------- |
| ![](docs\figures\mpc\mpc_low.png) | ![](docs\figures\mpc\mpc_medium.png) | ![](docs\figures\mpc\mpc_high.png) |

---

## 4. 参数敏感性实验

> 待完成：PP 前视距离、车辆最大转角、轴距等变量分析。

---

## 5. 运动学 / 动力学模型对比

> 待完成。

---

## 6. GPX 实际路线导航

> 待完成：寝室到教室路线规划、地图叠加、最大偏差位置与用时分析。
