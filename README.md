# NullPointer｜车辆动力学路径跟踪实验

> **Vehicle Dynamics Path-Tracking Project**
> 课程：车辆动力学与运动控制 ｜ 小组：NullPointer ｜ 状态：已完成 / **Completed**

本仓库实现并比较了 Pure Pursuit（PP）、运动学 LQR、动力学 LQR 和线性 MPC 四类路径跟踪控制器，并在标准测试路径、圆形稳态路径、导航重建路线及 GPX 实际路线中完成验证。
This repository implements and compares four path-tracking controllers—Pure Pursuit (PP), kinematic LQR, dynamic LQR, and linear MPC—on standard test paths, a steady-state circle, a reconstructed navigation route, and a real GPX route.

## 已完成工作 / Completed Work

| 工作项 / Item | 完成内容 / What was completed |
| --- | --- |
| 控制器实现 / Controllers | 补齐学生版 PP、运动学 LQR、动力学 LQR 和线性 MPC 的核心控制逻辑。 / Completed the core control logic in all four student implementations. |
| 标准场景验证 / Standard scenarios | 在双移线、直角弯、S 弯和圆形路径上完成统一仿真、轨迹记录和误差统计。 / Ran unified simulations with trajectory logging and error metrics on double-lane-change, right-angle, S-curve, and circle paths. |
| 理论验证 / Theory check | 在半径 12 m 圆形路径上验证了 \(r=v/R\) 与 \(a_n=v^2/R\) 的速度关系。 / Verified the speed relationships \(r=v/R\) and \(a_n=v^2/R\) on a 12 m-radius circle. |
| 导航路线实验 / Navigation route | 依据导航截图重建 868.9 m 的“寝室—教学楼”局部坐标路线，并完成跟踪与峰值误差定位。 / Reconstructed an 868.9 m dormitory-to-teaching-building route from navigation evidence and located peak tracking errors. |
| GPX 与地图 / GPX and map | 支持 GPX 重采样、离线 GeoJSON 底图叠加与结果保存；已验证 3.914 km GPX 路线。 / Added GPX resampling, offline GeoJSON map overlay, and reproducible output saving; validated on a 3.914 km GPX route. |
| MPC 改进 / MPC extension | 为 MPC 首步前轮转角加入变化率约束，比较执行器平滑性与跟踪误差。 / Added a first-step steering-rate constraint to MPC and compared actuator smoothness with tracking error. |

## 方法 / Methods

统一采用自行车模型、闭环路径跟踪和固定采样周期；每次实验输出轨迹、横向误差、航向误差、速度、控制输入和终点状态。
All experiments use a bicycle model, closed-loop path tracking, and a fixed sampling interval. Each run exports trajectories, lateral and heading errors, speed, control inputs, and terminal status.

| 控制器 / Controller | 方法概述 / Method |
| --- | --- |
| PP | 依据前视点几何关系计算转向。 / Computes steering from the geometry to a look-ahead point. |
| 运动学 LQR / Kinematic LQR | 对横向与航向误差的离散运动学模型进行 LQR 状态反馈。 / Applies LQR state feedback to a discrete kinematic error model. |
| 动力学 LQR / Dynamic LQR | 在控制状态中考虑侧偏角和横摆角速度。 / Includes sideslip and yaw-rate dynamics in the control state. |
| 线性 MPC / Linear MPC | 在线性化预测模型上优化状态误差、控制量和控制增量，并施加边界约束。 / Optimizes state error, control effort, and control increments on a linearized prediction model with bounds. |

## 关键结果 / Key Results

### 868.9 m 导航路线（5 m/s）/ 868.9 m Navigation Route (5 m/s)

四种控制器均到达终点。动力学 LQR 的平均绝对横向误差最低；运动学 LQR 的转向变化更激烈。
All four controllers reached the destination. Dynamic LQR achieved the lowest mean absolute lateral error, while kinematic LQR required more aggressive steering changes.

| 控制器 / Controller | 平均绝对横向误差 / Mean \(|e_y|\) | 最大绝对横向误差 / Max \(|e_y|\) | 平均转角变化率 / Mean \(|\dot\delta|\) |
| --- | ---: | ---: | ---: |
| PP | 0.049 m | 0.350 m | 0.032 rad/s |
| 运动学 LQR / Kinematic LQR | 0.035 m | 0.254 m | 0.655 rad/s |
| 动力学 LQR / Dynamic LQR | **0.010 m** | **0.090 m** | 0.026 rad/s |
| MPC | 0.122 m | 0.469 m | 0.314 rad/s |

在 7 m/s 下，运动学 LQR 的平均绝对横向误差升至 0.221 m，且转角达到 35° 限制；动力学 LQR 仍为 0.013 m，显示出对该路线高速工况更稳定的表现。
At 7 m/s, kinematic LQR rose to 0.221 m mean absolute lateral error and hit the 35° steering limit; dynamic LQR remained at 0.013 m, indicating more stable behavior on this route at the tested high speed.

### MPC 执行器约束对比（7 m/s 圆形路径）/ Actuator-Aware MPC (7 m/s Circle)

首步转角变化率约束将平均转角变化率从 0.665 降至 **0.357 rad/s**（约 **46%**），峰值从 1.735 降至 **0.785 rad/s**（约 **55%**）；两种方法均到达终点，平均绝对横向误差约为 0.166–0.168 m。
The first-step steering-rate constraint reduced mean steering rate from 0.665 to **0.357 rad/s** (about **46%**) and its peak from 1.735 to **0.785 rad/s** (about **55%**). Both methods reached the goal, with mean absolute lateral error around 0.166–0.168 m.

### GPX 路线验证（低速）/ GPX Route Validation (Low Speed)

在 3.914 km 的 `homework_route_1.gpx` 路线上，PP、运动学 LQR 和 MPC 均到达终点；平均横向误差分别为 **0.027 m、0.457 m、0.063 m**。
On the 3.914 km `homework_route_1.gpx` route, PP, kinematic LQR, and MPC all reached the goal, with mean lateral errors of **0.027 m, 0.457 m, and 0.063 m**, respectively.

## 实验可视化 / Experimental Visuals

### 导航路线轨迹与误差 / Navigation Trajectory and Error

![PP navigation trajectory and lateral error](outputs/nullpointer_completion/pp_navigation_tracking.png)

### 运动学与动力学 LQR 对比 / Kinematic vs. Dynamic LQR

![Kinematic and dynamic LQR comparison](outputs/nullpointer_completion/kinematic_dynamic_comparison.png)

### MPC 首步转角变化率约束 / MPC First-Step Steering-Rate Constraint

![Actuator-aware MPC comparison](outputs/mpc_rate_limit_highlight/mpc_rate_limit_dashboard.png)

## 快速复现 / Quick Reproduction

```powershell
pip install -r requirements.txt

# 标准场景 / standard scenario
python run_experiment.py --algo mpc --version student --route mixed_course --save-log --save-fig

# GPX + 离线地图 / GPX with offline map
python run_experiment.py --algo pp --version student --gpx data/gpx/homework_route_1.gpx --basemap geojson --basemap-file data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz --speed-mode low --save-log --save-fig
```

详细任务说明见 [课程任务书](vdm_lab/tasks/README.md)；完整实验报告见 [报告文件](deliverables/NullPointer_VDM_实验报告.docx)。原始数据、指标和图片位于 [`outputs/`](outputs/)；可用 `metrics.json` 和 `summary.png` 复核任一单次仿真。
See the [course task sheet](vdm_lab/tasks/README.md) for details and the [full report](deliverables/NullPointer_VDM_实验报告.docx) for the complete submission. Raw data, metrics, and figures are stored in [`outputs/`](outputs/); each individual run can be checked through `metrics.json` and `summary.png`.
