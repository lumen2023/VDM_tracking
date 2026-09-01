# 教师说明：VDM 路径跟踪实验

## 实验目标

学生需要掌握从参考路线到车辆控制量的完整闭环流程：

1. 生成仅前进参考轨迹 `x, y, yaw, curvature, s, target_speed`
2. 在轨迹上寻找最近点或前视目标点
3. 计算横向误差、航向误差和速度误差
4. 分别使用 PP、LQR、MPC 输出前轮转角
5. 使用统一纵向 PID 输出加速度
6. 通过 CSV、JSON、汇总图和 GIF 动图分析跟踪效果

## 与课程 PDF 的对应关系

课程材料 `VehicleDynamicsMobility_01_BicycleModel.pdf` 的重点已经接入代码：

- 运动学自行车模型图 `vdm_lab/KMLM.png` 对应 `vdm_lab/common/bicycle_model.py` 中的 `beta`、`psi_dot` 和连续时间导数；这两个量同时写入 `trajectory.csv` 的 `beta`、`yaw_rate` 字段。
- 圆周运动图 `vdm_lab/exp_cm.png` 对应日志中的 `curvature` 和 `normal_accel = v^2 * curvature`，并进入 `metrics.json` 的 `max_normal_acceleration_mps2`。
- 车辆几何和动力学参数 `lf`、`lr`、`m`、`Iz`、`Cf`、`Cr` 统一在 `vdm_lab/config/vehicle_params.py` 中设置。
- 三个课程题目位于 `vdm_lab/tasks/README.md`，可作为课后作业或实验报告模板。

## 参数与路线

车辆参数集中在 `vdm_lab/config/vehicle_params.py`，课程默认使用 `student_car`。学生调试车辆响应时只改参数文件或新增参数组，算法文件只从 `config.vehicle` 读取参数。

目标速度分为 `low`、`medium`、`high` 三档，默认使用低速 `low`。档位解析在 `vdm_lab/config/speed_profiles.py`，每条路线的具体速度值写在 `vdm_lab/config/routes.py`，便于教师根据课程难度调整。

路线集中在 `vdm_lab/config/routes.py`，当前包括：

- `double_lane_change`：强化双移线，横向位移更大、换道距离更短
- `right_angle`：强化直角弯，转弯半径更小
- `s_curve`：S 弯道
- `circle`：直线切入、一整圈圆和直线驶出，用于把 `kappa`、`delta_f`、`psi_dot`、`a_n` 与课程公式做定量核对
- `mixed_course`：综合路线

课堂建议先固定低速档位，让同一算法依次跑多条路线，再比较 `metrics.json` 中的误差和控制输入。学生掌握后再切换到中速或高速，观察横向误差和转角约束的变化。

## 学生版 TODO 对照

学生文件位于 `vdm_lab/student/`，参考答案位于 `vdm_lab/solutions/`。

| 算法 | 学生文件 | 关键填写点 | 参考答案 |
| --- | --- | --- | --- |
| PP | `pure_pursuit.py` | 前视距离、目标点搜索、`alpha`、几何转角公式 | `solutions/pure_pursuit.py` |
| LQR 运动学 | `lqr_kinematic.py` | 离散误差模型、Riccati 迭代、误差状态、曲率前馈 | `solutions/lqr_kinematic.py` |
| LQR 动力学 | `lqr_dynamic.py` | 车速保护、二自由度动力学矩阵、离散化、动力学前馈 | `solutions/lqr_dynamic.py` |
| MPC | `mpc.py` | 预测时域参考轨迹、线性化模型、QP 目标、速度和输入约束、滚动优化 | `solutions/mpc.py` |

## 建议课堂安排

1. 第一课时：运行完整答案版，解释统一仿真循环和数据记录。
2. 第二课时：学生填写 PP，观察前视距离对横向误差和路径切角的影响。
3. 第三课时：学生填写运动学 LQR，对比 PP 的几何控制和 LQR 的误差反馈控制。
4. 第四课时：讲解动力学 LQR 的车速保护和侧偏刚度参数，作为进阶内容。
5. 第五课时：学生填写 MPC，重点理解预测、线性化、目标函数和约束。

## 评分建议

推荐使用 `outputs/<时间戳>_<算法>/metrics.json` 中的指标评分：

- 必须项：`min_speed_mps >= 0.0`，`reached_goal == true`
- 跟踪精度：`mean_lateral_error_m`、`max_lateral_error_m`
- 终点效果：`finish_error_m`
- 控制合理性：`max_steer_rad`、`max_acceleration_mps2`、`max_normal_acceleration_mps2`
- 课程模型理解：`max_side_slip_beta_rad`、`max_yaw_rate_radps`

如果学生改动公共框架导致日志字段缺失，建议要求其恢复统一接口后再评分。

## GIF 制作

保留原仓库动图展示的教学方式，统一使用新入口生成：

```bash
python run_experiment.py --algo pp --version solution --route double_lane_change --save-gif
python run_experiment.py --algo lqr_kinematic --version solution --route right_angle --save-gif
python run_experiment.py --algo lqr_dynamic --version solution --route s_curve --save-gif
python run_experiment.py --algo pp --version solution --route circle --save-gif
python run_experiment.py --algo mpc --version solution --route mixed_course --save-gif
```

生成文件为 `outputs/<时间戳>_<算法>/animation.gif`。MPC 动图会额外显示当前预测轨迹。

如果需要边看实时动画边录制 GIF，使用：

```bash
python run_experiment.py --algo pp --version solution --route double_lane_change --animate --save-gif
```

实时动画和 GIF 默认开启历史车辆姿态虚影，且虚影会从起点开始一直保留到当前帧，便于学生观察控制量改变后车辆姿态如何逐步变化。课堂演示时可调节采样间隔；如果画面太密，也可以临时限制虚影数量：

```bash
python run_experiment.py --algo pp --route double_lane_change --save-gif --ghost-stride 8
python run_experiment.py --algo pp --route double_lane_change --save-gif --ghost-count 12 --ghost-stride 8
python run_experiment.py --algo pp --route double_lane_change --save-gif --no-history-ghosts
```

原仓库中与本实验相关的 PP、LQR、MPC 演示 GIF 已保留在 `vdm_lab/assets/original_gifs/`，可作为历史效果对照；正式实验验收建议以当前代码重新生成的 `animation.gif` 为准。

## 教学边界

本实验不考察全局路径规划、避障、泊车、倒车、混合 A*、状态格规划或 Frenet 轨迹优化。路线已经由平滑样条给出，学生的任务是完成车辆路径跟踪控制器。
