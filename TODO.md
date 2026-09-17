# 汇报之后要补的事项

明日答辩只展示思路、已有现象和卡点。下面是报告/幻灯片里承诺的后续工作，不要当成今晚已经完成。

统一口径仍为：`--version solution`、车辆 `student_car`、失败实验保留、公开材料匿名化寝室/教室。

## 组员 A · 速度与跟踪难度

- [ ] 补齐 `circle` 上 PP / 运动学 LQR / MPC × 低中高，共 9 组（`--save-log --save-fig`）
- [ ] 按 `|curvature - 1/12| < 0.005` 截取稳态圆弧，去掉切入/切出过渡
- [ ] 给出稳态 `steer`、`yaw_rate`、`normal_accel` 均值，并与理论值算相对误差
- [ ] 分开统计：切入瞬态最大横向误差 vs 圆中段稳态误差
- [ ] 记录高速是否转角饱和、振荡、未达目标速度、未到达终点
- [ ] 低速 vs 高速各留 1 个 GIF，其余用 `summary.png`

## 组员 B · 三个关键变量

- [ ] 把 `pp_base_lookahead` 扫参脚本留进仓库（CLI 没有该参数，需改 `LabConfig` 后调 `run_simulation()`）
- [ ] 完成 OAT：速度 `v`（3/5/7 m/s）、前视基数 `L_d`（1.5/3.0/6.0 m）、曲率（`circle` / `s_curve` / `right_angle`）
- [ ] 补 1 组交互：`high` × `pp_base_lookahead=1.5`
- [ ] 汇总表含 `reached_goal`、平均/最大横向误差、`J_delta`（转角变化率）
- [ ] 写出调参建议（高速加大前视；急弯不要只加速度）

## 组员 C · 运动学 / 动力学植物

- [ ] 实现 `DynamicBicycleBackend`（线性轮胎 \(F_y=C\alpha\)，状态含 \(v_y,r\)）
- [ ] 接到 `run_experiment.py` / `run_simulation(vehicle_backend=...)`，可切换运动学/动力学
- [ ] 同一 PP、`circle`、medium 上对比横向误差、`beta`、横摆
- [ ] 在报告中写清：现有 `lqr_dynamic` 只改控制器，默认植物仍是运动学
- [ ] 注明局限：线性轮胎、无摩擦圆、`dt=0.1 s`、低速用 0.5 m/s 地板

## 组员 D · 寝室到教室

- [ ] 用 BRouter 导出真实宿舍→教室 GPX，落在离线地图范围内（lon 118.792–118.837，lat 31.875–31.902）
- [ ] 与 GeoJSON 共用原点 `(118.8145, 31.8885)`，替换今晚的 `homework_route_1` 流程演示
- [ ] 同一路线、同一目标速度下补 PP / 运动学 LQR / MPC 三算法对比
- [ ] 统计用时、最大 \(|e_y|\) 及其经纬度、曲率、转角；图上标点
- [ ] 至少一组改进复测（降速或改控制器参数）
- [ ] 公开材料去掉精确寝室/教室名称和门牌

## 课程任务 1 里今晚没作为主线的部分

- [ ] 至少三条内置路线上 PP / 运动学 LQR / MPC 对比（不少于 9 组）
- [ ] `s_curve`、`mixed_course` 上补动态 LQR
- [ ] 任选 `lf` / `lr` / `max_steer` 做一组车辆参数变化实验
- [ ] 最大误差是出现在曲率峰值还是曲率突变后的滞后点（对着 `trajectory.csv` 写）

## RealCar（与 Python 动力学植物无关）

代码 TODO（PP-1…5、LQR-1…6）已在 `../RealCar/VDM_Class_TODO` 勾完；离线 42/42 通过。上车仍要做：

- [ ] 有 ROS/catkin 的环境里完整编译（本机无 ROS，离线通过 ≠ 实车可用）
- [ ] 学生 cpp 覆盖到车上工程并 `catkin_make --pkg driverless`
- [ ] launch 选 PP，低速跟完一条录制路径
- [ ] 同一路径再跑 LQR
- [ ] 实验结束用备份恢复车上二进制

## 报告成稿

- [ ] 把明日幻灯片上的口头结论写成正式章节（理论、设置、任务 1–3、改进、局限）
- [ ] 每张表写清完整命令和 `outputs/` 目录名
- [ ] 区分：实验现象、公式、原因判断；`normal_accel` 是 \(v^2\kappa_\mathrm{ref}\)，不是 IMU
- [ ] 写明仿真无灯控/行人/路权，用时不能当成真实通勤时间
