# VDM 路径跟踪仿真实验（学生作业复现与结果）

本仓库基于 [VDM_tracking](https://github.com/)（车辆动力学与运动控制课程实验框架，PP / LQR / MPC 路径跟踪）完成。本 README 记录了本次作业的**实验设计、运行方法、输出数据说明与结果汇总**，供提交与复现使用。

## 1. 作业要求

1. **分析并论证**：为什么车辆速度越快，路径跟踪的难度通常越大（结合理论分析与实验结果）；
2. **分析三个关键变量**（控制算法、目标速度、路径几何特征/曲率）变化对仿真结果的影响，完成仿真实验、可视化与数据分析。

## 2. 项目与实验概览

- 项目框架：`run_experiment.py`（统一仿真入口），控制器位于 `vdm_lab/solutions/`，车辆为仅前进自行车模型。
- 车辆参数组：`student_car`（轴距 2.5 m，最大转角 35°，最大加速度 2.0 m/s²，最大速度 12.0 m/s）。
- 路线：`mixed_course`（综合路线，含直线、缓弯、S 弯与换道；全长约 94.7 m，最大曲率 0.144 /m，最小半径约 6.9 m）。
- 实验共 **7 组**：

| # | 实验目录 | 算法 | 速度档 | 目标速度 |
|---|---------|------|--------|---------|
| 1 | `outputs/20260915_200948_pp_mixed_course_low` | PP | low | 4.0 m/s |
| 2 | `outputs/20260915_201035_lqr_kinematic_mixed_course_low` | LQR 运动学 | low | 4.0 m/s |
| 3 | `outputs/20260915_201042_lqr_dynamic_mixed_course_low` | LQR 动力学 | low | 4.0 m/s |
| 4 | `outputs/20260915_201053_mpc_mixed_course_low` | MPC | low | 4.0 m/s |
| 5 | `outputs/20260915_201214_pp_mixed_course_medium` | PP | medium | 7.0 m/s |
| 6 | `outputs/20260915_201217_pp_mixed_course_high` | PP | high | 9.0 m/s |
| 7 | `outputs/20260915_201231_pp_mixed_course_low` | PP | low | 4.0 m/s（重复，可重复性检验） |

## 3. 环境部署

```bash
conda create -n vdm-lab python=3.10 -y
conda activate vdm-lab
python -m pip install --upgrade pip
python -m pip install -r requirements.txt   # numpy scipy matplotlib cvxpy pillow
```

## 4. 实验运行命令（复现）

在仓库根目录执行，每次实验加上 `--save-log --save-fig` 即可得到与 `outputs/` 相同的 `trajectory.csv`、`metrics.json`、`reference_path.csv` 和 `summary.png`。

```bash
# 算法对比（低速，mixed_course）
python run_experiment.py --algo pp          --route mixed_course --speed-mode low    --save-log --save-fig
python run_experiment.py --algo lqr_kinematic --route mixed_course --speed-mode low --save-log --save-fig
python run_experiment.py --algo lqr_dynamic   --route mixed_course --speed-mode low --save-log --save-fig
python run_experiment.py --algo mpc           --route mixed_course --speed-mode low --save-log --save-fig

# 速度实验（PP，mixed_course，三档）
python run_experiment.py --algo pp --route mixed_course --speed-mode low    --save-log --save-fig
python run_experiment.py --algo pp --route mixed_course --speed-mode medium --save-log --save-fig
python run_experiment.py --algo pp --route mixed_course --speed-mode high   --save-log --save-fig
```

> 备注：`mixed_course` 三档目标速度为 low 4.0 / medium 7.0 / high 9.0 m/s（见 `vdm_lab/config/speed_profiles.py`）。

## 5. 输出文件说明

| 文件 | 说明 |
| --- | --- |
| `trajectory.csv` | 每仿真步的状态、控制量、β、横摆角速度、横向/航向误差、曲率、法向加速度 |
| `metrics.json` | 总体指标：平均/最大横向误差、终点误差、最大转角/加速度/法向加速度/侧偏角/横摆角速度、是否到达终点 |
| `reference_path.csv` | 控制器使用的重采样参考路径（里程、坐标、航向、曲率、目标速度） |
| `summary.png` | 轨迹、误差、速度、控制量汇总图 |

## 6. 结果汇总

### 6.1 速度对跟踪难度的影响（PP，mixed_course）

| 速度档 | 平均横向误差 /m | 最大横向误差 /m | 终点误差 /m | 最大法向加速度 /(m/s²) | 最大横摆角速度 /(rad/s) | 转角变化率 Jδ /(rad/s) | 到达终点 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| low（4 m/s） | 0.249 | 0.640 | 0.785 | 2.303 | 0.438 | 0.101 | 是 |
| medium（7 m/s） | 0.256 | 0.768 | 0.911 | 6.993 | 0.724 | 0.120 | 是 |
| high（9 m/s） | 0.253 | 0.845 | 0.890 | 11.403 | 0.891 | 0.114 | 是 |

**结论要点**：速度从 4 提升到 9 m/s（2.25 倍），最大法向加速度需求从 2.30 增至 11.40 m/s²（约 4.95 倍，近似按 v² 增长，理论 v²·κ_max = 2.30 / 7.05 / 11.66）；峰值横向误差从 0.640 增至 0.845 m，且均发生在同一急弯处（里程约 56 m，曲率约 0.128 /m）。平均横向误差变化不大，说明难度提升主要体现在**急弯处的瞬态/峰值误差**。

### 6.2 算法对比（低速，mixed_course）

| 算法 | 平均横向误差 /m | 最大横向误差 /m | 终点误差 /m | 最大转角 /rad | 转角变化率 Jδ /(rad/s) | |κ|-|e| 相关 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PP | 0.249 | 0.640 | 0.785 | 0.270 | 0.101 | 0.960 |
| LQR 运动学 | 0.183 | 0.414 | 0.729 | 0.372 | 0.511 | 0.631 |
| LQR 动力学 | 0.074 | 0.158 | 0.747 | 0.323 | 0.097 | −0.250 |
| MPC | 0.143 | 0.309 | 0.054 | 0.328 | 0.291 | 0.842 |

**结论要点**：低速下 LQR 动力学平均/最大误差最小，MPC 终点误差最小且加速度严格受限（max 2.0 m/s²），PP 误差最大但控制最平滑（Jδ 小）；LQR 运动学响应最快但转角变化率最大。

### 6.3 可重复性

PP 低速重复两次：平均横向误差 0.249 / 0.267 m，最大横向误差 0.640 / 0.660 m，差异约 3%–7%，结果可重复。

## 7. 报告

- 正式提交报告：`VDM路径跟踪仿真实验报告.docx`


## 8. 已知局限

- 仿真采用纯运动学/线性化模型，未建模轮胎非线性饱和、执行器延迟与真实道路限速；
- 高速档下车辆在急弯前会减速（峰值处实际速度略低于目标），因此实际最大法向加速度略低于 v²κ_max 理论值；
- 实验中三档速度均到达终点，未覆盖“失控/未到达”的极端速度场景。
