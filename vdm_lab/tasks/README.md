# 课程任务：自行车模型与路径跟踪

本任务对应 `VehicleDynamicsMobility_01_BicycleModel.pdf` 中的 Bicycle Model、Circular Motions 和 Path Tracking 内容。实验目标不仅是跑通代码，还要把课程公式、控制算法、仿真变量和真实路线联系起来。

本文件在原有课程框架下，将实验组织为三个递进任务：

1. 比较 PP、LQR 和 MPC 在不同场景、不同任务中的表现；
2. 在圆形路径上研究速度升高后跟踪难度增大的原因；
3. 自主规划寝室到教室的 GPX 路线，统计行驶时间和最大偏差位置并完成分析。

环境部署、完整命令行参数、内置路线、GPX 与离线地图的通用说明见主文档 [../../README.md](../../README.md)。

GPX 路线叠加离线 GeoJSON 底图的专项实验（含 Ubuntu / Windows 兼容说明）见 [gpx_geojson_task.md](gpx_geojson_task.md)。

> Windows 用户请不要把下面标为 `bash` 的多行命令直接粘贴到 PowerShell：
> Bash 的续行符是 `\`，PowerShell 的续行符是反引号 `` ` ``。默认 PP +
> GPX + GeoJSON 示例可分别通过 `bash scripts/run_pp.sh` 或
> `.\scripts\run_pp.ps1` 启动（均从仓库根目录执行）；也可跨平台运行
> `python examples/run_pp_demo.py`。

## 0. 实验前理论准备

### 0.1 运动学自行车模型

![Kinematic bicycle model](../KMLM.png)

根据图中的 `C`、`A`、`B`、`lf`、`lr`、`R`、`Rf`、`Rr`、`β`、`δf`，在后轮不转向 `δr = 0` 的假设下，有：

```text
beta = atan(lr / (lf + lr) * tan(delta_f))

x_dot = v * cos(psi + beta)
y_dot = v * sin(psi + beta)
psi_dot = v / (lf + lr) * tan(delta_f) * cos(beta)
```

实验前应完成以下准备：

1. 从几何关系推导侧偏角 `beta` 和横摆角速度 `psi_dot`；
2. 说明车辆位置 `(x, y)`、航向角 `psi`、速度 `v` 和前轮转角 `delta_f` 的含义；
3. 在以下代码中找到公式、车辆参数和状态更新对应的位置：

   - `vdm_lab/common/bicycle_model.py`
   - `vdm_lab/common/vehicle.py`
   - `vdm_lab/config/vehicle_params.py`

4. 在 `trajectory.csv` 中找到 `steer`、`beta` 和 `yaw_rate`，说明它们分别对应 `delta_f`、`beta` 和 `psi_dot`。

### 0.2 圆周运动、曲率与法向加速度

![Circular motion example](../exp_cm.png)

圆周运动的基本关系为：

```text
rho = 1 / kappa
a_n = v^2 / rho = v^2 * kappa
```

对于小侧偏、近似稳态的运动学自行车模型，还有：

```text
delta_f ≈ atan((lf + lr) * kappa)
psi_dot ≈ v * kappa
```

内置 `circle` 路线包含一段直线切入、半径 `R = 12 m` 的完整圆弧和一段直线驶出。驶出段用于避免闭环起点和终点重合导致仿真提前结束。圆弧段的理论曲率为：

```text
kappa = 1 / R = 1 / 12 ≈ 0.0833 1/m
```

### 0.3 输出文件和主要指标

使用 `--save-log` 后，每次实验会生成日志目录。分析时主要使用：

| 文件 | 用途 |
| --- | --- |
| `metrics.json` | 一次实验的总体误差、控制量和是否到达终点 |
| `trajectory.csv` | 每个仿真时刻的车辆状态、误差与控制输入 |
| `reference_path.csv` | 参考路径的位置、累计里程、曲率以及 GPX 经纬度 |
| `summary.png` | 路径、误差、速度和控制量汇总图 |
| `animation.gif` | 动态跟踪过程；可观察超调、振荡和历史车辆姿态 |

报告至少关注下列指标：

| 指标 | 含义 |
| --- | --- |
| `reached_goal` | 是否满足到达终点条件；未到达时不能把仿真结束时间当作有效用时 |
| `mean_lateral_error_m` | 全程平均横向误差 |
| `max_lateral_error_m` | 全程最大横向误差 |
| `mean_heading_error_rad` | 平均航向误差 |
| `max_steer_rad` | 最大前轮转角 |
| `max_normal_acceleration_mps2` | 最大法向加速度 |
| `max_side_slip_beta_rad` | 最大侧偏角 |
| `max_yaw_rate_radps` | 最大横摆角速度 |

### 0.4 公平对比原则

同一组对比实验必须使用相同的路线、车辆、速度档和仿真设置，只改变待比较的算法或速度。每张表中写清完整命令或参数；若某个算法未到达终点，应保留结果并分析失败原因，不能只删除失败实验。

---

## 实验任务 1：PP、LQR 和 MPC 在不同场景中的表现

### 1.1 实验目的

比较 Pure Pursuit（PP）、运动学 LQR、动态 LQR 和 MPC 面对不同路径几何特征时的精度、稳定性、控制平滑性与任务完成情况，并解释不同算法表现差异的原因。

### 1.2 场景选择

建议至少选取三类具有不同特征的路线：

| 路线 | 主要特征 | 重点观察 |
| --- | --- | --- |
| `double_lane_change` | 连续换道、曲率方向快速改变 | 瞬态响应、超调与回正速度 |
| `right_angle` | 急转弯、曲率突变明显 | 转弯前后的最大误差、是否饱和 |
| `s_curve` | 连续正负曲率 | 振荡、相位滞后和控制平滑性 |
| `mixed_course` | 直线、缓弯和急弯混合 | 综合适应能力与任务完成率 |

必做实验为 PP、运动学 LQR 和 MPC 在至少三条路线上的对比，共不少于 9 组。动态 LQR 建议在 `s_curve` 和 `mixed_course` 上补充测试，并结合其速度与轮胎模型假设解释结果。

### 1.3 运行方法

单次运行示例：

```bash
python run_experiment.py --algo pp --route double_lane_change --speed-mode medium --save-log --save-fig --save-gif
python run_experiment.py --algo lqr_kinematic --route right_angle --speed-mode medium --save-log --save-fig --save-gif
python run_experiment.py --algo mpc --route s_curve --speed-mode medium --save-log --save-fig --save-gif
```

也可以批量运行：

```bash
for route in double_lane_change right_angle s_curve; do
  for algo in pp lqr_kinematic mpc; do
    python run_experiment.py \
      --algo "$algo" \
      --route "$route" \
      --speed-mode medium \
      --save-log \
      --save-fig
  done
done
```

补充动态 LQR：

```bash
python run_experiment.py --algo lqr_dynamic --route s_curve --speed-mode medium --save-log --save-fig
python run_experiment.py --algo lqr_dynamic --route mixed_course --speed-mode medium --save-log --save-fig
```

### 1.4 数据整理

将每次实验的 `metrics.json` 汇总为下表：

| 路线 | 算法 | 到达终点 | 平均横向误差 / m | 最大横向误差 / m | 最大转角 / rad | 最大法向加速度 / m/s² | 最大横摆角速度 / rad/s |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| double_lane_change | PP |  |  |  |  |  |  |
| double_lane_change | LQR kinematic |  |  |  |  |  |  |
| double_lane_change | MPC |  |  |  |  |  |  |
| right_angle | PP |  |  |  |  |  |  |
| right_angle | LQR kinematic |  |  |  |  |  |  |
| right_angle | MPC |  |  |  |  |  |  |
| s_curve | PP |  |  |  |  |  |  |
| s_curve | LQR kinematic |  |  |  |  |  |  |
| s_curve | MPC |  |  |  |  |  |  |

除最大值外，建议利用 `trajectory.csv` 计算转角变化率均值，用于描述控制平滑性：

```text
J_delta = mean(abs(steer[k] - steer[k-1]) / dt)
```

### 1.5 分析要求

报告中回答以下问题：

1. 哪个算法的平均误差最小，哪个算法的最大误差最小？两者是否总是相同？
2. 在换道、直角弯和 S 弯中，算法排名是否发生变化？路径几何特征如何影响结果？
3. PP 的前视距离、LQR 的状态反馈、MPC 的预测与约束分别如何影响超调和控制平滑性？
4. 最大误差是否出现在曲率最大处，还是出现在曲率变化后的滞后位置？结合 `trajectory.csv` 和 `summary.png` 说明。
5. 修改 `student_car` 的 `lf`、`lr` 或 `max_steer` 后，PP 与 LQR 的 `beta`、`yaw_rate` 和横向误差发生了什么变化？任选一个参数完成附加实验。

### 1.6 提交内容

- 自行车模型的简要推导以及代码变量对应关系；
- 不少于 9 组实验的统一指标表；
- 至少三张具有代表性的 `summary.png`，或一组能清楚显示差异的 `animation.gif`；
- 对各算法适用场景、优缺点和失败情况的分析；
- 一组车辆参数变化实验及结论。

---

## 实验任务 2：圆形路径上为什么速度越快跟踪越难

### 2.1 实验目的

在曲率固定的圆形路径上隔离速度因素，验证 `a_n = v^2 * kappa`、`psi_dot ≈ v * kappa` 和转向约束之间的关系，并比较 PP、LQR、MPC 在低、中、高速下的稳态误差和瞬态响应。

圆形路径低速参考效果如下：

| PP 圆形路径 | LQR 运动学圆形路径 | MPC 圆形路径 |
| --- | --- | --- |
| ![PP circle](../assets/demo_gifs/pp_circle_low.gif) | ![LQR kinematic circle](../assets/demo_gifs/lqr_kinematic_circle_low.gif) | ![MPC circle](../assets/demo_gifs/mpc_circle_low.gif) |

### 2.2 理论预估

`student_car` 默认轴距为 `lf + lr = 2.5 m`。在半径 `R = 12 m` 的圆弧上，理论稳态转角近似为：

```text
delta_f ≈ atan(2.5 / 12) ≈ 0.2054 rad ≈ 11.77 deg
```

三档默认目标速度对应的理论值为：

| 速度档 | `v` / m/s | `psi_dot ≈ v/R` / rad/s | `a_n = v²/R` / m/s² |
| --- | ---: | ---: | ---: |
| low | 3.0 | 0.2500 | 0.7500 |
| medium | 5.0 | 0.4167 | 2.0833 |
| high | 7.0 | 0.5833 | 4.0833 |

从 `3 m/s` 提升到 `7 m/s` 时，速度约为原来的 `2.33` 倍，但法向加速度需求约为原来的 `5.44` 倍。所需稳态几何转角近似不变，横摆响应和轮胎侧向作用却必须更快建立；同时，固定采样周期内车辆前进距离更长，控制器留给误差修正的时间更短。这些因素共同导致高速跟踪更困难。

### 2.3 运行方法

每种算法都运行低、中、高三档，共 9 组：

```bash
for algo in pp lqr_kinematic mpc; do
  for speed in low medium high; do
    python run_experiment.py \
      --algo "$algo" \
      --route circle \
      --speed-mode "$speed" \
      --save-log \
      --save-fig \
      --save-gif
  done
done
```

若要只研究速度变化，也可以先固定 PP：

```bash
python run_experiment.py --algo pp --route circle --speed-mode low --save-log
python run_experiment.py --algo pp --route circle --speed-mode medium --save-log
python run_experiment.py --algo pp --route circle --speed-mode high --save-log
```

### 2.4 稳态圆弧数据选择

打开 `trajectory.csv`，选取满足下式的数据作为圆弧候选段：

```text
abs(curvature - 1/12) < 0.005
```

再去掉刚进入和即将驶出圆弧的过渡记录，分别统计：

- `steer`、`yaw_rate` 和 `normal_accel` 的平均值；
- `lateral_error` 的平均值、最大值和标准差；
- `beta` 的最大值；
- 转角变化率 `J_delta`。

将仿真均值与下列理论值对比，并给出相对误差：

```text
delta_f ≈ atan((lf + lr) * kappa)
psi_dot ≈ v * kappa
a_n = v^2 * kappa
relative_error = abs(simulation - theory) / abs(theory) * 100%
```

### 2.5 结果表

| 算法 | 速度档 | 到达终点 | 平均横向误差 / m | 最大横向误差 / m | 稳态平均转角 / rad | 稳态平均横摆角速度 / rad/s | 稳态平均法向加速度 / m/s² |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| PP | low |  |  |  |  |  |  |
| PP | medium |  |  |  |  |  |  |
| PP | high |  |  |  |  |  |  |
| LQR kinematic | low |  |  |  |  |  |  |
| LQR kinematic | medium |  |  |  |  |  |  |
| LQR kinematic | high |  |  |  |  |  |  |
| MPC | low |  |  |  |  |  |  |
| MPC | medium |  |  |  |  |  |  |
| MPC | high |  |  |  |  |  |  |

### 2.6 分析要求

1. 验证法向加速度是否近似随速度平方增长，并解释偏离理论值的原因。
2. 为什么理论稳态转角随速度近似不变，实际横向误差却可能随速度增大？
3. 区分切入圆弧时的瞬态最大误差和圆弧中段的稳态误差，判断问题主要来自哪一阶段。
4. 检查高速时是否出现转角饱和、振荡、相位滞后、速度未达到目标或未到达终点。
5. PP、LQR 和 MPC 中，哪个算法在高速圆弧上的精度和平滑性最好？该结论是否与任务 1 的其他路线一致？

### 2.7 提交内容

- 三个算法、三档速度的 9 组总体指标表；
- 稳定圆弧段的 `steer`、`yaw_rate`、`normal_accel` 和横向误差统计表；
- 理论值、仿真均值及其相对误差；
- 低速与高速的轨迹图或动画对比；
- 对“速度越快，跟踪难度越高”的公式分析和控制角度解释。

---

## 实验任务 3：自主规划寝室到教室的路线并跟踪

### 3.1 实验目的

使用实际道路数据规划一条从寝室到教室的路线，将 GPX 经纬度转换为以离线地图中心为原点的局部米制坐标，使用跟踪算法完成测试，并统计任务用时、最大偏差及其发生位置。

> 隐私提示：报告公开提交前，建议将寝室和教室名称匿名化，并检查截图、GPX 文件和经纬度是否包含不希望公开的精确位置。

### 3.2 路线准备

1. 在 BRouter Web 或其他路径规划工具中选择起点和终点，按实际通行方向规划汽车路线；
2. 导出 WGS84 坐标系的 GPX 文件，保存为：

   ```text
   data/gpx/dorm_to_classroom.gpx
   ```

3. 尽量保留较密集的路线点，运行时用 `--waypoint-ds 1.0` 重采样；
4. 确认 GPX 路线位于离线地图范围内。目前地图文件覆盖范围约为：

   ```text
   longitude: 118.792 ～ 118.837
   latitude:   31.875 ～ 31.902
   ```

5. 当前离线地图中心和局部坐标原点为：

   ```text
   (longitude, latitude) = (118.8145, 31.8885)
   ```

局部坐标中的 `(0, 0)` 对应该中心点；GPX 路径和 GeoJSON 地图必须使用同一个原点，才能正确叠加。

6. GPX路径规划工具地址：https://brouter.de/brouter-web

### 3.3 在离线地图上运行

先用 PP 检查路线和地图是否对齐：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/dorm_to_classroom.gpx \
  --basemap geojson \
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
  --map-origin 118.8145 31.8885 \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --animate \
  --save-log \
  --save-fig
```

地图对齐后，在相同目标速度下测试三个算法：

```bash
for algo in pp lqr_kinematic mpc; do
  python run_experiment.py \
    --algo "$algo" \
    --gpx data/gpx/dorm_to_classroom.gpx \
    --basemap geojson \
    --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
    --map-origin 118.8145 31.8885 \
    --target-speed 8 \
    --waypoint-ds 1.0 \
    --save-log \
    --save-fig
done
```

若只需要检查算法、不显示底图，可以保留同一 GPX 并改用：

```bash
python run_experiment.py --algo pp --gpx data/gpx/dorm_to_classroom.gpx --basemap none --target-speed 8 --waypoint-ds 1.0 --animate
```

### 3.4 统计用时和最大偏差位置

每次运行后，把 `OUTPUT_DIR` 改为该次实验实际生成的日志目录，然后执行：

```bash
OUTPUT_DIR='outputs/请替换为实际实验目录' python - <<'PY'
import csv
import json
import os
from pathlib import Path

output_dir = Path(os.environ['OUTPUT_DIR'])

with (output_dir / 'trajectory.csv').open(encoding='utf-8') as file:
    trajectory = list(csv.DictReader(file))

with (output_dir / 'reference_path.csv').open(encoding='utf-8') as file:
    reference = list(csv.DictReader(file))

with (output_dir / 'metrics.json').open(encoding='utf-8') as file:
    metrics = json.load(file)

if not trajectory or not reference:
    raise SystemExit('日志为空，无法统计')

peak = max(trajectory, key=lambda row: abs(float(row['lateral_error'])))
target_index = min(int(peak['target_index']), len(reference) - 1)
nearest_reference = reference[target_index]

duration = float(trajectory[-1]['time']) - float(trajectory[0]['time'])
route_length = float(reference[-1]['s_m'])
average_progress_speed = route_length / duration if duration > 0 else float('nan')

print(f"reached_goal: {metrics['reached_goal']}")
print(f'duration_s: {duration:.3f}')
print(f'route_length_m: {route_length:.3f}')
print(f'average_progress_speed_mps: {average_progress_speed:.3f}')
print(f"max_abs_lateral_error_m: {abs(float(peak['lateral_error'])):.3f}")
print(f"peak_time_s: {float(peak['time']):.3f}")
print(f"vehicle_xy_m: ({float(peak['x']):.3f}, {float(peak['y']):.3f})")
print(f"nearest_route_s_m: {float(nearest_reference['s_m']):.3f}")
print(
    'nearest_route_lon_lat: '
    f"({nearest_reference['lon_deg']}, {nearest_reference['lat_deg']})"
)
print(f"speed_at_peak_mps: {float(peak['speed']):.3f}")
print(f"curvature_at_peak_1pm: {float(peak['curvature']):.6f}")
print(f"steer_at_peak_rad: {float(peak['steer']):.6f}")
PY
```

注意：

- 只有 `reached_goal` 为 `true` 时，`duration_s` 才能作为“到达教室所用时间”；
- `vehicle_xy_m` 是相对地图中心点 `(118.8145, 31.8885)` 的局部坐标；
- `nearest_route_lon_lat` 是最大偏差时车辆目标点附近的经纬度，可用于在地图上标记位置；
- 最大横向误差有正负方向，排序时应取绝对值，报告中可以同时保留原始符号；
- `route_length / duration` 是沿参考路线的平均推进速度，不等同于车辆瞬时速度的算术平均值。

### 3.5 结果表

| 算法 | 到达终点 | 路线长度 / m | 用时 / s | 平均推进速度 / m/s | 平均横向误差 / m | 最大横向误差 / m | 最大偏差时刻 / s | 最大偏差位置（经纬度） |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| PP |  |  |  |  |  |  |  |  |
| LQR kinematic |  |  |  |  |  |  |  |  |
| MPC |  |  |  |  |  |  |  |  |

如果某个算法未到达终点，将“用时”记为“未完成”，同时另列仿真持续时间、最后到达的参考路径里程 `s_m` 和失败位置。

### 3.6 最大偏差位置分析

将脚本得到的最大偏差点标记在 `summary.png` 或地图截图上，并回答：

1. 最大偏差发生在急弯、连续弯、交叉口、路线起点，还是终点附近？
2. 该位置的曲率是否较大，或曲率符号是否刚刚发生变化？
3. 最大偏差与车辆速度、前轮转角、转角饱和和控制延迟有什么关系？
4. 三种算法的最大偏差是否出现在同一位置？若不同，说明各自的响应特征。
5. 调低目标速度或调整控制器参数后，最大偏差的位置和大小是否发生变化？至少选择一种改进方法复测。
6. 实际道路底图只用于空间背景，本仿真没有自动建模交通灯、行人、路权和真实道路限速；说明这一限制对“实际通勤时间”的影响。

### 3.7 提交内容

- 自主规划的 GPX 路线说明，包括匿名化的起终点、路线长度和地图范围；
- PP、运动学 LQR、MPC 的统一参数与完整运行命令；
- 三种算法的任务完成情况、用时和误差统计表；
- 标出最大偏差位置的地图或轨迹图，以及该点的时间、经纬度、曲率、速度和转角；
- 最大偏差成因分析和至少一组改进前后对比；
- 对仿真结果能否代表真实寝室到教室通勤时间的边界说明。

---

## 4. 实验报告建议结构

1. **理论与模型**：自行车模型推导、圆周运动关系、代码变量映射；
2. **实验设置**：软件环境、车辆参数、控制器参数、路线与速度；
3. **任务 1**：多算法、多场景对比结果与讨论；
4. **任务 2**：圆形路线速度实验、理论值验证与高速困难原因；
5. **任务 3**：自主路线、地图对齐、用时与最大偏差位置分析；
6. **改进实验**：参数修改、速度调整或控制器优化；
7. **结论与局限**：总结算法适用条件，并说明仿真与真实交通环境的差异。

## 5. 最终检查清单

- [ ] 所有对比实验仅改变一个目标变量，其他设置保持一致；
- [ ] 每组结果都记录了完整命令和输出目录；
- [ ] 指标表中包含 `reached_goal`，失败实验没有被忽略；
- [ ] 圆形路径的理论值和仿真值使用相同单位；
- [ ] GPX 路线与离线地图使用相同原点 `(118.8145, 31.8885)`；
- [ ] 最大偏差按横向误差绝对值查找，并记录对应经纬度；
- [ ] 图表具有标题、坐标轴、单位和图例；
- [ ] 报告区分了实验现象、公式推导和原因判断；
- [ ] 公开材料已处理寝室位置等隐私信息。
