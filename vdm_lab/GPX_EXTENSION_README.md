# VDM_tracking GPX 扩展

这个补丁基于现有 `lumen2023/VDM_tracking` 结构设计，不创建第二套控制器框架。

## 核心原则

原平台已经统一了：

```text
Path
VehicleState
ControlCommand
ControllerReference
```

因此 GPX 只负责：

```text
GPX / WGS84 经纬度
        ↓
局部 East / North 米制坐标
        ↓
现有 Path
        ↓
ReferenceTracker
        ↓
PP / LQR / MPC
```

已有控制算法文件不需要修改。

---

## 安装方法

把本补丁中的文件按照相同相对路径覆盖/复制到原仓库：

```text
VDM_tracking/
├── run_experiment.py                         # 覆盖
├── vdm_lab/common/types.py                  # 覆盖
├── vdm_lab/common/simulation.py             # 覆盖
├── vdm_lab/common/logging.py                # 覆盖
├── vdm_lab/common/visualization.py          # 覆盖
├── vdm_lab/common/gpx.py                    # 新增
├── vdm_lab/common/vehicle_backend.py        # 新增
└── data/gpx/homework_route_1.gpx            # 示例
```

原来的：

```text
vdm_lab/common/path.py
vdm_lab/solutions/pure_pursuit.py
vdm_lab/solutions/lqr_kinematic.py
vdm_lab/solutions/lqr_dynamic.py
vdm_lab/solutions/mpc.py
```

都不需要改。

---

## 运行原有路线

## 跨平台命令说明

`run_experiment.py` 的参数在 Ubuntu Bash 和 Windows PowerShell 中一致，
例如 `--algo`、`--gpx`、`--basemap-file` 和 `--waypoint-ds`。差异仅在多行
命令的续行符：本文件标为 `bash` 的示例使用 `\`；PowerShell 必须使用
反引号 `` ` ``，且反引号后不能有空格。

默认 PP + GPX + GeoJSON 示例已经分别封装为：

```bash
bash scripts/run_pp.sh
```

```powershell
.\scripts\run_pp.ps1
```

也可在任意支持 Python 的平台直接运行 `python examples/run_pp_demo.py`，
无需处理 shell 续行符。

仍然和以前一样：

```bash
python run_experiment.py \
  --algo pp \
  --route mixed_course \
  --animate
```

---

## 运行 GPX + Pure Pursuit

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/homework_route_1.gpx \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --animate
```

### 叠加 OpenStreetMap 平面地图

GPX 文件保留了 WGS84 经纬度，可以直接和 BRouter 使用的
OpenStreetMap 底图对齐：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/homework_route_1.gpx \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --basemap osm \
  --basemap-zoom 16 \
  --animate
```

可选参数：

```text
--basemap-opacity 0.72    # 底图透明度 0..1
--basemap-padding 100     # 路线外额外加载范围 [m]
--basemap-retries 3       # 单张瓦片网络重试次数
--basemap-strict          # 下载失败时终止（默认是警告后继续）
```

地图瓦片只会在第一次需要时下载，后续动画帧、GIF 和汇总图
会复用内存或本地缓存。图中会保留 `© OpenStreetMap contributors`
归属标记。

如果当前网络无法访问默认 OSM 服务，可指定你有权使用的
XYZ 地图服务：

```bash
--basemap-url 'https://your-tile-service/{z}/{x}/{y}.png'
```

程序会自动读取 `HTTPS_PROXY` / `HTTP_PROXY` 环境变量。默认情况下，
单张瓦片下载失败不再中断车辆仿真，未加载区域显示为浅灰色。

注意：BRouter/OSM 和 GPX 使用 WGS84。如果换用高德（GCJ-02）或
百度（BD-09）底图，必须先进行坐标系转换。

### 离线底图

如果仿真机无法稳定访问在线地图，可在另一台能联网的机器上
生成可移植 `.npz` 地图包。瓦片地址必须来自明确允许离线使用/
下载的服务：

```bash
python prepare_offline_basemap.py \
  --gpx data/gpx/homework_route_1.gpx \
  --tile-url 'https://your-authorized-service/{z}/{x}/{y}.png' \
  --attribution '地图数据提供者要求的归属文本' \
  --zoom 16 \
  --output data/maps/homework_route_1_z16.npz
```

将生成的文件复制到仿真机，运行：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/homework_route_1.gpx \
  --basemap local \
  --basemap-file data/maps/homework_route_1_z16.npz \
  --target-speed 8 \
  --animate
```

离线包保存原始 WGS84 地理边界，加载时会按当前 GPX 的原点重新
计算局部米制范围。因此不依赖生成包时的本地坐标原点。

OpenStreetMap Foundation 的公共 `tile.openstreetmap.org` 服务不允许为
离线使用预下载瓦片，不要将它用作上述 `--tile-url`。

### 直接使用 OSM GeoJSON 离线地图

对于已下载的 `.geojson` 或 `.geojson.xz` 矢量地图，不需要再生成
`.npz` 或访问任何网络服务：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/homework_route_1.gpx \
  --basemap geojson \
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --animate
```

对上述文件名，程序会把两个边界角点的几何中心自动设为
局部坐标原点：

```text
origin_lon = (118.792 + 118.837) / 2 = 118.8145
origin_lat = (31.875 + 31.902) / 2 = 31.8885
```

也可手动覆盖：

```bash
--map-origin 118.8145 31.8885
```

GeoJSON 会在内存中一次性栅格化，默认最长边为 2200 像素；
可用 `--basemap-max-pixels` 调整。道路、建筑、水系、绿地、铁路和
部分公交/信号点会作为平面场景背景绘制。

对于公里级路径，默认 `--view-mode auto` 会自动切换到：

```text
主窗口：车辆附近局部 90m × 90m
右上角：整条路线 overview
```

也可以手动：

```bash
--view-mode full
```

或者：

```bash
--view-mode follow --follow-radius 60
```

---

## 保存结果

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/homework_route_1.gpx \
  --target-speed 8 \
  --save-log \
  --save-fig \
  --save-gif
```

会额外得到：

```text
reference_path.csv
gpx_overview.png
```

其中 `reference_path.csv` 同时保存：

```text
s
x
y
yaw
curvature
target_speed
lat
lon
elevation
```

方便以后和 RTK/GNSS 实车数据对应。

---

## GPX 路线时间

原平台 `SimulationConfig.max_time=90s` 对 4km 路线不够。

GPX 模式默认自动按：

```text
路线长度 / 巡航速度 + 60s
```

扩展最大仿真时间。

也可以自己指定：

```bash
--max-time 700
```

指定后关闭自动扩展。

---

## 为什么不直接对 GPX 使用 CubicSpline

BRouter 等地图导出的 GPX 本质上是一条折线。

本扩展默认使用：

```text
累积弧长 + 线性重采样
```

因为对非常稀疏的 GPX 点直接做 CubicSpline，可能在道路拐角产生明显超调，
把参考轨迹平滑到道路之外。

如果原始 GPX 最大点间距非常大，程序会给出 warning。

注意：

> 重采样只能加密已有折线，不能恢复 GPX 文件本来没有提供的道路几何。

因此正式实验建议导出更高密度的 GPX。

---

# 外部车辆模型 / TruckSim 接口

新增：

```text
vdm_lab/common/vehicle_backend.py
```

原平台默认：

```python
KinematicBicycleBackend
```

行为和原仓库一致。

以后接 TruckSim 时：

```python
from vdm_lab.common.vehicle_backend import ExternalVehicleBackend
from vdm_lab.common.types import VehicleState, ControlCommand


class TruckSimBackend(ExternalVehicleBackend):

    def reset(self, initial_state, config):
        # 1. reset TruckSim
        # 2. read TruckSim X/Y/Yaw/V
        return VehicleState(
            x=...,
            y=...,
            yaw=...,
            v=...,
        )

    def dynamics_terms(self, state, command, config):
        yaw_rate = ...
        beta = ...
        return yaw_rate, beta

    def step(self, state, command, config, dt):
        # command.steer: 前轮转角 rad
        # command.acceleration: m/s^2

        # 1. steering / acceleration unit conversion
        # 2. send inputs to TruckSim
        # 3. advance one step
        # 4. read new state

        next_state = VehicleState(
            x=...,
            y=...,
            yaw=...,
            v=...,
        )

        applied_command = ControlCommand(
            acceleration=command.acceleration,
            steer=command.steer,
        )

        return next_state, applied_command
```

然后：

```python
path, records, predictions = run_simulation(
    controller,
    config=config,
    vehicle_backend=TruckSimBackend(...),
)
```

PP / LQR / MPC 都不用改。

---

# 数据流

```text
                         内置路线
                            │
                            ▼
                    generate_reference_path
                            │
                            │
GPX ──> generate_gpx_path ──┤
                            ▼
                           Path
                            │
                    ReferenceTracker
                            │
                            ▼
                   PP / LQR / MPC
                            │
                      ControlCommand
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
  KinematicBicycleBackend        TruckSimBackend
              │                           │
              └─────────────┬─────────────┘
                            ▼
                       VehicleState
                            │
                            └──── feedback
```
