# GPX + 离线 GeoJSON 底图路径跟踪任务

本任务使用真实 GPX 路线（WGS84 经纬度）叠加离线 OSM GeoJSON 平面地图，
运行 PP / 运动学 LQR / MPC 完成路径跟踪。全程无需联网，Ubuntu 与 Windows
均可完成。

> 部署、完整参数和在线 OSM 底图说明见根目录 [../../README.md](../../README.md)；
> GPX 坐标转换、离线 `.npz` 底图包和外部车辆模型见
> [../GPX_EXTENSION_README.md](../GPX_EXTENSION_README.md)。

## 0. 任务目标

1. 理解 GPX 经纬度 → 局部米制坐标的转换，以及底图与路线共用同一坐标原点的意义；
2. 使用离线 GeoJSON 底图运行默认演示，确认路线与地图正确叠加；
3. 在同一 GPX 路线、同一目标速度下对比 PP、运动学 LQR、MPC 的精度、平滑性与任务完成情况；
4. 定位最大横向偏差出现的位置，并结合底图与曲率分析原因。

## 1. 所需文件与地图覆盖范围

| 文件 | 说明 |
| --- | --- |
| `data/gpx/demo_route.gpx` | 默认演示路线，约 2.9 km，22 个轨迹点 |
| `data/gpx/homework_route_1.gpx` | 作业路线 1，58 个轨迹点 |
| `data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz` | 离线 OSM 矢量地图（xz 压缩） |

离线地图覆盖范围（WGS84）：

```text
经度 118.792 ~ 118.837
纬度  31.875 ~  31.902
```

程序会从文件名自动识别这四个边界，并把几何中心设为局部坐标原点：

```text
origin = ((118.792 + 118.837) / 2, (31.875 + 31.902) / 2)
       = (118.8145, 31.8885)
```

局部坐标中 `+x` 朝东、`+y` 朝北，单位米。GPX 路径与 GeoJSON 底图必须使用
同一个原点才能正确叠加；程序会自动保证这一点。

## 2. 环境准备（Ubuntu 与 Windows）

依赖见根目录 [../../requirements.txt](../../requirements.txt)：

```text
numpy>=1.23,<2.0
matplotlib
scipy
cvxpy
pillow
```

`.xz` 解压使用 Python 标准库 `lzma`，无需额外安装。

Ubuntu：

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Windows（PowerShell）：

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

注意事项：

- Ubuntu 部分发行版只有 `python3` 命令、没有 `python`；如提示
  `python: command not found`，把下文所有 `python` 换成 `python3`。
- Windows 建议用 `py` 启动器；不可用时用完整的 `python` 命令。
- `cvxpy` 用于 MPC，首次安装较慢属正常现象。

## 3. 快速启动（推荐，跨平台）

三种方式任选其一，均在仓库根目录执行：

```bash
# Ubuntu / WSL
bash scripts/run_pp.sh
```

```powershell
# Windows PowerShell
.\scripts\run_pp.ps1
```

```bash
# 任意平台（Windows / Ubuntu / macOS），无需处理任何 shell 语法
python examples/run_pp_demo.py
```

如果 PowerShell 报执行策略错误，改用：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_pp.ps1
```

这三种方式都会运行「PP + `demo_route.gpx` + 离线 GeoJSON」演示并弹出动画窗口。

## 4. 手动运行与命令差异

单行命令在 Bash 和 PowerShell 中完全相同（单引号两边都支持）：

```bash
python run_experiment.py --algo pp --gpx data/gpx/demo_route.gpx --basemap geojson --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' --target-speed 8 --waypoint-ds 1.0 --animate
```

多行命令需要续行符，Bash 用 `\`，PowerShell 用反引号 `` ` ``（续行符后不能有空格）：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/demo_route.gpx \
  --basemap geojson \
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --animate
```

```powershell
python run_experiment.py `
  --algo pp `
  --gpx data/gpx/demo_route.gpx `
  --basemap geojson `
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' `
  --target-speed 8 `
  --waypoint-ds 1.0 `
  --animate
```

### 关于逗号文件名的引号

地图文件名里有两个逗号：`118.792,31.875_118.837,31.902`。不同 shell 行为不同：

- **PowerShell**：逗号是数组分隔符，不加引号会被拆成多段，因此
  `--basemap-file` 的值必须用单引号包起来。
- **Bash**：逗号不是特殊字符，加不加单引号都可以（示例统一加引号，保持一致）。
- **cmd.exe**：单引号会被当作文件名的一部分传进去，导致找不到文件。请改用
  PowerShell，或直接运行 `python examples/run_pp_demo.py`，不要在 cmd 里粘贴
  带单引号的命令。

## 5. 实验任务

### 任务 A：默认演示与地图对齐

运行第 3 节的演示，确认车辆沿真实道路移动、地图正确叠加。观察长路线下
默认 `--view-mode auto` 的「主窗口 + 右上角全局小窗」效果，并尝试：

```bash
python run_experiment.py --algo pp --gpx data/gpx/demo_route.gpx --basemap geojson --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' --target-speed 8 --waypoint-ds 1.0 --view-mode full --animate
```

### 任务 B：三算法对比

在相同路线、相同目标速度下运行 PP、运动学 LQR、MPC，并保存日志和汇总图。

Bash：

```bash
for algo in pp lqr_kinematic mpc; do
  python run_experiment.py \
    --algo "$algo" \
    --gpx data/gpx/homework_route_1.gpx \
    --basemap geojson \
    --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
    --target-speed 8 \
    --waypoint-ds 1.0 \
    --save-log \
    --save-fig
done
```

PowerShell：

```powershell
foreach ($algo in @('pp','lqr_kinematic','mpc')) {
  python run_experiment.py `
    --algo $algo `
    --gpx data/gpx/homework_route_1.gpx `
    --basemap geojson `
    --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' `
    --target-speed 8 `
    --waypoint-ds 1.0 `
    --save-log `
    --save-fig
}
```

### 任务 C：验证坐标原点与地图覆盖范围

- 观察程序启动时打印的 `map_origin_lon=... map_origin_lat=...`，应与
  `(118.8145, 31.8885)` 一致；
- 用 `--map-origin 118.8145 31.8885` 手动指定，结果应与自动推断一致
  （程序保证 GPX 与底图共用同一原点）；
- 说明离线地图只覆盖经度 `118.792 ~ 118.837`、纬度 `31.875 ~ 31.902`；
  自建 GPX 若超出该范围，路线将无法叠加到底图上。

### 任务 D：指标统计与最大偏差定位

使用 `--save-log` 后，每次运行会生成一个 `outputs/` 下的实验目录：

| 文件 | 内容 |
| --- | --- |
| `trajectory.csv` | 每个仿真步的车辆状态、误差与控制输入 |
| `reference_path.csv` | 参考路径的里程、坐标、曲率、经纬度（列名 `s_m`、`lat_deg`、`lon_deg`） |
| `metrics.json` | 总体误差与控制量指标 |
| `summary.png` | 路径、误差、速度、控制量汇总图 |
| `gpx_overview.png` | GPX 路线 + 底图的全局概览 |

参考 [README.md](README.md) 任务 3.4 的统计脚本计算「用时」与「最大偏差位置」，
注意只有 `reached_goal` 为 `true` 时，用时才能代表真正跑完全程。

## 6. 分析要求

1. 三个算法的到达终点情况、平均/最大横向误差、最大转角与最大法向加速度如何排序？
2. 最大横向偏差出现在急弯、连续弯、交叉口还是路线起终点？该处曲率是否较大，
   或曲率符号是否刚发生变化？
3. 底图只提供空间背景，仿真未建模交通灯、行人、真实限速；说明这对
   「真实通行时间」的影响。
4. 把 `--target-speed` 调低（例如 `5`）后，偏差大小与出现位置如何变化？
   至少完成一组改进前后对比。

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| 报错「无法从 GeoJSON 文件名识别边界」 | 文件名不含 `_西,南_东,北` 格式边界，用 `--map-origin LON LAT` 手动指定 |
| 底图分辨率不够 | 提高 `--basemap-max-pixels`（默认 2200，最小 256） |
| 出现「GPX 存在较稀疏路段」警告 | 正常提示；重采样不能恢复缺失道路几何，建议导出更高密度 GPX |
| 路线未到终点仿真就结束 | 增加 `--max-time`（GPX 默认按路线长度自动延长，手动指定后关闭自动延长） |
| 只想看算法、不要底图 | `--basemap none` |
| Windows 找不到 `python` | 改用 `py` 或 `python3` |
| PowerShell 执行策略阻止脚本 | `powershell -ExecutionPolicy Bypass -File .\scripts\run_pp.ps1` |

## 8. 提交内容

- 三个算法在 `homework_route_1.gpx` 上的统一指标表（含 `reached_goal`）；
- 至少一张带底图的 `summary.png` 或 `gpx_overview.png`；
- 最大偏差位置说明（时间、经纬度、曲率、速度、转角）；
- 坐标原点验证结果与地图覆盖范围说明；
- 一组调低速度后的改进前后对比。
