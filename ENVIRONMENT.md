# 本地实验环境

## 使用方案

Miniconda 可以运行本项目：它包含 Conda，无需安装完整的 Anaconda。
不过，仓库只推荐通过 Conda 隔离 Python 环境，代码本身不依赖 Conda。
本机已有 uv 和 Python 3.11，因此本次直接复用它们创建项目内的 `.venv`，
没有另装 Miniconda，也没有修改系统 Python 或 shell 启动配置。

本次准备环境的目标是支持仓库任务书中的
[实验任务 1：PP、LQR 和 MPC 在不同场景中的表现](vdm_lab/tasks/README.md#实验任务-1pplqr-和-mpc-在不同场景中的表现)。
按任务书完成至少三条路线上的 PP、运动学 LQR、MPC 对比（不少于 9 组），
以及一组车辆参数变化实验、可视化和分析；动力学 LQR 为建议补充项。
不再以“三个关键变量变化”为任务定义。当前运行的示例仅用于验证环境。

- Python：3.11.15，复用本机已有解释器；README 推荐的 3.10 并非强制版本。
- 依赖：完整安装原 `requirements.txt`，包括 MPC 所需的 CVXPY / OSQP。
- 已验证的版本清单：[requirements.lock.txt](requirements.lock.txt)。
- 未额外安装 Jupyter、pandas、深度学习框架或 CUDA。
- 包文件通过硬链接与 uv 缓存共享，不要手动修改 `.venv` 内的第三方包文件。
- `.venv/` 和 `outputs/` 已被原仓库的 `.gitignore` 排除，不会提交到 Git。

## 日常使用

在仓库根目录执行：

```bash
source .venv/bin/activate
python --version
python run_experiment.py --algo pp --version solution --route double_lane_change --save-log --save-fig
```

结果位于终端打印的 `outputs/<时间戳>_pp_double_lane_change_low/`：

- `trajectory.csv`：逐时刻状态、控制量与误差。
- `reference_path.csv`：参考路径。
- `metrics.json`：汇总指标。
- `summary.png`：汇总图。

需要 GIF 时追加 `--save-gif`；需要实时窗口时追加 `--animate`。
GIF 导出会比单纯保存静态图慢，批量实验时可只给代表性结果生成 GIF。
实时动画结束后窗口会保留，关闭窗口后程序才继续退出。
请使用 `--version solution`；`student` 目录包含尚未填写的教学代码。

没有图形桌面或远程运行时，可以使用非交互式绘图后端：

```bash
MPLBACKEND=Agg python run_experiment.py --algo pp --version solution --route double_lane_change --save-log --save-fig
```

这条命令不弹出窗口，但仍可保存 PNG；也可以追加 `--save-gif`。
不要将 `MPLBACKEND=Agg` 与需要弹出窗口的 `--animate` 一起使用。

用完退出环境：

```bash
deactivate
```

也可以不激活环境，直接用 `.venv/bin/python` 替代上述 `python`。

## 在另一台机器复现

以下命令适用于已安装 uv 的 Linux 环境，在尚未创建 `.venv` 时执行：

```bash
uv venv --python 3.11 --prompt vdm-lab .venv
uv pip install --python .venv/bin/python --link-mode hardlink --only-binary :all: -r requirements.lock.txt
uv pip check --python .venv/bin/python
```

本机创建环境时使用了 `--no-python-downloads`，确认没有额外下载 Python。
另一台机器若没有 Python 3.11，uv 可以自动下载。版本清单仅在本机的
Linux x86_64 / Python 3.11 环境验证过，其他平台需重新做运行检查。
如果缓存和项目不在同一文件系统，硬链接可能不可用，此时改用
`--link-mode copy`，代价是额外的磁盘占用。

本环境没有单独安装 pip；安装额外依赖时使用
`uv pip install --python .venv/bin/python 包名`。原始依赖范围仍在
`requirements.txt`，固定版本清单用于复现本次已验证的组合。

如果组员已经使用 Miniconda，也可以在他们的独立环境中复现版本：

```bash
conda create -n vdm-lab python=3.11 pip -y
conda activate vdm-lab
python -m pip install -r requirements.lock.txt
```

同一个项目不需要同时维护 Conda 环境和 `.venv`。

## 本次环境检查

验证日期：2026-09-11。所有仿真使用原仓库的 `solution` 控制器，未修改算法或车辆参数。

- `uv pip check`：23 个包的依赖兼容性检查通过。
- NumPy、SciPy、Matplotlib、CVXPY、Pillow 导入正常。
- OSQP：简单约束优化求解通过。
- PP：低速双移线完整运行，到达终点；CSV、JSON、PNG 导出及 180 帧 GIF 解码检查通过。
- 运动学 LQR：低速直角弯完整运行，到达终点。
- 动力学 LQR：低速 S 弯完整运行，到达终点。
- MPC：综合路线运行 1 秒仿真时间，11 个采样点，成功生成预测轨迹及日志、静态图。
- TkAgg：图形窗口创建、绘制与关闭检查通过，项目实时动画的短时运行也通过。

MPC 检查命令中的 `--max-time 1` 是为了快速验证依赖和求解器，
其 `reached_goal=False` 是主动提前结束的结果，不代表算法完成率或跟踪性能。
这次没有测试 MPC 完整路线，也没有测试 GPX 或在线地图功能。

本次验证文件位于 `outputs/20260911_221353_*`；这些文件仅用于环境检查，
不能代替任务 1 要求的统一设置下的完整对比实验。

## 空间说明

安装后的 `.venv` 目录约 442 MiB（实测值会随字节码缓存略有增长）。
该数字包含与 uv 缓存硬链接共享的包文件，不等于新增磁盘占用；
不要简单把 `.venv` 和 uv 缓存的目录大小相加。
本次没有清理其他项目的 uv 缓存，也没有下载地图数据。

## 任务 1 的正式实验

正式实验已与上述环境检查数据分开保存。入口为
[reports/task1/README.md](reports/task1/README.md)，完整分析见
[reports/task1/report.md](reports/task1/report.md)。
原始数据位于 `outputs/task1/20260911_task1/`，包括 9 组主实验、
4 组新增参数实验和 2 组动力学 LQR 补充实验。
批量脚本为 `scripts/run_task1.py`，分析脚本为 `scripts/analyze_task1.py`，
均使用本环境已有依赖，无需另装 pandas 或 Jupyter。
