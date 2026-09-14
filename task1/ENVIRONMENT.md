# 任务 1 的 Python 环境

本目录对应 [仓库任务书的实验任务 1](../vdm_lab/tasks/README.md)：三算法、三路线对比，
以及车辆参数变化实验。入口见 [README.md](README.md)，报告见 [reports/report.md](reports/report.md)。

## 本机直接使用

本机已经准备好仓库根目录的 `.venv`，使用 Python 3.11.15 和 uv 管理依赖。
本次只整理作业目录，不移动这个已建环境，避免其激活脚本里的绝对路径失效，
也不重复安装一套依赖。它一直被原仓库忽略，不属于要提交的修改。

在仓库根目录执行：

```bash
source .venv/bin/activate
python --version
python -m unittest discover -s task1/tests -v
```

也可以不激活，直接用 `.venv/bin/python` 替代 `python`。用完执行 `deactivate` 退出。

## 在新机器创建环境

本项目不强制使用 Conda。Miniconda 本身包含 Conda，可以替代完整 Anaconda；
已有 uv 时也可以直接创建虚拟环境。固定依赖清单见 [requirements.lock.txt](requirements.lock.txt)。
下面将新环境也放在 `task1/` 内，在仓库根目录执行：

```bash
uv venv --python 3.11 --prompt vdm-lab task1/.venv
uv pip install --python task1/.venv/bin/python --link-mode hardlink --only-binary :all: -r task1/requirements.lock.txt
uv pip check --python task1/.venv/bin/python
source task1/.venv/bin/activate
```

若 uv 缓存与项目不在同一文件系统，可以将 `--link-mode hardlink` 改为 `copy`。
本机复用了已有 Python，未额外下载解释器。依赖组合在 Linux x86_64 / Python 3.11
下验证过，其他平台需要重新做运行检查。

已有 Miniconda 的组员也可以使用独立 Conda 环境：

```bash
conda create -n vdm-lab python=3.11 pip -y
conda activate vdm-lab
python -m pip install -r task1/requirements.lock.txt
```

同一个项目选择一种环境即可，不需要同时维护 Conda 和 `.venv`。
uv 创建的环境没有另装 pip；安装依赖使用 `uv pip install --python 环境中的python路径 包名`。

## 仿真、绘图与输出

```bash
python task1/scripts/run_task1.py --output task1/outputs/new_run --jobs 3
python task1/scripts/analyze_task1.py --input task1/outputs/new_run --output task1/outputs/new_report
```

批量脚本采用 `solution` 控制器和非交互式 Matplotlib 后端，不需要图形桌面。
它生成逐时刻 CSV、参考路径、JSON 指标、配置和静态图。新结果默认集中在
`task1/outputs/`，原始交付数据固定保留在 `task1/data/20260911_task1/`。

原仓库的 `run_experiment.py --animate` 实时动画入口仍可使用，但其默认输出规则仍属于原仓库，
不会被本目录改写。需要 GIF 时可使用原入口的 `--save-gif`；批量报告采用静态图已满足任务要求，
没有为每个正式案例重复导出 GIF。

## 验证与空间

- 已完整安装原 [requirements.txt](../requirements.txt) 的依赖，包括 MPC 所需的 CVXPY / OSQP。
- 23 个包的依赖兼容性检查通过，四种算法与图片/日志导出已验证。
- 正式实验数值与复现检查见 [tests/test_task1.py](tests/test_task1.py)。
- 初次环境检查数据已移动到 `task1/environment_checks/`，仅本地保留，不冒充正式实验。
- 本机现有环境约 442 MiB，包文件与 uv 缓存硬链接共享，目录大小不等于新增磁盘占用。
- 未额外安装 pandas、Jupyter、深度学习框架或 CUDA，也未清理其他项目的缓存。
- 本目录的 `.gitignore` 和 `.gitattributes` 只作用于 `task1/`，无需修改仓库根目录配置。
