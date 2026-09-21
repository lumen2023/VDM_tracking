# 任务二：圆形路径速度实验

本目录是本次作业的独立提交包。新增运行、统计和绘图代码全部集中在 `main.py`，不修改原有 `vdm_lab/` 和 `run_experiment.py`。

## 内容与代码来源

- `main.py`：批量实验、日志保存、统一圆弧筛选、总体/稳态/理论统计、比较图。
- `test_main.py`：统计口径及真实 PP 集成测试。
- `report.md`：实验设置、理论计算、实际结果、原因分析、局限。
- `ppt_outline.md`：约5分钟汇报的逐页提纲、讲稿和答辩准备。
- `results/baseline/`：9组基准实验的原始日志、配置、指标表与图片。

**控制器来源声明**：本作业实验复用教学仓库 `vdm_lab/solutions/` 中的 PP、运动学 LQR 和 Linear MPC 完整实现。新增贡献是任务二的实验组织、统一统计、理论验证、可视化和结果分析，不将原控制器声明为本次自行编写的算法。

## 环境和运行

需要 Python 3.10 或更新版本；本次实验实际使用 Python 3.12。运行依赖原教学仓库，保持 `task2/` 与 `vdm_lab/` 同级。**单独把 task2 移到任意目录不能独立运行**，老师可将提交目录放回同一教学仓库。环境目录不随提交包交付。

Windows PowerShell，从仓库根目录执行：

```powershell
py -3.12 -m venv task2/.venv
task2/.venv/Scripts/python.exe -m pip install -r task2/requirements-lock.txt
task2/.venv/Scripts/python.exe -B task2/main.py --output task2/results/my_run
```

若 `py -3.12` 不可用，用已安装的 Python 3.10+ 解释器创建环境，并按根目录 `requirements.txt` 安装依赖。锁定文件对应本次 Python 3.12 环境，旧版本可能需要重新解析兼容依赖。

Ubuntu/macOS：

```bash
python3 -m venv task2/.venv
task2/.venv/bin/python -m pip install -r requirements.txt
task2/.venv/bin/python -B task2/main.py --output task2/results/my_run
```

默认执行全部9组实验。`--output` 的路径若已存在，会拒绝运行，以保护原始结果。当前已保存 `results/baseline`，因此再次运行请指定新的结果目录。

只运行一个算法/速度，或只从已有日志重新分析：

```powershell
task2/.venv/Scripts/python.exe -B task2/main.py --algorithms pp --speeds low --output task2/results/pp_check
task2/.venv/Scripts/python.exe -B task2/main.py --analyze-only --output task2/results/baseline
task2/.venv/Scripts/python.exe -B -m unittest discover -s task2 -v
```

重分析会更新各组 `statistics.json`、汇总表与比较图，**不重新运行控制器，不覆盖原始轨迹、参考路径、配置及 metrics.json**。报告和讲稿使用随包的基准结果，新的实验需相应更新文本。

## 输出说明

| 文件 | 内容 |
|---|---|
| `manifest.json` | Python/依赖版本、模型源文件SHA-256、完整等价实验命令、完成/异常清单、求解器警告 |
| `runs/算法_速度/trajectory.csv` | 原始每步日志，SI单位 |
| `runs/算法_速度/reference_path.csv` | 原始参考路径，弧长列名为 `s_m` |
| `runs/算法_速度/config.json` | 本次全部车辆、仿真、控制器配置 |
| `runs/算法_速度/metrics.json` | 原框架总体指标，包含 `reached_goal` |
| `runs/算法_速度/statistics.json` | 当前新增统计的逐组版本，重分析时同步更新 |
| `runs/算法_速度/summary.png` | 原框架单次汇总图 |
| `overall_metrics.csv` | 总体误差、最大控制量、峰值阶段、到达情况 |
| `steady_metrics.csv` | 中间圆弧窗口统计、实际速度、侧偏角、转角变化率 |
| `theory_comparison.csv` | 标称速度与实际速度两种理论对照、相对误差 |
| `all_statistics.csv` / `analysis.json` | 所有新增统计 |
| `figures/` | 轨迹、误差、速度、转角、指标随速度变化、理论验证共6张比较图 |

CSV使用UTF-8 BOM，便于Excel打开；缺失统计留空而不是置零。异常实验写入manifest，其他组继续，脚本最终以非零退出码提示；未到终点与程序异常分别记录，不能混为一谈。

## 统一统计口径

1. 圆弧候选条件：`abs(curvature - 1/12) < 0.005`。
2. 按参考弧长去掉圆弧前后各20%，中间60%作为统一统计窗口。这是稳态候选区，仍需结合误差标准差和曲线判断是否稳定。
3. 平均/最大横向误差用绝对值；有符号均值和总体标准差另列，防止左右偏差相抵。
4. `J_delta = mean(abs(delta[k]-delta[k-1])/dt)`，仅使用选中且相邻的仿真步。
5. 相对误差：`abs(simulation-theory)/abs(theory)*100%`。理论为零时留空。
6. 实际速度修正：横摆角速度理论用 `mean(v)/R`，加速度理论用 `mean(v²)/R`，不把 `mean(v)²` 当成 `mean(v²)`。
7. `normal_accel` 是原日志的 `v² * reference.curvature`，是参考曲率需求量，不是独立测得的车辆法向加速度。额外的 `mean(v*yaw_rate)` 只在侧偏角近似不变时对应模型法向加速度近似，不能用来证明真实轮胎动力学。
8. 原默认后端仅限幅转角大小，没有统一执行转角速率限制。MPC约束预测时域内相邻转角，不能等同于全部控制器具有相同执行器速率约束。

## 提交建议

提交 `task2/` 内的源码、文档、锁定依赖和 `results/baseline/`。不提交 `.venv/`、`.cache/`、`__pycache__/`，也不需要提交其他课程任务的文件。若压缩包已生成，使用 `task2_submission.zip`。本轮只规划PPT，不包含正式PPTX，后续按提供的模板制作。
