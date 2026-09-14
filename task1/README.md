# 任务 1：独立作业目录

先看 [完整中文报告](reports/report.md)，或查看 [核心对比图](reports/figures/metrics_comparison.png)。

本次新增的代码、报告、图表、固定数据和依赖说明都在 `task1/` 内。
原仓库的 `vdm_lab/`、`run_experiment.py`、根目录 `.gitignore` 等文件不作修改。
这是依赖原项目的实验扩展，不另外复制一套仿真代码。

```text
task1/
├── README.md                 使用入口
├── ENVIRONMENT.md            环境说明
├── requirements.lock.txt     固定依赖版本
├── scripts/                  批量仿真、分析、打包脚本
├── tests/                    数据、复现与目录迁移测试
├── reports/                  完整报告、指标表和 12 张结果图
├── data/20260911_task1/       15 组正式实验的配置和原始日志
├── provenance/               原始实验脚本快照及路径映射
├── outputs/                  后续运行结果（本地生成，不提交）
├── artifacts/                ZIP 包（本地生成，不提交）
└── environment_checks/       初次环境检查数据（仅本地保留）
```

## 已完成的内容

- 9 组必做实验：PP、运动学 LQR、MPC × 双移线、直角弯、S 弯。
- 4 组新增参数实验：PP 和运动学 LQR 的转角上限 25°、15°；35°复用主实验。
- 2 组动力学 LQR 补充实验，共 15 组独立配置，均满足终点判据。
- 中文模型推导、结果分析、统一 CSV 表格和 12 张 PNG。

原始数据见 [data/20260911_task1](data/20260911_task1)，数值校验见
[reports/validation.json](reports/validation.json)。目录迁移没有重新解释或改写旧实验记录，
旧路径及哈希的定位方式见 [provenance/README.md](provenance/README.md)。

目录整理后，16 项测试通过；15 组实验在新路径下完整重跑，指标和轨迹与原数据一致
（数值比较容差为 `1e-8`）。95 个原始数据文件也与原提交逐一核对过，
除 Git 规范的 CSV 换行外未改写内容。

## 运行和验证

以下命令均在仓库根目录执行，先按 [环境说明](ENVIRONMENT.md) 激活 Python 环境：

```bash
python -m unittest discover -s task1/tests -v
python task1/scripts/run_task1.py --output task1/outputs/new_run --jobs 3
python task1/scripts/analyze_task1.py --input task1/outputs/new_run --output task1/outputs/new_report
```

新输出目录必须不存在，避免覆盖已有结果。不指定批量脚本的 `--output` 时，
默认写入 `task1/outputs/<时间戳>/`，不会把新结果散落到仓库根目录。
测试默认检查 `task1/data/20260911_task1/` 中的交付数据，新批次由分析脚本独立校验。

仅重绘已有数据并校验时，可输出到新的本地目录：

```bash
python task1/scripts/analyze_task1.py --input task1/data/20260911_task1 --output task1/outputs/report_preview
```

分析脚本只生成图表和 `data_tables.md`，不会生成或覆盖单独撰写的 `report.md` 正文。
如需更新已有图表，可显式使用 `--refresh-generated`；修改实验设置后需重新检查正文结论。

## 打包

```bash
python task1/scripts/package_task1.py
```

默认生成 `task1/artifacts/task1_submission.zip`。包内只有 `task1/` 一个顶层目录，
不包含虚拟环境、缓存、重复运行结果、旧 ZIP 或原仓库代码。将整个 `task1/` 放入
已克隆的 `VDM_tracking` 仓库即可使用；源码和课件链接也需要原仓库内容。
已有同名 ZIP 时脚本会拒绝覆盖；需要更新时显式使用 `--replace`。

已安装的根目录 `.venv` 仍作为本机共享运行环境复用，未重复安装，也不属于提交内容。
提交前请补充课程要求的姓名、学号、组员分工和封面格式；是否允许使用 `solution`
实现及辅助工具，请遵循课程要求。
