# 任务 1 交付说明

先阅读 [完整中文报告](report.md)。报告覆盖任务书第 1 节的模型推导、9 组对比实验、代表图、五个分析问题和车辆参数实验，另有动力学 LQR 补充实验。

- 正式仿真：15 组独立配置，全部满足终点判据；没有删除失败样本。
- 图表：`figures/` 中有 12 张 PNG，包括三张原框架的 `summary.png` 代表图。
- 数据表：`tables/` 中的 CSV 保留完整精度，使用 UTF-8 BOM，便于表格软件打开。
- 原始数据：仓库根目录 `outputs/task1/20260911_task1/`，每组均保存配置、逐时刻日志、指标和汇总图。
- 校验：`validation.json` 记录原始数据检查；12 项单元/数据/复现测试均通过。

9 组主实验之外，参数实验的 35°基准复用原结果，只新增 25°和 15°下的 PP、运动学 LQR 共 4 组；另有 2 组动力学 LQR 补充实验。

## 查看和复现

报告、CSV 和 PNG 无需运行 Python 即可查看。压缩包保留仓库相对目录结构；为保持图片和代码链接有效，请整体解压，不要只单独移动 `report.md`。

需要重跑时，将压缩包中的新增文件放入已克隆的 `VDM_tracking` 仓库并使用报告记录的代码版本；不要覆盖已有个人修改。批量脚本会记录 Git 提交号，因此重跑应在 Git 仓库中进行。按 `ENVIRONMENT.md` 准备依赖后，在仓库根目录执行：

```bash
python scripts/run_task1.py --output outputs/task1/new_run --jobs 3
python scripts/analyze_task1.py --input outputs/task1/new_run --output reports/task1_new_run
python -m unittest discover -s tests -v
```

新输出目录必须不存在。测试中的交付数据检查默认读取 `outputs/task1/20260911_task1/`；新的实验批次由分析脚本独立校验。新批次不会自动生成这份单独撰写的报告正文，修改实验设置后需重新分析结论。

重新打包本次交付：

```bash
python scripts/package_task1.py --output reports/task1_submission_new.zip
```

提交前请补充学校要求的姓名、学号、组员分工和封面格式；报告没有虚构这些信息。是否允许采用仓库 `solution` 实现及使用辅助工具，请遵循课程要求。
