# 原始实验的版本追溯

2026-09-11 的原始实验清单保留在
[data/20260911_task1/manifest.json](../data/20260911_task1/manifest.json)。
其中的源文件路径、Python 路径和哈希描述的是当时运行实验的目录布局，
不是本次迁移后的布局；没有为了让校验通过而改写这些历史记录。

[path_map.json](path_map.json) 仅为两个已迁移文件提供定位：

- 固定依赖清单移动到 `task1/requirements.lock.txt`，内容不变。
- 当时的批量脚本原样保留为 `run_task1_20260911.py.txt`，用于核对原始哈希，
  不作为当前运行入口。

当前运行入口为 [scripts/run_task1.py](../scripts/run_task1.py)，仅调整了目录定位、
默认输出位置和提示中的路径。其后重新运行生成的清单会记录新路径和新脚本哈希。
原仓库 `vdm_lab/` 与 `run_experiment.py` 未复制或改写，仍直接复用。

迁移前的数值数据、指标和配置保持不变；复现测试会用当前入口重新运行每种主要算法的一条路线，
并与原始结果逐字段比较。CSV 在 Git 中可能被规范为 LF 换行，但不改变数值。

旧 `run_info.json` 中的等价命令也是历史记录；重新运行请采用 `task1/README.md` 的新命令。
