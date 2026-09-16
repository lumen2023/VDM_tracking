# 任务 3：自主规划寝室到教室的路线并跟踪

先看 [完整中文实验报告](report/report_task3.html)。

本次新增的路线、实验数据、报告和分析脚本都在 `task3/` 内；GPX 路线按主项目约定放在
`data/gpx/dorm_to_classroom.gpx`。原仓库的 `vdm_lab/`、`run_experiment.py` 等文件不作修改。

```text
task3/
├── README.md                 使用入口
├── report/                   完整中文实验报告（HTML）
├── scripts/                  指标批量分析与最大偏差提取脚本
└── data/20260916_task3/      6 组正式实验的原始日志
    ├── pp__speed8/           PP，目标速度 8 m/s
    ├── lqr_kinematic__speed8/
    ├── mpc__speed8/
    ├── pp__speed5/           PP，目标速度 5 m/s（降速改进复测）
    ├── lqr_kinematic__speed5/
    └── mpc__speed5/
```

## 已完成的内容

- 使用 gpx.studio（基于 OSM）按实际通行方向规划寝室到教室路线，WGS84 坐标导出 GPX：
  27 个原始点，全长 774.1 m，`--waypoint-ds 1.0` 重采样为 776 个参考点。
- 路线完全落在离线 GeoJSON 底图覆盖范围内，局部坐标原点为任务书规定的 (118.8145, 31.8885)。
- PP、运动学 LQR、MPC 在目标速度 8 m/s 下各一组对比实验，三种算法均到达终点；
  精度排名 MPC（平均误差 0.029 m）> PP（0.050 m）> LQR（0.421 m）。
- 最大偏差定位：LQR / MPC 在 s ≈ 146 m 急弯处（LQR 转角饱和 0.611 rad），
  PP 在 s = 708 m 终点前直道（前视滞后"甩尾"）；
  标注图见 [report/figures/deviation_map.png](report/figures/deviation_map.png)。
- 仿真用时与真实通勤时间的边界说明见报告第七节。
- 降速改进复测（8 → 5 m/s，三算法共 3 组）：LQR 平均误差 -76.5%（最敏感），
  MPC -0.7%（几乎不变，自身会主动减速）；三算法法向加速度普降 53%~61%。

## 运行和验证

复现实验（仓库根目录，环境见主 README）：

```bash
python run_experiment.py \
  --algo pp \
  --gpx data/gpx/dorm_to_classroom.gpx \
  --basemap geojson \
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
  --map-origin 118.8145 31.8885 \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --save-log --save-fig
```

`--algo` 可换为 `lqr_kinematic`、`mpc`；`--target-speed 5` 对应降速复测组。

重新统计指标和最大偏差位置：

```bash
python task3/scripts/analyze_results.py
python task3/scripts/extract_peaks.py
python task3/scripts/plot_deviation_map.py   # 重新生成最大偏差位置标注图
```

## 隐私说明

报告公开前已将寝室和教室名称匿名化；GPX 经纬度和地图截图仅覆盖校园公开道路范围。
