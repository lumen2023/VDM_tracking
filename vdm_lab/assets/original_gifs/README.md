# 原仓库控制算法演示 GIF

这里保留原仓库中与本实验相关的 PP、LQR、MPC 演示动图，作为教师备课和课堂对照材料。

新实验请优先使用统一入口重新生成 GIF：

```bash
python run_experiment.py --algo pp --version solution --save-gif
python run_experiment.py --algo lqr_kinematic --version solution --save-gif
python run_experiment.py --algo lqr_dynamic --version solution --save-gif
python run_experiment.py --algo mpc --version solution --save-gif
```

生成的新动图会保存到 `outputs/<时间戳>_<算法>/animation.gif`。
