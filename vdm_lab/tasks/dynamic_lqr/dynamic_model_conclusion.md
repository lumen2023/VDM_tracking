================================================================
Dynamic LQR 与 Kinematic / Dynamic Vehicle Model 对比
——模型推导说明与实验结论
================================================================

作者：任皓雪
路线：s_curve (medium), mixed_course (medium)
车辆：student_car
版本：student（已与 solution 验证一致）


================================================================
第一部分：动力学模型推导说明
================================================================

一、为什么要用动力学模型？

运动学自行车模型假设轮胎与地面之间没有侧滑，车辆的横向运动完全由几何
关系决定。这一假设在低速、小转向角条件下是合理的，但当车速提高、
侧向加速度增大时，轮胎会出现明显的侧偏角，侧向力不再与几何转角成
简单比例关系，此时必须使用动力学模型才能准确描述车辆响应。

二、动力学模型中的物理量

符号     含义                        单位        对车辆响应的作用
------   --------------------------  ----------  ------------------------------
m        车辆质量                    kg          侧向力对加速度的影响
Iz       横摆转动惯量                kg·m^2      横摆力矩对横摆角速度的影响
Cf       前轮侧偏刚度                N/rad       前轮产生侧向力的能力
Cr       后轮侧偏刚度                N/rad       后轮产生侧向力的能力
lf       质心到前轴的距离            m           前轮侧向力对质心的力矩臂
lr       质心到后轴的距离            m           后轮侧向力对质心的力矩臂
L=lf+lr  轴距                        m           车辆几何比例
v        纵向速度                    m/s         动力学项中 1/v 的来源

三、动力学模型的物理链条

  方向盘转角 steer
        ↓
  轮胎侧偏角 slip angle
        ↓
  侧向轮胎力 lateral tire force
        ↓
  侧向加速度 + 横摆力矩
        ↓
  车辆侧向 / 横摆响应

四、线性二自由度车辆模型（横向误差动力学）

状态量取为：
  e_y       : 横向误差
  e_y_dot   : 横向误差变化率
  e_yaw     : 航向角误差
  e_yaw_dot : 航向角误差变化率

连续时间状态方程：

  A_c =
  [ 0,        1,                              0,               0                    ]
  [ 0,  -(cf+cr)/(m v),              (cf+cr)/m,        (lr cr - lf cf)/(m v)        ]
  [ 0,        0,                              0,               1                    ]
  [ 0,  (lr cr - lf cf)/(Iz v),   (lf cf - lr cr)/Iz,  -(lf^2 cf + lr^2 cr)/(Iz v)  ]

  B_c =
  [ 0          ]
  [ cf / m     ]
  [ 0          ]
  [ lf cf / Iz ]

输入 u = 前轮转角 δ。

离散化采用梯形法：

  A = pinv(I - 0.5 dt A_c) @ (I + 0.5 dt A_c)
  B = B_c * dt

五、运动学模型与动力学模型对比

                  Kinematic model           Dynamic model
--------------    ----------------------    --------------------------
状态              e_y, e_y_dot,             e_y, e_y_dot,
                  e_yaw, e_yaw_dot          e_yaw, e_yaw_dot
参数              仅 wheelbase, v           含 m, Iz, lf, lr, cf, cr, v
轮胎假设          无侧滑                    允许侧偏角
适用工况          低速、小转向              低速到高速
物理意义          几何约束                  力 / 力矩平衡

六、为什么运动学模型中没有 m、Iz、Cf、Cr？

运动学模型假设轮胎与地面之间“纯滚动、无侧滑”，车辆的运动完全由几何
关系决定，不涉及力与质量之间的平衡关系，因此这些力学参数不会出现。

七、为什么低速下运动学模型已经够用？

低速时轮胎侧偏角很小，侧向力与侧偏角近似线性，且侧向加速度很小。
动力学模型在低速下退化为运动学模型，两者结果接近。

八、为什么速度提高后动力学模型更重要？

速度提高 → 侧向加速度增大 → 轮胎侧偏角增大 → 侧向力进入非线性区间。
运动学模型无法描述这种非线性响应，必须使用包含 m、Iz、Cf、Cr 的
动力学模型。本实验在 s_curve 与 mixed_course 两条路线上都验证了这一点。


================================================================
第二部分：Dynamic LQR 控制器设计
================================================================

一、控制目标

让车辆沿参考路径行驶，使横向误差 e_y、航向误差 e_yaw 等状态收敛到零。

二、反馈项

采用 LQR 求解最优反馈增益 K：

  u_fb = -K x

其中 x = [e_y, e_y_dot, e_yaw, e_yaw_dot]^T。

K 通过离散代数 Riccati 迭代求解：

  P_{k+1} = Q + A^T P_k A
            - A^T P_k B (R + B^T P_k B)^{-1} B^T P_k A

  K = (R + B^T P B)^{-1} B^T P A

三、前馈项

为了消除连续弯道下的稳态横向误差，加入曲率前馈：

  δ_ff = L κ + kv v^2 κ - K[0,2] * yaw_steady_error

其中：
  L  = lf + lr
  κ  = 参考路径曲率
  kv = lr m / (2 cf L) - lf m / (2 cr L)

四、最终控制律

  δ = clamp(u_fb + δ_ff, -max_steer, +max_steer)

其中 max_steer 为车辆前轮最大转角，用于防止转角饱和。


================================================================
第三部分：实验结果
================================================================

实验设置：
  vehicle   = student_car
  version   = student
  speed_mode = medium
  routes    = s_curve, mixed_course

一、s_curve（medium）

指标                     Kinematic LQR     Dynamic LQR     改善
----------------------   ---------------   -------------   --------
steps                    150               152             —
reached_goal             True              True            
mean_lateral_error [m]   0.2164            0.0922          ↓ 57%
max_lateral_error  [m]   0.5469            0.2148          ↓ 61%
finish_error       [m]   0.8429            0.8976          略差
mean_heading_err [rad]   0.0516            0.0690          略差
max_steer         [rad]   0.6109            0.5598          ↓ 8%
max_beta          [rad]   0.3368            0.3036          ↓ 10%
max_yaw_rate    [rad/s]   1.7177            1.5542          ↓ 10%

二、mixed_course（medium）

指标                     Kinematic LQR     Dynamic LQR     改善
----------------------   ---------------   -------------   --------
steps                    184               183             —
reached_goal             True              True            
mean_lateral_error [m]   0.1568            0.0706          ↓ 55%
max_lateral_error  [m]   0.4770            0.1518          ↓ 68%
finish_error       [m]   0.9489            0.8231          ↓ 13%
mean_heading_err [rad]   0.0547            0.0466          ↓ 15%
max_steer         [rad]   0.6109            0.3344          ↓ 45%
max_beta          [rad]   0.3368            0.1720          ↓ 49%
max_yaw_rate    [rad/s]   1.8504            0.9585          ↓ 48%

三、图形观察

观察 outputs 目录下的 4 张 summary.png：

1. Kinematic LQR（s_curve）：
   lateral error 在弯道处达到 ±0.55 m，steer 曲线存在明显毛刺。

2. Dynamic LQR（s_curve）：
   lateral error 最大约 ±0.20 m，steer 曲线明显更平滑。

3. Kinematic LQR（mixed_course）：
   lateral error 存在高频锯齿抖动，幅度约 ±0.50 m。

4. Dynamic LQR（mixed_course）：
   lateral error 最大约 ±0.15 m，几乎无高频抖动。


================================================================
第四部分：结论与回答四个问题
================================================================

问题 1：为什么低速下运动学模型已经够用？

答：低速时轮胎侧偏角很小，侧向力与侧偏角近似线性，侧向加速度很小。
此时动力学模型退化为运动学模型，两者结果接近。本实验在 medium 速度
下两者都到达终点，也说明在该速度范围内动力学修正量虽小但已经能体现。

问题 2：为什么速度提高后动力学模型更重要？

答：速度提高导致侧向加速度增大，轮胎侧偏角随之增大，侧向力进入非线性
区间。运动学模型无法描述这种非线性，必须使用包含 m、Iz、Cf、Cr 的
动力学模型。本实验在连续弯道（s_curve、mixed_course）上，Dynamic LQR
的横向误差比 Kinematic LQR 减小 55%~68%，验证了该结论。

问题 3：mass、Iz、Cf、Cr 为什么在运动学模型中没有体现？

答：运动学模型假设轮胎无侧滑，车辆运动完全由几何关系决定，不涉及力与
质量的平衡，因此这些力学参数不会出现在方程中。

问题 4：Kinematic LQR 与 Dynamic LQR 在连续弯道下有什么差别？

答：在 s_curve 与 mixed_course 这两条连续弯道路线上：
  - Dynamic LQR 的 mean_lateral_error 比 Kinematic LQR 小 55%~57%；
  - Dynamic LQR 的 max_lateral_error 比 Kinematic LQR 小 61%~68%；
  - Dynamic LQR 的 max_beta、max_yaw_rate 更小，说明侧滑和横摆更可控；
  - Dynamic LQR 的 steer 更平滑，说明控制抖动更小；
  - 两者都到达终点，但 Dynamic LQR 的跟踪精度显著更高。

总结：
  动力学模型通过引入质量、转动惯量与轮胎侧偏刚度，
  能更准确地刻画高速与连续弯道下的车辆响应，
  从而让 LQR 控制器在同等速度下获得更小的横向误差和更平滑的控制量。


================================================================
第五部分：student 与 solution 一致性验证
================================================================

在 s_curve（medium）上分别运行：

  python run_experiment.py --algo lqr_dynamic --version student
      --route s_curve --speed-mode medium --save-log --save-fig
  python run_experiment.py --algo lqr_dynamic --version solution
      --route s_curve --speed-mode medium --save-log --save-fig

两者结果：

  指标                     student      solution
  ----------------------   ----------   ----------
  steps                    152          152
  reached_goal             True         True
  mean_lateral_error [m]   0.0922       0.0922
  max_lateral_error  [m]   0.2148       0.2148
  finish_error       [m]   0.8976       0.8976

结论：student 与 solution 完全一致，Dynamic LQR 实现正确。


