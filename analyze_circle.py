import csv
import math
import statistics
from pathlib import Path


# ============================================================
# 1. 修改成你自己的三个实验输出目录
# ============================================================

RUNS = {
    "low": Path("outputs/20260915_080400_pp_circle_low"),
    "medium": Path("outputs/20260915_080444_pp_circle_medium"),
    "high": Path("outputs/20260915_080458_pp_circle_high"),
}

# 三档目标速度
TARGET_SPEEDS = {
    "low": 3.0,
    "medium": 5.0,
    "high": 7.0,
}


# ============================================================
# 2. 圆形路径和车辆参数
# ============================================================

RADIUS = 12.0
KAPPA_THEORY = 1.0 / RADIUS

# student_car 默认轴距 lf + lr = 2.5 m
WHEELBASE = 2.5

# 任务书要求：
# abs(curvature - 1/12) < 0.005
CURVATURE_TOLERANCE = 0.005

# 为了排除刚进入/即将驶出圆弧的瞬态，
# 从候选圆弧数据首尾各去掉 10%
TRIM_RATIO = 0.10


# ============================================================
# 3. 辅助函数
# ============================================================

def mean(values):
    if not values:
        return float("nan")
    return sum(values) / len(values)


def relative_error(simulation, theory):
    """
    相对误差：
    abs(simulation - theory) / abs(theory) * 100%
    """
    if abs(theory) < 1e-12:
        return float("nan")

    return abs(simulation - theory) / abs(theory) * 100.0


def mean_steer_rate(rows):
    """
    计算任务书中的转角变化率：

        J_delta = mean(
            abs(steer[k] - steer[k-1]) / dt
        )

    单位：rad/s
    """
    rates = []

    for previous, current in zip(rows[:-1], rows[1:]):
        t0 = float(previous["time"])
        t1 = float(current["time"])

        dt = t1 - t0

        if dt <= 0:
            continue

        steer0 = float(previous["steer"])
        steer1 = float(current["steer"])

        rates.append(abs(steer1 - steer0) / dt)

    return mean(rates)


# ============================================================
# 4. 保存最终汇总结果
# ============================================================

results = []


# ============================================================
# 5. 分析 low / medium / high
# ============================================================

for speed_mode, output_dir in RUNS.items():

    trajectory_file = output_dir / "trajectory.csv"

    if not trajectory_file.exists():
        print(f"[ERROR] 找不到文件：{trajectory_file}")
        continue

    # --------------------------------------------------------
    # 读取 trajectory.csv
    # --------------------------------------------------------

    with trajectory_file.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as file:

        rows = list(csv.DictReader(file))

    if not rows:
        print(f"[ERROR] trajectory.csv 为空：{trajectory_file}")
        continue

    # --------------------------------------------------------
    # 找到圆弧候选段
    #
    # abs(curvature - 1/12) < 0.005
    # --------------------------------------------------------

    circle_rows = [
        row
        for row in rows
        if abs(
            float(row["curvature"]) - KAPPA_THEORY
        ) < CURVATURE_TOLERANCE
    ]

    if not circle_rows:
        print(f"[ERROR] {speed_mode}: 没有找到圆弧数据")
        continue

    # --------------------------------------------------------
    # 去掉圆弧首尾过渡区域
    # --------------------------------------------------------

    n_circle = len(circle_rows)

    trim = int(n_circle * TRIM_RATIO)

    # 至少保证不会全部删掉
    if trim > 0 and n_circle > 2 * trim:
        steady_rows = circle_rows[trim:-trim]
    else:
        steady_rows = circle_rows

    if not steady_rows:
        print(f"[ERROR] {speed_mode}: 稳态数据为空")
        continue

    # --------------------------------------------------------
    # 提取各物理量
    # --------------------------------------------------------

    times = [
        float(row["time"])
        for row in steady_rows
    ]

    speeds = [
        float(row["speed"])
        for row in steady_rows
    ]

    steers = [
        float(row["steer"])
        for row in steady_rows
    ]

    yaw_rates = [
        float(row["yaw_rate"])
        for row in steady_rows
    ]

    normal_accels = [
        float(row["normal_accel"])
        for row in steady_rows
    ]

    lateral_errors = [
        float(row["lateral_error"])
        for row in steady_rows
    ]

    betas = [
        float(row["beta"])
        for row in steady_rows
    ]

    curvatures = [
        float(row["curvature"])
        for row in steady_rows
    ]

    # --------------------------------------------------------
    # 实际仿真统计
    # --------------------------------------------------------

    target_speed = TARGET_SPEEDS[speed_mode]

    mean_speed = mean(speeds)
    mean_curvature = mean(curvatures)

    mean_steer = mean(steers)
    mean_yaw_rate = mean(yaw_rates)
    mean_normal_accel = mean(normal_accels)

    mean_lateral_error = mean(lateral_errors)

    max_abs_lateral_error = max(
        abs(value)
        for value in lateral_errors
    )

    # 这里把整个稳态区间视为完整数据总体，所以使用 pstdev
    lateral_error_std = statistics.pstdev(
        lateral_errors
    )

    max_abs_beta = max(
        abs(value)
        for value in betas
    )

    steer_rate = mean_steer_rate(steady_rows)

    # --------------------------------------------------------
    # 理论值 1：
    # 使用“目标速度”
    # --------------------------------------------------------

    theory_steer = math.atan(
        WHEELBASE * KAPPA_THEORY
    )

    theory_yaw_target = (
        target_speed * KAPPA_THEORY
    )

    theory_accel_target = (
        target_speed ** 2 * KAPPA_THEORY
    )

    # --------------------------------------------------------
    # 理论值 2：
    # 使用“仿真中的实际速度”
    #
    # yaw:
    # mean(v * kappa)
    #
    # normal acceleration:
    # mean(v^2 * kappa)
    #
    # 注意：
    # mean(v^2) != mean(v)^2
    # 所以这里直接对每个采样点计算，再取平均
    # --------------------------------------------------------

    theory_yaw_actual_speed = mean([
        speed * KAPPA_THEORY
        for speed in speeds
    ])

    theory_accel_actual_speed = mean([
        speed * speed * KAPPA_THEORY
        for speed in speeds
    ])

    # --------------------------------------------------------
    # 相对误差
    # --------------------------------------------------------

    steer_error_pct = relative_error(
        mean_steer,
        theory_steer,
    )

    yaw_target_error_pct = relative_error(
        mean_yaw_rate,
        theory_yaw_target,
    )

    yaw_actual_error_pct = relative_error(
        mean_yaw_rate,
        theory_yaw_actual_speed,
    )

    accel_target_error_pct = relative_error(
        mean_normal_accel,
        theory_accel_target,
    )

    accel_actual_error_pct = relative_error(
        mean_normal_accel,
        theory_accel_actual_speed,
    )

    # --------------------------------------------------------
    # 输出到终端
    # --------------------------------------------------------

    print()
    print("=" * 72)
    print(f"PP circle analysis: {speed_mode}")
    print("=" * 72)

    print(f"output dir:             {output_dir}")

    print()
    print("[Samples]")
    print(f"circle candidate rows:  {len(circle_rows)}")
    print(f"steady rows:            {len(steady_rows)}")

    print()
    print("[Speed]")
    print(
        f"target speed:           "
        f"{target_speed:.6f} m/s"
    )
    print(
        f"mean actual speed:      "
        f"{mean_speed:.6f} m/s"
    )

    print()
    print("[Curvature]")
    print(
        f"theory curvature:       "
        f"{KAPPA_THEORY:.6f} 1/m"
    )
    print(
        f"mean actual curvature:  "
        f"{mean_curvature:.6f} 1/m"
    )

    print()
    print("[Steering]")
    print(
        f"mean steer:             "
        f"{mean_steer:.6f} rad"
    )
    print(
        f"theory steer:           "
        f"{theory_steer:.6f} rad"
    )
    print(
        f"steer relative error:   "
        f"{steer_error_pct:.3f} %"
    )

    print()
    print("[Yaw rate]")
    print(
        f"mean yaw rate:          "
        f"{mean_yaw_rate:.6f} rad/s"
    )
    print(
        f"theory (target speed):  "
        f"{theory_yaw_target:.6f} rad/s"
    )
    print(
        f"error vs target theory: "
        f"{yaw_target_error_pct:.3f} %"
    )

    print(
        f"theory (actual speed):  "
        f"{theory_yaw_actual_speed:.6f} rad/s"
    )
    print(
        f"error vs actual theory: "
        f"{yaw_actual_error_pct:.3f} %"
    )

    print()
    print("[Normal acceleration]")
    print(
        f"mean normal accel:      "
        f"{mean_normal_accel:.6f} m/s^2"
    )
    print(
        f"theory (target speed):  "
        f"{theory_accel_target:.6f} m/s^2"
    )
    print(
        f"error vs target theory: "
        f"{accel_target_error_pct:.3f} %"
    )

    print(
        f"theory (actual speed):  "
        f"{theory_accel_actual_speed:.6f} m/s^2"
    )
    print(
        f"error vs actual theory: "
        f"{accel_actual_error_pct:.3f} %"
    )

    print()
    print("[Tracking error]")
    print(
        f"mean lateral error:     "
        f"{mean_lateral_error:.6f} m"
    )
    print(
        f"max abs lateral error:  "
        f"{max_abs_lateral_error:.6f} m"
    )
    print(
        f"lateral error std:      "
        f"{lateral_error_std:.6f} m"
    )

    print()
    print("[Vehicle response]")
    print(
        f"max abs beta:           "
        f"{max_abs_beta:.6f} rad"
    )
    print(
        f"mean steer rate J_delta:"
        f" {steer_rate:.6f} rad/s"
    )

    # --------------------------------------------------------
    # 保存本次结果，稍后写 CSV
    # --------------------------------------------------------

    results.append({
        "speed_mode": speed_mode,

        "target_speed_mps": target_speed,
        "mean_actual_speed_mps": mean_speed,

        "theory_curvature_1pm": KAPPA_THEORY,
        "mean_curvature_1pm": mean_curvature,

        "mean_steer_rad": mean_steer,
        "theory_steer_rad": theory_steer,
        "steer_error_pct": steer_error_pct,

        "mean_yaw_rate_radps": mean_yaw_rate,
        "theory_yaw_target_radps": theory_yaw_target,
        "yaw_target_error_pct": yaw_target_error_pct,
        "theory_yaw_actual_radps": theory_yaw_actual_speed,
        "yaw_actual_error_pct": yaw_actual_error_pct,

        "mean_normal_accel_mps2": mean_normal_accel,
        "theory_accel_target_mps2": theory_accel_target,
        "accel_target_error_pct": accel_target_error_pct,
        "theory_accel_actual_mps2": theory_accel_actual_speed,
        "accel_actual_error_pct": accel_actual_error_pct,

        "mean_lateral_error_m": mean_lateral_error,
        "max_abs_lateral_error_m": max_abs_lateral_error,
        "std_lateral_error_m": lateral_error_std,

        "max_abs_beta_rad": max_abs_beta,
        "mean_steer_rate_radps": steer_rate,

        "steady_samples": len(steady_rows),
    })


# ============================================================
# 6. 把最终结果保存成 CSV
# ============================================================

if results:

    output_csv = Path("circle_speed_analysis.csv")

    fieldnames = list(results[0].keys())

    with output_csv.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(results)

    print()
    print("=" * 72)
    print(
        f"分析完成，汇总结果已保存到："
        f"{output_csv.resolve()}"
    )
    print("=" * 72)

else:
    print("没有得到有效结果。")