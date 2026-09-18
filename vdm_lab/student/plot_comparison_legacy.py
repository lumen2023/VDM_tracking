"""Compare all student controllers on the mixed_course route (low speed).

Generates 6 figures:
  1. student_comparison.png      - overview (trajectory + bar + lateral/steer/speed)
  2. fig_trajectory_detail.png   - trajectory with error vectors & zoomed insets
  3. fig_error_analysis.png      - lateral error, heading error, CDF of lateral error
  4. fig_control_effort.png      - steer, acceleration, yaw rate, side slip
  5. fig_phase_and_scatter.png   - steer-accel phase portrait, curvature vs error scatter
  6. fig_radar_metrics.png       - radar/spider chart of normalised metrics
"""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

OUTPUTS = Path(__file__).resolve().parent.parent.parent / "outputs"

ALGO_DIRS = {
    "Pure Pursuit": "20260917_175646_pp_mixed_course_low",
    "LQR Kinematic": "20260917_175804_lqr_kinematic_mixed_course_low",
    "LQR Dynamic": "20260917_175808_lqr_dynamic_mixed_course_low",
    "MPC": "20260917_181609_mpc_mixed_course_low",
}

COLORS = {
    "Pure Pursuit": "#e74c3c",
    "LQR Kinematic": "#3498db",
    "LQR Dynamic": "#2ecc71",
    "MPC": "#9b59b6",
}

SHORT = {"Pure Pursuit": "PP", "LQR Kinematic": "LQR-K", "LQR Dynamic": "LQR-D", "MPC": "MPC"}


def load_trajectory(directory):
    rows = []
    with open(OUTPUTS / directory / "trajectory.csv", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def load_metrics(directory):
    with open(OUTPUTS / directory / "metrics.json") as f:
        return json.load(f)


def load_reference(directory):
    rows = []
    with open(OUTPUTS / directory / "reference_path.csv", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except ValueError:
                    parsed[k] = v
            rows.append(parsed)
    return rows


def _ts(data):
    return [r["time"] for r in data]


def save(fig, name):
    path = OUTPUTS / name
    fig.savefig(path, dpi=150, bbox_inches="tight")
    print(f"  saved: {path}")


# ── Figure 1: Overview ──────────────────────────────────────────────
def fig_overview(data, metrics, ref):
    ref_x = [p["x_m"] for p in ref]
    ref_y = [p["y_m"] for p in ref]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle("Student Controller Comparison - mixed_course / low speed",
                 fontsize=15, fontweight="bold")

    axes[0, 0].set_axis_off()
    axes[0, 1].set_axis_off()
    ax_traj = fig.add_subplot(2, 3, (1, 2))
    ax_traj.plot(ref_x, ref_y, "k--", lw=1.5, label="Reference", alpha=0.6)
    for name in ALGO_DIRS:
        ax_traj.plot([r["x"] for r in data[name]], [r["y"] for r in data[name]],
                     color=COLORS[name], lw=1.8, label=name)
    ax_traj.set_xlabel("x [m]"); ax_traj.set_ylabel("y [m]")
    ax_traj.set_title("Trajectory Tracking"); ax_traj.legend(fontsize=9)
    ax_traj.set_aspect("equal"); ax_traj.grid(True, alpha=0.3)

    ax_bar = axes[0, 2]
    names = list(ALGO_DIRS.keys())
    mean_lat = [metrics[n]["mean_lateral_error_m"] for n in names]
    max_lat = [metrics[n]["max_lateral_error_m"] for n in names]
    x_pos = np.arange(len(names)); w = 0.35
    b1 = ax_bar.bar(x_pos - w/2, mean_lat, w, label="Mean", color="#3498db", alpha=0.8)
    b2 = ax_bar.bar(x_pos + w/2, max_lat, w, label="Max", color="#e74c3c", alpha=0.8)
    ax_bar.set_xticks(x_pos); ax_bar.set_xticklabels(list(SHORT.values()), fontsize=9)
    ax_bar.set_ylabel("Error [m]"); ax_bar.set_title("Lateral Error")
    ax_bar.legend(fontsize=8); ax_bar.grid(True, alpha=0.3, axis="y")
    for b in list(b1) + list(b2):
        ax_bar.text(b.get_x()+b.get_width()/2, b.get_height(), f"{b.get_height():.3f}",
                    ha="center", va="bottom", fontsize=7)

    for ax, key, ylabel, title, deg in [
        (axes[1, 0], "lateral_error", "Lateral Error [m]", "Lateral Error vs Time", False),
        (axes[1, 1], "steer", "Steer [deg]", "Steering Angle vs Time", True),
        (axes[1, 2], "speed", "Speed [m/s]", "Speed vs Time", False),
    ]:
        for name in ALGO_DIRS:
            vals = [np.rad2deg(r[key]) if deg else r[key] for r in data[name]]
            ax.plot(_ts(data[name]), vals, color=COLORS[name], lw=1.2, label=name)
        ax.set_xlabel("Time [s]"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save(fig, "student_comparison.png")
    plt.close(fig)


# ── Figure 2: Trajectory detail with error vectors ─────────────────
def fig_trajectory_detail(data, ref):
    ref_x = np.array([p["x_m"] for p in ref])
    ref_y = np.array([p["y_m"] for p in ref])

    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle("Trajectory Detail - Error Vectors & Path Deviation",
                 fontsize=14, fontweight="bold")

    ax = axes[0]
    ax.plot(ref_x, ref_y, "k--", lw=2, label="Reference", alpha=0.5)
    for name in ALGO_DIRS:
        xs = [r["x"] for r in data[name]]
        ys = [r["y"] for r in data[name]]
        ax.plot(xs, ys, color=COLORS[name], lw=1.6, label=name, alpha=0.85)
        step = max(1, len(xs) // 25)
        for i in range(0, len(xs), step):
            lat_e = data[name][i]["lateral_error"]
            yaw = data[name][i]["yaw"]
            dx = -lat_e * np.sin(yaw)
            dy = lat_e * np.cos(yaw)
            ax.annotate("", xy=(xs[i], ys[i]), xytext=(xs[i]-dx, ys[i]-dy),
                        arrowprops=dict(arrowstyle="->", color=COLORS[name], lw=0.8, alpha=0.5))
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Trajectory with Lateral-Error Vectors")
    ax.legend(fontsize=8); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    ax2 = axes[1]
    for name in ALGO_DIRS:
        ts = _ts(data[name])
        ax2.plot(ts, [r["lateral_error"] for r in data[name]],
                 color=COLORS[name], lw=1.2, label=name)
    ts_ld = _ts(data["LQR Dynamic"])
    lat_ld = [r["lateral_error"] for r in data["LQR Dynamic"]]
    ax2.fill_between(ts_ld, 0, lat_ld,
                     color=COLORS["LQR Dynamic"], alpha=0.15, label="LQR-D envelope")
    ax2.set_xlabel("Time [s]"); ax2.set_ylabel("Lateral Error [m]")
    ax2.set_title("Lateral Error (with LQR-D envelope highlighted)")
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    save(fig, "fig_trajectory_detail.png")
    plt.close(fig)


# ── Figure 3: Error analysis ───────────────────────────────────────
def fig_error_analysis(data, metrics):
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle("Error Analysis", fontsize=14, fontweight="bold")

    ax = axes[0, 0]
    for name in ALGO_DIRS:
        ax.plot(_ts(data[name]), [r["lateral_error"] for r in data[name]],
                color=COLORS[name], lw=1.2, label=name)
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Lateral Error [m]")
    ax.set_title("Lateral Error vs Time"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    for name in ALGO_DIRS:
        ax.plot(_ts(data[name]), [np.rad2deg(r["heading_error"]) for r in data[name]],
                color=COLORS[name], lw=1.2, label=name)
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Heading Error [deg]")
    ax.set_title("Heading Error vs Time"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    for name in ALGO_DIRS:
        lat_err = np.sort([r["lateral_error"] for r in data[name]])
        cdf = np.arange(1, len(lat_err)+1) / len(lat_err)
        ax.plot(lat_err, cdf, color=COLORS[name], lw=1.8, label=name)
    ax.set_xlabel("Lateral Error [m]"); ax.set_ylabel("CDF")
    ax.set_title("CDF of Lateral Error"); ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    names = list(ALGO_DIRS.keys())
    keys = ["mean_lateral_error_m", "max_lateral_error_m",
            "mean_heading_error_rad", "max_steer_rad"]
    labels = ["Mean Lat[m]", "Max Lat[m]", "Mean Head[rad]", "Max Steer[rad]"]
    x_pos = np.arange(len(labels)); w = 0.18
    for i, name in enumerate(names):
        vals = [metrics[name][k] for k in keys]
        ax.bar(x_pos + i*w, vals, w, color=COLORS[name], label=SHORT[name], alpha=0.85)
    ax.set_xticks(x_pos + w*1.5); ax.set_xticklabels(labels, fontsize=8)
    ax.set_title("Multi-Metric Bar Comparison"); ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    save(fig, "fig_error_analysis.png")
    plt.close(fig)


# ── Figure 4: Control effort ───────────────────────────────────────
def fig_control_effort(data):
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle("Control Effort & Vehicle Dynamics", fontsize=14, fontweight="bold")

    specs = [
        (axes[0, 0], "steer", True, "Steering Angle [deg]", "Steering vs Time"),
        (axes[0, 1], "acceleration", False, "Acceleration [m/s^2]", "Acceleration vs Time"),
        (axes[1, 0], "yaw_rate", True, "Yaw Rate [deg/s]", "Yaw Rate vs Time"),
        (axes[1, 1], "beta", True, "Side Slip [deg]", "Side Slip Angle vs Time"),
    ]
    for ax, key, deg, ylabel, title in specs:
        for name in ALGO_DIRS:
            vals = [np.rad2deg(r[key]) if deg else r[key] for r in data[name]]
            ax.plot(_ts(data[name]), vals, color=COLORS[name], lw=1.2, label=name)
        ax.set_xlabel("Time [s]"); ax.set_ylabel(ylabel); ax.set_title(title)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save(fig, "fig_control_effort.png")
    plt.close(fig)


# ── Figure 5: Phase portrait & scatter ─────────────────────────────
def fig_phase_and_scatter(data):
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    fig.suptitle("Phase Portrait & Curvature-Error Scatter",
                 fontsize=14, fontweight="bold")

    ax = axes[0]
    for name in ALGO_DIRS:
        steers = [np.rad2deg(r["steer"]) for r in data[name]]
        accels = [r["acceleration"] for r in data[name]]
        ax.plot(steers, accels, color=COLORS[name], lw=0.6, alpha=0.5, label=name)
        ax.scatter(steers[0], accels[0], color=COLORS[name], s=60, marker="o", zorder=5)
        ax.scatter(steers[-1], accels[-1], color=COLORS[name], s=60, marker="s", zorder=5)
    ax.set_xlabel("Steer [deg]"); ax.set_ylabel("Acceleration [m/s^2]")
    ax.set_title("Steer-Acceleration Phase Portrait (o=start, s=end)")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1]
    for name in ALGO_DIRS:
        curvs = [abs(r["curvature"]) for r in data[name]]
        lats = [r["lateral_error"] for r in data[name]]
        ax.scatter(curvs, lats, color=COLORS[name], s=8, alpha=0.4, label=name)
    ax.set_xlabel("|Curvature| [1/m]"); ax.set_ylabel("Lateral Error [m]")
    ax.set_title("Curvature vs Lateral Error")
    ax.legend(fontsize=8, markerscale=3); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save(fig, "fig_phase_and_scatter.png")
    plt.close(fig)


# ── Figure 6: Radar / spider chart ─────────────────────────────────
def fig_radar_metrics(metrics):
    metric_keys = [
        "mean_lateral_error_m",
        "max_lateral_error_m",
        "mean_heading_error_rad",
        "max_steer_rad",
        "max_acceleration_mps2",
        "max_yaw_rate_radps",
        "max_side_slip_beta_rad",
    ]
    metric_labels = [
        "Mean Lat Err",
        "Max Lat Err",
        "Mean Head Err",
        "Max Steer",
        "Max Accel",
        "Max Yaw Rate",
        "Max Side Slip",
    ]
    names = list(ALGO_DIRS.keys())
    n_algos = len(names)
    n_metrics = len(metric_keys)

    raw = np.zeros((n_algos, n_metrics))
    for i, name in enumerate(names):
        for j, key in enumerate(metric_keys):
            raw[i, j] = metrics[name][key]

    ref_vals = raw.max(axis=0)
    ref_vals[ref_vals == 0] = 1.0
    normed = raw / ref_vals

    angles = np.linspace(0, 2 * np.pi, n_metrics, endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(9, 9), subplot_kw=dict(polar=True))
    fig.suptitle("Normalised Metrics Radar Chart", fontsize=14, fontweight="bold", y=0.98)

    for i, name in enumerate(names):
        vals = normed[i].tolist() + [normed[i, 0]]
        ax.plot(angles, vals, color=COLORS[name], lw=2, label=name)
        ax.fill(angles, vals, color=COLORS[name], alpha=0.1)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metric_labels, fontsize=9)
    ax.set_ylim(0, 1.05)
    ax.set_rticks(np.arange(0, 1.1, 0.2))
    ax.set_yticklabels([f"{v:.1f}" for v in np.arange(0, 1.1, 0.2)], fontsize=7)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1), fontsize=9)

    save(fig, "fig_radar_metrics.png")
    plt.close(fig)


# ── Main ────────────────────────────────────────────────────────────
def main():
    data = {}
    metrics = {}
    ref = None
    for name, directory in ALGO_DIRS.items():
        data[name] = load_trajectory(directory)
        metrics[name] = load_metrics(directory)
        if ref is None:
            ref = load_reference(directory)

    print("Generating figures...")
    fig_overview(data, metrics, ref)
    fig_trajectory_detail(data, ref)
    fig_error_analysis(data, metrics)
    fig_control_effort(data)
    fig_phase_and_scatter(data)
    fig_radar_metrics(metrics)

    print("\n" + "=" * 70)
    print(f"{'Algorithm':<18} {'Mean Lat[m]':<14} {'Max Lat[m]':<14} {'Reached':<10} {'Steps':<8}")
    print("=" * 70)
    for name in ALGO_DIRS:
        m = metrics[name]
        print(f"{name:<18} {m['mean_lateral_error_m']:<14.4f} "
              f"{m['max_lateral_error_m']:<14.4f} "
              f"{'Yes' if m['reached_goal'] else 'No':<10} {m['steps']:<8}")
    print("=" * 70)
    print(f"\nAll figures saved to: {OUTPUTS}")


if __name__ == "__main__":
    main()