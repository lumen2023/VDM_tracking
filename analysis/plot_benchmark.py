"""
为 9 组 Benchmark 生成代表性图片：
1. 每条路线 3 算法横向误差对比
2. 每条路线 3 算法 steer 对比（含饱和线）
3. 每条路线 3 算法轨迹对比
"""
import os
import glob
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUTS_DIR = os.path.join(PROJECT_ROOT, "outputs")
FIGS_DIR = os.path.join(PROJECT_ROOT, "analysis", "figs")
os.makedirs(FIGS_DIR, exist_ok=True)

ALGOS = ["pp", "lqr_kinematic", "mpc"]
ALGO_LABELS = {"pp": "PP", "lqr_kinematic": "LQR-K", "mpc": "MPC"}
ALGO_COLORS = {"pp": "tab:blue", "lqr_kinematic": "tab:orange", "mpc": "tab:green"}
ROUTES = ["double_lane_change", "right_angle", "s_curve"]
ROUTE_LABELS = {
    "double_lane_change": "Double Lane Change",
    "right_angle": "Right Angle",
    "s_curve": "S-Curve",
}
SPEED = "medium"


def find_run_dir(algo, route):
    pattern = os.path.join(OUTPUTS_DIR, f"*_{algo}_{route}_{SPEED}")
    cands = sorted(glob.glob(pattern), key=os.path.getmtime)
    return cands[-1] if cands else None


def load_traj(run_dir):
    path = os.path.join(run_dir, "trajectory.csv")
    return np.genfromtxt(path, delimiter=",", names=True)


def load_ref(run_dir):
    path = os.path.join(run_dir, "reference_path.csv")
    if not os.path.exists(path):
        return None
    return np.genfromtxt(path, delimiter=",", names=True)


def plot_error_per_route(route):
    plt.figure(figsize=(10, 4))
    for algo in ALGOS:
        d = find_run_dir(algo, route)
        if d is None:
            continue
        traj = load_traj(d)
        plt.plot(traj["time"], traj["lateral_error"],
                 label=ALGO_LABELS[algo], color=ALGO_COLORS[algo])
    plt.axhline(0, color="k", linewidth=0.5)
    plt.xlabel("time [s]")
    plt.ylabel("lateral error [m]")
    plt.title(f"Lateral error - {ROUTE_LABELS[route]} (medium)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    out = os.path.join(FIGS_DIR, f"error_{route}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"[OK] {out}")


def plot_steer_per_route(route):
    plt.figure(figsize=(10, 4))
    for algo in ALGOS:
        d = find_run_dir(algo, route)
        if d is None:
            continue
        traj = load_traj(d)
        plt.plot(traj["time"], np.rad2deg(traj["steer"]),
                 label=ALGO_LABELS[algo], color=ALGO_COLORS[algo])
    plt.axhline(35, color="r", linestyle="--", linewidth=0.8, label="limit 35 deg")
    plt.axhline(-35, color="r", linestyle="--", linewidth=0.8)
    plt.xlabel("time [s]")
    plt.ylabel("steer [deg]")
    plt.title(f"Steering - {ROUTE_LABELS[route]} (medium)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    out = os.path.join(FIGS_DIR, f"steer_{route}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"[OK] {out}")


def plot_traj_per_route(route):
    plt.figure(figsize=(7, 6))
    d0 = find_run_dir("pp", route)
    if d0:
        ref = load_ref(d0)
        if ref is not None and "x" in ref.dtype.names:
            plt.plot(ref["x"], ref["y"], "k--", linewidth=1, label="reference")
    for algo in ALGOS:
        d = find_run_dir(algo, route)
        if d is None:
            continue
        traj = load_traj(d)
        plt.plot(traj["x"], traj["y"], label=ALGO_LABELS[algo],
                 color=ALGO_COLORS[algo])
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Trajectory - {ROUTE_LABELS[route]} (medium)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    out = os.path.join(FIGS_DIR, f"traj_{route}.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"[OK] {out}")


def main():
    for route in ROUTES:
        plot_error_per_route(route)
        plot_steer_per_route(route)
        plot_traj_per_route(route)
    print(f"\n所有图保存到: {FIGS_DIR}")


if __name__ == "__main__":
    main()