"""
A-2: PP lookahead 参数敏感性分析。
从 outputs/ 下找带 lookahead_value.json 标记的 PP/s_curve/medium 实验，
生成 parameter_comparison.csv 并画出 lookahead 影响曲线。
"""
import os
import glob
import json
import csv
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUTS_DIR = os.path.join(PROJECT_ROOT, "outputs")
RESULT_CSV = os.path.join(PROJECT_ROOT, "parameter_comparison.csv")
FIGS_DIR = os.path.join(PROJECT_ROOT, "analysis", "figs")
os.makedirs(FIGS_DIR, exist_ok=True)


def load_metrics(run_dir):
    with open(os.path.join(run_dir, "metrics.json"), "r", encoding="utf-8") as f:
        return json.load(f)


def load_traj(run_dir):
    return np.genfromtxt(os.path.join(run_dir, "trajectory.csv"),
                         delimiter=",", names=True)


def compute_j_delta(traj):
    steer = np.asarray(traj["steer"], dtype=float)
    t = np.asarray(traj["time"], dtype=float)
    if len(steer) < 2:
        return float("nan")
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    return float(np.nanmean(np.abs(np.diff(steer)) / dt))


def collect():
    rows = []
    for d in sorted(glob.glob(os.path.join(OUTPUTS_DIR, "*_pp_s_curve_medium"))):
        marker = os.path.join(d, "lookahead_value.json")
        if not os.path.exists(marker):
            continue
        with open(marker, "r", encoding="utf-8") as f:
            lookahead = json.load(f)["pp_base_lookahead"]

        m = load_metrics(d)
        traj = load_traj(d)
        j_delta = compute_j_delta(traj)

        rows.append({
            "pp_base_lookahead_m": lookahead,
            "reached_goal": m.get("reached_goal"),
            "mean_lateral_error_m": m.get("mean_lateral_error_m"),
            "max_lateral_error_m": m.get("max_lateral_error_m"),
            "max_steer_rad": m.get("max_steer_rad"),
            "J_delta": j_delta,
            "steps": m.get("steps"),
            "finish_error_m": m.get("finish_error_m"),
            "run_dir": os.path.basename(d),
        })
    rows.sort(key=lambda r: r["pp_base_lookahead_m"])
    return rows


def main():
    rows = collect()
    if not rows:
        print("没找到带 marker 的实验结果")
        return

    fields = list(rows[0].keys())
    with open(RESULT_CSV, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print(f"[OK] 写入 {RESULT_CSV}\n")
    header = (f"{'lookahead':<12}{'reached':<10}{'mean_err':<12}"
              f"{'max_err':<12}{'max_steer':<12}{'J_delta':<12}")
    print(header)
    print("-" * len(header))
    for r in rows:
        print(f"{r['pp_base_lookahead_m']:<12.2f}"
              f"{str(r['reached_goal']):<10}"
              f"{r['mean_lateral_error_m']:<12.4f}"
              f"{r['max_lateral_error_m']:<12.4f}"
              f"{r['max_steer_rad']:<12.4f}"
              f"{r['J_delta']:<12.4f}")

    # ---- 画图 ----
    L = [r["pp_base_lookahead_m"] for r in rows]
    mean_err = [r["mean_lateral_error_m"] for r in rows]
    max_err = [r["max_lateral_error_m"] for r in rows]
    jd = [r["J_delta"] for r in rows]
    ms = [r["max_steer_rad"] for r in rows]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].plot(L, mean_err, "o-", color="tab:blue")
    axes[0, 0].set_xlabel("pp_base_lookahead [m]")
    axes[0, 0].set_ylabel("mean lateral error [m]")
    axes[0, 0].set_title("Mean lateral error vs lookahead")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(L, max_err, "o-", color="tab:red")
    axes[0, 1].set_xlabel("pp_base_lookahead [m]")
    axes[0, 1].set_ylabel("max lateral error [m]")
    axes[0, 1].set_title("Max lateral error vs lookahead")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(L, jd, "o-", color="tab:green")
    axes[1, 0].set_xlabel("pp_base_lookahead [m]")
    axes[1, 0].set_ylabel("J_delta [rad/s]")
    axes[1, 0].set_title("Steering rate (J_delta) vs lookahead")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(L, np.rad2deg(ms), "o-", color="tab:orange")
    axes[1, 1].axhline(35, color="r", linestyle="--", label="limit 35 deg")
    axes[1, 1].set_xlabel("pp_base_lookahead [m]")
    axes[1, 1].set_ylabel("max steer [deg]")
    axes[1, 1].set_title("Max steer vs lookahead")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    plt.tight_layout()
    out = os.path.join(FIGS_DIR, "lookahead_sensitivity.png")
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"\n[OK] 图片: {out}")

    # ---- 额外：画三条误差曲线叠加 ----
    plt.figure(figsize=(10, 4))
    colors = {1.5: "tab:blue", 3.0: "tab:orange", 5.0: "tab:green"}
    for r in rows:
        d = os.path.join(OUTPUTS_DIR, r["run_dir"])
        traj = load_traj(d)
        plt.plot(traj["time"], traj["lateral_error"],
                 label=f"Lf={r['pp_base_lookahead_m']} m",
                 color=colors.get(r["pp_base_lookahead_m"], "gray"))
    plt.axhline(0, color="k", linewidth=0.5)
    plt.xlabel("time [s]")
    plt.ylabel("lateral error [m]")
    plt.title("PP lateral error vs lookahead (s_curve, medium)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    out2 = os.path.join(FIGS_DIR, "lookahead_error_curves.png")
    plt.savefig(out2, dpi=150)
    plt.close()
    print(f"[OK] 图片: {out2}")


if __name__ == "__main__":
    main()