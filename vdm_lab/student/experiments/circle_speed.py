"""Circle speed sweep: PP / kinematic LQR / MPC × low / medium / high."""

from vdm_lab.student.experiments.runner import run_many


ALGOS = ("pp", "lqr_kinematic", "mpc")
SPEEDS = ("low", "medium", "high")
GIF_KEYS = {("pp", "low"), ("pp", "high")}


def main():
    return run_many(
        {
            "algo": algo,
            "label": f"circle_{algo}_{speed}",
            "route": "circle",
            "speed_mode": speed,
            "save_fig": True,
            "save_animation": (algo, speed) in GIF_KEYS,
        }
        for algo in ALGOS
        for speed in SPEEDS
    )


if __name__ == "__main__":
    main()
