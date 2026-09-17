"""GPX navigation on the existing homework route (no new BRouter export)."""

from vdm_lab.student.experiments.runner import GPX_ROUTE, run_many


ALGOS = ("pp", "lqr_kinematic", "mpc")
BASELINE_SPEED = 8.0
IMPROVED_SPEED = 5.0


def main():
    jobs = [
        {
            "algo": algo,
            "label": f"gpx_{algo}_v8",
            "gpx_file": GPX_ROUTE,
            "speed_mode": "medium",
            "target_speed": BASELINE_SPEED,
            "extra": {"gpx_group": "baseline_8mps"},
        }
        for algo in ALGOS
    ]
    jobs.append(
        {
            "algo": "lqr_kinematic",
            "label": "gpx_lqr_kinematic_v5",
            "gpx_file": GPX_ROUTE,
            "speed_mode": "medium",
            "target_speed": IMPROVED_SPEED,
            "extra": {"gpx_group": "lqr_slowdown_5mps"},
        }
    )
    return run_many(jobs)


if __name__ == "__main__":
    main()
