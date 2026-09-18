"""Course task-1 extras: multi-route comparison, dynamic LQR, vehicle params."""

from vdm_lab.student.experiments.runner import run_many


ROUTES = ("s_curve", "mixed_course", "right_angle")
ALGOS = ("pp", "lqr_kinematic", "mpc")


def main():
    jobs = [
        {
            "algo": algo,
            "label": f"task1_{algo}_{route}_medium",
            "route": route,
            "speed_mode": "medium",
            "extra": {"task1": "route_algo_grid"},
        }
        for route in ROUTES
        for algo in ALGOS
    ]
    jobs.extend(
        {
            "algo": "lqr_dynamic",
            "label": f"task1_lqr_dynamic_{route}_medium",
            "route": route,
            "speed_mode": "medium",
            "extra": {"task1": "dynamic_lqr"},
        }
        for route in ("s_curve", "mixed_course")
    )
    jobs.append(
        {
            "algo": "pp",
            "label": "task1_pp_mixed_course_medium_lf_shift",
            "route": "mixed_course",
            "speed_mode": "medium",
            "vehicle_updates": {"lf": 1.60, "lr": 0.90, "wheelbase": 2.50},
            "extra": {"task1": "vehicle_lf_lr"},
        }
    )
    return run_many(jobs)


if __name__ == "__main__":
    main()
