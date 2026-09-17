"""Lock the published student RESULTS.md tables against closed-loop reruns.

GPX cases are omitted here because they are too slow for the default suite;
they were checked against ``outputs/20260917_200917_student/index.json``.
"""

import unittest

from vdm_lab.student.experiments.runner import run_one


def _replay(**kwargs):
    kwargs.setdefault("save_fig", False)
    kwargs.setdefault("save_animation", False)
    kwargs.setdefault("persist", False)
    return run_one(**kwargs)


class PublishedResultsTests(unittest.TestCase):
    def test_plant_compare_table(self):
        cases = (
            ("kinematic", True, 0.279, 0.487),
            ("dynamic", True, 0.249, 0.468),
        )
        for plant, reached, mean_ey, max_ey in cases:
            with self.subTest(plant=plant):
                result = _replay(
                    algo="pp",
                    label=f"plant_pp_circle_medium_{plant}",
                    route="circle",
                    speed_mode="medium",
                    plant=plant,
                )
                self.assertEqual(result["reached_goal"], reached)
                self.assertEqual(round(result["mean_lateral_error_m"], 3), mean_ey)
                self.assertEqual(round(result["max_lateral_error_m"], 3), max_ey)

    def test_circle_speed_table(self):
        # Columns match RESULTS.md §4 at the printed rounding.
        expected = {
            ("pp", "low"): (3.000, 0.2140, 4.20, 0.2593, 3.73, 0.750, 0.00, 0.100, 0.438, True),
            ("pp", "medium"): (5.000, 0.2152, 4.77, 0.4345, 4.30, 2.083, 0.00, 0.121, 0.465, True),
            ("pp", "high"): (6.994, 0.2151, 4.74, 0.6077, 4.27, 4.077, 0.00, 0.158, 0.481, True),
            ("lqr_kinematic", "low"): (3.000, 0.2119, 3.19, 0.2569, 2.77, 0.750, 0.00, 0.036, 0.342, True),
            ("lqr_kinematic", "medium"): (5.000, 0.2134, 3.90, 0.4310, 3.44, 2.083, 0.00, 0.052, 0.372, True),
            ("lqr_kinematic", "high"): (6.994, 0.2133, 3.84, 0.6030, 3.46, 4.077, 0.00, 0.065, 0.433, True),
            ("mpc", "low"): (3.000, 0.2075, 1.01, 0.2513, 0.52, 0.750, 0.00, 0.012, 0.060, True),
            ("mpc", "medium"): (5.000, 0.2089, 1.72, 0.4218, 1.22, 2.083, 0.00, 0.019, 0.136, True),
            ("mpc", "high"): (6.993, 0.2092, 1.87, 0.5915, 1.50, 4.075, 0.00, 0.008, 0.103, True),
        }
        for (algo, speed), row in expected.items():
            with self.subTest(algo=algo, speed=speed):
                result = _replay(
                    algo=algo,
                    label=f"circle_{algo}_{speed}",
                    route="circle",
                    speed_mode=speed,
                )
                circle = result["circle"]
                (
                    mean_v,
                    mean_delta,
                    err_delta,
                    mean_r,
                    err_r,
                    mean_an,
                    err_an,
                    entry_ey,
                    steady_mae,
                    reached,
                ) = row
                self.assertEqual(result["reached_goal"], reached)
                self.assertEqual(round(circle["mean_speed"], 3), mean_v)
                self.assertEqual(round(circle["mean_steer"], 4), mean_delta)
                self.assertEqual(round(circle["rel_err_steer_pct"], 2), err_delta)
                self.assertEqual(round(circle["mean_yaw_rate"], 4), mean_r)
                self.assertEqual(round(circle["rel_err_yaw_rate_pct"], 2), err_r)
                self.assertEqual(round(circle["mean_normal_accel"], 3), mean_an)
                self.assertEqual(round(circle["rel_err_normal_accel_pct"], 2), err_an)
                self.assertEqual(round(circle["entry_max_abs_ey"], 3), entry_ey)
                self.assertEqual(round(circle["steady_mae_ey"], 3), steady_mae)

    def test_task1_table(self):
        expected = (
            ("pp", "s_curve", True, 0.339, 1.171, 15.0, True),
            ("lqr_kinematic", "s_curve", True, 0.211, 0.529, 21.0, True),
            ("mpc", "s_curve", True, 0.054, 0.207, 28.5, True),
            ("pp", "mixed_course", True, 0.261, 0.768, 1.5, True),
            ("lqr_kinematic", "mixed_course", True, 0.184, 0.451, 3.0, True),
            ("mpc", "mixed_course", True, 0.053, 0.227, 8.5, True),
            ("pp", "right_angle", True, 0.133, 1.106, 4.5, True),
            ("lqr_kinematic", "right_angle", True, 0.096, 0.647, 6.5, True),
            ("mpc", "right_angle", True, 0.017, 0.164, 6.0, True),
            ("lqr_dynamic", "s_curve", True, 0.059, 0.184, 5.0, True),
            ("lqr_dynamic", "mixed_course", True, 0.041, 0.106, 6.5, True),
        )
        for algo, route, reached, mean_ey, max_ey, lag, after_peak in expected:
            with self.subTest(algo=algo, route=route):
                result = _replay(
                    algo=algo,
                    label=f"task1_{algo}_{route}_medium",
                    route=route,
                    speed_mode="medium",
                )
                self.assertEqual(result["reached_goal"], reached)
                self.assertEqual(round(result["mean_lateral_error_m"], 3), mean_ey)
                self.assertEqual(round(result["max_lateral_error_m"], 3), max_ey)
                self.assertEqual(round(result["curvature_lag"]["lag_m"], 1), lag)
                self.assertEqual(
                    result["curvature_lag"]["error_after_peak"], after_peak
                )

        result = _replay(
            algo="pp",
            label="task1_pp_mixed_course_medium_lf_shift",
            route="mixed_course",
            speed_mode="medium",
            vehicle_updates={"lf": 1.60, "lr": 0.90, "wheelbase": 2.50},
        )
        self.assertTrue(result["reached_goal"])
        self.assertEqual(round(result["mean_lateral_error_m"], 3), 0.193)
        self.assertEqual(round(result["max_lateral_error_m"], 3), 0.639)
        self.assertEqual(round(result["curvature_lag"]["lag_m"], 1), 0.0)
        self.assertFalse(result["curvature_lag"]["error_after_peak"])
