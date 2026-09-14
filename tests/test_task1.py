"""Unit checks plus raw-data validation/replay when the delivered batch exists."""

from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from scripts.run_task1 import Case, ObservedController, extended_metrics, make_cases, make_config
from scripts.analyze_task1 import export_table, load_runs, validate_run
import numpy as np
from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import ControlCommand, Path as ReferencePath, StepRecord

ROOT = Path(__file__).resolve().parents[1]
DELIVERED = ROOT / "outputs" / "task1" / "20260911_task1"


class ExperimentDesignTests(unittest.TestCase):
    def test_matrix_has_nine_core_four_new_parameter_two_supplement_cases(self):
        cases = make_cases()
        self.assertEqual(len(cases), 15)
        self.assertEqual(len({case.name for case in cases}), 15)
        self.assertEqual(sum(case.group == "core" for case in cases), 9)
        self.assertEqual(sum(case.group == "steering" for case in cases), 4)
        self.assertEqual(sum(case.group == "supplement" for case in cases), 2)
        self.assertEqual(len(make_cases(False)), 13)

    def test_vehicle_parameter_experiment_changes_only_max_steer(self):
        original = make_config(Case("core", "right_angle", "pp"))
        altered = make_config(Case("steering", "right_angle", "pp", 15))
        self.assertNotEqual(original.vehicle.max_steer, altered.vehicle.max_steer)
        a, b = asdict(original.vehicle), asdict(altered.vehicle)
        a.pop("max_steer")
        b.pop("max_steer")
        self.assertEqual(a, b)
        self.assertEqual(asdict(original.sim), asdict(altered.sim))
        self.assertEqual(original.vehicle.wheelbase, original.vehicle.lf + original.vehicle.lr)

    def test_observer_does_not_change_original_simulation(self):
        case = Case("core", "double_lane_change", "pp")
        config = make_config(case)
        config.sim.max_time = 0.5
        module = load_controller("pp", "solution")
        _, unobserved, _ = run_simulation(module, config=config)
        observed = ObservedController(module, case)
        _, records, _ = run_simulation(observed, config=config)
        self.assertEqual(records, unobserved)
        self.assertEqual(observed.records, records)

    def test_observer_keeps_partial_records_if_controller_fails(self):
        class Broken:
            NAME = "Test controller"
            calls = 0

            def control(self, *args):
                self.calls += 1
                if self.calls == 2:
                    raise RuntimeError("injected test failure")
                return ControlCommand(acceleration=1.0, steer=0.0)

        case = Case("core", "right_angle", "pp")
        config = make_config(case)
        config.sim.max_time = 0.5
        observed = ObservedController(Broken(), case)
        with self.assertRaisesRegex(RuntimeError, "injected test failure"):
            run_simulation(observed, config=config)
        self.assertEqual(len(observed.records), 1)
        self.assertAlmostEqual(observed.failure_state["time_s"], 0.1)


class MetricTests(unittest.TestCase):
    def setUp(self):
        self.config = make_config(Case("core", "right_angle", "pp"))
        self.path = ReferencePath(x=np.array([0., 1., 2.]), y=np.zeros(3), yaw=np.zeros(3), curvature=np.zeros(3), s=np.array([0., 1., 2.]), target_speed=np.ones(3))
        self.record = StepRecord(time=0., x=0., y=1., yaw=0., speed=1., acceleration=0., steer=0., beta=0., yaw_rate=0.,
                                 target_index=0, lateral_error=1., heading_error=0., curvature=0., normal_accel=0., target_speed=1.)

    def test_absolute_error_and_rate_not_signed_averages(self):
        records = [self.record, replace(self.record, time=0.1, x=1., y=-2., lateral_error=-2., steer=0.1, target_index=1)]
        values = extended_metrics(self.path, records, self.config)
        self.assertEqual(values["mean_lateral_error_m"], 1.5)
        self.assertEqual(values["max_lateral_error_m"], 2.)
        self.assertAlmostEqual(values["rms_lateral_error_m"], np.sqrt(2.5))
        self.assertAlmostEqual(values["mean_abs_steer_rate_radps"], 1.)
        self.assertIsNone(values["completion_time_s"])
        self.assertFalse(values["reached_goal"])

    def test_completion_requires_both_position_and_speed(self):
        fast_at_goal = replace(self.record, x=2., y=0., target_index=2, lateral_error=0., time=1.)
        self.assertFalse(extended_metrics(self.path, [fast_at_goal], self.config)["reached_goal"])
        stopped = replace(fast_at_goal, speed=0.1)
        values = extended_metrics(self.path, [stopped], self.config)
        self.assertTrue(values["reached_goal"])
        self.assertEqual(values["completion_time_s"], 1.)
        self.assertEqual(values["mean_abs_steer_rate_radps"], 0.)

    def test_empty_trace_does_not_invent_metrics(self):
        values = extended_metrics(self.path, [], self.config)
        self.assertEqual(values["steps"], 0)
        self.assertIsNone(values["completion_time_s"])

    def test_markdown_pipes_are_escaped_but_csv_is_unchanged(self):
        with tempfile.TemporaryDirectory(prefix="vdm-table-test-") as directory:
            text = export_table(Path(directory), "example", [{"x": "a|b"}], [("x", "max |beta|")])
            self.assertIn("max \\|beta\\|", text)
            self.assertIn("a\\|b", text)
            self.assertIn("a|b", (Path(directory) / "example.csv").read_text(encoding="utf-8-sig"))


@unittest.skipUnless((DELIVERED / "results.json").exists(), "Delivered experiment data not available")
class DeliveredDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest, cls.runs = load_runs(DELIVERED)

    def test_all_fifteen_raw_traces_are_valid(self):
        self.assertEqual(len(self.runs), 15)
        for run in self.runs:
            with self.subTest(case=run["name"]):
                validate_run(run, self.manifest["speed_mode"])
                saved = json.loads((run["directory"] / "metrics.json").read_text())
                self.assertEqual(saved, run["metrics"])

    def test_simulation_source_matches_provenance(self):
        for relative, expected in self.manifest["source_sha256"].items():
            with self.subTest(file=relative):
                actual = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
                self.assertEqual(actual, expected)

    def test_pp_25_degree_limit_is_inactive(self):
        def get(limit):
            return next(run for run in self.runs if run["case"]["algo"] == "pp" and run["case"]["route"] == "right_angle" and run["case"]["max_steer_deg"] == limit)
        baseline, reduced = get(35), get(25)
        self.assertLess(baseline["metrics"]["max_steer_rad"], np.deg2rad(25))
        np.testing.assert_array_equal(baseline["trajectory"], reduced["trajectory"])

    def test_replay_one_complete_case_per_core_algorithm(self):
        for algo in ("pp", "lqr_kinematic", "mpc"):
            with self.subTest(algo=algo):
                saved = next(run for run in self.runs if run["case"] == {"group": "core", "route": "right_angle", "algo": algo, "max_steer_deg": 35.})
                config = make_config(Case(**saved["case"]), self.manifest["speed_mode"])
                _, records, _ = run_simulation(load_controller(algo, "solution"), config=config)
                self.assertEqual(len(records), len(saved["trajectory"]))
                for field in saved["trajectory"].dtype.names:
                    np.testing.assert_allclose([getattr(record, field) for record in records], saved["trajectory"][field], atol=1e-8, rtol=1e-8)


if __name__ == "__main__":
    unittest.main()
