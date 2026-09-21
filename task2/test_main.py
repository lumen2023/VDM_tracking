"""任务二统计口径测试：运行 python -m unittest discover -s task2 -v。"""
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

try:
    import main as sut
except ImportError:
    sut = None


class AnalysisTests(unittest.TestCase):
    def require_module(self):
        self.assertIsNotNone(sut, "任务二 main.py 尚未实现")

    def fixture(self):
        vehicle = SimpleNamespace(lf=1.25, lr=1.25, max_steer=0.6,
                                  max_steer_rate=0.8)
        config = SimpleNamespace(vehicle=vehicle, sim=SimpleNamespace(dt=1.0))
        path = SimpleNamespace(curvature=[0] + [1 / 12] * 11 + [0],
                               s=list(range(13)))
        rows = [dict(time=float(i), target_index=i, curvature=k,
                     speed=3.0, steer=0.2 + 0.01 * i,
                     yaw_rate=0.25, normal_accel=0.75,
                     lateral_error=(-1 if i % 2 else 1) * 0.1,
                     heading_error=0.01, beta=0.1, x=float(i), y=0.0)
                for i, k in enumerate(path.curvature)]
        return config, path, rows

    def test_theory_uses_speed_squared_and_vehicle_wheelbase(self):
        # 错用 v/R 计算加速度或忽略轴距时本测试失败。
        self.require_module()
        car = SimpleNamespace(lf=1.25, lr=1.25)
        low = sut.theoretical_values(3, car)
        high = sut.theoretical_values(7, car)
        self.assertAlmostEqual(low['normal_accel'], 0.75)
        self.assertAlmostEqual(high['normal_accel'], 4.083333333333333)
        self.assertAlmostEqual(high['yaw_rate'], 0.5833333333333334)
        self.assertAlmostEqual(low['steer'], 0.2053953891897674)
        self.assertEqual(low['steer'], high['steer'])

    def test_relative_error_has_explicit_zero_and_nonfinite_handling(self):
        self.require_module()
        self.assertAlmostEqual(sut.relative_error(2.2, 2.0), 10.0)
        self.assertIsNone(sut.relative_error(1.0, 0.0))
        with self.assertRaises(ValueError):
            sut.relative_error(float('nan'), 1.0)

    def test_steady_selection_trims_by_arc_progress_not_elapsed_time(self):
        # 将全部圆弧或两端过渡记录混入稳态统计时失败。
        self.require_module()
        _, path, rows = self.fixture()
        self.assertEqual(sut.select_steady_indices(rows, path), list(range(3, 10)))
        with self.assertRaises(ValueError):
            sut.select_steady_indices(rows, path, trim_fraction=0.5)

    def test_no_circle_returns_empty_instead_of_using_straight_records(self):
        self.require_module()
        _, path, rows = self.fixture()
        path.curvature = [0] * 13
        self.assertEqual(sut.select_steady_indices(rows, path), [])

    def test_summary_uses_absolute_error_and_actual_speed(self):
        # 正负误差相抵、忽略未到终点或用目标速度代替实际速度时失败。
        self.require_module()
        config, path, rows = self.fixture()
        for row in rows:
            row['speed'] = 2.0
            row['normal_accel'] = 1 / 3
        result = sut.summarize_run('pp', 'low', config, path, rows,
                                   {'reached_goal': False}, target_speed=3)
        self.assertFalse(result['reached_goal'])
        self.assertAlmostEqual(result['steady_mean_abs_error_m'], 0.1)
        self.assertAlmostEqual(result['steady_mean_speed_mps'], 2.0)
        self.assertAlmostEqual(result['normal_accel_actual_speed_theory'], 1 / 3)
        self.assertAlmostEqual(result['normal_accel_actual_speed_relative_error_pct'], 0)
        self.assertAlmostEqual(result['steady_steer_activity_radps'], 0.01)

    def test_steer_activity_does_not_bridge_nonconsecutive_segments(self):
        self.require_module()
        _, _, rows = self.fixture()
        self.assertIsNone(sut.steering_activity(rows, [1, 3]))
        self.assertAlmostEqual(sut.steering_activity(rows, [1, 2, 3]), 0.01)

    def test_numpy_vehicle_parameter_can_be_used_for_saturation_fraction(self):
        self.require_module()
        import numpy as np
        config, path, rows = self.fixture()
        config.vehicle.max_steer = np.float64(0.25)
        result = sut.summarize_run('pp', 'low', config, path, rows,
                                   {'reached_goal': True}, target_speed=3)
        self.assertAlmostEqual(result['steer_saturation_fraction'], 8 / 13)

    def test_phase_error_separates_entry_central_and_exit(self):
        self.require_module()
        config, path, rows = self.fixture()
        rows[2]['lateral_error'] = 5.0
        rows[5]['lateral_error'] = -2.0
        rows[11]['lateral_error'] = 3.0
        result = sut.summarize_run('pp', 'low', config, path, rows,
                                   {'reached_goal': True}, target_speed=3)
        self.assertEqual(result.get('entry_max_abs_error_m'), 5.0)
        self.assertEqual(result.get('central_arc_max_abs_error_m'), 2.0)
        self.assertEqual(result.get('exit_max_abs_error_m'), 3.0)

    def test_acceleration_correction_uses_mean_square_not_square_mean(self):
        self.require_module()
        config, path, rows = self.fixture()
        for i in range(3, 10):
            rows[i]['speed'] = 1.0 if i < 6 else 3.0
            rows[i]['normal_accel'] = rows[i]['speed'] ** 2 / 12
        result = sut.summarize_run('pp', 'low', config, path, rows,
                                   {'reached_goal': True}, target_speed=3)
        self.assertAlmostEqual(result['normal_accel_actual_speed_theory'], 39 / 84)
        self.assertEqual(result.get('steady_min_speed_mps'), 1.0)
        self.assertEqual(result.get('steady_max_speed_mps'), 3.0)

    def test_write_table_preserves_failure_and_has_no_fake_zero_for_missing_data(self):
        self.require_module()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'table.csv'
            sut.write_table(output, [{'algorithm': 'pp', 'reached_goal': False,
                                      'steady_mean_speed_mps': None}])
            import csv
            with output.open(encoding='utf-8-sig', newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]['reached_goal'], 'False')
            self.assertEqual(rows[0]['steady_mean_speed_mps'], '')

    def test_real_pp_pipeline_outputs_figures_and_reanalysis_preserves_raw_log(self):
        # 未保存真实日志、统计表、图表，或重分析覆盖原始数据时失败。
        self.require_module()
        self.assertTrue(callable(getattr(sut, 'run_experiments', None)), '实验入口未实现')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'experiment'
            summaries = sut.run_experiments(output, ['pp'], ['low'], make_figures=True)
            self.assertEqual(len(summaries), 1)
            self.assertGreater(summaries[0]['steady_samples'], 10)
            self.assertEqual(summaries[0]['target_speed_mps'], 3.0)
            raw = output / 'runs' / 'pp_low' / 'trajectory.csv'
            before = raw.read_bytes()
            self.assertTrue((output / 'overall_metrics.csv').is_file())
            self.assertTrue((output / 'figures' / 'metrics_vs_speed.png').is_file())
            self.assertTrue((output / 'runs' / 'pp_low' / 'summary.png').is_file())
            import json
            stats_file = output / 'runs' / 'pp_low' / 'statistics.json'
            stats_file.write_text('{"stale": true}', encoding='utf-8')
            again = sut.analyze_existing(output, make_figures=False)
            self.assertAlmostEqual(again[0]['steady_mean_abs_error_m'],
                                   summaries[0]['steady_mean_abs_error_m'])
            self.assertEqual(raw.read_bytes(), before)
            self.assertAlmostEqual(json.loads(stats_file.read_text(encoding='utf-8'))
                                   .get('steady_mean_abs_error_m', -1),
                                   summaries[0]['steady_mean_abs_error_m'])

    def test_rerun_refuses_existing_results_instead_of_overwriting(self):
        self.require_module()
        self.assertTrue(callable(getattr(sut, 'run_experiments', None)), '实验入口未实现')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'experiment'
            output.mkdir()
            marker = output / 'manifest.json'
            marker.write_text('{"keep": true}', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                sut.run_experiments(output, ['pp'], ['low'], make_figures=False)
            self.assertEqual(marker.read_text(encoding='utf-8'), '{"keep": true}')

    def test_plot_failure_preserves_completed_simulation_for_reanalysis(self):
        self.require_module()
        # 仅替换有文件系统副作用的绘图边界；仿真、日志与统计全部真实执行。
        import sys
        sys.path.insert(0, str(sut.ROOT))
        from unittest.mock import patch
        import json
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'experiment'
            with patch('vdm_lab.common.visualization.save_summary', side_effect=OSError('render unavailable')):
                results = sut.run_experiments(output, ['pp'], ['low'], make_figures=True)
            self.assertEqual(len(results), 1)
            manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(manifest['runs'][0]['status'], 'completed')
            self.assertIn('render unavailable', manifest['runs'][0]['figure_error'])
            self.assertEqual(len(sut.analyze_existing(output, make_figures=False)), 1)


if __name__ == '__main__':
    unittest.main()
