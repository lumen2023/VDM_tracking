"""任务二：所有新增实验与分析代码集中在本文件；原控制器不改动。

从仓库根目录运行：task2/.venv/Scripts/python.exe task2/main.py
统计函数只依赖标准库，运行与绘图才导入原仓库和第三方库。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import sys
from dataclasses import asdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASK_DIR = Path(__file__).resolve().parent
RADIUS = 12.0
ALGORITHMS = ('pp', 'lqr_kinematic', 'mpc')
SPEEDS = {'low': 3.0, 'medium': 5.0, 'high': 7.0}
TITLES = {'pp': 'PP', 'lqr_kinematic': 'LQR kinematic', 'mpc': 'MPC'}
# 缓存也保留在独立任务目录；禁止弹出交互窗口。
os.environ.setdefault('MPLCONFIGDIR', str(TASK_DIR / '.cache' / 'matplotlib'))
os.environ.setdefault('MPLBACKEND', 'Agg')


def theoretical_values(speed, vehicle, radius=RADIUS):
    """课程的小侧偏稳态近似；速度和半径以 SI 单位传入。"""
    if not math.isfinite(speed) or speed < 0 or radius <= 0:
        raise ValueError('速度必须非负且有限，半径必须大于零')
    return {'steer': math.atan((vehicle.lf + vehicle.lr) / radius),
            'yaw_rate': speed / radius, 'normal_accel': speed * speed / radius}


def relative_error(simulation, theory):
    if not math.isfinite(simulation) or not math.isfinite(theory):
        raise ValueError('相对误差不接受非有限数')
    return None if theory == 0 else abs(simulation - theory) / abs(theory) * 100


def select_steady_indices(rows, path, trim_fraction=0.2, tolerance=0.005):
    """按参考弧长剔除圆弧前后各20%，所有算法用同一空间窗口。

    这只是统一的“稳态候选窗口”，不会自动证明控制器已经收敛。
    """
    if not 0 <= trim_fraction < 0.5:
        raise ValueError('trim_fraction 必须在 [0, 0.5) 内')
    arc = [i for i, k in enumerate(path.curvature) if abs(k - 1 / RADIUS) < tolerance]
    if not arc:
        return []
    start, end = float(path.s[arc[0]]), float(path.s[arc[-1]])
    margin = (end - start) * trim_fraction
    return [i for i, row in enumerate(rows)
            if abs(row['curvature'] - 1 / RADIUS) < tolerance
            and start + margin <= float(path.s[int(row['target_index'])]) <= end - margin]


def steering_activity(rows, indices):
    """只用连续仿真步计算 mean(|Δδ|/Δt)，不跨越被剔除的间隙。"""
    rates = [abs(rows[j]['steer'] - rows[i]['steer']) /
             (rows[j]['time'] - rows[i]['time'])
             for i, j in zip(indices, indices[1:])
             if j == i + 1 and rows[j]['time'] > rows[i]['time']]
    return statistics.mean(rates) if rates else None


def summarize_run(algorithm, speed_mode, config, path, rows, metrics, target_speed):
    if not rows:
        raise ValueError('不能统计空日志')
    indices = select_steady_indices(rows, path)
    selected = [rows[i] for i in indices]
    nominal = theoretical_values(target_speed, config.vehicle)
    arc = [i for i, k in enumerate(path.curvature) if abs(k - 1 / RADIUS) < 0.005]
    arc_start = float(path.s[arc[0]]) if arc else None
    arc_end = float(path.s[arc[-1]]) if arc else None
    arc_length = arc_end - arc_start if arc else None
    lower = arc_start + 0.2 * arc_length if arc else None
    upper = arc_end - 0.2 * arc_length if arc else None
    peak = max(rows, key=lambda row: abs(row['lateral_error']))
    peak_s = float(path.s[int(peak['target_index'])])
    stage = ('straight' if arc_start is None or not arc_start <= peak_s <= arc_end
             else 'entry' if peak_s < lower else 'exit' if peak_s > upper else 'central_arc')
    result = dict(metrics)
    result.update(algorithm=algorithm, speed_mode=speed_mode, target_speed_mps=target_speed,
                  duration_s=rows[-1]['time'] - rows[0]['time'],
                  mean_lateral_error_m=statistics.mean(abs(r['lateral_error']) for r in rows),
                  max_lateral_error_m=max(abs(r['lateral_error']) for r in rows),
                  peak_time_s=peak['time'], peak_progress_m=peak_s, peak_stage=stage,
                  steady_samples=len(selected), steady_s_start_m=lower, steady_s_end_m=upper,
                  steady_time_start_s=selected[0]['time'] if selected else None,
                  steady_time_end_s=selected[-1]['time'] if selected else None,
                  steer_theory=nominal['steer'], yaw_rate_theory=nominal['yaw_rate'],
                  normal_accel_theory=nominal['normal_accel'],
                  all_steer_activity_radps=steering_activity(rows, list(range(len(rows)))),
                  steer_saturation_fraction=statistics.mean(
                      int(abs(r['steer']) >= 0.99 * config.vehicle.max_steer) for r in rows),
                  max_observed_steer_rate_radps=max(
                      [abs(b['steer'] - a['steer']) / (b['time'] - a['time'])
                       for a, b in zip(rows, rows[1:]) if b['time'] > a['time']] or [0]),
                  analysis_warning='' if len(selected) >= 10 else 'too_few_central_arc_samples')
    keys = ('steady_mean_speed_mps', 'steady_mean_abs_error_m', 'steady_mean_signed_error_m',
            'steady_max_abs_error_m', 'steady_signed_error_std_m', 'steady_beta_max_rad',
            'steady_steer_activity_radps', 'steady_steer_mean_rad', 'steady_yaw_rate_mean_radps',
            'steady_normal_accel_mean_mps2', 'steady_model_v_yaw_rate_mean_mps2',
            'steer_relative_error_pct', 'yaw_rate_relative_error_pct', 'normal_accel_relative_error_pct',
            'yaw_rate_actual_speed_theory', 'normal_accel_actual_speed_theory',
            'yaw_rate_actual_speed_relative_error_pct', 'normal_accel_actual_speed_relative_error_pct')
    result.update(dict.fromkeys(keys))
    for phase in ('straight', 'entry', 'central_arc', 'exit'):
        phase_rows = []
        for row in rows:
            progress = float(path.s[int(row['target_index'])])
            row_phase = ('straight' if arc_start is None or not arc_start <= progress <= arc_end
                         else 'entry' if progress < lower else 'exit' if progress > upper else 'central_arc')
            if phase == row_phase:
                phase_rows.append(row)
        result[phase + '_max_abs_error_m'] = max((abs(r['lateral_error']) for r in phase_rows), default=None)
    result.update(steady_min_speed_mps=None, steady_max_speed_mps=None, steady_speed_std_mps=None)
    if selected:
        speed = statistics.mean(r['speed'] for r in selected)
        # E[v²]，不能用 E[v]² 代替，以免忽略速度波动。
        actual_an = statistics.mean(r['speed'] ** 2 / RADIUS for r in selected)
        result.update(steady_mean_speed_mps=speed,
                      steady_min_speed_mps=min(r['speed'] for r in selected),
                      steady_max_speed_mps=max(r['speed'] for r in selected),
                      steady_speed_std_mps=statistics.pstdev(r['speed'] for r in selected),
                      steady_mean_abs_error_m=statistics.mean(abs(r['lateral_error']) for r in selected),
                      steady_mean_signed_error_m=statistics.mean(r['lateral_error'] for r in selected),
                      steady_max_abs_error_m=max(abs(r['lateral_error']) for r in selected),
                      steady_signed_error_std_m=statistics.pstdev(r['lateral_error'] for r in selected),
                      steady_beta_max_rad=max(abs(r['beta']) for r in selected),
                      steady_steer_activity_radps=steering_activity(rows, indices),
                      steady_steer_mean_rad=statistics.mean(r['steer'] for r in selected),
                      steady_yaw_rate_mean_radps=statistics.mean(r['yaw_rate'] for r in selected),
                      steady_normal_accel_mean_mps2=statistics.mean(r['normal_accel'] for r in selected),
                      steady_model_v_yaw_rate_mean_mps2=statistics.mean(r['speed'] * r['yaw_rate'] for r in selected),
                      yaw_rate_actual_speed_theory=speed / RADIUS,
                      normal_accel_actual_speed_theory=actual_an)
        for field, label in [('steer', 'steady_steer_mean_rad'),
                             ('yaw_rate', 'steady_yaw_rate_mean_radps'),
                             ('normal_accel', 'steady_normal_accel_mean_mps2')]:
            result[field + '_relative_error_pct'] = relative_error(result[label], nominal[field])
        result['yaw_rate_actual_speed_relative_error_pct'] = relative_error(
            result['steady_yaw_rate_mean_radps'], speed / RADIUS)
        result['normal_accel_actual_speed_relative_error_pct'] = relative_error(
            result['steady_normal_accel_mean_mps2'], actual_an)
    return result


def write_table(output, rows):
    """UTF-8 BOM CSV 可直接用 Excel 打开；缺失值留空，不伪装成零。"""
    if not rows:
        raise ValueError('表格不能为空')
    with Path(output).open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_json(output, value):
    with Path(output).open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False,
                  default=lambda item: item.tolist() if hasattr(item, 'tolist') else str(item))


def save_analysis(output, summaries):
    write_json(output / 'analysis.json', summaries)
    write_table(output / 'all_statistics.csv', summaries)
    common = ['algorithm', 'speed_mode', 'target_speed_mps', 'reached_goal']
    groups = {
        'overall_metrics.csv': ['mean_lateral_error_m', 'max_lateral_error_m',
            'mean_heading_error_rad', 'max_steer_rad', 'max_normal_acceleration_mps2',
            'max_side_slip_beta_rad', 'max_yaw_rate_radps', 'duration_s', 'peak_stage',
            'peak_time_s', 'peak_progress_m', 'steer_saturation_fraction',
            'max_observed_steer_rate_radps'],
        'steady_metrics.csv': ['steady_samples', 'steady_s_start_m', 'steady_s_end_m',
            'steady_time_start_s', 'steady_time_end_s', 'steady_mean_speed_mps',
            'steady_min_speed_mps', 'steady_max_speed_mps', 'steady_speed_std_mps',
            'steady_mean_abs_error_m', 'steady_max_abs_error_m', 'steady_mean_signed_error_m',
            'steady_signed_error_std_m', 'steady_steer_mean_rad', 'steady_yaw_rate_mean_radps',
            'steady_normal_accel_mean_mps2', 'steady_model_v_yaw_rate_mean_mps2',
            'steady_beta_max_rad', 'steady_steer_activity_radps', 'analysis_warning'],
        'theory_comparison.csv': ['steady_mean_speed_mps', 'steer_theory',
            'steady_steer_mean_rad', 'steer_relative_error_pct', 'yaw_rate_theory',
            'steady_yaw_rate_mean_radps', 'yaw_rate_relative_error_pct', 'normal_accel_theory',
            'steady_normal_accel_mean_mps2', 'normal_accel_relative_error_pct',
            'yaw_rate_actual_speed_theory', 'yaw_rate_actual_speed_relative_error_pct',
            'normal_accel_actual_speed_theory', 'normal_accel_actual_speed_relative_error_pct']}
    groups['overall_metrics.csv'] += ['entry_max_abs_error_m', 'central_arc_max_abs_error_m',
                                      'exit_max_abs_error_m', 'straight_max_abs_error_m']
    for filename, fields in groups.items():
        write_table(output / filename, [{key: r.get(key) for key in common + fields} for r in summaries])


def make_comparison_figures(output, summaries, bundles):
    """科学数据图，无装饰性绘图；对应汇报的证据图集中输出。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False})
    figures = output / 'figures'
    figures.mkdir(exist_ok=True)
    algorithms = [a for a in ALGORITHMS if any(r['algorithm'] == a for r in summaries)]
    colors = {'low': '#2373B4', 'medium': '#D18B15', 'high': '#C73E45'}

    def finish(fig, name):
        fig.tight_layout()
        fig.savefig(figures / name, dpi=200, bbox_inches='tight')
        plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, (field, title) in zip(axes.flat, [
            ('mean_lateral_error_m', 'Whole-run mean absolute error [m]'),
            ('max_lateral_error_m', 'Whole-run maximum absolute error [m]'),
            ('steady_mean_abs_error_m', 'Central-arc mean absolute error [m]'),
            ('steady_steer_activity_radps', 'Central-arc steering activity [rad/s]')]):
        for algorithm in algorithms:
            group = sorted([r for r in summaries if r['algorithm'] == algorithm],
                           key=lambda r: r['target_speed_mps'])
            ax.plot([r['target_speed_mps'] for r in group],
                    [r.get(field) if r.get(field) is not None else np.nan for r in group],
                    '-o', label=TITLES[algorithm])
        ax.set(title=title, xlabel='Target speed [m/s]')
        ax.set_xticks([3, 5, 7])
        ax.grid(alpha=0.2)
        ax.legend()
    finish(fig, 'metrics_vs_speed.png')

    fig, axes = plt.subplots(len(algorithms), 2, figsize=(12, 4 * len(algorithms)), squeeze=False)
    for i, algorithm in enumerate(algorithms):
        for j, speed_mode in enumerate(['low', 'high']):
            ax = axes[i, j]
            bundle = bundles.get((algorithm, speed_mode))
            ax.set(title=f'{TITLES[algorithm]}: {speed_mode}', xlabel='x [m]', ylabel='y [m]')
            if bundle:
                path, rows = bundle
                ax.plot(path.x, path.y, '--', color='#666666', label='Reference')
                ax.plot([r['x'] for r in rows], [r['y'] for r in rows],
                        color=colors[speed_mode], label='Vehicle')
                ax.plot(path.x[0], path.y[0], 'o', color='#18855B', label='Start')
                ax.plot(path.x[-1], path.y[-1], 's', color='#333333', label='Goal')
                ax.set_aspect('equal', adjustable='box')
                ax.set_xlim(-3, 38)
                ax.set_ylim(-5, 30)
                ax.legend(loc='upper left')
            else:
                ax.text(0.5, 0.5, 'Not included', ha='center', transform=ax.transAxes)
            ax.grid(alpha=0.2)
    finish(fig, 'trajectory_low_high.png')

    for filename, field, ylabel in [('error_by_progress.png', 'lateral_error', 'Signed lateral error [m]'),
                                     ('speed_by_progress.png', 'speed', 'Actual speed [m/s]'),
                                     ('steer_by_progress.png', 'steer', 'Front steering [rad]')]:
        fig, axes = plt.subplots(len(algorithms), 1, figsize=(12, 3.2 * len(algorithms)), squeeze=False)
        for i, algorithm in enumerate(algorithms):
            ax = axes[i, 0]
            group = [r for r in summaries if r['algorithm'] == algorithm]
            for r in group:
                path, rows = bundles[(algorithm, r['speed_mode'])]
                progress = [path.s[int(row['target_index'])] for row in rows]
                ax.plot(progress, [row[field] for row in rows],
                        color=colors[r['speed_mode']], label=f"{r['speed_mode']} ({r['target_speed_mps']:g} m/s)")
            sample = group[0]
            if sample['steady_s_start_m'] is not None:
                ax.axvspan(sample['steady_s_start_m'], sample['steady_s_end_m'],
                           color='#18855B', alpha=0.09, label='Central-arc statistics window')
            ax.set(title=TITLES[algorithm], xlabel='Nearest-reference progress [m]', ylabel=ylabel)
            ax.grid(alpha=0.2)
            ax.legend(ncol=2)
        finish(fig, filename)

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    for ax, field, title in zip(axes, ['steer', 'yaw_rate', 'normal_accel'],
                               ['Central-arc steering [rad]', 'Central-arc yaw rate [rad/s]',
                                'Reference-curvature acceleration [m/s2]']):
        theory = [next(r[field + '_theory'] for r in summaries if r['speed_mode'] == mode)
                  for mode in SPEEDS if any(r['speed_mode'] == mode for r in summaries)]
        speeds = [SPEEDS[mode] for mode in SPEEDS if any(r['speed_mode'] == mode for r in summaries)]
        ax.plot(speeds, theory, '--', color='black', label='Target-speed theory')
        measured = {'steer': 'steady_steer_mean_rad', 'yaw_rate': 'steady_yaw_rate_mean_radps',
                    'normal_accel': 'steady_normal_accel_mean_mps2'}[field]
        for algorithm in algorithms:
            group = sorted([r for r in summaries if r['algorithm'] == algorithm],
                           key=lambda r: r['target_speed_mps'])
            ax.plot([r['target_speed_mps'] for r in group],
                    [r[measured] if r[measured] is not None else np.nan for r in group],
                    '-o', label=TITLES[algorithm])
        ax.set(title=title, xlabel='Target speed [m/s]')
        ax.set_xticks([3, 5, 7])
        ax.grid(alpha=0.2)
        ax.legend()
    finish(fig, 'theory_validation.png')


def run_experiments(output, algorithms=ALGORITHMS, speed_modes=tuple(SPEEDS), make_figures=True):
    """复用教学答案版，输出只写入任务二指定目录；绝不覆盖已有目录。"""
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f'结果目录已存在：{output}；请选择新 --output 或 --analyze-only')
    if not algorithms or not speed_modes or any(a not in ALGORITHMS for a in algorithms) or any(s not in SPEEDS for s in speed_modes):
        raise ValueError('算法或速度档选择无效')
    sys.path.insert(0, str(ROOT)) if str(ROOT) not in sys.path else None
    import importlib.metadata
    import platform
    import warnings
    from datetime import datetime
    from vdm_lab.common.simulation import load_controller, run_simulation
    from vdm_lab.common.types import LabConfig
    from vdm_lab.common.logging import compute_metrics, save_records, save_reference_path, save_metrics, save_predictions
    from vdm_lab.common.visualization import save_summary
    from vdm_lab.config.vehicle_params import make_vehicle_config

    output.mkdir(parents=True)
    manifest = {'created_at': datetime.now().isoformat(), 'python': platform.python_version(),
                'controller_version': 'solution', 'route': 'circle', 'vehicle': 'student_car',
                'steady_selection': 'abs(kappa-1/12)<0.005; central 60% of reference arc length',
                'packages': {name: importlib.metadata.version(name) for name in
                             ['numpy', 'scipy', 'matplotlib', 'cvxpy', 'osqp', 'pillow']},
                'source_sha256': {str(p.relative_to(ROOT)).replace('\\', '/'): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in sorted((ROOT / 'vdm_lab').rglob('*.py'))}, 'runs': []}
    summaries, bundles = [], {}
    write_json(output / 'manifest.json', manifest)
    for algorithm in algorithms:
        for speed_mode in speed_modes:
            print(f'Running {algorithm} / {speed_mode} ...', flush=True)
            directory = output / 'runs' / f'{algorithm}_{speed_mode}'
            directory.mkdir(parents=True)
            config = LabConfig(vehicle=make_vehicle_config('student_car'))
            config.sim.route_name = 'circle'
            config.sim.speed_mode = speed_mode
            write_json(directory / 'config.json', asdict(config))
            item = {'algorithm': algorithm, 'speed_mode': speed_mode,
                    'directory': str(directory.relative_to(output)).replace('\\', '/'),
                    'equivalent_command': f'python run_experiment.py --algo {algorithm} --version solution --route circle --vehicle student_car --speed-mode {speed_mode} --save-log --save-fig'}
            try:
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter('always')
                    path, records, predictions = run_simulation(load_controller(algorithm, 'solution'), config=config)
                item['warnings'] = sorted(set(str(w.message) for w in caught))
                metrics = compute_metrics(path, records)
                save_records(directory, records)
                save_reference_path(directory, path)
                save_metrics(directory, metrics)
                save_predictions(directory, predictions)
                rows = [asdict(r) for r in records]
                summary = summarize_run(algorithm, speed_mode, config, path, rows, metrics, SPEEDS[speed_mode])
                write_json(directory / 'statistics.json', summary)
                summaries.append(summary)
                bundles[(algorithm, speed_mode)] = (path, rows)
                item['status'] = 'completed'
                item['reached_goal'] = metrics['reached_goal']
                if make_figures:
                    try:
                        save_summary(path, records, directory / 'summary.png', TITLES[algorithm])
                    except Exception as figure_exc:
                        item['figure_error'] = f'{type(figure_exc).__name__}: {figure_exc}'
                        print(f"  Figure warning: {item['figure_error']}", flush=True)
                print(f"  reached_goal={metrics['reached_goal']}, mean_error={summary['mean_lateral_error_m']:.4f} m, central_samples={summary['steady_samples']}", flush=True)
            except Exception as exc:
                # 保留失败清单，后续其他实验继续；主入口最后返回非零状态。
                item.update(status='error', error=f'{type(exc).__name__}: {exc}')
                print(f"  FAILED: {item['error']}", flush=True)
            manifest['runs'].append(item)
            write_json(output / 'manifest.json', manifest)
    if summaries:
        save_analysis(output, summaries)
        if make_figures:
            make_comparison_figures(output, summaries, bundles)
    return summaries


def analyze_existing(output, make_figures=True):
    """从已保存日志重算统计/图表，不重新运行控制器，不改原始日志。"""
    sys.path.insert(0, str(ROOT)) if str(ROOT) not in sys.path else None
    import numpy as np
    from vdm_lab.common.types import LabConfig, VehicleConfig, SimulationConfig, ControllerConfig, Path as ReferencePath
    output = Path(output).resolve()
    manifest = json.loads((output / 'manifest.json').read_text(encoding='utf-8'))
    summaries, bundles = [], {}
    for item in manifest['runs']:
        if item['status'] != 'completed':
            continue
        directory = output / item['directory']
        data = json.loads((directory / 'config.json').read_text(encoding='utf-8'))
        controller = {key: np.asarray(v) if isinstance(v, list) else v for key, v in data['controller'].items()}
        config = LabConfig(VehicleConfig(**data['vehicle']), SimulationConfig(**data['sim']), ControllerConfig(**controller))
        with (directory / 'trajectory.csv').open(encoding='utf-8', newline='') as stream:
            rows = [{key: int(v) if key == 'target_index' else float(v)
                     for key, v in row.items()} for row in csv.DictReader(stream)]
        with (directory / 'reference_path.csv').open(encoding='utf-8', newline='') as stream:
            reference = list(csv.DictReader(stream))
        arrays = {key: np.asarray([float(r[column]) for r in reference]) for key, column in
                  [('x', 'x_m'), ('y', 'y_m'), ('yaw', 'yaw_rad'), ('curvature', 'curvature_1pm'),
                   ('s', 's_m'), ('target_speed', 'target_speed_mps')]}
        path = ReferencePath(**arrays)
        metrics = json.loads((directory / 'metrics.json').read_text(encoding='utf-8'))
        summary = summarize_run(item['algorithm'], item['speed_mode'], config,
                                path, rows, metrics, SPEEDS[item['speed_mode']])
        write_json(directory / 'statistics.json', summary)
        summaries.append(summary)
        bundles[(item['algorithm'], item['speed_mode'])] = (path, rows)
    if not summaries:
        raise ValueError('没有可分析的已完成实验')
    save_analysis(output, summaries)
    if make_figures:
        make_comparison_figures(output, summaries, bundles)
    return summaries


def main():
    parser = argparse.ArgumentParser(description='任务二：圆形路径三算法三速度实验')
    parser.add_argument('--output', type=Path, default=TASK_DIR / 'results' / 'baseline')
    parser.add_argument('--algorithms', nargs='+', choices=ALGORITHMS, default=list(ALGORITHMS))
    parser.add_argument('--speeds', nargs='+', choices=SPEEDS, default=list(SPEEDS))
    parser.add_argument('--analyze-only', action='store_true', help='从已保存日志重算统计与比较图')
    parser.add_argument('--no-figures', action='store_true')
    args = parser.parse_args()
    summaries = (analyze_existing(args.output, not args.no_figures) if args.analyze_only else
                 run_experiments(args.output, args.algorithms, args.speeds, not args.no_figures))
    print(f'Analyzed {len(summaries)} experiments. Output: {args.output.resolve()}', flush=True)
    manifest = json.loads((args.output / 'manifest.json').read_text(encoding='utf-8'))
    return 1 if any(r['status'] == 'error' or 'figure_error' in r for r in manifest['runs']) else 0


if __name__ == '__main__':
    raise SystemExit(main())
