"""Thin entry point using the group's unchanged simulation and student controllers."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=ROOT,
                        help='Group repository root; omit after overlaying this delivery')
    parser.add_argument('--algo', choices=['pp', 'lqr_kinematic', 'mpc'], required=True)
    parser.add_argument('--cruise', type=float, default=3.0)
    parser.add_argument('--case', required=True)
    args = parser.parse_args()
    if Path(args.case).name != args.case or args.case in ('.', '..'):
        parser.error('case must be one directory name')
    sys.path.insert(0, str(args.repo.resolve()))
    from campus_route import load_route, ORIGIN
    from vdm_lab.common.simulation import load_controller, run_simulation
    from vdm_lab.common.types import LabConfig
    from vdm_lab.config.vehicle_params import make_vehicle_config
    from vdm_lab.common.logging import save_records, save_reference_path, save_metrics, compute_metrics
    from vdm_lab.common.visualization import save_summary

    output = ROOT/'outputs/campus'/args.case
    output.mkdir(parents=True, exist_ok=False)
    route = load_route(args.cruise)
    config = LabConfig(vehicle=make_vehicle_config('student_car'))
    config.sim.dt = 0.1
    config.sim.waypoint_ds = 0.2
    config.sim.max_time = 900.0
    config.sim.auto_extend_gpx_time = False
    config.sim.route_name = 'campus_smooth'
    config.sim.target_speed = args.cruise
    config.sim.coordinate_origin_lon, config.sim.coordinate_origin_lat = ORIGIN
    def serial(obj):
        if isinstance(obj, np.ndarray): return obj.tolist()
        raise TypeError(type(obj).__name__)
    inputs = ['vdm_lab/student/pure_pursuit.py', 'vdm_lab/student/lqr_kinematic.py',
              'vdm_lab/student/mpc.py', 'vdm_lab/common/simulation.py',
              'vdm_lab/common/reference.py', 'vdm_lab/common/vehicle.py',
              'vdm_lab/common/vehicle_backend.py', 'vdm_lab/common/bicycle_model.py',
              'vdm_lab/common/types.py', 'vdm_lab/config/vehicle_params.py']
    hashes = {f: hashlib.sha256((args.repo/f).read_bytes()).hexdigest() for f in inputs}
    command = f'python analysis/run_campus.py --algo {args.algo} --cruise {args.cruise:g} --case {args.case}'
    manifest = dict(algorithm=args.algo, version='student', vehicle='student_car',
        command=command, output_dir=f'outputs/campus/{args.case}', config=asdict(config),
        baseline_sha256=hashes,
        plan_sha256=hashlib.sha256((ROOT/'data/planned_path.csv').read_bytes()).hexdigest(),
        python=sys.version, numpy=np.__version__)
    (output/'config.json').write_text(json.dumps(manifest, indent=2, default=serial), encoding='utf-8')
    begin = time.perf_counter()
    path, records, _ = run_simulation(load_controller(args.algo, 'student'), config,
                                     path_override=route)
    metrics = compute_metrics(path, records)
    metrics.update(duration_s=float(records[-1].time), route_length_m=float(path.s[-1]),
                   wall_clock_s=time.perf_counter()-begin)
    save_records(output, records)
    save_reference_path(output, path)
    save_metrics(output, metrics)
    save_summary(path, records, output/'summary.png',
                 f'{args.algo} | campus | cruise cap {args.cruise:g} m/s')
    print(json.dumps(dict(case=args.case, output_dir=str(output), **metrics)), flush=True)


if __name__ == '__main__':
    main()
