"""Reproducible task-1 experiments; original controllers and plant stay unchanged."""

from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import warnings

# Keep small linear algebra operations from oversubscribing worker processes.
for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[variable] = "1"
os.environ["MPLBACKEND"] = "Agg"
TASK_ROOT = Path(__file__).resolve().parents[1]
ROOT = TASK_ROOT.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from vdm_lab.common.bicycle_model import normal_acceleration, yaw_rate_from_steer, front_steer_slip_angle
from vdm_lab.common.logging import compute_metrics, save_metrics, save_predictions, save_records, save_reference_path
from vdm_lab.common.simulation import build_reference_path, load_controller, run_simulation
from vdm_lab.common.types import LabConfig, StepRecord
from vdm_lab.common.vehicle import limit_command
from vdm_lab.common.visualization import save_summary
from vdm_lab.config.vehicle_params import make_vehicle_config

ROUTES = ("double_lane_change", "right_angle", "s_curve")
ALGORITHMS = ("pp", "lqr_kinematic", "mpc")


@dataclass(frozen=True)
class Case:
    group: str
    route: str
    algo: str
    max_steer_deg: float = 35.0

    @property
    def name(self):
        return f"{self.group}__{self.route}__{self.algo}__steer{self.max_steer_deg:g}"


def make_cases(include_dynamic=True):
    cases = [Case("core", route, algo) for route in ROUTES for algo in ALGORITHMS]
    cases += [Case("steering", "right_angle", algo, limit)
              for limit in (25.0, 15.0) for algo in ("pp", "lqr_kinematic")]
    if include_dynamic:
        cases += [Case("supplement", route, "lqr_dynamic") for route in ("s_curve", "mixed_course")]
    return cases


def make_config(case, speed_mode="medium"):
    config = LabConfig(vehicle=make_vehicle_config("student_car"))
    config.sim.route_name = case.route
    config.sim.speed_mode = speed_mode
    config.vehicle.max_steer = math.radians(case.max_steer_deg)
    return config


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=json_default, allow_nan=False) + "\n", encoding="utf-8")


def extended_metrics(path, records, config):
    """Preserve repository metrics and add explicitly named diagnostic quantities."""
    if not records:
        return {"steps": 0, "reached_goal": False, "completion_time_s": None}
    result = compute_metrics(path, records)
    times = np.array([r.time for r in records])
    errors = np.abs([r.lateral_error for r in records])
    steer = np.array([r.steer for r in records])
    speed = np.array([r.speed for r in records])
    curvature = np.abs([r.curvature for r in records])
    rate = np.abs(np.diff(steer) / np.diff(times)) if len(times) > 1 else np.array([])
    peak = int(np.argmax(errors))
    peak_record = records[peak]
    curvature_peaks = np.flatnonzero(np.isclose(curvature, curvature.max(), rtol=1e-6, atol=1e-10))
    indices = np.array([r.target_index for r in records], dtype=int)
    moving = speed > 0.5
    result.update({
        "end_time_s": float(times[-1]),
        "completion_time_s": float(times[-1]) if result["reached_goal"] else None,
        "rms_lateral_error_m": float(np.sqrt(np.mean(errors**2))),
        "p95_lateral_error_m": float(np.percentile(errors, 95)),
        "mean_abs_steer_rate_radps": float(rate.mean()) if rate.size else 0.0,
        "max_abs_steer_rate_radps": float(rate.max()) if rate.size else 0.0,
        "steer_rate_above_config_fraction": float(np.mean(rate > config.vehicle.max_steer_rate + 1e-6)) if rate.size else 0.0,
        "steer_saturation_fraction": float(np.mean(np.abs(steer) >= config.vehicle.max_steer - 1e-6)),
        "moving_mean_lateral_error_m": float(errors[moving].mean()) if moving.any() else None,
        "moving_mean_abs_steer_rate_radps": float(rate[moving[:-1] & moving[1:]].mean()) if rate.size and np.any(moving[:-1] & moving[1:]) else None,
        "moving_duration_s": float(moving.sum() * config.sim.dt),
        "mean_speed_mps": float(speed.mean()),
        "reference_progress_fraction": float(path.s[indices.max()] / path.s[-1]),
        "peak_error_time_s": float(peak_record.time),
        "peak_error_reference_s_m": float(path.s[peak_record.target_index]),
        "peak_error_x_m": float(peak_record.x),
        "peak_error_y_m": float(peak_record.y),
        "abs_curvature_at_peak_error_1pm": float(curvature[peak]),
        "max_visited_abs_curvature_1pm": float(curvature.max()),
        "first_peak_curvature_time_s": float(times[curvature_peaks[0]]),
        "last_peak_curvature_time_s": float(times[curvature_peaks[-1]]),
        "max_reference_abs_curvature_1pm": float(np.max(np.abs(path.curvature))),
    })
    return result


class ObservedController:
    """Observe successful steps without changing returned control commands.

    The duplicate trace is only a recovery journal if the original simulation
    raises: its local records would otherwise be inaccessible to the caller.
    """

    def __init__(self, module, case):
        self.module = module
        self.NAME = module.NAME
        self.case = case
        self.records = []
        self.predictions = []
        self.control_seconds = []
        self.sim_time = 0.0
        self.failure_state = None

    def control(self, state, reference, previous_control, config):
        started = time.perf_counter()
        try:
            command = self.module.control(state, reference, previous_control, config)
        except Exception:
            self.failure_state = {"time_s": self.sim_time, "state": asdict(state), "target_index": reference.nearest_index}
            raise
        finally:
            self.control_seconds.append(time.perf_counter() - started)
        limited = limit_command(command, config.vehicle)
        self.records.append(StepRecord(
            time=self.sim_time, x=state.x, y=state.y, yaw=state.yaw, speed=state.v,
            acceleration=limited.acceleration, steer=limited.steer,
            beta=front_steer_slip_angle(limited.steer, config.vehicle),
            yaw_rate=yaw_rate_from_steer(state.v, limited.steer, config.vehicle),
            target_index=reference.nearest_index, lateral_error=reference.lateral_error,
            heading_error=reference.heading_error, curvature=reference.curvature,
            normal_accel=normal_acceleration(state.v, reference.curvature), target_speed=reference.target_speed,
        ))
        if hasattr(limited, "prediction"):
            self.predictions.append((self.sim_time, limited.prediction))
        self.sim_time += config.sim.dt
        if len(self.records) % 100 == 0:
            print(f"  {self.case.name}: {len(self.records)} samples", flush=True)
        return command


def run_case(case, output_root, speed_mode):
    destination = Path(output_root) / "runs" / case.name
    destination.mkdir(parents=True, exist_ok=False)
    config = make_config(case, speed_mode)
    path = build_reference_path(config)
    write_json(destination / "config.json", {"case": asdict(case), "config": asdict(config)})
    save_reference_path(destination, path)
    observed = ObservedController(load_controller(case.algo, "solution"), case)
    print(f"START {case.name}", flush=True)
    started = time.perf_counter()
    failure = None
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        try:
            _, records, predictions = run_simulation(observed, config=config, path_override=path)
            assert records == observed.records, "Recovery journal differs from canonical simulation records"
        except Exception as exc:
            records, predictions = observed.records, observed.predictions
            failure = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc(), "failure_state": observed.failure_state}
    wall_seconds = time.perf_counter() - started
    metrics = extended_metrics(path, records, config)
    if failure is not None:
        metrics["reached_goal"] = False
        metrics["completion_time_s"] = None
    status = "error" if failure else ("reached_goal" if metrics["reached_goal"] else "time_limit")
    warning_counts = Counter(f"{w.category.__name__}: {w.message}" for w in caught)
    write_json(destination / "run_info.json", {
        "status": status, "simulation_wall_seconds": wall_seconds,
        "mean_control_wall_seconds": float(np.mean(observed.control_seconds)) if observed.control_seconds else None,
        "warnings": dict(warning_counts), "failure": failure,
        "command_equivalent": (f"python run_experiment.py --algo {case.algo} --version solution --route {case.route} --speed-mode {speed_mode} --save-log --save-fig"
                               if case.max_steer_deg == 35 else "Use task1/scripts/run_task1.py with the saved config.json; original CLI has no max_steer override."),
    })
    if records:
        save_records(destination, records)
        save_predictions(destination, predictions)
        save_summary(path, records, destination / "summary.png", f"{observed.NAME} | {case.route} | {speed_mode} | steer limit {case.max_steer_deg:g} deg")
    # Strict JSON also rejects non-finite metrics, avoiding silently invalid data.
    write_json(destination / "metrics.json", metrics)
    print(f"DONE  {case.name}: {status}, n={metrics['steps']}, mean error={metrics.get('mean_lateral_error_m', float('nan')):.4f} m", flush=True)
    return {"name": case.name, "case": asdict(case), "status": status, "metrics": metrics}


def source_hashes():
    files = sorted((ROOT / "vdm_lab").rglob("*.py"))
    files += [ROOT / "run_experiment.py", TASK_ROOT / "requirements.lock.txt", Path(__file__).resolve()]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=TASK_ROOT / "outputs" / datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--jobs", type=int, default=3)
    parser.add_argument("--speed-mode", choices=("low", "medium", "high"), default="medium")
    parser.add_argument("--skip-dynamic", action="store_true")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    output = args.output.resolve()
    if output.exists():
        parser.error(f"Output already exists; choose a new directory: {output}")
    output.mkdir(parents=True)
    cases = make_cases(not args.skip_dynamic)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version, "executable": sys.executable,
        "packages": {name: version(name) for name in ("numpy", "scipy", "matplotlib", "cvxpy", "osqp", "pillow")},
        "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": source_hashes(),
        "speed_mode": args.speed_mode, "jobs": args.jobs, "version": "solution",
        "cases": [asdict(case) for case in cases],
        "note": "Deterministic simulation, no injected noise, one run per fixed configuration. Steering baseline is reused from core cases, not rerun.",
    }
    write_json(output / "manifest.json", manifest)
    results = []
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(run_case, case, output, args.speed_mode): case for case in cases}
        for future in as_completed(futures):
            case = futures[future]
            try:
                results.append(future.result())
            except Exception as exc:
                # Do not conceal worker/serialization errors as ordinary timeouts.
                results.append({"name": case.name, "case": asdict(case), "status": "worker_error", "error": str(exc)})
                print(f"WORKER ERROR {case.name}: {exc}", file=sys.stderr, flush=True)
            write_json(output / "results.json", sorted(results, key=lambda result: result["name"]))
    print(f"Results: {output}", flush=True)
    errors = [result for result in results if result["status"] in {"error", "worker_error"}]
    if errors:
        raise SystemExit(f"{len(errors)} execution errors; diagnostics and successful/partial data were preserved.")


if __name__ == "__main__":
    main()
