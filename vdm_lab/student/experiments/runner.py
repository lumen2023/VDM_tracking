"""Shared student-experiment helpers.

All batch scripts call ``run_simulation()`` with student controllers and write
logs under ``outputs/<batch>_student/<label>/``.  Controllers are the student
implementations, not ``--version solution``.
"""

from __future__ import annotations

import json
import math
import os
from datetime import datetime
from pathlib import Path as FsPath

import matplotlib

matplotlib.use("Agg")

import numpy as np

from vdm_lab.common.logging import (
    compute_metrics,
    save_metrics,
    save_predictions,
    save_records,
    save_reference_path,
)
from vdm_lab.common.simulation import load_controller, run_simulation
from vdm_lab.common.types import LabConfig
from vdm_lab.common.visualization import save_gif, save_summary
from vdm_lab.config.vehicle_params import make_vehicle_config
from vdm_lab.student.dynamic_bicycle_backend import DynamicBicycleBackend


BATCH_ENV = "VDM_STUDENT_BATCH"
CIRCLE_KAPPA = 1.0 / 12.0
CIRCLE_KAPPA_TOL = 0.005
GPX_ROUTE = "data/gpx/homework_route_1.gpx"
GPX_ORIGIN_LON = 118.8145
GPX_ORIGIN_LAT = 31.8885
REPO_ROOT = FsPath(__file__).resolve().parents[3]


def batch_id():
    if BATCH_ENV not in os.environ:
        os.environ[BATCH_ENV] = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.environ[BATCH_ENV]


def batch_root():
    root = REPO_ROOT / "outputs" / f"{batch_id()}_student"
    root.mkdir(parents=True, exist_ok=True)
    return root


def make_output_dir(label):
    path = batch_root() / label
    path.mkdir(parents=True, exist_ok=True)
    return path


def make_lab_config(
    route="circle",
    speed_mode="low",
    target_speed=None,
    gpx_file=None,
    lookahead=None,
    vehicle_updates=None,
    max_time=None,
):
    config = LabConfig(vehicle=make_vehicle_config("student_car"))
    config.sim.route_name = route
    config.sim.speed_mode = speed_mode
    if target_speed is not None:
        config.sim.target_speed = float(target_speed)
    if gpx_file is not None:
        config.sim.gpx_file = str(gpx_file)
        config.sim.coordinate_origin_lon = GPX_ORIGIN_LON
        config.sim.coordinate_origin_lat = GPX_ORIGIN_LAT
    if lookahead is not None:
        config.controller.pp_base_lookahead = float(lookahead)
    if max_time is not None:
        config.sim.max_time = float(max_time)
        config.sim.auto_extend_gpx_time = False
    if vehicle_updates:
        for name, value in vehicle_updates.items():
            setattr(config.vehicle, name, value)
    return config


def make_backend(plant):
    if plant == "dynamic":
        return DynamicBicycleBackend()
    if plant in {None, "kinematic"}:
        return None
    raise ValueError(f"unknown plant {plant}")


def _longest_true_span(mask):
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0 or not np.any(mask):
        return None
    padded = np.concatenate(([False], mask, [False]))
    edges = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    i = int(np.argmax(ends - starts))
    return int(starts[i]), int(ends[i])


def _trim_span(span, trim_frac=0.15):
    start, end = span
    length = end - start
    if length < 4:
        return span
    trim = max(1, int(round(length * trim_frac)))
    if 2 * trim >= length:
        trim = max(1, length // 5)
    return start + trim, end - trim


def _record_mean(records, name):
    return float(np.mean([getattr(r, name) for r in records]))


def _record_abs_max(records, name):
    return float(np.max(np.abs([getattr(r, name) for r in records])))


def steering_activity(records, dt):
    if len(records) < 2 or dt <= 0.0:
        return float("nan")
    steers = np.array([r.steer for r in records], dtype=float)
    return float(np.mean(np.abs(np.diff(steers)) / dt))


def _relative_error_pct(sim, theory):
    theory = float(theory)
    if not math.isfinite(theory) or abs(theory) < 1.0e-12:
        return float("nan")
    return abs(float(sim) - theory) / abs(theory) * 100.0


def circle_analysis(records, config):
    mask = np.array(
        [abs(r.curvature - CIRCLE_KAPPA) < CIRCLE_KAPPA_TOL for r in records],
        dtype=bool,
    )
    span = _longest_true_span(mask)
    if span is None:
        return {"n_steady": 0}
    raw_start, raw_end = span
    steady_start, steady_end = _trim_span(span)
    entry = records[:raw_start]
    steady = records[steady_start:steady_end]
    if not steady:
        return {"n_steady": 0}

    wheelbase = float(config.vehicle.lf + config.vehicle.lr)
    mean_speed = _record_mean(steady, "speed")
    mean_steer = _record_mean(steady, "steer")
    mean_yaw_rate = _record_mean(steady, "yaw_rate")
    mean_normal = _record_mean(steady, "normal_accel")
    theory_steer = math.atan(wheelbase / 12.0)
    theory_r = mean_speed / 12.0
    theory_an = mean_speed * mean_speed / 12.0
    steers = np.array([r.steer for r in steady], dtype=float)
    demeaned = steers - np.mean(steers)
    sign = np.sign(demeaned)
    sign[sign == 0.0] = 1.0
    sign_changes = int(np.sum(sign[1:] * sign[:-1] < 0.0)) if len(sign) > 1 else 0

    return {
        "n_circle_raw": int(raw_end - raw_start),
        "n_steady": len(steady),
        "steady_time_s": [steady[0].time, steady[-1].time],
        "mean_speed": mean_speed,
        "mean_steer": mean_steer,
        "mean_yaw_rate": mean_yaw_rate,
        "mean_normal_accel": mean_normal,
        "theory_steer": theory_steer,
        "theory_yaw_rate": theory_r,
        "theory_normal_accel": theory_an,
        "rel_err_steer_pct": _relative_error_pct(mean_steer, theory_steer),
        "rel_err_yaw_rate_pct": _relative_error_pct(mean_yaw_rate, theory_r),
        "rel_err_normal_accel_pct": _relative_error_pct(mean_normal, theory_an),
        "entry_max_abs_ey": (
            _record_abs_max(entry, "lateral_error") if entry else float("nan")
        ),
        "steady_mae_ey": float(
            np.mean(np.abs([r.lateral_error for r in steady]))
        ),
        "steady_max_abs_ey": _record_abs_max(steady, "lateral_error"),
        "steer_std": float(np.std(steers)),
        "oscillation_sign_changes": sign_changes,
        "j_delta_steady": steering_activity(steady, config.sim.dt),
    }


def gpx_peak_error(path, records):
    if not records:
        return {}
    abs_ey = np.array([abs(r.lateral_error) for r in records], dtype=float)
    idx = int(np.argmax(abs_ey))
    record = records[idx]
    path_index = min(max(int(record.target_index), 0), len(path.x) - 1)
    lat = getattr(path, "lat", None)
    lon = getattr(path, "lon", None)
    path_s = float(path.s[path_index]) if path.s is not None else float("nan")
    return {
        "max_abs_ey": float(abs_ey[idx]),
        "time_s": float(record.time),
        "x_m": float(record.x),
        "y_m": float(record.y),
        "path_index": path_index,
        "path_s_m": path_s,
        "curvature_1pm": float(record.curvature),
        "steer_rad": float(record.steer),
        "lat_deg": None if lat is None else float(lat[path_index]),
        "lon_deg": None if lon is None else float(lon[path_index]),
    }


def curvature_lag(path, records):
    if not records or path is None or len(path.s) == 0:
        return {}
    abs_ey = np.array([abs(r.lateral_error) for r in records], dtype=float)
    ey_idx = int(np.argmax(abs_ey))
    rec = records[ey_idx]
    path_index = min(max(int(rec.target_index), 0), len(path.s) - 1)
    s_at_max_ey = float(path.s[path_index])
    abs_kappa = np.abs(np.asarray(path.curvature, dtype=float))
    kappa_idx = int(np.argmax(abs_kappa))
    s_at_peak_kappa = float(path.s[kappa_idx])
    return {
        "max_abs_ey": float(abs_ey[ey_idx]),
        "s_at_max_ey_m": s_at_max_ey,
        "time_at_max_ey_s": float(rec.time),
        "peak_abs_kappa": float(abs_kappa[kappa_idx]),
        "s_at_peak_kappa_m": s_at_peak_kappa,
        "lag_m": s_at_max_ey - s_at_peak_kappa,
        "error_after_peak": bool(s_at_max_ey > s_at_peak_kappa + 1.0e-6),
    }


def _steer_saturation(records, max_steer):
    if not records:
        return False, float("nan")
    abs_steer = np.array([abs(r.steer) for r in records], dtype=float)
    threshold = 0.99 * abs(float(max_steer))
    hits = abs_steer >= threshold
    return bool(np.any(hits)), float(np.mean(hits))


def run_one(
    algo,
    label,
    *,
    route="circle",
    speed_mode="low",
    target_speed=None,
    gpx_file=None,
    lookahead=None,
    vehicle_updates=None,
    plant="kinematic",
    save_fig=True,
    save_animation=False,
    max_time=None,
    extra=None,
    persist=True,
):
    controller = load_controller(algo, "student")
    config = make_lab_config(
        route=route,
        speed_mode=speed_mode,
        target_speed=target_speed,
        gpx_file=gpx_file,
        lookahead=lookahead,
        vehicle_updates=vehicle_updates,
        max_time=max_time,
    )
    backend = make_backend(plant)
    path, records, predictions = run_simulation(
        controller,
        config=config,
        vehicle_backend=backend,
    )
    metrics = compute_metrics(path, records)

    output_dir = None
    if persist:
        output_dir = make_output_dir(label)
        save_records(output_dir, records)
        save_reference_path(output_dir, path)
        save_predictions(output_dir, predictions)
        save_metrics(output_dir, metrics)
        if save_fig:
            save_summary(
                path,
                records,
                output_dir / "summary.png",
                getattr(controller, "NAME", algo),
            )
        if save_animation:
            save_gif(
                path,
                records,
                predictions,
                output_dir / "animation.gif",
                getattr(controller, "NAME", algo),
                config.vehicle,
                max_frames=120,
                show_history_ghosts=False,
            )

    speeds = np.array([r.speed for r in records], dtype=float)
    saturated, sat_frac = _steer_saturation(records, config.vehicle.max_steer)
    nominal_target = (
        float(target_speed)
        if target_speed is not None
        else float(np.nanmax([r.target_speed for r in records]))
    )
    analysis = {
        "label": label,
        "algo": algo,
        "version": "student",
        "route": route if gpx_file is None else FsPath(gpx_file).name,
        "speed_mode": speed_mode,
        "target_speed": None if target_speed is None else float(target_speed),
        "pp_base_lookahead": float(config.controller.pp_base_lookahead),
        "plant": plant,
        "output_dir": (
            "" if output_dir is None else str(output_dir.relative_to(REPO_ROOT))
        ),
        "duration_s": float(records[-1].time) if records else float("nan"),
        "mean_speed": float(np.mean(speeds)) if len(speeds) else float("nan"),
        "max_speed": float(np.max(speeds)) if len(speeds) else float("nan"),
        "final_speed": float(speeds[-1]) if len(speeds) else float("nan"),
        "nominal_target_speed": nominal_target,
        "reached_target_speed": bool(
            len(speeds) and math.isfinite(nominal_target) and np.max(speeds) >= 0.95 * nominal_target
        ),
        "j_delta": steering_activity(records, config.sim.dt),
        "steer_saturated": saturated,
        "steer_saturation_frac": sat_frac,
        "vehicle_max_steer": float(config.vehicle.max_steer),
        "lf": float(config.vehicle.lf),
        "lr": float(config.vehicle.lr),
        "wheelbase": float(config.vehicle.lf + config.vehicle.lr),
        **metrics,
    }
    if route == "circle" and gpx_file is None:
        analysis["circle"] = circle_analysis(records, config)
    if gpx_file is not None:
        analysis["gpx"] = gpx_peak_error(path, records)
    analysis["curvature_lag"] = curvature_lag(path, records)
    if extra:
        analysis["extra"] = extra

    if persist:
        analysis_path = output_dir / "analysis.json"
        with analysis_path.open("w", encoding="utf-8") as handle:
            json.dump(analysis, handle, ensure_ascii=False, indent=2)

    print(
        f"{label}: reached_goal={metrics['reached_goal']} "
        f"max_|ey|={metrics['max_lateral_error_m']:.3f} m "
        f"dir={analysis['output_dir']}"
    )
    return analysis


def run_many(jobs):
    """Run ``run_one`` for each mapping that includes ``algo`` and ``label``."""
    results = []
    for job in jobs:
        job = dict(job)
        results.append(run_one(job.pop("algo"), job.pop("label"), **job))
    return results
