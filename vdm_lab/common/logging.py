import csv
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path as FsPath

import numpy as np


def create_output_dir(label):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = FsPath("outputs") / f"{stamp}_{label}"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def save_records(output_dir, records):
    csv_path = output_dir / "trajectory.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(records[0]).keys()))
        writer.writeheader()
        for record in records:
            writer.writerow(asdict(record))
    return csv_path


def save_predictions(output_dir, predictions):
    if not predictions:
        return None
    path = output_dir / "mpc_predictions.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "horizon_index", "x", "y", "v", "yaw"])
        for time, prediction in predictions:
            for i in range(prediction.shape[1]):
                writer.writerow([time, i, prediction[0, i], prediction[1, i], prediction[2, i], prediction[3, i]])
    return path


def compute_metrics(path, records):
    lateral_errors = np.array([abs(r.lateral_error) for r in records], dtype=float)
    heading_errors = np.array([abs(r.heading_error) for r in records], dtype=float)
    speeds = np.array([r.speed for r in records], dtype=float)
    steers = np.array([abs(r.steer) for r in records], dtype=float)
    accelerations = np.array([abs(r.acceleration) for r in records], dtype=float)
    last = records[-1]
    finish_error = float(np.hypot(last.x - path.x[-1], last.y - path.y[-1]))

    return {
        "mean_lateral_error_m": float(lateral_errors.mean()),
        "max_lateral_error_m": float(lateral_errors.max()),
        "finish_error_m": finish_error,
        "mean_heading_error_rad": float(heading_errors.mean()),
        "max_steer_rad": float(steers.max()),
        "max_acceleration_mps2": float(accelerations.max()),
        "min_speed_mps": float(speeds.min()),
        "steps": len(records),
        "reached_goal": bool(finish_error < 1.5 and last.speed < 0.5),
    }


def save_metrics(output_dir, metrics):
    path = output_dir / "metrics.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    return path
