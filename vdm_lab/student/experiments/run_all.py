"""Run every student batch experiment from the repository root."""

from __future__ import annotations

import json

from vdm_lab.student.experiments import (
    circle_speed,
    gpx_nav,
    lookahead_oat,
    plant_compare,
    task1_extra,
)
from vdm_lab.student.experiments.runner import batch_root


def main():
    results = []
    for module in (
        circle_speed,
        lookahead_oat,
        plant_compare,
        gpx_nav,
        task1_extra,
    ):
        print(f"=== {module.__name__} ===")
        results.extend(module.main())

    root = batch_root()
    index_path = root / "index.json"
    with index_path.open("w", encoding="utf-8") as handle:
        json.dump(results, handle, ensure_ascii=False, indent=2)
    print(f"wrote {index_path} ({len(results)} runs)")
    return results


if __name__ == "__main__":
    main()
