"""Student experiment package.

Existing batch scripts remain importable (``circle_speed``, ``run_all``, ...).
The GPT-6 Pro research harness lives in ``study`` and is re-exported here so
``python -m vdm_lab.student.experiments`` and
``from vdm_lab.student.experiments import Case, OUT`` keep working.
"""

from vdm_lab.student.experiments.study import (
    ALGORITHMS,
    OUT,
    ROOT,
    Case,
    analyze,
    config_for,
    local_projection,
    main,
    prepared_path,
    run_case,
    suite_cases,
    write_csv,
    write_json,
)

__all__ = [
    "ALGORITHMS",
    "OUT",
    "ROOT",
    "Case",
    "analyze",
    "config_for",
    "local_projection",
    "main",
    "prepared_path",
    "run_case",
    "suite_cases",
    "write_csv",
    "write_json",
]
