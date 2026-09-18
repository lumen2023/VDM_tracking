"""Compatibility entry point for the refined, data-driven figure generator.

The former script depended on nonexistent timestamped folders. Run from the
repository root: python -m vdm_lab.student.plot_comparison --help
The older mixed-course helper remains at plot_comparison_legacy.py.
"""
from vdm_lab.student.visualizations import HERE, main

if __name__ == "__main__":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=HERE / "results")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    main(args.results, args.out)
