"""Run the default Pure Pursuit GPX demo on Windows, Linux, or macOS."""

from pathlib import Path
import subprocess
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    command = [
        sys.executable,
        str(REPOSITORY_ROOT / "run_experiment.py"),
        "--algo",
        "pp",
        "--gpx",
        "data/gpx/demo_route.gpx",
        "--basemap",
        "geojson",
        "--basemap-file",
        "data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz",
        "--target-speed",
        "8",
        "--waypoint-ds",
        "1.0",
        "--animate",
    ]
    subprocess.run(command, cwd=REPOSITORY_ROOT, check=True)


if __name__ == "__main__":
    main()
