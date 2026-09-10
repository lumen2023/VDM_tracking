#!/usr/bin/env bash
# Run the default GPX + offline GeoJSON Pure Pursuit demonstration from Bash.
set -euo pipefail

cd "$(dirname "$0")/.."

python run_experiment.py \
  --algo pp \
  --gpx data/gpx/demo_route.gpx \
  --basemap geojson \
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' \
  --target-speed 8 \
  --waypoint-ds 1.0 \
  --animate
