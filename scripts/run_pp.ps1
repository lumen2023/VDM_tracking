# Run the default GPX + offline GeoJSON Pure Pursuit demonstration from PowerShell.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

Set-Location (Join-Path $PSScriptRoot '..')

python run_experiment.py `
  --algo pp `
  --gpx data/gpx/demo_route.gpx `
  --basemap geojson `
  --basemap-file 'data/planet_118.792,31.875_118.837,31.902.osm.geojson.xz' `
  --target-speed 8 `
  --waypoint-ds 1.0 `
  --animate
