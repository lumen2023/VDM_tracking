"""Campus route adapter: immutable smooth geometry, shared feasible speed profile."""
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ORIGIN = (118.8145, 31.8885)
R = 6378137.0


def speed_profile(s, curvature, cruise):
    if not np.isfinite(cruise) or cruise <= 0:
        raise ValueError('Cruise speed must be finite and positive')
    speed = np.minimum(cruise, np.sqrt(0.7 / np.maximum(abs(curvature), 1e-5)))
    speed[0] = min(speed[0], 0.5)
    speed[-1] = 0.0
    for i in range(1, len(s)):
        speed[i] = min(speed[i], np.sqrt(speed[i-1]**2 + 2*0.7*(s[i]-s[i-1])))
    for i in range(len(s)-2, -1, -1):
        speed[i] = min(speed[i], np.sqrt(speed[i+1]**2 + 2*0.8*(s[i+1]-s[i])))
    return speed


def to_lonlat(x, y):
    return (ORIGIN[0] + np.rad2deg(np.asarray(x)/(R*np.cos(np.deg2rad(ORIGIN[1])))),
            ORIGIN[1] + np.rad2deg(np.asarray(y)/R))


def load_route(cruise=3.0):
    from vdm_lab.common.types import Path as ReferencePath
    p = np.genfromtxt(ROOT/'data/planned_path.csv', delimiter=',', names=True)
    if not np.isfinite(np.column_stack([p[n] for n in p.dtype.names])).all():
        raise ValueError('Nonfinite route data')
    if not np.all(np.diff(p['s_m']) > 0):
        raise ValueError('Route station must increase')
    speed = speed_profile(p['s_m'], p['curvature_1pm'], cruise)
    return ReferencePath(x=p['x_m'], y=p['y_m'], yaw=p['yaw_rad'],
        curvature=p['curvature_1pm'], s=p['s_m'], target_speed=speed,
        lat=p['lat_deg'], lon=p['lon_deg'], source='campus_smooth_plan')
