import math

import numpy as np

from vdm_lab.common.geometry import pi_to_pi
from vdm_lab.common.types import ControllerReference


class ReferenceTracker:
    def __init__(self, path, search_window=30):
        self.path = path
        self.search_window = search_window
        self.index = 0

    def nearest(self, state):
        start = self.index
        end = min(len(self.path.x), start + self.search_window)
        dx = state.x - self.path.x[start:end]
        dy = state.y - self.path.y[start:end]
        local_index = int(np.argmin(np.hypot(dx, dy)))
        self.index = start + local_index

        yaw_ref = self.path.yaw[self.index]
        path_to_vehicle = np.array([state.x - self.path.x[self.index], state.y - self.path.y[self.index]])
        normal_left = np.array([-math.sin(yaw_ref), math.cos(yaw_ref)])
        lateral_error = float(np.dot(path_to_vehicle, normal_left))
        heading_error = pi_to_pi(state.yaw - yaw_ref)

        return ControllerReference(
            path=self.path,
            nearest_index=self.index,
            lateral_error=lateral_error,
            heading_error=heading_error,
            curvature=float(self.path.curvature[self.index]),
            target_speed=float(self.path.target_speed[self.index]),
        )


def distance_to_goal(state, path):
    return float(math.hypot(state.x - path.x[-1], state.y - path.y[-1]))
