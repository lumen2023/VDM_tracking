import math
import unittest

from vdm_lab.common.types import ControlCommand, LabConfig, VehicleState
from vdm_lab.common.vehicle import update_state
from vdm_lab.student.dynamic_bicycle_backend import (
    KINEMATIC_SPEED_FLOOR,
    DynamicBicycleBackend,
)


def _straight_command(acceleration=0.0, steer=0.0):
    return ControlCommand(acceleration=acceleration, steer=steer)


class DynamicBicycleBackendTests(unittest.TestCase):
    def setUp(self):
        self.config = LabConfig()
        self.dt = self.config.sim.dt
        self.backend = DynamicBicycleBackend()

    def test_dynamics_terms_use_internal_sideslip_not_geometric_beta(self):
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=5.0)
        command = _straight_command(steer=0.35)
        self.backend.reset(state, self.config)

        yaw_rate, beta = self.backend.dynamics_terms(state, command, self.config)

        self.assertEqual(yaw_rate, 0.0)
        self.assertEqual(beta, 0.0)

    def test_zero_steer_decays_lateral_velocity_and_yaw_rate(self):
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=5.0)
        self.backend.reset(state, self.config)
        self.backend.v_y = 1.2
        self.backend.r = 0.4
        command = _straight_command(steer=0.0)

        for _ in range(40):
            yaw_rate, beta = self.backend.dynamics_terms(
                state, command, self.config
            )
            state, command = self.backend.step(
                state, command, self.config, self.dt
            )
            self.assertTrue(math.isfinite(state.x))
            self.assertTrue(math.isfinite(state.y))
            self.assertTrue(math.isfinite(state.yaw))
            self.assertTrue(math.isfinite(state.v))
            self.assertTrue(math.isfinite(self.backend.v_y))
            self.assertTrue(math.isfinite(self.backend.r))
            self.assertTrue(math.isfinite(yaw_rate))
            self.assertTrue(math.isfinite(beta))

        self.assertLess(abs(self.backend.v_y), 1.0e-3)
        self.assertLess(abs(self.backend.r), 1.0e-3)

    def test_steady_circle_yaw_rate_matches_speed_times_curvature(self):
        vehicle = self.config.vehicle
        speed = 5.0
        kappa = 1.0 / 12.0
        steer = (vehicle.lf + vehicle.lr) * kappa
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=speed)
        command = _straight_command(steer=steer)
        self.backend.reset(state, self.config)

        for _ in range(80):
            state, command = self.backend.step(
                state, command, self.config, self.dt
            )

        yaw_rate, beta = self.backend.dynamics_terms(
            state, command, self.config
        )
        expected = speed * kappa
        self.assertTrue(math.isfinite(beta))
        self.assertAlmostEqual(yaw_rate, expected, delta=0.05 * expected)
        self.assertAlmostEqual(self.backend.r, expected, delta=0.05 * expected)

    def test_low_speed_floor_uses_kinematic_step_and_keeps_state_finite(self):
        self.assertEqual(KINEMATIC_SPEED_FLOOR, 0.5)
        state = VehicleState(x=1.0, y=-0.5, yaw=0.3, v=0.2)
        command = ControlCommand(acceleration=1.5, steer=0.4)
        self.backend.reset(state, self.config)

        next_dynamic, applied = self.backend.step(
            state, command, self.config, self.dt
        )
        next_kinematic, kinematic_applied = update_state(
            state, command, self.config.vehicle, self.dt
        )

        self.assertAlmostEqual(next_dynamic.x, next_kinematic.x, places=12)
        self.assertAlmostEqual(next_dynamic.y, next_kinematic.y, places=12)
        self.assertAlmostEqual(next_dynamic.yaw, next_kinematic.yaw, places=12)
        self.assertAlmostEqual(next_dynamic.v, next_kinematic.v, places=12)
        self.assertAlmostEqual(applied.steer, kinematic_applied.steer, places=12)
        self.assertTrue(math.isfinite(self.backend.v_y))
        self.assertTrue(math.isfinite(self.backend.r))
        self.assertLess(abs(self.backend.v_y), 10.0)
        self.assertLess(abs(self.backend.r), 10.0)

    def test_reset_clears_internal_lateral_states(self):
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=4.0)
        self.backend.v_y = 3.0
        self.backend.r = -1.5
        restored = self.backend.reset(state, self.config)
        self.assertIs(restored, state)
        self.assertEqual(self.backend.v_y, 0.0)
        self.assertEqual(self.backend.r, 0.0)


if __name__ == "__main__":
    unittest.main()
