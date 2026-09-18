import math
import unittest
import warnings
from unittest import mock

import numpy as np

from vdm_lab.common.reference import ReferenceTracker
from vdm_lab.common.simulation import run_simulation
from vdm_lab.common.types import (
    ControlCommand,
    ControllerReference,
    LabConfig,
    Path,
    VehicleState,
)
from vdm_lab.common.vehicle import update_state
from vdm_lab.student import lqr_dynamic, lqr_kinematic, mpc, pure_pursuit
from vdm_lab.student._controller_utils import remaining_path_distance


CONTROLLERS = (pure_pursuit, lqr_kinematic, lqr_dynamic, mpc)


def _loop_reference(target_speed=7.0):
    path = Path(
        x=np.array([0.0, 10.0, 0.0]),
        y=np.array([0.0, 0.0, 0.0]),
        yaw=np.zeros(3),
        curvature=np.zeros(3),
        s=np.array([0.0, 50.0, 100.0]),
        target_speed=np.full(3, target_speed),
    )
    reference = ControllerReference(
        path=path,
        nearest_index=0,
        lateral_error=0.0,
        heading_error=0.0,
        curvature=0.0,
        target_speed=target_speed,
    )
    return path, reference


class ControllerRegressionTests(unittest.TestCase):
    def assert_bounded_command(self, command, previous, config):
        self.assertTrue(math.isfinite(command.acceleration))
        self.assertTrue(math.isfinite(command.steer))
        self.assertGreaterEqual(
            command.acceleration, -config.vehicle.max_decel
        )
        self.assertLessEqual(command.acceleration, config.vehicle.max_accel)
        self.assertLessEqual(abs(command.steer), config.vehicle.max_steer)
        self.assertLessEqual(
            abs(command.steer - previous.steer),
            config.vehicle.max_steer_rate * config.sim.dt + 1.0e-12,
        )

    def test_remaining_distance_uses_route_progress_not_endpoint_distance(self):
        path, reference = _loop_reference()
        endpoint_distance = math.hypot(
            path.x[0] - path.x[-1], path.y[0] - path.y[-1]
        )
        self.assertEqual(endpoint_distance, 0.0)
        self.assertEqual(
            remaining_path_distance(path, reference.nearest_index), 100.0
        )

    def test_loop_start_does_not_trigger_premature_goal_braking(self):
        _, reference = _loop_reference()
        for controller in (pure_pursuit, lqr_kinematic, lqr_dynamic):
            with self.subTest(controller=controller.NAME):
                command = controller.control(
                    VehicleState(x=0.0, y=0.0, yaw=0.0, v=0.0),
                    reference,
                    ControlCommand(acceleration=0.0, steer=0.0),
                    LabConfig(),
                )
                self.assertGreater(command.acceleration, 0.0)

    def test_dynamic_feedforward_uses_axle_stiffness_convention(self):
        config = LabConfig()
        A, B = lqr_dynamic.build_dynamic_model(7.0, config)
        K = lqr_dynamic.solve_lqr(
            A,
            B,
            config.controller.lqr_q,
            config.controller.lqr_r,
            config.controller.lqr_eps,
            config.controller.lqr_max_iter,
        )
        actual = lqr_dynamic.dynamic_feedforward(
            7.0, 1.0 / 12.0, K, config
        )
        self.assertAlmostEqual(actual, 0.08895278816714257, places=10)

    def test_mpc_prediction_matches_shared_plant_step(self):
        config = LabConfig()
        state = VehicleState(x=3.0, y=-2.0, v=7.0, yaw=0.6)
        command = ControlCommand(acceleration=0.5, steer=0.4)
        plant_state, _ = update_state(
            state, command, config.vehicle, config.sim.dt
        )
        predicted = mpc.update_kinematic_array(
            np.array([state.x, state.y, state.v, state.yaw]),
            command.acceleration,
            command.steer,
            config,
        )
        expected = np.array(
            [
                plant_state.x,
                plant_state.y,
                plant_state.v,
                plant_state.yaw,
            ]
        )
        self.assertLess(float(np.max(np.abs(predicted - expected))), 1.0e-12)

    def test_mpc_jacobian_matches_finite_differences(self):
        config = LabConfig()
        state = np.array([3.0, -2.0, 7.0, 0.6])
        control = np.array([0.0, 0.4])
        A, B, _ = mpc.linear_model(
            state[2], state[3], control[1], config
        )
        epsilon = 1.0e-6
        state_basis = np.eye(4)
        input_basis = np.eye(2)
        finite_A = np.column_stack(
            [
                (
                    mpc.update_kinematic_array(
                        state + epsilon * state_basis[i], *control, config
                    )
                    - mpc.update_kinematic_array(
                        state - epsilon * state_basis[i], *control, config
                    )
                )
                / (2.0 * epsilon)
                for i in range(4)
            ]
        )
        finite_B = np.column_stack(
            [
                (
                    mpc.update_kinematic_array(
                        state,
                        *(control + epsilon * input_basis[i]),
                        config,
                    )
                    - mpc.update_kinematic_array(
                        state,
                        *(control - epsilon * input_basis[i]),
                        config,
                    )
                )
                / (2.0 * epsilon)
                for i in range(2)
            ]
        )
        self.assertLess(float(np.max(np.abs(A - finite_A))), 1.0e-7)
        self.assertLess(float(np.max(np.abs(B - finite_B))), 1.0e-7)

    def test_mpc_qp_is_translation_invariant(self):
        try:
            import cvxpy  # noqa: F401
        except ImportError:
            self.skipTest("cvxpy is not installed")

        config = LabConfig()
        horizon = config.controller.mpc_horizon
        state = np.array([4321.0, -7654.0, 7.0, 0.35])
        acceleration = np.zeros(horizon)
        steer = np.full(horizon, 0.08)
        nominal = mpc.predict_motion(
            state,
            acceleration,
            steer,
            np.zeros((4, horizon + 1)),
            config,
        )
        reference = nominal.copy()
        reference[0, :] += np.linspace(0.0, 0.8, horizon + 1)
        reference[1, :] += np.linspace(0.0, 0.3, horizon + 1)
        mpc._WORKSPACE_CACHE.clear()
        first = mpc.solve_linear_mpc(
            reference, nominal, state, steer, config
        )

        offset = np.array([1.0e6, -2.0e6])
        shifted_state = state.copy()
        shifted_state[:2] += offset
        shifted_nominal = nominal.copy()
        shifted_nominal[:2, :] += offset[:, None]
        shifted_reference = reference.copy()
        shifted_reference[:2, :] += offset[:, None]
        mpc._WORKSPACE_CACHE.clear()
        second = mpc.solve_linear_mpc(
            shifted_reference,
            shifted_nominal,
            shifted_state,
            steer,
            config,
        )

        self.assertLess(
            float(np.max(np.abs(first[0] - second[0]))), 1.0e-3
        )
        self.assertLess(
            float(np.max(np.abs(first[1] - second[1]))), 1.0e-3
        )

    def test_mpc_failure_is_bounded_and_observable(self):
        config = LabConfig()
        path, _ = _loop_reference()
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=3.0)
        reference = ReferenceTracker(path).nearest(state)
        previous = ControlCommand(acceleration=0.0, steer=0.2)

        failure = mpc.MPCSolverError(
            "injected failure",
            status="user_limit",
            iterations=10000,
            solve_time_ms=12.5,
        )
        mpc.reset_diagnostics()
        with mock.patch.object(mpc, "_solve_linear_mpc", side_effect=failure):
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                command = mpc.control(state, reference, previous, config)

        diagnostics = mpc.get_diagnostics()
        self.assertTrue(
            any("user_limit" in str(item.message) for item in caught)
        )
        self.assertGreaterEqual(
            command.acceleration, -config.vehicle.max_decel
        )
        self.assertLessEqual(command.acceleration, 0.0)
        self.assertLessEqual(
            abs(command.steer - previous.steer),
            config.vehicle.max_steer_rate * config.sim.dt + 1.0e-12,
        )
        self.assertEqual(diagnostics.fallback_count, 1)
        self.assertEqual(diagnostics.controller_status, "fallback")
        self.assertEqual(diagnostics.solver_status, "user_limit")
        self.assertEqual(diagnostics.solver_iterations, 10000)

    def test_geometry_mismatch_is_rejected(self):
        _, reference = _loop_reference()
        for controller in CONTROLLERS:
            with self.subTest(controller=controller.NAME):
                config = LabConfig()
                config.vehicle.wheelbase += 0.1
                with self.assertRaisesRegex(
                    ValueError, r"lf \+ vehicle\.lr"
                ):
                    controller.control(
                        VehicleState(x=0.0, y=0.0, yaw=0.0, v=0.0),
                        reference,
                        ControlCommand(acceleration=0.0, steer=0.0),
                        config,
                    )

    def test_invalid_states_produce_bounded_fail_stop_commands(self):
        _, reference = _loop_reference()
        states = (
            VehicleState(x=math.nan, y=0.0, yaw=0.0, v=3.0),
            VehicleState(x=0.0, y=0.0, yaw=math.inf, v=3.0),
            VehicleState(x=0.0, y=0.0, yaw=0.0, v=99.0),
        )
        for controller in CONTROLLERS:
            for state in states:
                with self.subTest(controller=controller.NAME, state=state):
                    config = LabConfig()
                    previous = ControlCommand(acceleration=0.0, steer=0.2)
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore")
                        command = controller.control(
                            state, reference, previous, config
                        )
                    self.assert_bounded_command(command, previous, config)

    def test_empty_reference_produces_bounded_fail_stop_commands(self):
        empty = np.array([], dtype=float)
        path = Path(
            x=empty,
            y=empty,
            yaw=empty,
            curvature=empty,
            s=empty,
            target_speed=empty,
        )
        reference = ControllerReference(
            path=path,
            nearest_index=0,
            lateral_error=0.0,
            heading_error=0.0,
            curvature=0.0,
            target_speed=0.0,
        )
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=3.0)
        for controller in CONTROLLERS:
            with self.subTest(controller=controller.NAME):
                config = LabConfig()
                previous = ControlCommand(acceleration=0.0, steer=0.2)
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    command = controller.control(
                        state, reference, previous, config
                    )
                self.assert_bounded_command(command, previous, config)

    def test_invalid_safety_limits_are_rejected(self):
        _, reference = _loop_reference()
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=0.0)
        previous = ControlCommand(acceleration=0.0, steer=0.0)
        for controller in CONTROLLERS:
            with self.subTest(controller=controller.NAME):
                config = LabConfig()
                config.vehicle.max_steer = 0.0
                with self.assertRaises(ValueError):
                    controller.control(state, reference, previous, config)

    def test_lqr_preview_regressions_on_high_speed_sharp_routes(self):
        cases = (
            (lqr_kinematic, "right_angle", 0.6),
            (lqr_dynamic, "s_curve", 0.3),
        )
        for controller, route, error_limit in cases:
            with self.subTest(controller=controller.NAME, route=route):
                config = LabConfig()
                config.sim.route_name = route
                config.sim.speed_mode = "high"
                path, records, _ = run_simulation(controller, config)
                peak_error = max(abs(record.lateral_error) for record in records)
                self.assertLess(peak_error, error_limit)
                self.assertLess(
                    math.hypot(
                        records[-1].x - path.x[-1],
                        records[-1].y - path.y[-1],
                    ),
                    1.5,
                )

    def test_mpc_one_step_does_not_exceed_reference_speed(self):
        try:
            import cvxpy  # noqa: F401
        except ImportError:
            self.skipTest("cvxpy is not installed")

        config = LabConfig()
        target_speed = 3.0
        path, reference = _straight_reference(target_speed=target_speed, length=40.0)
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=2.85)
        previous = ControlCommand(acceleration=2.0, steer=0.0)
        mpc.reset_diagnostics()
        command = mpc.control(state, reference, previous, config)
        next_speed = state.v + command.acceleration * config.sim.dt
        self.assertLessEqual(next_speed, target_speed + 1.0e-9)
        self.assertEqual(mpc.get_diagnostics().controller_status, "ok")

    def test_mpc_brakes_when_already_above_reference_speed(self):
        try:
            import cvxpy  # noqa: F401
        except ImportError:
            self.skipTest("cvxpy is not installed")

        config = LabConfig()
        path, _ = _straight_reference(target_speed=3.0, length=40.0)
        tracker = ReferenceTracker(path)
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=4.2)
        reference = tracker.nearest(state)
        previous = ControlCommand(acceleration=0.0, steer=0.0)
        mpc.reset_diagnostics()
        command = mpc.control(state, reference, previous, config)
        self.assertLess(command.acceleration, 0.0)
        self.assertEqual(mpc.get_diagnostics().controller_status, "ok")
        self.assertLessEqual(
            state.v + command.acceleration * config.sim.dt,
            state.v - 0.5 * config.vehicle.max_decel * config.sim.dt,
        )

    def test_mpc_closed_loop_cruise_stays_at_or_below_target(self):
        try:
            import cvxpy  # noqa: F401
        except ImportError:
            self.skipTest("cvxpy is not installed")

        config = LabConfig()
        config.sim.target_speed = 3.0
        config.sim.max_time = 12.0
        path, _ = _straight_reference(target_speed=3.0, length=40.0)
        _, records, _ = run_simulation(mpc, config, path_override=path)
        peak_speed = max(record.speed for record in records)
        self.assertLessEqual(peak_speed, 3.0 + 1.0e-6)

    def test_lqr_kinematic_slows_for_previewed_sharp_corner(self):
        config = LabConfig()
        path, reference = _straight_then_corner_reference(
            target_speed=8.0,
            corner_s=10.0,
            kappa=0.80,
        )
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=8.0)
        command = lqr_kinematic.control(
            state,
            reference,
            ControlCommand(acceleration=0.0, steer=0.0),
            config,
        )
        self.assertLess(command.acceleration, -0.5)

    def test_pp_slows_when_steer_rate_cannot_track_s_curve(self):
        config = LabConfig()
        config.vehicle.max_steer_rate = math.radians(15.0)
        path, reference = _straight_then_corner_reference(
            target_speed=6.5,
            corner_s=8.0,
            kappa=0.27,
        )
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=6.5)
        command = pure_pursuit.control(
            state,
            reference,
            ControlCommand(acceleration=0.0, steer=0.0),
            config,
        )
        self.assertLess(command.acceleration, -0.5)


class StudentSpeedConstraintTests(unittest.TestCase):
    def test_preview_max_abs_curvature_uses_interval_max_not_endpoint(self):
        from vdm_lab.student._controller_utils import (
            preview_max_abs_curvature,
            preview_path_curvature,
        )

        _, reference = _straight_then_corner_reference(
            target_speed=8.0,
            corner_s=6.0,
            kappa=0.90,
            length=20.0,
        )
        reference.path.curvature = np.where(
            reference.path.s >= 12.0, 0.10, reference.path.curvature
        )
        self.assertAlmostEqual(
            preview_max_abs_curvature(reference, 14.0), 0.90, places=6
        )
        self.assertAlmostEqual(
            preview_path_curvature(reference, 14.0), 0.10, places=6
        )

    def test_tracking_speed_limit_keeps_default_lab_speeds(self):
        from vdm_lab.student._controller_utils import tracking_speed_limit

        vehicle = LabConfig().vehicle
        cases = (
            (7.0, 1.0 / 12.0),
            (6.5, 0.2705),
            (8.0, 0.2705),
            (7.0, 0.20),
        )
        for target, kappa in cases:
            with self.subTest(target=target, kappa=kappa):
                self.assertGreaterEqual(
                    tracking_speed_limit(target, kappa, vehicle), target - 1.0e-9
                )

    def test_tracking_speed_limit_slows_for_tight_steer_rate_or_sharp_corner(self):
        from vdm_lab.student._controller_utils import tracking_speed_limit

        vehicle = LabConfig().vehicle
        vehicle.max_steer_rate = math.radians(15.0)
        slowed = tracking_speed_limit(6.5, 0.27, vehicle)
        self.assertLess(slowed, 3.6)
        self.assertGreater(slowed, 2.5)

        vehicle = LabConfig().vehicle
        sharp = tracking_speed_limit(8.0, 0.80, vehicle)
        self.assertLess(sharp, 4.0)
        self.assertGreater(sharp, 2.0)

    def test_command_target_speed_sees_corner_before_arrival(self):
        from vdm_lab.student._controller_utils import command_target_speed

        _, reference = _straight_then_corner_reference(
            target_speed=8.0,
            corner_s=10.0,
            kappa=0.80,
        )
        vehicle = LabConfig().vehicle
        self.assertLess(
            command_target_speed(8.0, reference, 8.0, vehicle), 4.0
        )

    def test_horizon_speed_caps_are_reachable_and_hold_cruise(self):
        config = LabConfig()
        horizon = config.controller.mpc_horizon
        z_ref = np.zeros((4, horizon + 1))
        z_ref[2, :] = 3.0
        overspeed = mpc._horizon_speed_caps(4.2, z_ref, config)
        dt = config.sim.dt
        max_decel = config.vehicle.max_decel
        self.assertAlmostEqual(overspeed[0], 4.2 - max_decel * dt, places=6)
        self.assertTrue(np.all(np.diff(overspeed) <= 1.0e-12))
        self.assertGreaterEqual(overspeed[-1], 3.0 - 1.0e-9)

        cruise = mpc._horizon_speed_caps(3.0, z_ref, config)
        np.testing.assert_allclose(cruise, 3.0)

    def test_mpc_horizon_reference_caps_speed_on_sharp_curvature(self):
        config = LabConfig()
        path, reference = _straight_then_corner_reference(
            target_speed=8.0,
            corner_s=0.0,
            kappa=0.80,
        )
        reference.nearest_index = 0
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=8.0)
        z_ref = mpc.nearest_horizon_reference(state, reference, config)
        self.assertTrue(np.all(z_ref[2, :] < 4.0))

    def test_lqr_dynamic_slows_for_previewed_sharp_corner(self):
        config = LabConfig()
        _, reference = _straight_then_corner_reference(
            target_speed=8.0,
            corner_s=10.0,
            kappa=0.80,
        )
        command = lqr_dynamic.control(
            VehicleState(x=0.0, y=0.0, yaw=0.0, v=8.0),
            reference,
            ControlCommand(acceleration=0.0, steer=0.0),
            config,
        )
        self.assertLess(command.acceleration, -0.5)

    def test_pp_does_not_slow_on_default_s_curve_at_45deg_s(self):
        config = LabConfig()
        _, reference = _straight_then_corner_reference(
            target_speed=6.5,
            corner_s=8.0,
            kappa=0.27,
        )
        command = pure_pursuit.control(
            VehicleState(x=0.0, y=0.0, yaw=0.0, v=6.5),
            reference,
            ControlCommand(acceleration=0.0, steer=0.0),
            config,
        )
        self.assertGreater(command.acceleration, -0.2)
        self.assertLess(command.acceleration, 0.2)

    def test_pp_and_lqr_one_step_do_not_exceed_cruise_target(self):
        _, reference = _straight_reference(target_speed=3.0, length=40.0)
        state = VehicleState(x=0.0, y=0.0, yaw=0.0, v=2.85)
        previous = ControlCommand(acceleration=2.0, steer=0.0)
        config = LabConfig()
        for controller in (pure_pursuit, lqr_kinematic, lqr_dynamic):
            with self.subTest(controller=controller.NAME):
                command = controller.control(state, reference, previous, config)
                next_speed = state.v + command.acceleration * config.sim.dt
                self.assertLessEqual(next_speed, 3.0 + 1.0e-9)

    def test_mpc_circle_cruise_stays_at_or_below_target(self):
        try:
            import cvxpy  # noqa: F401
        except ImportError:
            self.skipTest("cvxpy is not installed")

        config = LabConfig()
        config.sim.route_name = "circle"
        config.sim.target_speed = 3.0
        config.sim.max_time = 45.0
        _, records, _ = run_simulation(mpc, config)
        self.assertLessEqual(max(record.speed for record in records), 3.0 + 1.0e-6)
        self.assertEqual(mpc.get_diagnostics().controller_status, "ok")

    def test_pp_s_curve_completes_with_steer_rate_15deg_s(self):
        config = LabConfig()
        config.sim.route_name = "s_curve"
        config.sim.speed_mode = "medium"
        config.sim.max_time = 60.0
        config.vehicle.max_steer_rate = math.radians(15.0)
        path, records, _ = run_simulation(pure_pursuit, config)
        end_dist = math.hypot(
            records[-1].x - path.x[-1], records[-1].y - path.y[-1]
        )
        self.assertLess(end_dist, 1.5)
        self.assertLess(records[-1].speed, 1.0)
        self.assertLess(records[-1].time, 60.0)


def _straight_reference(target_speed=3.0, length=40.0, ds=0.5):
    s = np.arange(0.0, length + ds, ds)
    path = Path(
        x=s.copy(),
        y=np.zeros_like(s),
        yaw=np.zeros_like(s),
        curvature=np.zeros_like(s),
        s=s,
        target_speed=np.full_like(s, target_speed),
    )
    path.target_speed[-1] = 0.0
    reference = ControllerReference(
        path=path,
        nearest_index=0,
        lateral_error=0.0,
        heading_error=0.0,
        curvature=0.0,
        target_speed=target_speed,
    )
    return path, reference


def _straight_then_corner_reference(target_speed, corner_s, kappa, length=30.0, ds=0.5):
    s = np.arange(0.0, length + ds, ds)
    curvature = np.where(s >= corner_s, float(kappa), 0.0)
    path = Path(
        x=s.copy(),
        y=np.zeros_like(s),
        yaw=np.zeros_like(s),
        curvature=curvature,
        s=s,
        target_speed=np.full_like(s, target_speed),
    )
    path.target_speed[-1] = 0.0
    reference = ControllerReference(
        path=path,
        nearest_index=0,
        lateral_error=0.0,
        heading_error=0.0,
        curvature=0.0,
        target_speed=target_speed,
    )
    return path, reference


if __name__ == "__main__":
    unittest.main()
