from __future__ import annotations

import math
import unittest

import numpy as np

import arm
from arm import config as arm_config
import chassis
from chassis import ChassisPathError


class MotionPathTest(unittest.TestCase):
    def test_chassis_direct_path(self) -> None:
        path = chassis.direct((0.0, 0.0, 0.0), (0.2, 0.1, math.radians(10.0)), 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (3,))
        np.testing.assert_allclose(path(0.0, order=0), (0.0, 0.0, 0.0), atol=1e-6)
        np.testing.assert_allclose(path(path.duration, order=0), (0.2, 0.1, math.radians(10.0)), atol=1e-6)

    def test_chassis_s_cross_direction_guard(self) -> None:
        with self.assertRaises(ChassisPathError):
            chassis.s_cross((0.0, 0.0, 0.0), (1.2, 0.2, 0.0), 1.0)

    def test_chassis_s_cross_path(self) -> None:
        path = chassis.s_cross((-1.4, -0.5, 0.0), (1.4, 0.5, math.radians(90.0)), 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (3,))
        np.testing.assert_allclose(path(0.0, order=0), (-1.4, -0.5, 0.0), atol=1e-6)
        np.testing.assert_allclose(path(path.duration, order=0), (1.4, 0.5, math.radians(90.0)), atol=1e-6)

    def test_arm_move_path(self) -> None:
        path = arm.move((0.055, 0.0, 0.0, 0.30, 1.0), (0.08, 0.0, 0.0, 0.32, 0.5), 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (5,))
        self.assertAlmostEqual(float(path(0.0, order=0)[0]), 0.30, places=6)
        self.assertAlmostEqual(float(path(path.duration, order=0)[0]), 0.32, places=6)
        self.assertAlmostEqual(float(path(path.duration, order=0)[4]), 0.5, places=6)

    def test_arm_servo_targets_do_not_constrain_toppra(self) -> None:
        start = (0.055, 0.0, 0.0, 0.30, 1.0)
        same_motion_small_servo = (0.08, 0.0, 0.0, 0.32, 0.9)
        same_motion_large_servo = (0.08, 0.0, 0.0, 0.32, 0.0)

        path_small = arm.move(start, same_motion_small_servo, 1.0)
        path_large = arm.move(start, same_motion_large_servo, 1.0)

        self.assertAlmostEqual(path_small.duration, path_large.duration, places=6)
        dq = np.asarray(path_large(path_large.duration / 2.0, order=1), dtype=float)
        self.assertEqual(dq.shape, (5,))
        np.testing.assert_allclose(dq[3:], (0.0, 0.0), atol=1e-9)

    def test_arm_servo_only_change_keeps_kind_busy_until_servo_can_arrive(self) -> None:
        path = arm.grip_lift((0.24, 5.8, 3.8, 0.0, 1.0), 0.0, 1.0)
        servo_only = arm.move((0.055, 0.0, 0.0, 0.30, 1.0), (0.055, 0.0, 0.0, 0.30, 0.0), 1.0)

        self.assertGreater(path.duration, 0.0)
        self.assertAlmostEqual(
            servo_only.duration,
            1.0 / arm_config.GRIPPER_OPENING_SERVO_V_LIMIT,
            places=6,
        )
        np.testing.assert_allclose(servo_only(0.0, order=1), (0.0, 0.0, 0.0, 0.0, 0.0), atol=1e-9)
        self.assertAlmostEqual(float(servo_only(0.0, order=0)[4]), 0.0, places=6)

    def test_arm_set_gripper_keeps_end_effector_pose_and_changes_opening(self) -> None:
        state = (0.055, 0.0, 0.0, 0.30, 1.0)
        path = arm.set_gripper(state, 0.0)
        kin = arm.FiveBarKinematics()
        q1, q2, gripper_yaw = kin.ik(state[0], state[1], state[2])

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (5,))
        np.testing.assert_allclose(path(0.0, order=0)[:4], (state[3], q1, q2, gripper_yaw), atol=1e-6)
        np.testing.assert_allclose(path(path.duration, order=0)[:4], (state[3], q1, q2, gripper_yaw), atol=1e-6)
        self.assertAlmostEqual(float(path(0.0, order=0)[4]), 0.0, places=6)
        self.assertAlmostEqual(
            path.duration,
            1.0 / arm_config.GRIPPER_OPENING_SERVO_V_LIMIT,
            places=6,
        )

    def test_arm_move_converts_chassis_relative_gripper_yaw_to_lower_passive_link_frame(self) -> None:
        chassis_relative_yaw = 0.0
        start = (0.08, 0.0, chassis_relative_yaw, 0.30, 1.0)
        end = (0.055, 0.0, 0.0, 0.32, 0.5)
        path = arm.move(start, end, 1.0)
        kin = arm.FiveBarKinematics()
        q1, q2, start_servo_target = kin.ik(start[0], start[1], chassis_relative_yaw)
        end_q1, end_q2, end_servo_target = kin.ik(end[0], end[1], end[2])
        x, y, yaw = kin.fk(q1, q2, start_servo_target)
        end_x, end_y, end_yaw = kin.fk(end_q1, end_q2, end_servo_target)

        np.testing.assert_allclose(path(0.0, order=0)[:3], (start[3], q1, q2), atol=1e-6)
        self.assertAlmostEqual(float(path(0.0, order=0)[3]), end_servo_target, places=6)
        self.assertAlmostEqual(x, start[0], places=6)
        self.assertAlmostEqual(y, start[1], places=6)
        self.assertAlmostEqual(yaw, chassis_relative_yaw, places=6)
        self.assertAlmostEqual(end_x, end[0], places=6)
        self.assertAlmostEqual(end_y, end[1], places=6)
        self.assertAlmostEqual(end_yaw, end[2], places=6)

    def test_arm_ik_fk_round_trip_with_outward_elbow_branch(self) -> None:
        kin = arm.FiveBarKinematics()
        for expected_x, expected_y, expected_yaw in (
            (-0.08, 0.0, 0.0),
            (0.08, 0.0, 0.0),
            (-0.055, 0.02, 0.2),
            (0.055, 0.02, -0.2),
        ):
            q1, q2, gripper_yaw = kin.ik(expected_x, expected_y, expected_yaw)
            x, y, yaw = kin.fk(q1, q2, gripper_yaw)
            self.assertAlmostEqual(x, expected_x, places=6)
            self.assertAlmostEqual(y, expected_y, places=6)
            self.assertAlmostEqual(yaw, expected_yaw, places=6)

    def test_arm_ik_center_line_uses_complementary_outward_angles(self) -> None:
        kin = arm.FiveBarKinematics()
        for expected_x, expected_q1_side, expected_q2_side in (
            (0.3, -1, 1),
            (0.08, -1, 1),
            (-0.08, 1, -1),
            (-0.3, 1, -1),
        ):
            q1, q2, gripper_yaw = kin.ik(expected_x, 0.0, 0.0)
            x, y, yaw = kin.fk(q1, q2, gripper_yaw)
            self.assertAlmostEqual(q1 + q2, 2.0 * math.pi, places=6)
            self.assertEqual(math.copysign(1, q1 - math.pi), expected_q1_side)
            self.assertEqual(math.copysign(1, q2 - math.pi), expected_q2_side)
            self.assertAlmostEqual(x, expected_x, places=6)
            self.assertAlmostEqual(y, 0.0, places=6)
            self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_arm_ik_margin_only_rejects_outer_boundary(self) -> None:
        kin = arm.FiveBarKinematics()
        q1, q2, gripper_yaw = kin.ik(0.0, 0.0, 0.0)
        self.assertAlmostEqual(q1, math.pi, places=6)
        self.assertAlmostEqual(q2, math.pi, places=6)
        self.assertGreaterEqual(gripper_yaw, 0.0)
        self.assertLessEqual(gripper_yaw, 2.0 * math.pi)

        with self.assertRaises(arm.ArmKinematicsError):
            kin.ik(0.388, 0.0, 0.0)

    def test_arm_move_cross_half_plane_path(self) -> None:
        path = arm.move((0.3, 0.0, 0.0, 0.30, 1.0), (-0.3, 0.0, 0.0, 0.30, 0.5), 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (5,))
        self.assertGreater(path.duration, 0.0)
        samples = np.asarray(path(np.linspace(0.0, path.duration, 20), order=0), dtype=float)
        self.assertTrue(np.all(samples[:, 1] >= 0.0))
        self.assertTrue(np.all(samples[:, 1] <= 2.0 * math.pi))
        self.assertTrue(np.all(samples[:, 2] >= 0.0))
        self.assertTrue(np.all(samples[:, 2] <= 2.0 * math.pi))
        np.testing.assert_allclose(samples[:, 1] + samples[:, 2], 2.0 * math.pi, atol=1e-6)

    def test_arm_half_plane_precomputed_joint_states_keep_chassis_yaw_zero(self) -> None:
        kin = arm.FiveBarKinematics()
        for q1, q2, gripper_yaw in arm_config.HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X:
            _, _, yaw = kin.fk(q1, q2, gripper_yaw)
            self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_arm_prepare_pick_path(self) -> None:
        path = arm.prepare_pick((0.055, 0.0, 0.0, 0.30, 1.0), (0.08, 0.0, 0.0, 0.24, 0.8), 0.36, 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (5,))
        self.assertAlmostEqual(float(path(0.0, order=0)[0]), 0.30, places=6)
        self.assertAlmostEqual(float(path(path.duration, order=0)[0]), 0.24, places=6)

    def test_arm_grip_lift_path(self) -> None:
        path = arm.grip_lift((0.24, 5.8, 3.8, 0.0, 1.0), 0.0, 1.0)

        self.assertEqual(np.asarray(path(0.0, order=0)).shape, (5,))
        self.assertAlmostEqual(float(path(path.duration, order=0)[0]), 0.37, places=6)
        self.assertAlmostEqual(float(path(path.duration, order=0)[4]), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
