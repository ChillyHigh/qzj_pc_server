from __future__ import annotations

import math
import unittest

from chassis.geometry import is_drive_pose_colliding, plan_avoidance_path
from plan.setting import FIELD_X_MAX, FIELD_X_MIN, FIELD_Y_MAX, FIELD_Y_MIN


class TestChassisFieldBounds(unittest.TestCase):
    def test_pose_collides_when_chassis_body_crosses_field_boundary(self) -> None:
        self.assertTrue(is_drive_pose_colliding(FIELD_X_MAX, 0.0, 0.0, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(FIELD_X_MIN, 0.0, math.pi, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(0.0, FIELD_Y_MAX, 0.0, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(0.0, FIELD_Y_MIN, 0.0, skip_boxes=True))

    def test_pose_inside_field_does_not_collide_with_boundary(self) -> None:
        self.assertFalse(is_drive_pose_colliding(0.3, 0.0, 0.0, skip_boxes=True))

    def test_theta_star_rejects_endpoint_when_chassis_body_crosses_field_boundary(self) -> None:
        with self.assertRaisesRegex(ValueError, "终点"):
            plan_avoidance_path((0.3, 0.0, 0.0), (FIELD_X_MAX, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
