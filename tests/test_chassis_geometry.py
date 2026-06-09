from __future__ import annotations

import math
import unittest

from chassis.geometry import is_drive_pose_colliding, plan_avoidance_path
from plan.setting import (
    CHASSIS_HALF_Y,
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
    FUNNEL_SIDE_EXTENSION_Y,
    OBSTACLE_RADIUS,
)


class TestChassisFieldBounds(unittest.TestCase):
    def test_pose_collides_when_chassis_body_crosses_field_boundary(self) -> None:
        self.assertTrue(is_drive_pose_colliding(FIELD_X_MAX, 0.0, 0.0, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(FIELD_X_MIN, 0.0, math.pi, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(0.0, FIELD_Y_MAX, 0.0, skip_boxes=True))
        self.assertTrue(is_drive_pose_colliding(0.0, FIELD_Y_MIN, 0.0, skip_boxes=True))

    def test_pose_inside_field_does_not_collide_with_boundary(self) -> None:
        self.assertFalse(is_drive_pose_colliding(0.3, 0.0, 0.0, skip_boxes=True))

    def test_funnel_extension_does_not_collide_with_field_boundary(self) -> None:
        y = FIELD_Y_MAX - CHASSIS_HALF_Y - FUNNEL_SIDE_EXTENSION_Y / 2.0
        self.assertFalse(is_drive_pose_colliding(0.3, y, 0.0, skip_boxes=True))

    def test_theta_star_rejects_endpoint_when_chassis_body_crosses_field_boundary(self) -> None:
        with self.assertRaisesRegex(ValueError, "终点"):
            plan_avoidance_path((0.3, 0.0, 0.0), (FIELD_X_MAX, 0.0, 0.0))

    def test_funnel_extension_collides_with_middle_obstacle(self) -> None:
        y = CHASSIS_HALF_Y + OBSTACLE_RADIUS + FUNNEL_SIDE_EXTENSION_Y / 2.0
        self.assertTrue(is_drive_pose_colliding(-1.0, y, 0.0, skip_boxes=True))

    def test_funnel_extension_collides_with_pickup_box(self) -> None:
        x = -1.855 + 0.105 + CHASSIS_HALF_Y + FUNNEL_SIDE_EXTENSION_Y / 2.0
        self.assertTrue(is_drive_pose_colliding(x, 0.500, math.pi / 2.0))

    def test_funnel_extension_does_not_collide_with_drop_box(self) -> None:
        y = 0.875 - 0.105 - CHASSIS_HALF_Y - FUNNEL_SIDE_EXTENSION_Y / 2.0
        self.assertFalse(is_drive_pose_colliding(1.355, y, 0.0))


if __name__ == "__main__":
    unittest.main()
