from __future__ import annotations

import math
import unittest

from planner import config


def _rotate(point: tuple[float, float], yaw: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * point[0] - s * point[1], s * point[0] + c * point[1]


def _pose_chassis_local(point: tuple[float, float], pose: config.Pose) -> tuple[float, float]:
    center_offset = _rotate(config.CHASSIS_CENTER_FROM_DRIVE, pose[config.DRIVE_YAW])
    center_world = (pose[config.DRIVE_X] + center_offset[0], pose[config.DRIVE_Y] + center_offset[1])
    return _rotate(
        (point[0] - center_world[0], point[1] - center_world[1]),
        -pose[config.DRIVE_YAW],
    )


def _pose_arm_local(point: tuple[float, float], pose: config.Pose) -> tuple[float, float]:
    chassis_local = _pose_chassis_local(point, pose)
    return (
        chassis_local[0] - config.ARM_ORIGIN_IN_CHASSIS_CENTER[0],
        chassis_local[1] - config.ARM_ORIGIN_IN_CHASSIS_CENTER[1],
    )


def _distance_to_chassis_rect(point: tuple[float, float], pose: config.Pose) -> float:
    local = _pose_chassis_local(point, pose)
    dx = max(abs(local[0]) - config.CHASSIS_LENGTH / 2.0, 0.0)
    dy = max(abs(local[1]) - config.CHASSIS_WIDTH / 2.0, 0.0)
    return math.hypot(dx, dy)


def _rect_corners(rect: config.TargetRect) -> tuple[tuple[float, float], ...]:
    center = rect[config.TARGET_CENTER]
    half_size = rect[config.TARGET_HALF_SIZE]
    return (
        (center[0] - half_size[0], center[1] - half_size[1]),
        (center[0] - half_size[0], center[1] + half_size[1]),
        (center[0] + half_size[0], center[1] + half_size[1]),
        (center[0] + half_size[0], center[1] - half_size[1]),
    )


def _oriented_chassis_corners(pose: config.Pose) -> tuple[tuple[float, float], ...]:
    center_offset = _rotate(config.CHASSIS_CENTER_FROM_DRIVE, pose[config.DRIVE_YAW])
    center = (pose[config.DRIVE_X] + center_offset[0], pose[config.DRIVE_Y] + center_offset[1])
    corners = []
    for sx, sy in ((-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)):
        local = (sx * config.CHASSIS_LENGTH / 2.0, sy * config.CHASSIS_WIDTH / 2.0)
        offset = _rotate(local, pose[config.DRIVE_YAW])
        corners.append((center[0] + offset[0], center[1] + offset[1]))
    return tuple(corners)


def _polygon_axes(poly: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
    axes = []
    for idx, point in enumerate(poly):
        next_point = poly[(idx + 1) % len(poly)]
        edge = (next_point[0] - point[0], next_point[1] - point[1])
        length = math.hypot(edge[0], edge[1])
        axes.append((-edge[1] / length, edge[0] / length))
    return tuple(axes)


def _project_polygon(poly: tuple[tuple[float, float], ...], axis: tuple[float, float]) -> tuple[float, float]:
    values = [point[0] * axis[0] + point[1] * axis[1] for point in poly]
    return min(values), max(values)


def _rects_overlap(pose: config.Pose, rect: config.TargetRect) -> bool:
    chassis = _oriented_chassis_corners(pose)
    target = _rect_corners(rect)
    for axis in _polygon_axes(chassis) + _polygon_axes(target):
        chassis_min, chassis_max = _project_polygon(chassis, axis)
        target_min, target_max = _project_polygon(target, axis)
        if chassis_max <= target_min or target_max <= chassis_min:
            return False
    return True


class PlannerConfigTest(unittest.TestCase):
    def test_pickup_poses_put_box_on_arm_y_zero(self) -> None:
        for position_id, pose in config.PICKUP_POSES.items():
            box_center = config.TARGET_RECTS[position_id][config.TARGET_CENTER]
            box_in_arm = _pose_arm_local(box_center, pose)

            self.assertAlmostEqual(box_in_arm[0], pose[config.ARM_X], places=6)
            self.assertAlmostEqual(box_in_arm[1], pose[config.ARM_Y], places=6)

    def test_pickup_poses_do_not_touch_middle_obstacles(self) -> None:
        for pose in config.PICKUP_POSES.values():
            for obstacle in config.OBSTACLE_CENTERS:
                distance = _distance_to_chassis_rect(obstacle, pose)
                self.assertGreater(distance, config.OBSTACLE_RADIUS)

    def test_pickup_poses_use_zero_gripper_yaw(self) -> None:
        for pose in config.PICKUP_POSES.values():
            self.assertEqual(pose[config.GRIPPER_YAW], 0.0)

    def test_funnel_drop_poses_align_box_center_x_and_edge_y(self) -> None:
        for position_id, carrier_poses in config.DROP_POSES.items():
            for carrier in ("upper_funnel", "lower_funnel"):
                pose = carrier_poses[carrier]
                center_local = _pose_chassis_local(
                    config.TARGET_RECTS[position_id][config.TARGET_CENTER],
                    pose,
                )
                edge_local = _pose_chassis_local(
                    config.FUNNEL_DROP_BOX_EDGE_POINTS[position_id][carrier],
                    pose,
                )
                expected_y = (
                    config.UPPER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y
                    if carrier == "upper_funnel"
                    else config.LOWER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y
                )

                self.assertAlmostEqual(center_local[0], config.FUNNEL_EDGE_IN_CHASSIS_CENTER_X, places=6)
                self.assertAlmostEqual(edge_local[1], expected_y, places=6)

    def test_funnel_drop_poses_do_not_overlap_target_box(self) -> None:
        for position_id, carrier_poses in config.DROP_POSES.items():
            for carrier in ("upper_funnel", "lower_funnel"):
                self.assertFalse(_rects_overlap(carrier_poses[carrier], config.TARGET_RECTS[position_id]))

    def test_gripper_drop_poses_put_box_center_on_arm_y_zero(self) -> None:
        for position_id, carrier_poses in config.DROP_POSES.items():
            pose = carrier_poses["gripper"]
            box_center = config.TARGET_RECTS[position_id][config.TARGET_CENTER]
            arm_local = _pose_arm_local(box_center, pose)
            self.assertAlmostEqual(arm_local[0], pose[config.ARM_X], places=6)
            self.assertAlmostEqual(arm_local[1], pose[config.ARM_Y], places=6)

    def test_gripper_drop_poses_do_not_overlap_drop_boxes(self) -> None:
        for carrier_poses in config.DROP_POSES.values():
            pose = carrier_poses["gripper"]
            for position_id, rect in config.TARGET_RECTS.items():
                if position_id >= 4:
                    self.assertFalse(_rects_overlap(pose, rect))

    def test_gripper_drop_yaw_aligns_gripper_and_box_long_edges(self) -> None:
        expected_gripper_yaw = {
            4: math.radians(45.0),
            5: 0.0,
            6: 0.0,
            7: 0.0,
            8: math.radians(135.0),
        }
        for position_id, yaw in expected_gripper_yaw.items():
            pose = config.DROP_POSES[position_id]["gripper"]
            self.assertAlmostEqual(pose[config.GRIPPER_YAW], yaw, places=6)


if __name__ == "__main__":
    unittest.main()
