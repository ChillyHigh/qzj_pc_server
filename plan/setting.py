from __future__ import annotations

# ---- 坐标下标 ----------------------------------------------------------------
# Pose 是 7 元组 (drive_x, drive_y, drive_yaw, arm_x, arm_y, gripper_yaw, h)
DRIVE_X = 0
DRIVE_Y = 1
DRIVE_YAW = 2
ARM_X = 3
ARM_Y = 4
GRIPPER_YAW = 5
H = 6

# TargetRect 下标：(center, half_size)
TARGET_CENTER = 0
TARGET_HALF_SIZE = 1

# TargetRect 类型：((center_x, center_y), (half_x, half_y))
TargetRect = tuple[tuple[float, float], tuple[float, float]]

# ---- 底盘几何（相对四主动轮对角线交点） -----------------------------------------------
# 碰撞检查以驱动轮中心为原点，前后左右范围不得碰到障碍物
# 前方（+x 方向）：底盘局部坐标 x 正方向
CHASSIS_HALF_X_FRONT = 0.215
CHASSIS_HALF_X_REAR = 0.195
CHASSIS_HALF_Y = 0.345
FUNNEL_SIDE_EXTENSION_Y = 0.020
# 底盘几何中心相对驱动轮中心的偏移（画图 / 换算用）
CHASSIS_CENTER_FROM_DRIVE = (0.035, 0.0)
# 总长宽（画图 / 换算用）
CHASSIS_LENGTH = CHASSIS_HALF_X_FRONT + CHASSIS_HALF_X_REAR
CHASSIS_WIDTH = CHASSIS_HALF_Y * 2.0

# ---- 场地范围 ----------------------------------------------------------------
FIELD_X_MIN = -2.0
FIELD_X_MAX = 2.0
FIELD_Y_MIN = -1.0
FIELD_Y_MAX = 1.0

# ---- 机械臂 ----------------------------------------------------------------
# 机械臂原点在底盘几何中心坐标系下的偏移
ARM_ORIGIN_IN_CHASSIS_CENTER = (-0.110, 0.0)

# ---- 场地障碍物 ---------------------------------------------------------------
OBSTACLE_RADIUS = 0.051
OBSTACLE_CENTERS = ((-1.000, 0.000), (1.000, 0.000))

# ---- 目标箱 ------------------------------------------------------------------
# key = 箱编号 1-8，value = ((center_x, center_y), (half_x, half_y))
TARGET_RECTS: dict[int, TargetRect] = {
    1: ((-1.855, 0.500), (0.105, 0.150)),
    2: ((-1.855, -0.500), (0.105, 0.150)),
    3: ((-1.600, 0.000), (0.105, 0.150)),
    4: ((1.640, 0.875), (0.150, 0.105)),
    5: ((1.875, 0.400), (0.105, 0.150)),
    6: ((1.875, 0.000), (0.105, 0.150)),
    7: ((1.875, -0.400), (0.105, 0.150)),
    8: ((1.640, -0.875), (0.150, 0.105)),
}
