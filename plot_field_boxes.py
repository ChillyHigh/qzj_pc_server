from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle


# Standalone copy of the field settings. Do not import project modules here.
FIELD_X_MIN = -2.0
FIELD_X_MAX = 2.0
FIELD_Y_MIN = -1.0
FIELD_Y_MAX = 1.0

# box_id: ((center_x, center_y), (half_x, half_y))
BOX_RECTS = {
    1: ((-1.855, 0.500), (0.105, 0.150)),
    2: ((-1.855, -0.500), (0.105, 0.150)),
    3: ((-1.600, 0.000), (0.105, 0.150)),
    4: ((1.640, 0.875), (0.150, 0.105)),
    5: ((1.875, 0.400), (0.105, 0.150)),
    6: ((1.875, 0.000), (0.105, 0.150)),
    7: ((1.875, -0.400), (0.105, 0.150)),
    8: ((1.640, -0.875), (0.150, 0.105)),
}

# Chassis pose uses the active-wheel diagonal intersection, not geometry center.
# box_id: (drive_x, drive_y, yaw_rad)
BOX_123_CHASSIS_POSES = {
    1: (-1.480, 0.530, 0.0),
    2: (-1.480, -0.530, 0.0),
    3: (-1.300, 0.000, 0.0),
}

OBSTACLE_CENTERS = ((-1.000, 0.000), (1.000, 0.000))
OBSTACLE_RADIUS = 0.051

CHASSIS_HALF_X_FRONT = 0.215
CHASSIS_HALF_X_REAR = 0.195
CHASSIS_HALF_Y = 0.345
CHASSIS_CENTER_FROM_DRIVE = (0.035, 0.0)

OUTPUT_PATH = Path("field_boxes_chassis_123.png")


def rotate_translate(
    x: float,
    y: float,
    pose: tuple[float, float, float],
) -> tuple[float, float]:
    px, py, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return px + x * c - y * s, py + x * s + y * c


def chassis_corners(pose: tuple[float, float, float]) -> list[tuple[float, float]]:
    center_x, center_y = CHASSIS_CENTER_FROM_DRIVE
    local = [
        (center_x + CHASSIS_HALF_X_FRONT, center_y + CHASSIS_HALF_Y),
        (center_x + CHASSIS_HALF_X_FRONT, center_y - CHASSIS_HALF_Y),
        (center_x - CHASSIS_HALF_X_REAR, center_y - CHASSIS_HALF_Y),
        (center_x - CHASSIS_HALF_X_REAR, center_y + CHASSIS_HALF_Y),
    ]
    return [rotate_translate(x, y, pose) for x, y in local]


def draw_box(ax, box_id: int) -> None:
    (cx, cy), (hx, hy) = BOX_RECTS[box_id]
    face = "#f6c177" if box_id <= 3 else "#9ccfd8"
    rect = Rectangle(
        (cx - hx, cy - hy),
        hx * 2.0,
        hy * 2.0,
        facecolor=face,
        edgecolor="#232136",
        linewidth=1.4,
        alpha=0.9,
    )
    ax.add_patch(rect)
    ax.text(cx, cy, f"Box {box_id}", ha="center", va="center", fontsize=9)


def draw_chassis(ax, box_id: int, pose: tuple[float, float, float]) -> None:
    drive_x, drive_y, yaw = pose
    poly = Polygon(
        chassis_corners(pose),
        closed=True,
        facecolor="#ea9a97",
        edgecolor="#b4637a",
        linewidth=1.8,
        alpha=0.35,
    )
    ax.add_patch(poly)
    ax.plot(drive_x, drive_y, marker="o", color="#b4637a", markersize=4)

    arrow_len = 0.18
    ax.arrow(
        drive_x,
        drive_y,
        arrow_len * math.cos(yaw),
        arrow_len * math.sin(yaw),
        head_width=0.035,
        head_length=0.045,
        color="#b4637a",
        length_includes_head=True,
    )
    ax.text(
        drive_x,
        drive_y + 0.41,
        f"Chassis for Box {box_id}\n({drive_x:.3f}, {drive_y:.3f}, {math.degrees(yaw):.0f} deg)",
        ha="center",
        va="bottom",
        fontsize=8,
        color="#7d3c54",
    )


def main() -> None:
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.set_aspect("equal", adjustable="box")

    field = Rectangle(
        (FIELD_X_MIN, FIELD_Y_MIN),
        FIELD_X_MAX - FIELD_X_MIN,
        FIELD_Y_MAX - FIELD_Y_MIN,
        fill=False,
        edgecolor="#232136",
        linewidth=2.0,
    )
    ax.add_patch(field)

    for box_id in sorted(BOX_RECTS):
        draw_box(ax, box_id)

    for ox, oy in OBSTACLE_CENTERS:
        ax.add_patch(
            Circle(
                (ox, oy),
                OBSTACLE_RADIUS,
                facecolor="#56526e",
                edgecolor="#232136",
                linewidth=1.0,
                alpha=0.9,
            )
        )
        ax.text(ox, oy + 0.09, "Obstacle", ha="center", va="bottom", fontsize=8)

    for box_id, pose in BOX_123_CHASSIS_POSES.items():
        draw_chassis(ax, box_id, pose)

    ax.axhline(0.0, color="#9893a5", linewidth=0.8, linestyle="--")
    ax.axvline(0.0, color="#9893a5", linewidth=0.8, linestyle="--")
    ax.set_xlim(FIELD_X_MIN - 0.12, FIELD_X_MAX + 0.12)
    ax.set_ylim(FIELD_Y_MIN - 0.12, FIELD_Y_MAX + 0.12)
    ax.set_xlabel("x / m")
    ax.set_ylabel("y / m")
    ax.set_title("Field boxes and chassis poses for Box 1/2/3")
    ax.grid(True, linewidth=0.4, alpha=0.35)

    fig.tight_layout()
    fig.savefig(OUTPUT_PATH, dpi=180)
    print(f"saved: {OUTPUT_PATH.resolve()}")


if __name__ == "__main__":
    main()
