from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from itertools import product
from typing import Literal


Carrier = Literal["gripper", "upper_funnel", "lower_funnel"]
Point = tuple[float, float]


@dataclass(frozen=True)
class TargetRect:
    point_id: int
    center: Point
    half_x: float
    half_y: float


@dataclass(frozen=True)
class LinkSolution:
    q1: float
    q2: float
    upper_elbow: Point
    lower_elbow: Point
    endpoint: Point


@dataclass(frozen=True)
class PoseCandidate:
    point_id: int
    carrier: Carrier
    x: float
    y: float
    yaw: float
    q1: float | None
    q2: float | None
    link_solution: LinkSolution | None
    gripper_rect_center: Point | None
    note: str


CHASSIS_HALF_X = 0.180
CHASSIS_HALF_Y = 0.335

MOTOR_X = 0.110
MOTOR_Y_OFFSET = 0.080
ACTIVE_LINK = 0.160
PASSIVE_LINK = 0.240

UPPER_FUNNEL_OUTLET = (-0.070, CHASSIS_HALF_Y)
LOWER_FUNNEL_OUTLET = (-0.070, -CHASSIS_HALF_Y)

GRIPPER_HALF_X = 0.065
GRIPPER_HALF_Y = 0.1275

FIELD_HALF_X = 2.0
FIELD_HALF_Y = 1.0
OBSTACLE_RADIUS = 0.051
OBSTACLE_CLEARANCE = 0.0
OBSTACLES = (
    (-1.0, 0.0),
    (1.0, 0.0),
)


TARGET_RECTS: dict[int, TargetRect] = {
    1: TargetRect(1, (-1.822, 0.500), 0.105, 0.150),
    2: TargetRect(2, (-1.822, -0.500), 0.105, 0.150),
    3: TargetRect(3, (-1.565, 0.000), 0.105, 0.150),
    4: TargetRect(4, (1.605, 0.840), 0.150, 0.105),
    5: TargetRect(5, (1.840, 0.400), 0.105, 0.150),
    6: TargetRect(6, (1.840, 0.000), 0.105, 0.150),
    7: TargetRect(7, (1.840, -0.400), 0.105, 0.150),
    8: TargetRect(8, (1.605, -0.840), 0.150, 0.105),
}

CARDINAL_YAWS = (
    -math.pi,
    -math.pi / 2.0,
    0.0,
    math.pi / 2.0,
    math.pi,
)

GRIPPER_YAWS = tuple(math.radians(deg) for deg in range(-180, 181, 30))
GRIPPER_Q_SAMPLES = tuple(math.radians(deg) for deg in range(-180, 181, 5))
GRIPPER_CONTACT_OFFSETS = (
    (0.0, 0.0),
    (-GRIPPER_HALF_X, 0.0),
    (GRIPPER_HALF_X, 0.0),
    (0.0, -GRIPPER_HALF_Y),
    (0.0, GRIPPER_HALF_Y),
)


def rotate(v: Point, yaw: float) -> Point:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * v[0] - s * v[1], s * v[0] + c * v[1]


def _circle_intersections(c1: Point, r1: float, c2: Point, r2: float) -> tuple[Point, Point]:
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    d = math.hypot(dx, dy)
    if d <= 0.0:
        raise ValueError("circle centers are coincident")
    if d > r1 + r2 or d < abs(r1 - r2):
        raise ValueError("circles do not intersect")

    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h_sq = r1 * r1 - a * a
    if h_sq < -1e-9:
        raise ValueError("circle intersection has negative height")
    h = math.sqrt(max(h_sq, 0.0))

    px = c1[0] + a * dx / d
    py = c1[1] + a * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    return (px + rx, py + ry), (px - rx, py - ry)


def _q_from_elbow(motor: Point, elbow: Point) -> float | None:
    q = math.atan2(motor[0] - elbow[0], elbow[1] - motor[1])
    if -math.pi - 1e-9 <= q <= math.pi + 1e-9:
        return min(max(q, -math.pi), math.pi)
    return None


def solve_arm_to_local_endpoint(endpoint: Point) -> LinkSolution:
    """Solve the planar five-bar pose for a local endpoint.

    The endpoint is expressed in robot-local x/y. The returned solution uses
    q=0 as +y and q increasing counterclockwise toward -x, matching yaw.
    """

    upper_motor = (MOTOR_X, MOTOR_Y_OFFSET)
    lower_motor = (MOTOR_X, -MOTOR_Y_OFFSET)

    upper_options = _circle_intersections(upper_motor, ACTIVE_LINK, endpoint, PASSIVE_LINK)
    lower_options = _circle_intersections(lower_motor, ACTIVE_LINK, endpoint, PASSIVE_LINK)

    best: LinkSolution | None = None
    best_score = math.inf
    for upper_elbow, lower_elbow in product(upper_options, lower_options):
        q1 = _q_from_elbow(upper_motor, upper_elbow)
        q2 = _q_from_elbow(lower_motor, lower_elbow)
        if q1 is None or q2 is None:
            continue
        score = abs(q1 - math.radians(30.0)) + abs(q2 - math.radians(150.0))
        if score < best_score:
            best_score = score
            best = LinkSolution(
                q1=q1,
                q2=q2,
                upper_elbow=upper_elbow,
                lower_elbow=lower_elbow,
                endpoint=endpoint,
            )

    if best is None:
        raise ValueError(f"endpoint is outside arm workspace: ({endpoint[0]:.3f}, {endpoint[1]:.3f})")
    return best


def solve_arm_forward_options(q1: float, q2: float) -> tuple[LinkSolution, ...]:
    """Compute all five-bar endpoint branches from q1/q2."""

    upper_motor = (MOTOR_X, MOTOR_Y_OFFSET)
    lower_motor = (MOTOR_X, -MOTOR_Y_OFFSET)
    upper_elbow = (
        upper_motor[0] - ACTIVE_LINK * math.sin(q1),
        upper_motor[1] + ACTIVE_LINK * math.cos(q1),
    )
    lower_elbow = (
        lower_motor[0] - ACTIVE_LINK * math.sin(q2),
        lower_motor[1] + ACTIVE_LINK * math.cos(q2),
    )
    try:
        endpoint_options = _circle_intersections(upper_elbow, PASSIVE_LINK, lower_elbow, PASSIVE_LINK)
    except ValueError:
        return ()
    return tuple(
        LinkSolution(
            q1=q1,
            q2=q2,
            upper_elbow=upper_elbow,
            lower_elbow=lower_elbow,
            endpoint=endpoint,
        )
        for endpoint in endpoint_options
    )


def solve_arm_forward(q1: float, q2: float) -> LinkSolution | None:
    """Compatibility helper: return the left branch for quick inspection."""

    options = solve_arm_forward_options(q1, q2)
    if not options:
        return None
    return min(options, key=lambda solution: solution.endpoint[0])


def _rect_extent_along(rect: TargetRect, direction: Point) -> float:
    return abs(direction[0]) * rect.half_x + abs(direction[1]) * rect.half_y


def _base_inside_field(x: float, y: float) -> bool:
    return -FIELD_HALF_X <= x <= FIELD_HALF_X and -FIELD_HALF_Y <= y <= FIELD_HALF_Y


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _rect_corners(center: Point, half_x: float, half_y: float, yaw: float = 0.0) -> list[Point]:
    local = (
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    )
    return [(center[0] + rotated[0], center[1] + rotated[1]) for rotated in (rotate(corner, yaw) for corner in local)]


def _polygon_axes(corners: list[Point]) -> list[Point]:
    axes: list[Point] = []
    for index, current in enumerate(corners):
        nxt = corners[(index + 1) % len(corners)]
        edge = (nxt[0] - current[0], nxt[1] - current[1])
        length = math.hypot(edge[0], edge[1])
        if length <= 0.0:
            raise ValueError("degenerate polygon edge")
        axes.append((-edge[1] / length, edge[0] / length))
    return axes


def _project_polygon(corners: list[Point], axis: Point) -> tuple[float, float]:
    values = [_dot(corner, axis) for corner in corners]
    return min(values), max(values)


def _polygons_intersect(a: list[Point], b: list[Point]) -> bool:
    for axis in _polygon_axes(a) + _polygon_axes(b):
        min_a, max_a = _project_polygon(a, axis)
        min_b, max_b = _project_polygon(b, axis)
        if max_a <= min_b or max_b <= min_a:
            return False
    return True


def _point_in_oriented_rect(point: Point, center: Point, half_x: float, half_y: float, yaw: float) -> bool:
    local = rotate((point[0] - center[0], point[1] - center[1]), -yaw)
    return abs(local[0]) <= half_x and abs(local[1]) <= half_y


def _segment_distance_to_point(a: Point, b: Point, p: Point) -> float:
    ab = (b[0] - a[0], b[1] - a[1])
    ap = (p[0] - a[0], p[1] - a[1])
    ab_len_sq = _dot(ab, ab)
    if ab_len_sq <= 0.0:
        return math.hypot(ap[0], ap[1])
    t = max(0.0, min(1.0, _dot(ap, ab) / ab_len_sq))
    closest = (a[0] + ab[0] * t, a[1] + ab[1] * t)
    return math.hypot(p[0] - closest[0], p[1] - closest[1])


def _oriented_rect_intersects_circle(
    center: Point,
    half_x: float,
    half_y: float,
    yaw: float,
    circle_center: Point,
    radius: float,
) -> bool:
    if _point_in_oriented_rect(circle_center, center, half_x, half_y, yaw):
        return True
    corners = _rect_corners(center, half_x, half_y, yaw)
    for index, current in enumerate(corners):
        nxt = corners[(index + 1) % len(corners)]
        if _segment_distance_to_point(current, nxt, circle_center) <= radius:
            return True
    return False


def _chassis_corners_inside_field(corners: list[Point]) -> bool:
    return all(
        -FIELD_HALF_X <= corner[0] <= FIELD_HALF_X and -FIELD_HALF_Y <= corner[1] <= FIELD_HALF_Y
        for corner in corners
    )


def _chassis_intersects_targets(chassis_corners: list[Point]) -> bool:
    for rect in TARGET_RECTS.values():
        target_corners = _rect_corners(rect.center, rect.half_x, rect.half_y)
        if _polygons_intersect(chassis_corners, target_corners):
            return True
    return False


def _chassis_intersects_obstacles(center: Point, yaw: float) -> bool:
    radius = OBSTACLE_RADIUS + OBSTACLE_CLEARANCE
    return any(
        _oriented_rect_intersects_circle(center, CHASSIS_HALF_X, CHASSIS_HALF_Y, yaw, obstacle, radius)
        for obstacle in OBSTACLES
    )


def _chassis_pose_valid(x: float, y: float, yaw: float) -> bool:
    center = (x, y)
    corners = _rect_corners(center, CHASSIS_HALF_X, CHASSIS_HALF_Y, yaw)
    if not _chassis_corners_inside_field(corners):
        return False
    if _chassis_intersects_targets(corners):
        return False
    if _chassis_intersects_obstacles(center, yaw):
        return False
    return True


def _dedupe_key(pose: PoseCandidate) -> tuple[str, int, int, int]:
    return (
        pose.carrier,
        round(pose.x * 1000),
        round(pose.y * 1000),
        round(math.degrees(pose.yaw)),
    )


def _gripper_can_cover_target(endpoint_world: Point, yaw: float, rect: TargetRect) -> bool:
    gripper_corners = _rect_corners(endpoint_world, GRIPPER_HALF_X, GRIPPER_HALF_Y, yaw)
    target_corners = _rect_corners(rect.center, rect.half_x, rect.half_y)
    return any(_point_in_oriented_rect(corner, endpoint_world, GRIPPER_HALF_X, GRIPPER_HALF_Y, yaw) for corner in target_corners) or any(
        _point_in_oriented_rect(corner, rect.center, rect.half_x, rect.half_y, 0.0) for corner in gripper_corners
    )


def _gripper_candidates(rect: TargetRect, yaw: float) -> list[PoseCandidate]:
    candidates: list[PoseCandidate] = []
    seen: set[tuple[str, int, int, int]] = set()
    for q1 in GRIPPER_Q_SAMPLES:
        for q2 in GRIPPER_Q_SAMPLES:
            for link_solution in solve_arm_forward_options(q1, q2):
                # The opened gripper is a rectangle centered at the five-bar endpoint.
                # Sample target points inside this rectangle and infer the base pose.
                for contact_offset in GRIPPER_CONTACT_OFFSETS:
                    target_contact = (rect.center[0] + contact_offset[0], rect.center[1] + contact_offset[1])
                    endpoint_offset = rotate(link_solution.endpoint, yaw)
                    base = (target_contact[0] - endpoint_offset[0], target_contact[1] - endpoint_offset[1])
                    endpoint_world = (base[0] + endpoint_offset[0], base[1] + endpoint_offset[1])
                    if not _gripper_can_cover_target(endpoint_world, yaw, rect):
                        continue
                    if not _base_inside_field(base[0], base[1]):
                        continue
                    if not _chassis_pose_valid(base[0], base[1], yaw):
                        continue

                    candidate = PoseCandidate(
                        point_id=rect.point_id,
                        carrier="gripper",
                        x=base[0],
                        y=base[1],
                        yaw=yaw,
                        q1=link_solution.q1,
                        q2=link_solution.q2,
                        link_solution=link_solution,
                        gripper_rect_center=link_solution.endpoint,
                        note="opened gripper rectangle covers target",
                    )
                    key = _dedupe_key(candidate)
                    if key in seen:
                        continue
                    seen.add(key)
                    candidates.append(candidate)
    return candidates


def _funnel_candidate(rect: TargetRect, carrier: Carrier, yaw: float) -> PoseCandidate | None:
    if carrier == "upper_funnel":
        outlet_local = UPPER_FUNNEL_OUTLET
        side_normal = rotate((0.0, 1.0), yaw)
    elif carrier == "lower_funnel":
        outlet_local = LOWER_FUNNEL_OUTLET
        side_normal = rotate((0.0, -1.0), yaw)
    else:
        raise ValueError(f"invalid funnel carrier: {carrier}")

    # The target is on the funnel side of the chassis. Put the chassis side on
    # the near edge of the target rectangle, so contact is tangent, not overlap.
    target_extent = _rect_extent_along(rect, side_normal)
    target_near_edge = (rect.center[0] - side_normal[0] * target_extent, rect.center[1] - side_normal[1] * target_extent)
    base = (target_near_edge[0] - side_normal[0] * CHASSIS_HALF_Y, target_near_edge[1] - side_normal[1] * CHASSIS_HALF_Y)
    if not _base_inside_field(base[0], base[1]):
        return None
    if not _chassis_pose_valid(base[0], base[1], yaw):
        return None
    return PoseCandidate(
        point_id=rect.point_id,
        carrier=carrier,
        x=base[0],
        y=base[1],
        yaw=yaw,
        q1=None,
        q2=None,
        link_solution=None,
        gripper_rect_center=None,
        note="funnel outlet touches target near edge",
    )


@lru_cache(maxsize=None)
def candidate_poses(point_id: int) -> tuple[PoseCandidate, ...]:
    """Return geometry candidates for target point 1..8.

    The candidates are computed from current MuJoCo dimensions. They are
    geometric hypotheses for planning/debugging, not validated trajectories.
    """

    if point_id not in TARGET_RECTS:
        raise ValueError(f"unknown point id: {point_id}")

    rect = TARGET_RECTS[point_id]
    candidates: list[PoseCandidate] = []

    for yaw in GRIPPER_YAWS:
        candidates.extend(_gripper_candidates(rect, yaw))

    if point_id >= 4:
        for carrier in ("upper_funnel", "lower_funnel"):
            for yaw in CARDINAL_YAWS:
                candidate = _funnel_candidate(rect, carrier, yaw)
                if candidate is not None:
                    candidates.append(candidate)

    return tuple(candidates)


def _plot_polygon(ax, corners: list[Point], **kwargs) -> None:
    xs = [p[0] for p in corners] + [corners[0][0]]
    ys = [p[1] for p in corners] + [corners[0][1]]
    ax.plot(xs, ys, **kwargs)


def _draw_arm(ax, pose: PoseCandidate) -> None:
    if pose.link_solution is None:
        return

    base = (pose.x, pose.y)
    upper_motor_offset = rotate((MOTOR_X, MOTOR_Y_OFFSET), pose.yaw)
    lower_motor_offset = rotate((MOTOR_X, -MOTOR_Y_OFFSET), pose.yaw)
    upper_elbow_offset = rotate(pose.link_solution.upper_elbow, pose.yaw)
    lower_elbow_offset = rotate(pose.link_solution.lower_elbow, pose.yaw)
    endpoint_offset = rotate(pose.link_solution.endpoint, pose.yaw)
    upper_motor = (base[0] + upper_motor_offset[0], base[1] + upper_motor_offset[1])
    lower_motor = (base[0] + lower_motor_offset[0], base[1] + lower_motor_offset[1])
    upper_elbow = (base[0] + upper_elbow_offset[0], base[1] + upper_elbow_offset[1])
    lower_elbow = (base[0] + lower_elbow_offset[0], base[1] + lower_elbow_offset[1])
    endpoint = (base[0] + endpoint_offset[0], base[1] + endpoint_offset[1])

    ax.plot(
        [upper_motor[0], upper_elbow[0], endpoint[0], lower_elbow[0], lower_motor[0]],
        [upper_motor[1], upper_elbow[1], endpoint[1], lower_elbow[1], lower_motor[1]],
        marker="o",
        color="tab:red",
        linewidth=2.0,
    )
    gripper_corners = _rect_corners(endpoint, GRIPPER_HALF_X, GRIPPER_HALF_Y, pose.yaw)
    _plot_polygon(ax, gripper_corners, color="tab:purple", linewidth=2.0)


def debug_plot_point(point_id: int) -> None:
    """Plot candidates for one point with matplotlib.

    Usage:
        python -c "from pc_server.planning_geometry import debug_plot_point; debug_plot_point(4)"
    """

    import matplotlib.pyplot as plt

    rect = TARGET_RECTS.get(point_id)
    if rect is None:
        raise ValueError(f"unknown point id: {point_id}")

    poses = candidate_poses(point_id)
    if not poses:
        raise ValueError(f"no candidate poses for point {point_id}")

    fig, ax = plt.subplots(figsize=(8, 6))
    _plot_polygon(
        ax,
        _rect_corners(rect.center, rect.half_x, rect.half_y),
        color="black",
        linewidth=2.5,
        label=f"target {point_id}",
    )

    colors = {
        "gripper": "tab:blue",
        "upper_funnel": "tab:green",
        "lower_funnel": "tab:orange",
    }

    for pose in poses:
        chassis = _rect_corners((pose.x, pose.y), CHASSIS_HALF_X, CHASSIS_HALF_Y, pose.yaw)
        _plot_polygon(ax, chassis, color=colors[pose.carrier], alpha=0.35, linewidth=1.0)
        ax.scatter([pose.x], [pose.y], color=colors[pose.carrier], s=12)

    gripper_pose = next((pose for pose in poses if pose.carrier == "gripper"), None)
    if gripper_pose is None:
        raise ValueError(f"no gripper pose for point {point_id}")
    _draw_arm(ax, gripper_pose)

    ax.scatter([rect.center[0]], [rect.center[1]], color="black", s=35, zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_title(f"Point {point_id}: chassis candidates and arm endpoint")
    ax.set_xlabel("world x / m")
    ax.set_ylabel("world y / m")
    ax.legend(handles=[], labels=[])
    plt.show()


def print_candidate_counts(point_id: int) -> None:
    counts: dict[str, int] = {}
    for pose in candidate_poses(point_id):
        counts[pose.carrier] = counts.get(pose.carrier, 0) + 1
    count_text = ", ".join(f"{carrier}={count}" for carrier, count in sorted(counts.items()))
    print(f"point {point_id}: {count_text}")


def print_candidates(point_id: int, *, limit: int = 40) -> None:
    poses = candidate_poses(point_id)
    for index, pose in enumerate(poses[:limit]):
        q_text = ""
        if pose.q1 is not None and pose.q2 is not None:
            q_text = f", q1={math.degrees(pose.q1):.1f}, q2={math.degrees(pose.q2):.1f}"
        print(
            f"{index:02d} {pose.carrier:12s} "
            f"x={pose.x:+.3f}, y={pose.y:+.3f}, yaw={math.degrees(pose.yaw):+6.1f} deg"
            f"{q_text}  # {pose.note}"
        )
    if len(poses) > limit:
        print(f"... {len(poses) - limit} more candidates omitted")


if __name__ == "__main__":
    for point in range(1, 9):
        debug_plot_point(point)
