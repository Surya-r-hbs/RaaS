"""Conservative X/Y grid planning for the Webots warehouse."""

import heapq
import math
import time


GRID_RESOLUTION_METERS = 0.2
SAFETY_CLEARANCE_METERS = 0.1
MAX_EXPANSIONS = 100_000
PROGRESS_TIMEOUT_SECONDS = 10.0
MIN_PROGRESS_METERS = 0.05


class ProgressMonitor:
    def __init__(
        self,
        timeout_seconds=PROGRESS_TIMEOUT_SECONDS,
        minimum_progress=MIN_PROGRESS_METERS,
    ):
        self.timeout_seconds = timeout_seconds
        self.minimum_progress = minimum_progress
        self.best_distance = None
        self.last_progress_at = time.monotonic()

    def reset(self):
        self.best_distance = None
        self.last_progress_at = time.monotonic()

    def stalled(self, distance, now=None):
        now = time.monotonic() if now is None else now
        if (
            self.best_distance is None
            or self.best_distance - distance >= self.minimum_progress
        ):
            self.best_distance = distance
            self.last_progress_at = now
            return False
        return now - self.last_progress_at > self.timeout_seconds


def _xy(point, label):
    try:
        x = float(point["x"])
        y = float(point["y"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain numeric x and y coordinates") from error
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"{label} coordinates must be finite")
    return x, y


def _bounds_xy(bounds):
    try:
        minimum = bounds["min"]
        maximum = bounds["max"]
        return (
            float(minimum["x"]),
            float(minimum["y"]),
            float(maximum["x"]),
            float(maximum["y"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("Obstacle bounds must include X/Y min and max") from error


def _point_blocked(x, y, obstacles, inflation, floor_bounds):
    floor_min_x, floor_min_y, floor_max_x, floor_max_y = floor_bounds
    if (
        x < floor_min_x + inflation
        or x > floor_max_x - inflation
        or y < floor_min_y + inflation
        or y > floor_max_y - inflation
    ):
        return True

    for obstacle in obstacles:
        min_x, min_y, max_x, max_y = _bounds_xy(obstacle["bounds"])
        if (
            min_x - inflation <= x <= max_x + inflation
            and min_y - inflation <= y <= max_y + inflation
        ):
            return True
    return False


def _line_is_clear(start, end, obstacles, inflation, floor_bounds):
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, math.ceil(distance / (GRID_RESOLUTION_METERS / 2)))
    for index in range(steps + 1):
        ratio = index / steps
        x = start[0] + (end[0] - start[0]) * ratio
        y = start[1] + (end[1] - start[1]) * ratio
        if _point_blocked(x, y, obstacles, inflation, floor_bounds):
            return False
    return True


def _smooth_path(points, obstacles, inflation, floor_bounds):
    if len(points) < 3:
        return points
    smoothed = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        candidate = len(points) - 1
        while candidate > anchor + 1 and not _line_is_clear(
            points[anchor],
            points[candidate],
            obstacles,
            inflation,
            floor_bounds,
        ):
            candidate -= 1
        smoothed.append(points[candidate])
        anchor = candidate
    return smoothed


def _a_star(start, goal, obstacles, floor_bounds, inflation):
    if _point_blocked(*start, obstacles, inflation, floor_bounds):
        raise ValueError("Robot pose is inside an obstacle or outside the safe floor area")
    if _point_blocked(*goal, obstacles, inflation, floor_bounds):
        raise ValueError("Goal is inside an obstacle or outside the safe floor area")

    min_x, min_y, max_x, max_y = floor_bounds
    width = max(1, math.floor((max_x - min_x) / GRID_RESOLUTION_METERS) + 1)
    height = max(1, math.floor((max_y - min_y) / GRID_RESOLUTION_METERS) + 1)

    def cell(point):
        return (
            round((point[0] - min_x) / GRID_RESOLUTION_METERS),
            round((point[1] - min_y) / GRID_RESOLUTION_METERS),
        )

    def location(index):
        return (
            min_x + index[0] * GRID_RESOLUTION_METERS,
            min_y + index[1] * GRID_RESOLUTION_METERS,
        )

    start_cell = cell(start)
    goal_cell = cell(goal)

    def blocked(index):
        if not 0 <= index[0] < width or not 0 <= index[1] < height:
            return True
        return _point_blocked(
            *location(index),
            obstacles,
            inflation,
            floor_bounds,
        )

    if blocked(start_cell) or blocked(goal_cell):
        raise ValueError("No collision-free grid cell is available near the pose or goal")

    def heuristic(index):
        dx = abs(index[0] - goal_cell[0])
        dy = abs(index[1] - goal_cell[1])
        return GRID_RESOLUTION_METERS * (
            max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        )

    open_heap = [(heuristic(start_cell), 0.0, start_cell)]
    came_from = {}
    cost_so_far = {start_cell: 0.0}
    visited = 0

    while open_heap and visited < MAX_EXPANSIONS:
        _, cost, current = heapq.heappop(open_heap)
        if cost != cost_so_far.get(current):
            continue
        visited += 1
        if current == goal_cell:
            break

        for dx, dy in (
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ):
            neighbor = (current[0] + dx, current[1] + dy)
            if blocked(neighbor):
                continue
            if dx and dy and (
                blocked((current[0] + dx, current[1]))
                or blocked((current[0], current[1] + dy))
            ):
                continue
            step_cost = GRID_RESOLUTION_METERS * (
                math.sqrt(2) if dx and dy else 1.0
            )
            new_cost = cost + step_cost
            if new_cost >= cost_so_far.get(neighbor, math.inf):
                continue
            came_from[neighbor] = current
            cost_so_far[neighbor] = new_cost
            heapq.heappush(
                open_heap,
                (new_cost + heuristic(neighbor), new_cost, neighbor),
            )

    if goal_cell not in cost_so_far:
        raise ValueError("No collision-free route exists to the requested goal")

    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    grid_path = [location(index) for index in cells]
    grid_path[0] = start
    grid_path[-1] = goal
    return _smooth_path(grid_path, obstacles, inflation, floor_bounds)


def plan_route(
    start_position,
    goal_position,
    obstacles,
    floor_bounds,
    footprint_radius,
    clearance=SAFETY_CLEARANCE_METERS,
):
    """Plan a footprint-inflated route, including reachable station approaches."""
    start = _xy(start_position, "Robot pose")
    goal = _xy(goal_position, "Goal")
    try:
        floor = _bounds_xy(floor_bounds)
        radius = float(footprint_radius)
        clearance = float(clearance)
    except (TypeError, ValueError) as error:
        raise ValueError("Floor bounds, footprint, or clearance is invalid") from error
    if not all(math.isfinite(value) for value in (*floor, radius, clearance)):
        raise ValueError("Floor bounds, footprint, or clearance must be finite")
    if radius < 0 or clearance < 0:
        raise ValueError("Robot footprint and safety clearance cannot be negative")

    inflation = radius + clearance
    floor_min_x, floor_min_y, floor_max_x, floor_max_y = floor
    if (
        goal[0] < floor_min_x + inflation
        or goal[0] > floor_max_x - inflation
        or goal[1] < floor_min_y + inflation
        or goal[1] > floor_max_y - inflation
    ):
        raise ValueError("Goal is outside the safe floor area")
    blockers = []
    for obstacle in obstacles:
        min_x, min_y, max_x, max_y = _bounds_xy(obstacle["bounds"])
        if (
            min_x - inflation <= goal[0] <= max_x + inflation
            and min_y - inflation <= goal[1] <= max_y + inflation
        ):
            blockers.append(obstacle)
    if not blockers:
        return {
            "path": _a_star(start, goal, obstacles, floor, inflation),
            "approach": False,
            "goal": {"x": goal[0], "y": goal[1]},
        }

    if len(blockers) != 1 or not blockers[0].get("approachAllowed"):
        names = ", ".join(
            str(obstacle.get("name") or obstacle.get("id") or "unknown obstacle")
            for obstacle in blockers
        )
        raise ValueError(f"Goal is not safely reachable; it conflicts with {names}")

    obstacle = blockers[0]
    min_x, min_y, max_x, max_y = _bounds_xy(obstacle["bounds"])
    offset = inflation + GRID_RESOLUTION_METERS
    candidates = (
        (min_x - offset, goal[1]),
        (max_x + offset, goal[1]),
        (goal[0], min_y - offset),
        (goal[0], max_y + offset),
        (min_x - offset, min_y - offset),
        (min_x - offset, max_y + offset),
        (max_x + offset, min_y - offset),
        (max_x + offset, max_y + offset),
    )
    routes = []
    for candidate in candidates:
        try:
            path = _a_star(start, candidate, obstacles, floor, inflation)
        except ValueError:
            continue
        length = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(path, path[1:])
        )
        routes.append((length, path, candidate))

    if not routes:
        raise ValueError(
            f"No safe approach pose is reachable for station "
            f"{obstacle.get('name') or obstacle.get('id')}"
        )
    _, path, approach = min(routes, key=lambda route: route[0])
    return {
        "path": path,
        "approach": True,
        "goal": {"x": approach[0], "y": approach[1]},
        "station": obstacle.get("name") or obstacle.get("id"),
    }
