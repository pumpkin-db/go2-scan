import math
from collections import deque


class OccupancyGridView:
    """Small, ROS-message-compatible view used for safe staging selection."""

    def __init__(self, msg, occupied_threshold=50):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        q = msg.info.origin.orientation
        self.origin_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.data = msg.data
        self.occupied_threshold = occupied_threshold

    def cell(self, point):
        dx, dy = point[0] - self.origin_x, point[1] - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(local_x / self.resolution)), int(math.floor(local_y / self.resolution))

    def inside(self, cell):
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def value(self, cell):
        return self.data[cell[1] * self.width + cell[0]] if self.inside(cell) else -1

    def free(self, cell):
        value = self.value(cell)
        return 0 <= value < self.occupied_threshold

    def clearance_free(self, point, radius):
        center = self.cell(point)
        cells = int(math.ceil(radius / self.resolution))
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                if math.hypot(dx, dy) * self.resolution <= radius and not self.free(
                        (center[0] + dx, center[1] + dy)):
                    return False
        return True

    def connected(self, start, goal):
        start_cell, goal_cell = self.cell(start), self.cell(goal)
        if not self.free(goal_cell):
            return False
        if not self.free(start_cell):
            start_cell = self._nearest_free(start_cell, 0.5)
        if start_cell is None:
            return False
        queue, seen = deque([start_cell]), {start_cell}
        while queue:
            cell = queue.popleft()
            if cell == goal_cell:
                return True
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (cell[0] + dx, cell[1] + dy)
                if nxt not in seen and self.free(nxt):
                    seen.add(nxt)
                    queue.append(nxt)
        return False

    def _nearest_free(self, center, max_distance):
        cells = int(math.ceil(max_distance / self.resolution))
        candidates = []
        for dy in range(-cells, cells + 1):
            for dx in range(-cells, cells + 1):
                cell = (center[0] + dx, center[1] + dy)
                if self.free(cell):
                    candidates.append((dx * dx + dy * dy, cell))
        return min(candidates)[1] if candidates else None


def select_safe_staging(grid, robot, entry, heading, nominal_distance=1.0,
                        clearance=0.25, distances=None, lateral_offsets=None):
    """Return one current-map-valid executable staging point, or None."""
    norm = math.hypot(*heading)
    if norm < 1e-6:
        return None
    hx, hy = heading[0] / norm, heading[1] / norm
    lx, ly = -hy, hx
    distances = distances or (nominal_distance, 0.8, 1.2, 0.6, 1.6, 2.0)
    lateral_offsets = lateral_offsets or (0.0, 0.2, -0.2, 0.4, -0.4)
    seen = set()
    for distance in distances:
        for lateral in lateral_offsets:
            candidate = (entry[0] - distance * hx + lateral * lx,
                         entry[1] - distance * hy + lateral * ly)
            key = (round(candidate[0], 4), round(candidate[1], 4))
            if key in seen:
                continue
            seen.add(key)
            if (grid.clearance_free(candidate, clearance) and
                    grid.connected(robot, candidate)):
                return candidate
    return None


def staging_arrived(robot, accepted, tolerance):
    """Arrival contract consumes only authoritative accepted staging."""
    return math.hypot(robot[0] - accepted[0], robot[1] - accepted[1]) <= tolerance


def accept_planner_endpoint(current, endpoint, max_adjustment=1.0):
    """Validate SCAN's reported final endpoint before it becomes authoritative."""
    if math.hypot(endpoint[0] - current[0], endpoint[1] - current[1]) > max_adjustment:
        return None
    return endpoint
