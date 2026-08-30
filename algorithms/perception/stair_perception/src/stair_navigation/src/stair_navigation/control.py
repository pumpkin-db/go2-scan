import math
from collections import deque


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_staging(entry, heading, distance):
    norm = math.hypot(*heading)
    if norm < 1e-6:
        raise ValueError('degenerate stair heading')
    return (entry[0] - distance * heading[0] / norm,
            entry[1] - distance * heading[1] / norm)


def same_track_geometry(reference_entry, reference_heading, candidate_entry, candidate_heading,
                        max_entry_distance=0.8, max_heading_deg=25.0):
    if math.hypot(candidate_entry[0] - reference_entry[0],
                  candidate_entry[1] - reference_entry[1]) > max_entry_distance:
        return False
    rn, cn = math.hypot(*reference_heading), math.hypot(*candidate_heading)
    if rn < 1e-6 or cn < 1e-6:
        return False
    cosine = ((reference_heading[0] * candidate_heading[0] +
               reference_heading[1] * candidate_heading[1]) / (rn * cn))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine)))) <= max_heading_deg


def mission_extent_expands(current_entry, current_exit, current_rise,
                           candidate_entry, candidate_exit, candidate_rise,
                           tolerance=0.05):
    """Accept tracker-validated canonical growth; never shrink a selected mission."""
    current_heading, current_length = CorridorFollower.geometry(current_entry, current_exit)
    candidate_heading, candidate_length = CorridorFollower.geometry(candidate_entry, candidate_exit)
    if not same_track_geometry(current_entry, current_heading,
                               candidate_entry, candidate_heading):
        return False
    if candidate_length < current_length - tolerance or candidate_rise < current_rise - tolerance:
        return False
    return (candidate_length > current_length + tolerance or
            candidate_rise > current_rise + tolerance)


def stair_state_owns_control(state_name):
    # COMPLETE/FAILED remain fail-closed until a later lifecycle owner explicitly releases them.
    return state_name != 'IDLE'


def landing_reacquire_score(previous_heading, candidate_heading, candidate_entry, robot_pose,
                            candidate_rise, last_seen, landing_since, max_range=4.0,
                            max_vertical=0.9, min_turn_deg=120.0,
                            pre_landing_freshness=2.0, observation_count=3,
                            min_observations=2, confidence=1.0, min_confidence=0.45):
    """Return planar approach distance for a valid switchback flight, else None."""
    if (last_seen < landing_since - pre_landing_freshness or candidate_rise <= 0.0 or
            observation_count < min_observations or confidence < min_confidence):
        return None
    previous_norm = math.hypot(*previous_heading)
    candidate_norm = math.hypot(*candidate_heading)
    if previous_norm < 1e-6 or candidate_norm < 1e-6:
        return None
    cosine = ((previous_heading[0] * candidate_heading[0] +
               previous_heading[1] * candidate_heading[1]) /
              (previous_norm * candidate_norm))
    turn = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    if turn < min_turn_deg:
        return None
    planar = math.hypot(candidate_entry[0] - robot_pose[0],
                        candidate_entry[1] - robot_pose[1])
    vertical = abs(candidate_entry[2] - robot_pose[2])
    return planar if planar <= max_range and vertical <= max_vertical else None


class ExitVerifier:
    def __init__(self, episode_start_z, expected_rise, final_entry, final_exit,
                 min_height_ratio=0.75, progress_tolerance=0.30,
                 stability_window=1.0, max_z_span=0.08, max_xy_span=0.08,
                 settle_time=2.0):
        self.episode_start_z = episode_start_z
        self.expected_rise = expected_rise
        self.final_entry = final_entry
        self.heading, self.length = CorridorFollower.geometry(final_entry, final_exit)
        self.min_height_ratio = min_height_ratio
        self.progress_tolerance = progress_tolerance
        self.stability_window = stability_window
        self.max_z_span = max_z_span
        self.max_xy_span = max_xy_span
        self.settle_time = settle_time
        self.samples = deque()

    def update(self, stamp, position):
        self.samples.append((stamp, position))
        cutoff = stamp - self.stability_window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def ready(self, now, started_at):
        if now - started_at < self.settle_time or len(self.samples) < 2:
            return False
        if self.samples[-1][0] - self.samples[0][0] < 0.8 * self.stability_window:
            return False
        latest = self.samples[-1][1]
        height_gain = latest[2] - self.episode_start_z
        if height_gain < self.min_height_ratio * self.expected_rise:
            return False
        progress = CorridorFollower().progress(latest[:2], self.final_entry, self.heading)
        if progress < self.length - self.progress_tolerance:
            return False
        xs = [sample[1][0] for sample in self.samples]
        ys = [sample[1][1] for sample in self.samples]
        zs = [sample[1][2] for sample in self.samples]
        return (math.hypot(max(xs) - min(xs), max(ys) - min(ys)) <= self.max_xy_span and
                max(zs) - min(zs) <= self.max_z_span)


class MotionArbiterCore:
    def __init__(self, timeout=0.3):
        self.timeout = timeout
        self.stair_active = False
        self.nav = (None, 0.0)
        self.stair = (None, 0.0)

    def update(self, source, command, now):
        if source == 'nav':
            self.nav = (command, now)
        elif source == 'stair':
            self.stair = (command, now)

    def select(self, now):
        command, stamp = self.stair if self.stair_active else self.nav
        return command if command is not None and now - stamp <= self.timeout else None


class CorridorFollower:
    def __init__(self, forward_speed=0.13, lateral_gain=0.8, yaw_gain=1.5,
                 max_lateral=0.12, max_yaw=0.6):
        self.forward_speed = forward_speed
        self.lateral_gain = lateral_gain
        self.yaw_gain = yaw_gain
        self.max_lateral = max_lateral
        self.max_yaw = max_yaw

    @staticmethod
    def geometry(entry, exit_):
        dx, dy = exit_[0] - entry[0], exit_[1] - entry[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            raise ValueError('degenerate stair centerline')
        return (dx / length, dy / length), length

    def progress(self, position, entry, heading):
        return (position[0] - entry[0]) * heading[0] + (position[1] - entry[1]) * heading[1]

    def command(self, position, yaw, entry, heading):
        lateral = (-heading[1], heading[0])
        error = (position[0] - entry[0]) * lateral[0] + (position[1] - entry[1]) * lateral[1]
        world_vx = self.forward_speed * heading[0] - self.lateral_gain * error * lateral[0]
        world_vy = self.forward_speed * heading[1] - self.lateral_gain * error * lateral[1]
        c, s = math.cos(yaw), math.sin(yaw)
        body_x = c * world_vx + s * world_vy
        body_y = -s * world_vx + c * world_vy
        target_yaw = math.atan2(heading[1], heading[0])
        return (max(0.0, body_x), clamp(body_y, self.max_lateral),
                clamp(self.yaw_gain * wrap(target_yaw - yaw), self.max_yaw))


class TerrainProfileCore:
    def __init__(self, entry, exit_, rise, anchor_z, anchor_progress=0.0):
        self.entry = entry
        self.heading, self.length = CorridorFollower.geometry(entry, exit_)
        self.rise = max(0.0, rise)
        self.anchor_z = anchor_z
        self.anchor_progress = max(0.0, min(self.length, anchor_progress))

    def target(self, position):
        progress = CorridorFollower().progress(position, self.entry, self.heading)
        denominator = max(1e-6, self.length - self.anchor_progress)
        ratio = max(0.0, min(1.0, (progress - self.anchor_progress) / denominator))
        return self.anchor_z + ratio * self.rise
