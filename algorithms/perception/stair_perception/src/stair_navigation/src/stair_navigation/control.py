import math


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


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
