from collections import deque
import math
import statistics


class StablePoseWindow:
    def __init__(self, duration=1.0, max_xy_span=0.08, max_z_span=0.04):
        self.duration = duration
        self.max_xy_span = max_xy_span
        self.max_z_span = max_z_span
        self.samples = deque()

    def clear(self):
        self.samples.clear()

    def add(self, stamp, pose):
        self.samples.append((stamp, pose))
        while self.samples and stamp - self.samples[0][0] > self.duration + 0.5:
            self.samples.popleft()

    def stable(self):
        if len(self.samples) < 2 or self.samples[-1][0] - self.samples[0][0] < self.duration:
            return False
        xs = [sample[1][0] for sample in self.samples]
        ys = [sample[1][1] for sample in self.samples]
        zs = [sample[1][2] for sample in self.samples]
        return (math.hypot(max(xs) - min(xs), max(ys) - min(ys)) <= self.max_xy_span and
                max(zs) - min(zs) <= self.max_z_span)

    def floor_z(self, body_height):
        if not self.stable():
            raise ValueError('pose window is not stable')
        return statistics.median(sample[1][2] for sample in self.samples) - body_height


def relative_z_band(floor_z_ref, min_above, max_above):
    if min_above < 0.0 or max_above <= min_above:
        raise ValueError('invalid active-floor z band')
    return floor_z_ref + min_above, floor_z_ref + max_above


class FloorHandoffGate:
    """Accept each completed stair episode exactly once."""
    def __init__(self):
        self.current_episode = 0
        self.pending = None
        self.processed = set()

    def observe(self, episode_id):
        if episode_id <= self.current_episode:
            return False
        self.current_episode = episode_id
        if self.pending is not None and self.pending < episode_id:
            self.pending = None
        return True

    def request(self, episode_id):
        if (episode_id <= 0 or episode_id != self.current_episode or
                episode_id == self.pending or episode_id in self.processed):
            return False
        self.pending = episode_id
        return True

    def commit(self):
        if self.pending is None:
            return False
        self.processed.add(self.pending)
        self.pending = None
        return True
