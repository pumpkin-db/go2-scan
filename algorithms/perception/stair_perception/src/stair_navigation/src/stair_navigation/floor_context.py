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
