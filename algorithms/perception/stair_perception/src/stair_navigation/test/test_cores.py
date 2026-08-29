#!/usr/bin/env python3
import math
import unittest

from stair_navigation.control import CorridorFollower, MotionArbiterCore, TerrainProfileCore


class CoreTests(unittest.TestCase):
    def test_arbiter_is_fail_closed_and_exclusive(self):
        core = MotionArbiterCore(timeout=0.3)
        core.update('nav', 'nav', 1.0)
        self.assertEqual(core.select(1.1), 'nav')
        core.stair_active = True
        self.assertIsNone(core.select(1.1))
        core.update('stair', 'stair', 1.2)
        self.assertEqual(core.select(1.3), 'stair')
        self.assertIsNone(core.select(1.6))

    def test_corridor_progress_and_correction(self):
        follower = CorridorFollower()
        heading, length = follower.geometry((0.0, 0.0), (0.0, 3.0))
        self.assertAlmostEqual(length, 3.0)
        self.assertAlmostEqual(follower.progress((0.0, 1.0), (0.0, 0.0), heading), 1.0)
        vx, vy, wz = follower.command((0.2, 1.0), math.pi / 2, (0.0, 0.0), heading)
        self.assertGreater(vx, 0.0)
        self.assertGreater(vy, 0.0)
        self.assertAlmostEqual(wz, 0.0)

    def test_terrain_profile_is_monotonic_and_bounded(self):
        profile = TerrainProfileCore((0.0, 0.0), (0.0, 3.0), 2.4, 0.35)
        values = [profile.target((0.0, y)) for y in (-1.0, 0.0, 1.5, 3.0, 4.0)]
        self.assertEqual(values, sorted(values))
        self.assertAlmostEqual(values[0], 0.35)
        self.assertAlmostEqual(values[-1], 2.75)


if __name__ == '__main__':
    unittest.main()
