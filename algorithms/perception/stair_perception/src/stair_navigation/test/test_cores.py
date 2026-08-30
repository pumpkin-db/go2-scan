#!/usr/bin/env python3
import math
import unittest

from stair_navigation.control import (CorridorFollower, MotionArbiterCore, TerrainProfileCore,
                                      ExitVerifier, compute_staging, landing_reacquire_score,
                                      mission_extent_expands, same_track_geometry,
                                      stair_state_owns_control)


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

    def test_landing_reacquire_rejects_stale_same_direction_track(self):
        score = landing_reacquire_score(
            previous_heading=(0.0, -1.0), candidate_heading=(0.0, -1.0),
            candidate_entry=(13.05, -0.15, 3.26), robot_pose=(12.53, 0.38, 2.95),
            candidate_rise=1.87, last_seen=31.8, landing_since=34.3)
        self.assertIsNone(score)

    def test_landing_reacquire_accepts_fresh_switchback_track(self):
        score = landing_reacquire_score(
            previous_heading=(-0.10, -1.0), candidate_heading=(0.05, 1.0),
            candidate_entry=(14.40, 1.82, 3.67), robot_pose=(12.53, 0.38, 2.95),
            candidate_rise=1.03, last_seen=32.604, landing_since=34.0,
            observation_count=2, confidence=0.63)
        self.assertIsNotNone(score)
        self.assertLess(score, 3.0)

    def test_landing_reacquire_rejects_single_frame_candidate(self):
        score = landing_reacquire_score(
            previous_heading=(0.0, -1.0), candidate_heading=(0.0, 1.0),
            candidate_entry=(14.2, 1.5, 3.5), robot_pose=(12.5, 0.3, 3.0),
            candidate_rise=1.2, last_seen=33.5, landing_since=34.0,
            observation_count=1, confidence=0.9)
        self.assertIsNone(score)

    def test_exit_verifier_requires_height_progress_and_stability(self):
        verifier = ExitVerifier(episode_start_z=0.35, expected_rise=4.0,
                                final_entry=(14.2, 1.5), final_exit=(14.3, 2.9))
        for i in range(11):
            verifier.update(10.0 + 0.1 * i, (14.29, 2.75, 3.95 + 0.002 * (i % 2)))
        self.assertTrue(verifier.ready(11.0, 9.0))

    def test_exit_verifier_rejects_unsettled_or_insufficient_height(self):
        verifier = ExitVerifier(episode_start_z=0.35, expected_rise=4.0,
                                final_entry=(14.2, 1.5), final_exit=(14.3, 2.9))
        for i in range(11):
            verifier.update(10.0 + 0.1 * i, (14.29, 2.75, 2.0 + 0.1 * i))
        self.assertFalse(verifier.ready(11.0, 9.0))

    def test_staging_is_behind_entry_on_horizontal_floor(self):
        self.assertEqual(compute_staging((12.8, 3.0), (0.0, -1.0), 1.0), (12.8, 4.0))

    def test_reverify_matches_geometry_not_track_id(self):
        self.assertTrue(same_track_geometry((12.8, 3.0), (0.0, -1.0),
                                            (12.9, 3.1), (-0.1, -1.0)))
        self.assertFalse(same_track_geometry((12.8, 3.0), (0.0, -1.0),
                                             (14.2, 1.5), (0.0, 1.0)))

    def test_mission_snapshot_accepts_only_monotonic_canonical_expansion(self):
        current = ((12.75, 2.81), (12.62, 1.06), 1.74)
        expanded = ((12.86, 2.81), (12.76, 0.39), 2.41)
        partial = ((12.70, 2.20), (12.65, 1.10), 1.10)
        self.assertTrue(mission_extent_expands(*current, *expanded))
        self.assertFalse(mission_extent_expands(*current, *partial))

    def test_stair_keeps_control_after_terminal_state(self):
        self.assertFalse(stair_state_owns_control('IDLE'))
        self.assertTrue(stair_state_owns_control('COMPLETE'))
        self.assertTrue(stair_state_owns_control('FAILED'))


if __name__ == '__main__':
    unittest.main()
