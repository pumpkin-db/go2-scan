#!/usr/bin/env python3
import math
import unittest

from stair_navigation.control import (CorridorFollower, MotionArbiterCore, TerrainProfileCore,
                                      ExitVerifier, compute_staging, landing_reacquire_score,
                                      mission_extent_expands, same_track_geometry,
                                      stair_state_owns_control, StairEpisodeLifecycle)
from stair_navigation.floor_context import (FloorHandoffGate, StablePoseWindow,
                                             relative_z_band)
from stair_navigation.multifloor import MultiFloorLifecycle
from stair_navigation.safe_staging import (OccupancyGridView, accept_planner_endpoint,
                                            select_safe_staging, staging_arrived)


class Attr:
    pass


def grid_message(width=30, height=30, resolution=0.1):
    msg = Attr(); msg.info = Attr(); msg.info.origin = Attr()
    msg.info.origin.position = Attr(); msg.info.origin.orientation = Attr()
    msg.info.width, msg.info.height, msg.info.resolution = width, height, resolution
    msg.info.origin.position.x = msg.info.origin.position.y = 0.0
    q = msg.info.origin.orientation
    q.x = q.y = q.z = 0.0; q.w = 1.0
    msg.data = [0] * (width * height)
    return msg


def occupy(msg, x, y):
    cell_x, cell_y = int(x / msg.info.resolution), int(y / msg.info.resolution)
    msg.data[cell_y * msg.info.width + cell_x] = 100


class CoreTests(unittest.TestCase):
    def test_safe_staging_accepts_free_nominal_goal(self):
        grid = OccupancyGridView(grid_message())
        accepted = select_safe_staging(grid, (0.5, 1.5), (2.0, 1.5), (1.0, 0.0),
                                       clearance=0.15)
        self.assertEqual(accepted, (1.0, 1.5))

    def test_safe_staging_replaces_occupied_nominal_goal(self):
        msg = grid_message()
        occupy(msg, 1.0, 1.5)
        accepted = select_safe_staging(OccupancyGridView(msg), (0.5, 1.5),
                                       (2.0, 1.5), (1.0, 0.0), clearance=0.15)
        self.assertIsNotNone(accepted)
        self.assertNotEqual(accepted, (1.0, 1.5))
        self.assertTrue(staging_arrived(accepted, accepted, 0.35))
        self.assertFalse(staging_arrived(accepted, (1.0, 1.5), 0.05))

    def test_safe_staging_endpoint_is_single_accepted_value(self):
        grid = OccupancyGridView(grid_message())
        accepted = select_safe_staging(grid, (0.5, 1.5), (2.0, 1.5), (1.0, 0.0),
                                       clearance=0.15)
        scan_endpoint = accept_planner_endpoint(accepted, (accepted[0] + 0.2, accepted[1]))
        arrival_target = scan_endpoint
        self.assertEqual(scan_endpoint, arrival_target)

    def test_safe_staging_rejects_implausible_planner_endpoint(self):
        self.assertIsNone(accept_planner_endpoint((1.0, 1.0), (3.0, 1.0)))

    def test_safe_staging_fails_when_no_candidate_is_free(self):
        msg = grid_message()
        msg.data = [-1] * len(msg.data)
        accepted = select_safe_staging(OccupancyGridView(msg), (0.5, 1.5),
                                       (2.0, 1.5), (1.0, 0.0), clearance=0.15)
        self.assertIsNone(accepted)

    def test_partial_mission_old_entry_requires_current_map_connection(self):
        msg = grid_message()
        for y in range(msg.info.height):
            msg.data[y * msg.info.width + 12] = 100
        accepted = select_safe_staging(OccupancyGridView(msg), (0.5, 1.5),
                                       (2.4, 1.5), (1.0, 0.0), clearance=0.15,
                                       distances=(0.6,), lateral_offsets=(0.0,))
        self.assertIsNone(accepted)

    def test_arbiter_is_fail_closed_and_exclusive(self):
        core = MotionArbiterCore(timeout=0.3)
        core.update('nav', 'nav', 1.0)
        self.assertEqual(core.select(1.1), 'nav')
        core.stair_active = True
        self.assertIsNone(core.select(1.1))
        core.update('stair', 'stair', 1.2)
        self.assertEqual(core.select(1.3), 'stair')
        core.handoff_active = True
        self.assertIsNone(core.select(1.3))
        core.handoff_active = False
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

    def test_floor_reference_requires_stable_pose_and_relative_band(self):
        window = StablePoseWindow(duration=1.0, max_xy_span=0.08, max_z_span=0.04)
        for i in range(11):
            window.add(i * 0.1, (14.0 + 0.002 * (i % 2), 3.5, 4.73))
        self.assertTrue(window.stable())
        self.assertAlmostEqual(window.floor_z(0.35), 4.38)
        self.assertEqual(relative_z_band(4.38, 0.2, 1.0), (4.58, 5.38))

    def test_complete_latches_past_episode_timeout(self):
        life = StairEpisodeLifecycle()
        life.start(1.0)
        life.transition('COMPLETE', 30.0)
        self.assertFalse(life.timed_out(200.0, 120.0))
        self.assertEqual(life.state, 'COMPLETE')

    def test_failed_rejects_late_timer_transition(self):
        life = StairEpisodeLifecycle()
        life.start(1.0)
        life.transition('FAILED', 2.0)
        self.assertFalse(life.timed_out(500.0, 120.0))
        self.assertFalse(life.transition('COMPLETE', 3.0))
        self.assertEqual(life.state, 'FAILED')

    def test_explicit_reset_starts_clean_independent_episode(self):
        life = StairEpisodeLifecycle()
        first_id = life.start(1.0)
        life.transition('COMPLETE', 2.0)
        self.assertTrue(life.reset(10.0))
        second_id = life.start(20.0)
        self.assertEqual(second_id, first_id + 1)
        self.assertEqual(life.started_at, 20.0)
        self.assertFalse(life.timed_out(21.0, 120.0))

    def test_terminal_state_disallows_motion(self):
        life = StairEpisodeLifecycle()
        life.start(1.0)
        life.transition('COMPLETE', 2.0)
        self.assertFalse(life.motion_allowed())

    def test_handoff_is_one_shot_per_episode(self):
        gate = FloorHandoffGate()
        gate.observe(7)
        floor_id = 0
        self.assertTrue(gate.request(7)); floor_id += 1
        self.assertFalse(gate.request(7))
        self.assertTrue(gate.commit())
        self.assertFalse(gate.request(7))
        self.assertEqual(floor_id, 1)
        self.assertEqual(gate.processed, {7})

    def test_new_episode_can_handoff_once_after_previous(self):
        gate = FloorHandoffGate()
        gate.observe(7)
        self.assertTrue(gate.request(7)); self.assertTrue(gate.commit())
        gate.observe(8)
        self.assertTrue(gate.request(8)); self.assertTrue(gate.commit())
        self.assertEqual(gate.processed, {7, 8})

    def test_late_old_completion_cannot_pollute_new_episode(self):
        gate = FloorHandoffGate()
        gate.observe(7)
        gate.observe(8)
        self.assertFalse(gate.request(7))
        self.assertTrue(gate.request(8))

    def test_multifloor_complete_requires_available_stair(self):
        life = MultiFloorLifecycle(floor_id=0, session_id=4)
        self.assertFalse(life.exploration_complete(False))
        self.assertEqual(life.state, 'EXPLORE')
        self.assertTrue(life.exploration_complete(True))
        self.assertFalse(life.exploration_complete(True))
        self.assertEqual(life.transition_count, 1)

    def test_multifloor_pause_requires_invalidated_target(self):
        life = MultiFloorLifecycle(session_id=4)
        life.exploration_complete(True)
        self.assertFalse(life.explorer_paused(False))
        self.assertTrue(life.explorer_paused(True))
        self.assertEqual(life.state, 'STAIR_APPROACH')

    def test_multifloor_handoff_cannot_reset_early(self):
        life = MultiFloorLifecycle(floor_id=0, session_id=4)
        life.exploration_complete(True); life.explorer_paused(True)
        life.approach_handoff(); life.stair_complete()
        self.assertFalse(life.floor_ready(True, True, True))
        self.assertFalse(life.observe_floor(0))
        self.assertEqual(life.state, 'FLOOR_HANDOFF')

    def test_multifloor_new_floor_requires_fresh_map_pose_and_stop(self):
        life = MultiFloorLifecycle(floor_id=0, session_id=4)
        life.exploration_complete(True); life.explorer_paused(True)
        life.approach_handoff(); life.stair_complete(); life.observe_floor(1)
        self.assertFalse(life.floor_ready(False, True, True))
        self.assertFalse(life.floor_ready(True, False, True))
        self.assertFalse(life.floor_ready(True, True, False))
        self.assertTrue(life.floor_ready(True, True, True))
        self.assertEqual(life.state, 'RESET_EXPLORER')

    def test_multifloor_reset_requires_new_session_and_cleared_complete(self):
        life = MultiFloorLifecycle(floor_id=0, session_id=4)
        life.exploration_complete(True); life.explorer_paused(True)
        life.approach_handoff(); life.stair_complete(); life.observe_floor(1)
        life.floor_ready(True, True, True)
        self.assertFalse(life.explorer_reset(4, True))
        self.assertFalse(life.explorer_reset(5, False))
        self.assertTrue(life.explorer_reset(5, True))
        self.assertEqual((life.state, life.floor_id, life.session_id), ('EXPLORE', 1, 5))


if __name__ == '__main__':
    unittest.main()
