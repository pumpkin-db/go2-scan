#!/usr/bin/env python3
import math
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from stair_perception.msg import StairTrack, StairTrackArray
from stair_navigation.control import (CorridorFollower, ExitVerifier, clamp,
                                      landing_reacquire_score, wrap)


class StairTraverser:
    IDLE, ACQUIRE, ALIGN, ASCEND, LANDING_SCAN, EXIT_VERIFY, COMPLETE, FAILED = range(8)
    NAMES = ['IDLE', 'ACQUIRE', 'ALIGN', 'ASCEND', 'LANDING_SCAN', 'EXIT_VERIFY',
             'COMPLETE', 'FAILED']

    def __init__(self):
        self.pose = None
        self.yaw = 0.0
        self.tracks = []
        self.active_track = None
        self.used_ids = set()
        self.state = self.IDLE
        self.state_since = rospy.Time.now()
        self.episode_since = None
        self.episode_start_z = None
        self.expected_rise = 0.0
        self.exit_verifier = None
        self.flights = 0
        self.required_flights = rospy.get_param('~required_flights', 2)
        self.auto_start = rospy.get_param('~auto_start', True)
        self.acquire_range = rospy.get_param('~acquire_range', 4.0)
        self.entry_tolerance = rospy.get_param('~entry_tolerance', 0.45)
        self.landing_scan_time = rospy.get_param('~landing_scan_time', 3.0)
        self.landing_min_turn_deg = rospy.get_param('~landing_min_turn_deg', 120.0)
        self.landing_confirm_observations = rospy.get_param('~landing_confirm_observations', 2)
        self.landing_min_confidence = rospy.get_param('~landing_min_confidence', 0.45)
        self.exit_verify_timeout = rospy.get_param('~exit_verify_timeout', 8.0)
        self.pose_timeout = rospy.get_param('~pose_timeout', 0.5)
        self.max_episode_time = rospy.get_param('~max_episode_time', 120.0)
        self.follower = CorridorFollower(rospy.get_param('~forward_speed', 0.13))
        self.cmd_pub = rospy.Publisher('/cmd_vel_stair', Twist, queue_size=1)
        self.active_pub = rospy.Publisher('/stair_episode/active', Bool, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/stair_episode/state', String, queue_size=1, latch=True)
        self.track_pub = rospy.Publisher('/stair_episode/active_track', StairTrack,
                                         queue_size=1, latch=True)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber('/stair_perception/tracks', StairTrackArray, self.tracks_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.publish_state()

    def pose_cb(self, msg):
        self.pose = msg.pose.pose.position
        self.pose_seen_at = rospy.Time.now()
        q = msg.pose.pose.orientation
        self.yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

    def tracks_cb(self, msg):
        self.tracks = list(msg.tracks)

    def set_state(self, state):
        self.state = state
        self.state_since = rospy.Time.now()
        self.publish_state()
        rospy.loginfo('[stair_traverser] state=%s flights=%d', self.NAMES[state], self.flights)

    def publish_state(self):
        active = self.state not in (self.IDLE, self.COMPLETE)
        self.active_pub.publish(Bool(active))
        self.state_pub.publish(String(self.NAMES[self.state]))

    def choose_track(self):
        if self.pose is None:
            return None
        choices = []
        for track in self.tracks:
            if track.state != StairTrack.CONFIRMED or track.id in self.used_ids:
                continue
            entry = track.entry_pose.position
            planar = math.hypot(entry.x - self.pose.x, entry.y - self.pose.y)
            vertical = abs(entry.z - self.pose.z)
            if planar <= self.acquire_range and vertical <= 0.9:
                choices.append((planar, track))
        return min(choices, key=lambda item: item[0])[1] if choices else None

    def activate_track(self, track):
        self.active_track = track
        self.used_ids.add(track.id)
        self.expected_rise += max(0.0, track.rise)
        self.track_pub.publish(track)
        self.set_state(self.ACQUIRE)

    def choose_landing_reacquire_track(self):
        if self.pose is None or self.active_track is None:
            return None
        previous = (self.active_track.heading.x, self.active_track.heading.y)
        robot = (self.pose.x, self.pose.y, self.pose.z)
        choices = []
        for track in self.tracks:
            if track.id in self.used_ids:
                continue
            entry = track.entry_pose.position
            score = landing_reacquire_score(
                previous, (track.heading.x, track.heading.y), (entry.x, entry.y, entry.z), robot,
                track.rise, track.last_seen.to_sec(), self.state_since.to_sec(),
                self.acquire_range, 0.9, self.landing_min_turn_deg, 2.0,
                track.observation_count, self.landing_confirm_observations,
                track.confidence, self.landing_min_confidence)
            if score is not None:
                choices.append((score, track))
        return min(choices, key=lambda item: item[0])[1] if choices else None

    def stop(self):
        self.cmd_pub.publish(Twist())

    def fail(self, reason):
        rospy.logerr('[stair_traverser] FAIL: %s', reason)
        self.stop()
        self.set_state(self.FAILED)

    def tick(self, _event):
        now = rospy.Time.now()
        if self.pose is None:
            self.stop()
            return
        if (self.state not in (self.IDLE, self.COMPLETE, self.FAILED) and
                (now - self.pose_seen_at).to_sec() > self.pose_timeout):
            self.fail('localization timeout')
            return
        if self.episode_since and (now - self.episode_since).to_sec() > self.max_episode_time:
            self.fail('episode timeout')
            return
        if self.state == self.IDLE:
            self.stop()
            track = self.choose_track() if self.auto_start else None
            if track:
                self.episode_since = now
                self.episode_start_z = self.pose.z
                self.expected_rise = 0.0
                self.activate_track(track)
            return
        if self.state in (self.COMPLETE, self.FAILED):
            self.stop()
            return

        entry = self.active_track.entry_pose.position if self.active_track else None
        exit_ = self.active_track.exit_pose.position if self.active_track else None
        if entry is None or exit_ is None:
            self.fail('missing active track')
            return
        entry_xy, exit_xy = (entry.x, entry.y), (exit_.x, exit_.y)
        heading, length = self.follower.geometry(entry_xy, exit_xy)
        position = (self.pose.x, self.pose.y)
        target_yaw = math.atan2(heading[1], heading[0])

        cmd = Twist()
        elapsed = (now - self.state_since).to_sec()
        if self.state == self.ACQUIRE:
            dx, dy = entry.x - self.pose.x, entry.y - self.pose.y
            distance = math.hypot(dx, dy)
            if distance <= self.entry_tolerance:
                self.stop(); self.set_state(self.ALIGN); return
            if elapsed > 20.0:
                self.fail('entry timeout'); return
            desired = math.atan2(dy, dx)
            error = wrap(desired - self.yaw)
            cmd.angular.z = clamp(1.5 * error, 0.6)
            cmd.linear.x = 0.12 if abs(error) < 0.45 else 0.0
        elif self.state == self.ALIGN:
            error = wrap(target_yaw - self.yaw)
            if abs(error) < 0.12:
                self.stop(); self.set_state(self.ASCEND); return
            if elapsed > 12.0:
                self.fail('align timeout'); return
            cmd.angular.z = clamp(1.5 * error, 0.6)
        elif self.state == self.ASCEND:
            progress = self.follower.progress(position, entry_xy, heading)
            if progress >= length - 0.18:
                self.flights += 1
                self.stop(); self.set_state(self.LANDING_SCAN); return
            if elapsed > 45.0:
                self.fail('flight timeout'); return
            cmd.linear.x, cmd.linear.y, cmd.angular.z = self.follower.command(
                position, self.yaw, entry_xy, heading)
        elif self.state == self.LANDING_SCAN:
            self.stop()
            if elapsed < self.landing_scan_time:
                return
            if self.flights >= self.required_flights:
                entry = self.active_track.entry_pose.position
                exit_ = self.active_track.exit_pose.position
                self.exit_verifier = ExitVerifier(
                    self.episode_start_z, self.expected_rise,
                    (entry.x, entry.y), (exit_.x, exit_.y))
                self.set_state(self.EXIT_VERIFY)
                return
            next_track = self.choose_landing_reacquire_track()
            if next_track:
                self.activate_track(next_track)
            elif elapsed > 12.0:
                self.fail('next flight not found')
            return
        elif self.state == self.EXIT_VERIFY:
            self.stop()
            self.exit_verifier.update(now.to_sec(), (self.pose.x, self.pose.y, self.pose.z))
            if self.exit_verifier.ready(now.to_sec(), self.state_since.to_sec()):
                self.set_state(self.COMPLETE)
            elif elapsed > self.exit_verify_timeout:
                self.fail('upper floor exit verification timeout')
            return
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('stair_traverser')
    StairTraverser()
    rospy.spin()
