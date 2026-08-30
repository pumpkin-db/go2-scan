#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from stair_perception.msg import StairTrack, StairTrackArray
from stair_navigation.control import same_track_geometry
from stair_navigation.safe_staging import (OccupancyGridView, accept_planner_endpoint,
                                           select_safe_staging, staging_arrived)


class StairApproach:
    WAIT_TRACK, NAVIGATE, REVERIFY, HANDOFF, DONE, FAILED = range(6)
    NAMES = ['WAIT_TRACK', 'NAVIGATE', 'REVERIFY', 'HANDOFF', 'DONE', 'FAILED']

    def __init__(self):
        self.auto_start = rospy.get_param('~auto_start', True)
        self.enabled = self.auto_start
        self.pose = None
        self.pose_seen_at = rospy.Time(0)
        self.tracks = []
        self.nav_cmd = Twist()
        self.nav_cmd_seen_at = rospy.Time(0)
        self.stair_active = False
        self.selected = None
        self.requested = None
        self.accepted_staging_pose = None
        self.active_map = None
        self.path_published_at = rospy.Time(0)
        self.stop_since = None
        self.state = self.WAIT_TRACK
        self.state_since = rospy.Time.now()
        self.staging_distance = rospy.get_param('~staging_distance', 1.0)
        self.staging_clearance = rospy.get_param('~staging_clearance', 0.25)
        self.staging_distances = rospy.get_param('~staging_distances',
                                                 [1.0, 0.8, 1.2, 0.6, 1.6, 2.0])
        self.staging_lateral_offsets = rospy.get_param('~staging_lateral_offsets',
                                                       [0.0, 0.2, -0.2, 0.4, -0.4])
        self.arrival_tolerance = rospy.get_param('~arrival_tolerance', 0.35)
        self.body_height = rospy.get_param('~body_height', 0.35)
        self.track_range = rospy.get_param('~track_range', 6.0)
        self.nav_timeout = rospy.get_param('~nav_timeout', 45.0)
        self.reverify_timeout = rospy.get_param('~reverify_timeout', 5.0)
        self.pose_timeout = rospy.get_param('~pose_timeout', 0.5)
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        self.accepted_pub = rospy.Publisher('/stair_approach/accepted_staging_pose',
                                            PoseStamped, queue_size=1, latch=True)
        self.start_pub = rospy.Publisher('/stair_episode/start_track', StairTrack, queue_size=1)
        self.state_pub = rospy.Publisher('/stair_approach/state', String, queue_size=1, latch=True)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber('/active_floor_map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/stair_perception/tracks', StairTrackArray, self.tracks_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel_nav', Twist, self.nav_cb, queue_size=1)
        rospy.Subscriber('/scan_planner_node/goal_point', Marker, self.scan_goal_cb, queue_size=2)
        rospy.Subscriber('/stair_episode/active', Bool, self.active_cb, queue_size=1)
        rospy.Subscriber('/stair_approach/start', Bool, self.start_cb, queue_size=1)
        rospy.Subscriber('/stair_approach/reset', Bool, self.reset_cb, queue_size=1)
        rospy.Subscriber('/stair_approach/mission_track', StairTrack, self.mission_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.publish_state()

    def pose_cb(self, msg):
        self.pose = msg.pose.pose.position
        self.pose_seen_at = rospy.Time.now()

    def map_cb(self, msg):
        self.active_map = OccupancyGridView(msg)

    def tracks_cb(self, msg):
        self.tracks = list(msg.tracks)

    def nav_cb(self, msg):
        self.nav_cmd = msg
        self.nav_cmd_seen_at = rospy.Time.now()

    def scan_goal_cb(self, msg):
        # A two-pose staging path uses marker id=1 for both raw and final SCAN endpoint.
        if (self.accepted_staging_pose is None or msg.id != 1 or
                msg.header.stamp < self.path_published_at):
            return
        current = self.accepted_staging_pose.pose.position
        endpoint = accept_planner_endpoint((current.x, current.y),
                                           (msg.pose.position.x, msg.pose.position.y))
        if endpoint is None:
            rospy.logerr('[stair_approach] reject implausible SCAN endpoint=(%.2f, %.2f)',
                         msg.pose.position.x, msg.pose.position.y)
            return
        if math.hypot(endpoint[0] - current.x, endpoint[1] - current.y) <= 0.01:
            return
        current.x, current.y = endpoint
        self.accepted_pub.publish(self.accepted_staging_pose)
        rospy.loginfo('[stair_approach] accepted_staging updated from SCAN=(%.2f, %.2f)',
                      endpoint[0], endpoint[1])

    def active_cb(self, msg):
        self.stair_active = msg.data

    def start_cb(self, msg):
        if msg.data and self.state == self.WAIT_TRACK:
            self.enabled = True

    def mission_cb(self, msg):
        if self.state == self.WAIT_TRACK and msg.state == StairTrack.CONFIRMED:
            self.requested = msg

    def reset_cb(self, msg):
        if not msg.data or self.state not in (self.DONE, self.FAILED):
            return
        self.selected = None
        self.requested = None
        self.accepted_staging_pose = None
        self.stop_since = None
        self.enabled = self.auto_start
        self.set_state(self.WAIT_TRACK)

    def set_state(self, state):
        self.state = state
        self.state_since = rospy.Time.now()
        self.stop_since = None
        self.publish_state()
        rospy.loginfo('[stair_approach] state=%s', self.NAMES[state])

    def publish_state(self):
        self.state_pub.publish(String(self.NAMES[self.state]))

    def fail(self, reason):
        rospy.logerr('[stair_approach] FAIL: %s', reason)
        self.set_state(self.FAILED)

    def choose_track(self):
        if self.requested is not None:
            return self.requested
        choices = []
        for track in self.tracks:
            if track.state != StairTrack.CONFIRMED:
                continue
            entry = track.entry_pose.position
            distance = math.hypot(entry.x - self.pose.x, entry.y - self.pose.y)
            if distance <= self.track_range and abs(entry.z - self.pose.z) <= 0.9:
                choices.append((distance, track))
        return min(choices, key=lambda item: item[0])[1] if choices else None

    def publish_staging_path(self):
        entry = self.selected.entry_pose.position
        heading = self.selected.heading
        if self.active_map is None:
            return False
        accepted = select_safe_staging(
            self.active_map, (self.pose.x, self.pose.y), (entry.x, entry.y),
            (heading.x, heading.y), self.staging_distance, self.staging_clearance,
            self.staging_distances, self.staging_lateral_offsets)
        if accepted is None:
            return False
        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'world'
        floor_z = self.pose.z - self.body_height
        accepted_msg = PoseStamped()
        accepted_msg.header = path.header
        accepted_msg.pose.position.x = accepted[0]
        accepted_msg.pose.position.y = accepted[1]
        accepted_msg.pose.position.z = floor_z
        yaw = math.atan2(heading.y, heading.x)
        accepted_msg.pose.orientation.z = math.sin(0.5 * yaw)
        accepted_msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.accepted_staging_pose = accepted_msg
        for x, y in ((self.pose.x, self.pose.y), accepted):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = floor_z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_published_at = now
        self.accepted_pub.publish(accepted_msg)
        self.path_pub.publish(path)
        rospy.loginfo('[stair_approach] accepted_staging=(%.2f, %.2f, %.2f) track=%d',
                      accepted[0], accepted[1], floor_z, self.selected.id)
        return True

    def reverified_track(self):
        reference_entry = self.selected.entry_pose.position
        reference_heading = self.selected.heading
        for track in self.tracks:
            entry, heading = track.entry_pose.position, track.heading
            if (track.state == StairTrack.CONFIRMED and track.last_seen >= self.state_since and
                    same_track_geometry((reference_entry.x, reference_entry.y),
                                        (reference_heading.x, reference_heading.y),
                                        (entry.x, entry.y), (heading.x, heading.y))):
                return track
        return None

    def tick(self, _event):
        now = rospy.Time.now()
        if self.state in (self.DONE, self.FAILED):
            return
        if self.pose is None:
            return
        if (now - self.pose_seen_at).to_sec() > self.pose_timeout:
            self.fail('localization timeout')
            return
        elapsed = (now - self.state_since).to_sec()
        if self.state == self.WAIT_TRACK:
            if not self.enabled:
                return
            if self.path_pub.get_num_connections() == 0:
                return
            self.selected = self.choose_track()
            if self.selected:
                if self.publish_staging_path():
                    self.set_state(self.NAVIGATE)
                else:
                    self.fail('no safe connected staging pose')
        elif self.state == self.NAVIGATE:
            accepted = self.accepted_staging_pose.pose.position
            speed = math.hypot(self.nav_cmd.linear.x, self.nav_cmd.linear.y) + abs(self.nav_cmd.angular.z)
            nav_fresh = (now - self.nav_cmd_seen_at).to_sec() <= 0.5
            if (staging_arrived((self.pose.x, self.pose.y), (accepted.x, accepted.y),
                                self.arrival_tolerance) and nav_fresh and speed <= 0.03):
                if self.stop_since is None:
                    self.stop_since = now
                elif (now - self.stop_since).to_sec() >= 0.8:
                    self.set_state(self.REVERIFY)
            else:
                self.stop_since = None
            if elapsed > self.nav_timeout:
                self.fail('SCAN staging timeout')
        elif self.state == self.REVERIFY:
            track = self.reverified_track()
            if track:
                self.start_pub.publish(track)
                self.set_state(self.HANDOFF)
            elif elapsed > self.reverify_timeout:
                self.fail('stair reverify timeout')
        elif self.state == self.HANDOFF:
            if self.stair_active:
                self.set_state(self.DONE)
            elif elapsed > 2.0:
                self.fail('stair control handoff timeout')


if __name__ == '__main__':
    rospy.init_node('stair_approach')
    StairApproach()
    rospy.spin()
