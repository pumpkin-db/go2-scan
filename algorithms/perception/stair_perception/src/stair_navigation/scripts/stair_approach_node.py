#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from stair_perception.msg import StairTrack, StairTrackArray
from stair_navigation.control import compute_staging, same_track_geometry


class StairApproach:
    WAIT_TRACK, NAVIGATE, REVERIFY, HANDOFF, DONE, FAILED = range(6)
    NAMES = ['WAIT_TRACK', 'NAVIGATE', 'REVERIFY', 'HANDOFF', 'DONE', 'FAILED']

    def __init__(self):
        self.pose = None
        self.pose_seen_at = rospy.Time(0)
        self.tracks = []
        self.nav_cmd = Twist()
        self.nav_cmd_seen_at = rospy.Time(0)
        self.stair_active = False
        self.selected = None
        self.staging = None
        self.stop_since = None
        self.state = self.WAIT_TRACK
        self.state_since = rospy.Time.now()
        self.staging_distance = rospy.get_param('~staging_distance', 1.0)
        self.arrival_tolerance = rospy.get_param('~arrival_tolerance', 0.35)
        self.body_height = rospy.get_param('~body_height', 0.35)
        self.track_range = rospy.get_param('~track_range', 6.0)
        self.nav_timeout = rospy.get_param('~nav_timeout', 45.0)
        self.reverify_timeout = rospy.get_param('~reverify_timeout', 5.0)
        self.pose_timeout = rospy.get_param('~pose_timeout', 0.5)
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        self.start_pub = rospy.Publisher('/stair_episode/start_track', StairTrack, queue_size=1)
        self.state_pub = rospy.Publisher('/stair_approach/state', String, queue_size=1, latch=True)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber('/stair_perception/tracks', StairTrackArray, self.tracks_cb, queue_size=1)
        rospy.Subscriber('/cmd_vel_nav', Twist, self.nav_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/active', Bool, self.active_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.publish_state()

    def pose_cb(self, msg):
        self.pose = msg.pose.pose.position
        self.pose_seen_at = rospy.Time.now()

    def tracks_cb(self, msg):
        self.tracks = list(msg.tracks)

    def nav_cb(self, msg):
        self.nav_cmd = msg
        self.nav_cmd_seen_at = rospy.Time.now()

    def active_cb(self, msg):
        self.stair_active = msg.data

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
        self.staging = compute_staging((entry.x, entry.y), (heading.x, heading.y),
                                       self.staging_distance)
        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'world'
        floor_z = self.pose.z - self.body_height
        for x, y in ((self.pose.x, self.pose.y), self.staging):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = floor_z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        rospy.loginfo('[stair_approach] staging=(%.2f, %.2f, %.2f) track=%d',
                      self.staging[0], self.staging[1], floor_z, self.selected.id)

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
            if self.path_pub.get_num_connections() == 0:
                return
            self.selected = self.choose_track()
            if self.selected:
                self.publish_staging_path()
                self.set_state(self.NAVIGATE)
        elif self.state == self.NAVIGATE:
            distance = math.hypot(self.pose.x - self.staging[0], self.pose.y - self.staging[1])
            speed = math.hypot(self.nav_cmd.linear.x, self.nav_cmd.linear.y) + abs(self.nav_cmd.angular.z)
            nav_fresh = (now - self.nav_cmd_seen_at).to_sec() <= 0.5
            if distance <= self.arrival_tolerance and nav_fresh and speed <= 0.03:
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
