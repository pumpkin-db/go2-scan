#!/usr/bin/env python3
import json

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Int32, String
from stair_perception.msg import StairTrack, StairTrackArray
from stair_navigation.floor_context import StablePoseWindow
from stair_navigation.multifloor import MultiFloorLifecycle


class MultiFloorSupervisor:
    def __init__(self):
        self.life = MultiFloorLifecycle()
        self.ariadne_status = 'STARTING'
        self.ariadne_session = 0
        self.bridge_target_valid = False
        self.approach_state = 'WAIT_TRACK'
        self.stair_state = 'IDLE'
        self.floor_state = 'INITIALIZING'
        self.confirmed_tracks = {}
        self.traversed_ids = set()
        self.last_completed_episode = 0
        self.floor_changed_at = rospy.Time(0)
        self.map_seen_at = rospy.Time(0)
        self.map_has_data = False
        self.pose_seen_at = rospy.Time(0)
        self.pose_xy = None
        self.pose_window = StablePoseWindow(duration=1.0, max_xy_span=0.08, max_z_span=0.04)

        self.state_pub = rospy.Publisher('/multifloor/state', String, queue_size=1, latch=True)
        self.pause_pub = rospy.Publisher('/ariadne/lifecycle/pause', Bool, queue_size=1, latch=True)
        self.reset_pub = rospy.Publisher('/ariadne/lifecycle/reset_for_floor', Bool, queue_size=1)
        self.approach_start_pub = rospy.Publisher('/stair_approach/start', Bool, queue_size=1)
        self.approach_track_pub = rospy.Publisher('/stair_approach/mission_track', StairTrack, queue_size=1)
        self.approach_reset_pub = rospy.Publisher('/stair_approach/reset', Bool, queue_size=1)
        self.episode_reset_pub = rospy.Publisher('/stair_episode/reset', Bool, queue_size=1)

        rospy.Subscriber('/ariadne/status', String, self.ariadne_status_cb, queue_size=1)
        rospy.Subscriber('/ariadne/session_id', Int32, self.session_cb, queue_size=1)
        rospy.Subscriber('/ariadne/bridge/target_valid', Bool, self.target_valid_cb, queue_size=1)
        rospy.Subscriber('/stair_perception/tracks', StairTrackArray, self.tracks_cb, queue_size=1)
        rospy.Subscriber('/stair_approach/state', String, self.approach_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/state', String, self.stair_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/completed_id', Int32, self.completed_cb, queue_size=1)
        rospy.Subscriber('/floor_context/floor_id', Int32, self.floor_cb, queue_size=1)
        rospy.Subscriber('/floor_context/state', String, self.floor_state_cb, queue_size=1)
        rospy.Subscriber('/floor_context/transition', String, self.transition_cb, queue_size=1)
        rospy.Subscriber('/active_floor_map', OccupancyGrid, self.map_cb, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        self.publish_state()
        self.pause_pub.publish(Bool(False))

    def publish_state(self):
        self.state_pub.publish(String(self.life.state))

    def ariadne_status_cb(self, msg): self.ariadne_status = msg.data
    def session_cb(self, msg):
        self.ariadne_session = msg.data
        if self.life.session_id == 0 and self.life.state == 'EXPLORE':
            self.life.session_id = msg.data
    def target_valid_cb(self, msg): self.bridge_target_valid = msg.data
    def approach_cb(self, msg): self.approach_state = msg.data
    def stair_cb(self, msg): self.stair_state = msg.data
    def completed_cb(self, msg): self.last_completed_episode = max(self.last_completed_episode, msg.data)
    def floor_state_cb(self, msg): self.floor_state = msg.data

    def tracks_cb(self, msg):
        for track in msg.tracks:
            if track.state == StairTrack.CONFIRMED and track.id not in self.traversed_ids:
                self.confirmed_tracks[track.id] = track

    def transition_cb(self, msg):
        try:
            for value in str(json.loads(msg.data).get('stair_id', '')).split('-'):
                if value:
                    self.traversed_ids.add(int(value))
        except (TypeError, ValueError, json.JSONDecodeError):
            rospy.logwarn_throttle(2.0, '[multifloor] invalid transition record')

    def floor_cb(self, msg):
        if self.life.observe_floor(msg.data):
            self.confirmed_tracks = {}
            self.pose_window.clear()
            self.floor_changed_at = rospy.Time.now()
            self.map_seen_at = rospy.Time(0)
            self.map_has_data = False
            self.publish_state()

    def map_cb(self, msg):
        self.map_seen_at = rospy.Time.now()
        self.map_has_data = bool(msg.info.width and msg.info.height and
                                 any(value != -1 for value in msg.data))

    def pose_cb(self, msg):
        now = rospy.Time.now()
        p = msg.pose.pose.position
        self.pose_seen_at = now
        self.pose_xy = (p.x, p.y)
        self.pose_window.add(now.to_sec(), (p.x, p.y, p.z))

    def available_track(self):
        choices = [track for track_id, track in self.confirmed_tracks.items()
                   if track_id not in self.traversed_ids]
        if not choices or self.pose_xy is None:
            return None
        return min(choices, key=lambda track:
                   (track.entry_pose.position.x - self.pose_xy[0]) ** 2 +
                   (track.entry_pose.position.y - self.pose_xy[1]) ** 2)

    def tick(self, _event):
        now = rospy.Time.now()
        changed = False
        if self.life.state == 'EXPLORE':
            track = self.available_track()
            if self.ariadne_status == 'COMPLETE' and track is not None:
                changed = self.life.exploration_complete(True)
                if changed:
                    self.approach_track_pub.publish(track)
                    self.pause_pub.publish(Bool(True))
        elif self.life.state == 'PAUSE_EXPLORER':
            changed = self.life.explorer_paused(
                self.ariadne_status == 'PAUSED' and not self.bridge_target_valid)
            if changed:
                self.approach_start_pub.publish(Bool(True))
        elif self.life.state == 'STAIR_APPROACH':
            changed = self.approach_state == 'DONE' and self.life.approach_handoff()
        elif self.life.state == 'STAIR_TRAVERSE':
            changed = (self.stair_state == 'COMPLETE' and self.last_completed_episode > 0 and
                       self.life.stair_complete())
        elif self.life.state == 'WAIT_FLOOR_MAP':
            map_fresh = (self.map_has_data and self.map_seen_at >= self.floor_changed_at and
                         (now - self.map_seen_at).to_sec() <= 1.0 and self.floor_state == 'ACTIVE')
            pose_valid = ((now - self.pose_seen_at).to_sec() <= 0.5)
            changed = self.life.floor_ready(map_fresh, pose_valid, self.pose_window.stable())
            if changed:
                self.reset_pub.publish(Bool(True))
                self.episode_reset_pub.publish(Bool(True))
                self.approach_reset_pub.publish(Bool(True))
        elif self.life.state == 'RESET_EXPLORER':
            changed = self.life.explorer_reset(
                self.ariadne_session, self.ariadne_status in ('PAUSED', 'EXPLORING'))
            if changed:
                self.pause_pub.publish(Bool(False))
        if changed:
            self.publish_state()
            rospy.loginfo('[multifloor] state=%s floor=%d session=%d',
                          self.life.state, self.life.floor_id, self.life.session_id)


if __name__ == '__main__':
    rospy.init_node('multifloor_supervisor')
    MultiFloorSupervisor()
    rospy.spin()
