#!/usr/bin/env python3
import json

import rospy
from dynamic_reconfigure.client import Client
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Int32, String
from stair_perception.msg import StairTrack
from stair_navigation.floor_context import StablePoseWindow, relative_z_band


class FloorManager:
    def __init__(self):
        self.floor_id = 0
        self.floor_z_ref = None
        self.stair_state = 'IDLE'
        self.state = 'INITIALIZING'
        self.handoff_pending = False
        self.tracks = []
        self.transitions = []
        self.body_height = rospy.get_param('~body_height', 0.35)
        self.min_above = rospy.get_param('~occupancy_min_above_floor', 0.2)
        self.max_above = rospy.get_param('~occupancy_max_above_floor', 1.0)
        self.min_floor_separation = rospy.get_param('~min_floor_separation', 0.8)
        self.server = rospy.get_param('~octomap_server', '/active_floor_octomap')
        self.window = StablePoseWindow(
            rospy.get_param('~stable_duration', 1.0),
            rospy.get_param('~stable_xy_span', 0.08),
            rospy.get_param('~stable_z_span', 0.04))
        self.client = None

        self.id_pub = rospy.Publisher('/floor_context/floor_id', Int32, queue_size=1, latch=True)
        self.z_pub = rospy.Publisher('/floor_context/z_ref', Float64, queue_size=1, latch=True)
        self.min_pub = rospy.Publisher('/floor_context/active_z_min', Float64, queue_size=1, latch=True)
        self.max_pub = rospy.Publisher('/floor_context/active_z_max', Float64, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/floor_context/state', String, queue_size=1, latch=True)
        self.handoff_pub = rospy.Publisher('/floor_handoff/active', Bool, queue_size=1, latch=True)
        self.transition_pub = rospy.Publisher('/floor_context/transition', String, queue_size=1, latch=True)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/state', String, self.stair_state_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/active_track', StairTrack, self.track_cb, queue_size=4)
        rospy.Timer(rospy.Duration(0.1), self.tick)
        self.publish_context()

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        self.window.add(rospy.get_time(), (p.x, p.y, p.z))

    def stair_state_cb(self, msg):
        previous = self.stair_state
        self.stair_state = msg.data
        if msg.data == 'COMPLETE' and previous != 'COMPLETE' and self.floor_z_ref is not None:
            self.handoff_pending = True
            self.state = 'FLOOR_HANDOFF'
            self.window.clear()
            self.publish_context()

    def track_cb(self, msg):
        if not self.tracks or self.tracks[-1].id != msg.id:
            self.tracks.append(msg)
        else:
            self.tracks[-1] = msg

    def configure_projection(self, floor_z_ref):
        z_min, z_max = relative_z_band(floor_z_ref, self.min_above, self.max_above)
        if self.client is None:
            self.client = Client(self.server, timeout=2.0)
        self.client.update_configuration({
            'occupancy_min_z': z_min,
            'occupancy_max_z': z_max,
            'incremental_2D_projection': False})
        return z_min, z_max

    def publish_context(self):
        self.id_pub.publish(Int32(self.floor_id))
        self.state_pub.publish(String(self.state))
        self.handoff_pub.publish(Bool(self.state == 'FLOOR_HANDOFF'))
        if self.floor_z_ref is not None:
            z_min, z_max = relative_z_band(self.floor_z_ref, self.min_above, self.max_above)
            self.z_pub.publish(Float64(self.floor_z_ref))
            self.min_pub.publish(Float64(z_min))
            self.max_pub.publish(Float64(z_max))

    @staticmethod
    def point(p):
        return {'x': p.x, 'y': p.y, 'z': p.z}

    def publish_transition(self, from_floor, to_floor):
        first = self.tracks[0] if self.tracks else None
        last = self.tracks[-1] if self.tracks else None
        record = {
            'from_floor': from_floor,
            'to_floor': to_floor,
            'stair_id': '-'.join(str(track.id) for track in self.tracks),
            'entry': self.point(first.entry_pose.position) if first else None,
            'exit': self.point(last.exit_pose.position) if last else None,
            'status': 'TRAVERSED'}
        self.transitions.append(record)
        encoded = json.dumps(record, sort_keys=True)
        self.transition_pub.publish(String(encoded))
        rospy.set_param('/floor_context/last_transition', encoded)
        rospy.set_param('/floor_context/transitions', json.dumps(self.transitions, sort_keys=True))
        self.tracks = []

    def tick(self, _event):
        if self.floor_z_ref is None:
            if not self.window.stable():
                return
            floor_z_ref = self.window.floor_z(self.body_height)
            try:
                self.configure_projection(floor_z_ref)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, '[floor_manager] projection unavailable: %s', exc)
                return
            self.floor_z_ref = floor_z_ref
            self.state = 'ACTIVE'
            self.publish_context()
            rospy.loginfo('[floor_manager] floor=0 z_ref=%.3f', self.floor_z_ref)
            return

        if not self.handoff_pending or not self.window.stable():
            return
        new_z_ref = self.window.floor_z(self.body_height)
        if new_z_ref < self.floor_z_ref + self.min_floor_separation:
            rospy.logerr('[floor_manager] rejected handoff: z_ref %.3f -> %.3f',
                         self.floor_z_ref, new_z_ref)
            self.state = 'FAILED'
            self.handoff_pending = False
            self.publish_context()
            return
        try:
            self.configure_projection(new_z_ref)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, '[floor_manager] handoff projection unavailable: %s', exc)
            return
        old_floor = self.floor_id
        self.floor_id += 1
        self.floor_z_ref = new_z_ref
        self.publish_transition(old_floor, self.floor_id)
        self.state = 'ACTIVE'
        self.handoff_pending = False
        self.publish_context()
        rospy.loginfo('[floor_manager] handoff %d->%d z_ref=%.3f',
                      old_floor, self.floor_id, self.floor_z_ref)


if __name__ == '__main__':
    rospy.init_node('floor_manager')
    FloorManager()
    rospy.spin()
