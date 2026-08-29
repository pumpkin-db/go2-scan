#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from stair_perception.msg import StairTrack
from stair_navigation.control import CorridorFollower, TerrainProfileCore


class TerrainFollowSimAdapter:
    def __init__(self):
        self.pose = None
        self.active = False
        self.track_id = None
        self.profile = None
        self.pub = rospy.Publisher('/sim/body_z_target', Float64, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/active', Bool, self.active_cb, queue_size=1)
        rospy.Subscriber('/stair_episode/active_track', StairTrack, self.track_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.tick)

    def pose_cb(self, msg):
        self.pose = msg.pose.pose.position

    def active_cb(self, msg):
        self.active = msg.data

    def track_cb(self, msg):
        if self.pose is None or msg.id == self.track_id:
            return
        entry = (msg.entry_pose.position.x, msg.entry_pose.position.y)
        exit_ = (msg.exit_pose.position.x, msg.exit_pose.position.y)
        heading, _ = CorridorFollower.geometry(entry, exit_)
        position = (self.pose.x, self.pose.y)
        anchor = CorridorFollower().progress(position, entry, heading)
        self.profile = TerrainProfileCore(entry, exit_, msg.rise, self.pose.z, anchor)
        self.track_id = msg.id

    def tick(self, _event):
        if self.active and self.pose is not None and self.profile is not None:
            self.pub.publish(Float64(self.profile.target((self.pose.x, self.pose.y))))


if __name__ == '__main__':
    rospy.init_node('terrain_follow_sim_adapter')
    TerrainFollowSimAdapter()
    rospy.spin()
