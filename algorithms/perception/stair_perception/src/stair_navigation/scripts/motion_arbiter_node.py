#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from stair_navigation.control import MotionArbiterCore


class MotionArbiterNode:
    def __init__(self):
        self.core = MotionArbiterCore(rospy.get_param('~timeout', 0.3))
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/cmd_vel_nav', Twist, self.command_cb, callback_args='nav', queue_size=1)
        rospy.Subscriber('/cmd_vel_stair', Twist, self.command_cb, callback_args='stair', queue_size=1)
        rospy.Subscriber('/stair_episode/active', Bool, self.active_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.tick)

    def command_cb(self, msg, source):
        self.core.update(source, msg, rospy.get_time())

    def active_cb(self, msg):
        self.core.stair_active = msg.data

    def tick(self, _event):
        self.pub.publish(self.core.select(rospy.get_time()) or Twist())


if __name__ == '__main__':
    rospy.init_node('motion_arbiter')
    MotionArbiterNode()
    rospy.spin()
