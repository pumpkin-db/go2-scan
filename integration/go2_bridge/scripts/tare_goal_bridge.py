#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 胶水：TARE 的 /way_point（geometry_msgs/PointStamped，lookahead 单点）
          → SCAN-Planner 的 /move_base_simple/goal（geometry_msgs/PoseStamped，navi_mode=1 输入）

为什么这么做：TARE 原始的执行输出是「狗前方 lookahead 单点」（随狗移动平滑更新），
而不是「整条路径」。SCAN-Planner 的 navi_mode=1（MANUAL_TARGET）正好是「朝单点目标走」。
这样狗连续追 lookahead 点，不会被「每秒发整条路径 → 每秒重规划」逼得来回掉头。
"""
import time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped


class TareGoalBridge:
    def __init__(self):
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.waypoint_sub = rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)
        self.start_time = time.time()  # 真实时间（wall clock），不用 sim 时间（sim 时间启动初期会跑得比真实快）
        self.start_delay = rospy.get_param('~start_delay', 15.0)  # 启动后静止秒数
        rospy.loginfo('[tare_goal_bridge] ready: /way_point -> /move_base_simple/goal (start_delay=%.1fs)',
                      self.start_delay)

    def waypoint_cb(self, msg):
        # 启动后前 start_delay 秒不转发目标（狗静止、只扫描建图，避免初始地图不全导致穿墙）
        if time.time() - self.start_time < self.start_delay:
            return
        goal = PoseStamped()
        goal.header = msg.header
        goal.header.frame_id = 'world'
        goal.pose.position = msg.point
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


if __name__ == '__main__':
    rospy.init_node('tare_goal_bridge')
    TareGoalBridge()
    rospy.spin()
