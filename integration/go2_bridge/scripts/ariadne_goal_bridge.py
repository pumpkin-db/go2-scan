#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""A2 出口胶水：ARiADNE 的 /way_point(PointStamped) -> SCAN-Planner 的 /initial_path(nav_msgs/Path)。

navi_mode=3 的 pathCallback(scan_replan_fsm.cpp:357) 语义：
  - poses[0] 被当作全局轨迹起点 -> 必须发机器人当前位置
  - 终点取 poses.back()，每个 z 自动 +body_height(0.4) -> 这里发 z=0
  - frame_id 不校验，但按仓库约定填 'world'
  - 每条新 Path 触发整体重规划；走完一段 FSM 进 WAIT_TARGET 等下一条

去重门控：ARiADNE 以 2.5Hz 重发同一航点直至机器人抵达，若每条都转发会触发 2.5Hz 全量重规划。
只有航点相对上次转发移动超过 ~repub_dist 才转发（首条必转）。
不做墙钟 start_delay（tare_goal_bridge 的教训，见 PROGRESS.md 六-5）：就绪判据用数据流本身。
"""

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path


class AriadneGoalBridge:
    def __init__(self):
        self.repub_dist = rospy.get_param('~repub_dist', 1.0)
        self.frame_id = rospy.get_param('~frame_id', 'world')

        self.robot_xy = None            # 最近一次 body_pose 位置
        self.last_sent_wp = None        # 上次转发的航点（None=尚未转发过）
        self.last_robot_xy_sent = None

        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.body_pose_cb, queue_size=1)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)

        rospy.loginfo('[ariadne_goal_bridge] ready: /way_point + /quad_0/body_pose -> /initial_path '
                      '(repub_dist=%.1fm)', self.repub_dist)

    def body_pose_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = np.array([p.x, p.y])

    def waypoint_cb(self, msg):
        if self.robot_xy is None:
            # 还没有里程计就发的航点无法构造合法 Path（起点缺失），丢弃等下一条
            rospy.logwarn_throttle(5.0, '[ariadne_goal_bridge] no odometry yet, drop way_point')
            return

        wp = np.array([msg.point.x, msg.point.y])

        # 去重：与上次转发的航点几乎相同则跳过（ARiADNE 会持续重发同一目标）
        if self.last_sent_wp is not None and np.linalg.norm(wp - self.last_sent_wp) < self.repub_dist:
            return

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()

        robot_pose = PoseStamped()
        robot_pose.header = path.header
        robot_pose.pose.position.x = float(self.robot_xy[0])
        robot_pose.pose.position.y = float(self.robot_xy[1])
        robot_pose.pose.position.z = 0.0
        robot_pose.pose.orientation.w = 1.0

        wp_pose = PoseStamped()
        wp_pose.header = path.header
        wp_pose.pose.position.x = float(wp[0])
        wp_pose.pose.position.y = float(wp[1])
        wp_pose.pose.position.z = 0.0
        wp_pose.pose.orientation.w = 1.0

        path.poses = [robot_pose, wp_pose]
        self.path_pub.publish(path)

        self.last_sent_wp = wp
        self.last_robot_xy_sent = self.robot_xy.copy()
        rospy.loginfo('[ariadne_goal_bridge] /initial_path: [%.2f, %.2f] -> [%.2f, %.2f]',
                      self.robot_xy[0], self.robot_xy[1], wp[0], wp[1])


if __name__ == '__main__':
    rospy.init_node('ariadne_goal_bridge')
    AriadneGoalBridge()
    rospy.spin()
