#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ARiADNE 航点 → SCAN-Planner 参考路径桥（A2 架构唯一自研胶水）。

官方契约：rl_planner 输出 /way_point(PointStamped, ≤2.5Hz)，由下游 waypoint
follower 导航。本机用 SCAN-Planner navi_mode=3 替代 follower：
每条 /initial_path(nav_msgs/Path) 触发一次全局参考重建，走完一段进 WAIT_TARGET。
由此推出四条接口纪律（源码依据见 scan_replan_fsm.cpp pathCallback）：
  1. Path 只需两点 [当前位置, 航点]——SCAN 对稀疏路径自动插中间点
  2. poses[0] 必须是轨迹起点（SCAN 把首点当 trajectory start）
  3. z 发 0（SCAN 内部自动 +body_height）
  4. 必须去重：way_point 以固定频率重复发布同一目标，
     不去重会反复触发 SCAN 整体重规划
frame_id 用 world（SCAN 全局系；数值与 map 恒等，由 map→world 静态 TF 桥保证）。
"""

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path


class WaypointBridge(object):
    def __init__(self):
        self.repub_dist = float(rospy.get_param('~repub_dist', 1.0))
        self.robot_xy = None
        self.last_sent = None
        # latch 不开：latch 会让迟连接的订阅者收到旧 Path 触发意外重规划
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def waypoint_cb(self, msg):
        if self.robot_xy is None:
            return  # 未收到里程计前不发：SCAN 需要真实起点
        wp = (msg.point.x, msg.point.y)
        if self.last_sent is not None:
            d = ((wp[0] - self.last_sent[0]) ** 2 +
                 (wp[1] - self.last_sent[1]) ** 2) ** 0.5
            if d <= self.repub_dist:
                return  # 同目标去重
        now = rospy.Time.now()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'world'
        for x, y in (self.robot_xy, wp):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'world'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.last_sent = wp
        rospy.loginfo('[ariadne_goal_bridge] 转发航点 (%.2f, %.2f)', wp[0], wp[1])


if __name__ == '__main__':
    rospy.init_node('ariadne_goal_bridge')
    WaypointBridge()
    rospy.spin()
