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

2026-08-28 楼梯主线扩展：
  - /nav_source 门控（'ariadne'|'stair'）：stair_mission_manager 接管期间
    本桥停发 /initial_path，保证单一有效控制源（禁止双源竞争）。
  - 5s 周期重发当前目标：SCAN respawn/断连后必然重新拿到有效路径
    （修复 respawn+去重门导致的永久冻结，Bug A）。
  - /floor_reset：切层后清 last_sent，二层新目标立即可转发。
"""
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, String


class WaypointBridge(object):
    def __init__(self):
        self.repub_dist = float(rospy.get_param('~repub_dist', 1.0))
        self.robot_xy = None
        self.last_sent = None
        self.nav_source = 'ariadne'
        # latch 不开：latch 会让迟连接的订阅者收到旧 Path 触发意外重规划
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/way_point', PointStamped, self.waypoint_cb, queue_size=1)
        rospy.Subscriber('/nav_source', String, self.nav_source_cb, queue_size=2)
        rospy.Subscriber('/floor_reset', Empty, self.floor_reset_cb, queue_size=2)
        rospy.Timer(rospy.Duration(5.0), self.resend_tick)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def nav_source_cb(self, msg):
        if msg.data != self.nav_source:
            self.nav_source = msg.data
            rospy.logwarn('[ariadne_goal_bridge] nav_source=%s', self.nav_source)

    def floor_reset_cb(self, _):
        self.last_sent = None
        rospy.logwarn('[ariadne_goal_bridge] floor_reset: last_sent cleared')

    def send_path_to(self, wp):
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

    def resend_tick(self, _):
        """Bug A 兜底：SCAN respawn/断连后周期拿到当前有效路径。"""
        if self.nav_source != 'ariadne' or self.last_sent is None or self.robot_xy is None:
            return
        self.send_path_to(self.last_sent)

    def waypoint_cb(self, msg):
        if self.nav_source != 'ariadne':
            return  # 楼梯接管期间：单一有效控制源
        if self.robot_xy is None:
            return  # 未收到里程计前不发：SCAN 需要真实起点
        wp = (msg.point.x, msg.point.y)
        if self.last_sent is not None:
            d = ((wp[0] - self.last_sent[0]) ** 2 +
                 (wp[1] - self.last_sent[1]) ** 2) ** 0.5
            if d <= self.repub_dist:
                return  # 同目标去重
        self.send_path_to(wp)
        self.last_sent = wp
        rospy.loginfo('[ariadne_goal_bridge] 转发航点 (%.2f, %.2f)', wp[0], wp[1])


if __name__ == '__main__':
    rospy.init_node('ariadne_goal_bridge')
    WaypointBridge()
    rospy.spin()
