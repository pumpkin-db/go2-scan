#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 边界节点：发布 /navigation_boundary（场景 bbox 矩形），限制 TARE 视点在场景内。

背景：kCheckTerrainCollision=false 时，TARE 视点采样不做地形碰撞，视点/lookahead 点
可能落在场景外（墙外），导致狗穿墙。TARE 的 viewpoint_manager 会用 /navigation_boundary
（多边形）限制视点：视点不在多边形内就跳过。所以发布场景 bbox 矩形即可根治穿墙。

输入参数：~xmin ~xmax ~ymin ~ymax（场景 bbox，默认 indoor_1 的值）
输出：/navigation_boundary（geometry_msgs/PolygonStamped，latch）
"""
import rospy
from geometry_msgs.msg import PolygonStamped, Point32


class NavigationBoundaryPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/navigation_boundary', PolygonStamped, queue_size=1, latch=True)
        self.xmin = rospy.get_param('~xmin', -8.44)
        self.xmax = rospy.get_param('~xmax', 45.0)
        self.ymin = rospy.get_param('~ymin', -3.65)
        self.ymax = rospy.get_param('~ymax', 30.4)
        self.publish()

    def publish(self):
        msg = PolygonStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = rospy.Time.now()
        corners = [(self.xmin, self.ymin), (self.xmax, self.ymin),
                   (self.xmax, self.ymax), (self.xmin, self.ymax)]
        for x, y in corners:
            p = Point32()
            p.x = x
            p.y = y
            p.z = 0.0
            msg.polygon.points.append(p)
        self.pub.publish(msg)
        rospy.loginfo('[navigation_boundary_publisher] boundary x[%.1f, %.1f] y[%.1f, %.1f]',
                      self.xmin, self.xmax, self.ymin, self.ymax)


if __name__ == '__main__':
    rospy.init_node('navigation_boundary_publisher')
    NavigationBoundaryPublisher()
    rospy.spin()
