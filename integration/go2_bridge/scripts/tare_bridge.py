#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 转发节点：把 Gazebo 的世界系点云（livox 插件直接输出，frame=world）转发给 TARE 探索决策层。

TARE 需要（frame 都硬编码 kWorldFrameID="map"）：
- /registered_scan         : 世界系点云（PointXYZ），配准后的激光
- /state_estimation_at_scan: 世界系 base 位姿（Odometry）

本节点：
- 订阅 /mid360_points（世界系点云，frame=world）→ 原样转发为 /registered_scan（无需坐标变换）
- 订阅 /quad_0/body_pose（base 世界系位姿）→ 转发为 /state_estimation_at_scan

（仿真里 livox 插件直接输出世界系真实位置，故无需再变换）
"""
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class TareBridge:
    def __init__(self):
        self.world_frame = rospy.get_param('~world_frame', 'world')

        self.reg_pub = rospy.Publisher('/registered_scan', PointCloud2, queue_size=1)
        self.state_pub = rospy.Publisher('/state_estimation_at_scan', Odometry, queue_size=1)
        self.cloud_sub = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        self.body_pose_sub = rospy.Subscriber('/quad_0/body_pose', Odometry, self.body_pose_cb, queue_size=1)
        rospy.loginfo('[tare_bridge] ready: /mid360_points(world) -> /registered_scan (frame=%s)',
                      self.world_frame)

    def body_pose_cb(self, msg):
        # 转发 base 位姿为 TARE 的 state_estimation（frame 对齐到 world_frame）
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.world_frame
        out.child_frame_id = 'base'
        out.pose = msg.pose
        out.twist = msg.twist
        self.state_pub.publish(out)

    def cloud_cb(self, msg):
        # 点云已是世界系（livox 插件直接输出），原样转发，无需坐标变换
        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.world_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        self.reg_pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('tare_bridge')
    TareBridge()
    rospy.spin()
