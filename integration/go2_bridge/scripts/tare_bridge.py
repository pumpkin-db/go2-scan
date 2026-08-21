#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 配准节点：把 Gazebo 的传感器系点云配准到世界系，发布给 TARE 探索决策层。

TARE 需要（frame 都硬编码 kWorldFrameID="map"）：
- /registered_scan         : 世界系点云（PointXYZ），配准后的激光
- /state_estimation_at_scan: 世界系 base 位姿（Odometry）

本节点：
- 订阅 /mid360_points（传感器系）+ /quad_0/lidar_pose（传感器世界系位姿）→ 变换到世界系 → /registered_scan
- 订阅 /quad_0/body_pose（base 世界系位姿）→ 转发为 /state_estimation_at_scan

（仿真里没有 FAST-LIO2，Gazebo livox 插件只出传感器系 /mid360_points，所以要配准）
"""
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix


class TareBridge:
    def __init__(self):
        self.lidar_pose = None
        self.world_frame = rospy.get_param('~world_frame', 'world')

        self.reg_pub = rospy.Publisher('/registered_scan', PointCloud2, queue_size=1)
        self.state_pub = rospy.Publisher('/state_estimation_at_scan', Odometry, queue_size=1)
        self.cloud_sub = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        self.lidar_pose_sub = rospy.Subscriber('/quad_0/lidar_pose', Odometry, self.lidar_pose_cb, queue_size=1)
        self.body_pose_sub = rospy.Subscriber('/quad_0/body_pose', Odometry, self.body_pose_cb, queue_size=1)
        rospy.loginfo('[tare_bridge] ready: /mid360_points + /quad_0/lidar_pose -> /registered_scan (frame=%s)',
                      self.world_frame)

    def lidar_pose_cb(self, msg):
        self.lidar_pose = msg.pose.pose

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
        if self.lidar_pose is None:
            return
        pts = np.array([[p[0], p[1], p[2]]
                        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=np.float64)
        if len(pts) == 0:
            return
        # 传感器系 → 世界系（用 lidar_pose 的位姿）
        p = self.lidar_pose.position
        q = self.lidar_pose.orientation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        pts_w = pts @ R.T + np.array([p.x, p.y, p.z])

        header = rospy.Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.world_frame
        self.reg_pub.publish(pc2.create_cloud_xyz32(header, pts_w.astype(np.float32)))


if __name__ == '__main__':
    rospy.init_node('tare_bridge')
    TareBridge()
    rospy.spin()
