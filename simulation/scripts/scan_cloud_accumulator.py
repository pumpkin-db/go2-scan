#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 累积节点：把每帧 /mid360_points 变换到世界系并体素累积，发布 /scan_map。
- 订阅 /mid360_points（传感器坐标）+ /quad_0/lidar_pose（LiDAR 世界系位姿）
- 每帧变换到世界系，体素(0.05m)降采样累积
- 1Hz 发布累积点云到 /scan_map（frame world）

用途：RViz 里对照「狗实际扫到的点云(scan_map)」vs「场景完整点云(map)」vs「占据地图(occ_map)」。
"""
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix


class ScanAccumulator:
    def __init__(self):
        self.pose = None
        self.voxel = rospy.get_param('~voxel_size', 0.05)
        self.accum = {}  # (ix,iy,iz) -> (x,y,z)
        self.pub = rospy.Publisher('/scan_map', PointCloud2, queue_size=1)
        self.sub_cloud = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber('/quad_0/lidar_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_cb)
        rospy.loginfo('[scan_cloud_accumulator] ready: /mid360_points -> /scan_map (voxel=%.2fm)', self.voxel)

    def pose_cb(self, msg):
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        if self.pose is None:
            return
        pts = np.array([[p[0], p[1], p[2]]
                        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)],
                       dtype=np.float64)
        if len(pts) == 0:
            return
        # 变换到世界系
        p = self.pose.position
        q = self.pose.orientation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        pts_w = pts @ R.T + np.array([p.x, p.y, p.z])

        # 体素降采样累积（逐点，室内规模点云可接受）
        keys = (pts_w / self.voxel).astype(np.int64)
        for i in range(len(pts_w)):
            k = (keys[i, 0], keys[i, 1], keys[i, 2])
            if k not in self.accum:
                self.accum[k] = pts_w[i]

    def publish_cb(self, _event):
        if not self.accum:
            return
        pts = np.array(list(self.accum.values()), dtype=np.float32)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        cloud = pc2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)


if __name__ == '__main__':
    rospy.init_node('scan_cloud_accumulator')
    ScanAccumulator()
    rospy.spin()
