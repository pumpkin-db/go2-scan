#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 累积节点：把每帧 /mid360_points（已是世界系，frame=world，由 livox 插件直接输出）
            按距离过滤后体素累积，发布 /scan_map。

【纯累积版，add-only】—— 体素一旦被扫到就永久保留，不删除。

输入：/mid360_points（世界系点云，frame=world）
输出：/scan_map（累积体素点云，frame world，1Hz）
服务：/scan_map/save 保存、/scan_map/clear 清空
"""
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse


class ScanAccumulator:
    def __init__(self):
        self.pose = None  # 仅作距离过滤参考基准（不做坐标变换）
        self.voxel = rospy.get_param('~voxel_size', 0.05)
        self.max_range = rospy.get_param('~max_range', 12.0)
        self.min_range = rospy.get_param('~min_range', 0.3)
        self.publish_max_points = rospy.get_param('~publish_max_points', 100000)
        self.save_dir = rospy.get_param('~save_dir', '/tmp')
        self.cells = {}  # (ix,iy,iz) -> 1.0（纯累积，只加不删）

        self.pub = rospy.Publisher('/scan_map', PointCloud2, queue_size=1)
        self.sub_cloud = rospy.Subscriber('/mid360_points', PointCloud2, self.cloud_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber('/quad_0/lidar_pose', Odometry, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0), self.publish_cb)
        rospy.Service('/scan_map/save', Trigger, self.save_cb)
        rospy.Service('/scan_map/clear', Empty, self.clear_cb)
        rospy.loginfo('[scan_cloud_accumulator] ready: world-frame accumulate map, voxel=%.2fm', self.voxel)

    def pose_cb(self, msg):
        # 仅保存用作距离过滤基准（排除狗自身点），不用于坐标变换
        self.pose = msg.pose.pose

    def cloud_cb(self, msg):
        # 2026-08-26 改 numpy 直析：noetic pc2.read_points 对本链路点云存在
        # unpack_from 越界(struct.error)，逐帧异常导致积累量骤减（bench_fix2
        # 实证 scan 仅 4.9k 点）。三字段 step16 布局由 velodyne 插件保证。
        n = msg.width * msg.height
        if n == 0 or len(msg.data) < n * msg.point_step:
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
        xyz = arr[:, :12].copy().view(np.float32).reshape(n, 3)
        pts = xyz[~np.isnan(xyz).any(axis=1)].astype(np.float64)
        if len(pts) == 0:
            return
        # 点云已是世界系（livox 插件直接输出），无需坐标变换
        # 距离过滤基准：用 dog 传感器位置（若无则原点）
        if self.pose is not None:
            origin = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        else:
            origin = np.array([0.0, 0.0, 0.0])

        dist = np.linalg.norm(pts - origin, axis=1)
        mask = (dist >= self.min_range) & (dist <= self.max_range)
        pts_w = pts[mask]
        if len(pts_w) == 0:
            return

        # 体素累积（floor 正确处理负数坐标，add-only 只加不删）
        keys = np.floor(pts_w / self.voxel).astype(np.int64)
        for k in keys:
            key = (int(k[0]), int(k[1]), int(k[2]))
            if key not in self.cells:
                self.cells[key] = 1.0

    def publish_cb(self, _event):
        if not self.cells:
            return
        # 向量化（不用逐点列表推导式，避免大点云阻塞主线程）
        keys = np.array(list(self.cells.keys()), dtype=np.int64)
        if len(keys) > self.publish_max_points:
            sel = np.random.choice(len(keys), self.publish_max_points, replace=False)
            keys = keys[sel]
        pts = ((keys + 0.5) * self.voxel).astype(np.float32)
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        self.pub.publish(pc2.create_cloud_xyz32(header, pts))

    def save_cb(self, _req):
        try:
            keys = np.array(list(self.cells.keys()), dtype=np.int64)
            pts = ((keys + 0.5) * self.voxel).astype(np.float32)
            pcd_path = self.save_dir + '/scan_map_occupancy.pcd'
            with open(pcd_path, 'w') as f:
                f.write('# .PCD v0.7 - Point Cloud Data file format\n')
                f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
                f.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n' % len(pts))
                f.write('POINTS %d\nDATA ascii\n' % len(pts))
                for row in pts:
                    f.write('%.4f %.4f %.4f\n' % (row[0], row[1], row[2]))
            return TriggerResponse(success=True, message='saved %d points to %s' % (len(pts), pcd_path))
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))

    def clear_cb(self, _req):
        self.cells.clear()
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('scan_cloud_accumulator')
    ScanAccumulator()
    rospy.spin()
