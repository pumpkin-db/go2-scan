#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
雷达射线取证探针（2026-08-25 Depot 排查）。
输出：帧数 / 点云去重数（关键多样性指标）/ 传感器平面命中数 / 模型列表。
用法：probe_cloud_diversity.py [采样秒数=20]
"""
import sys
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rospy.init_node('cloud_forensics', anonymous=True)
cap = {'c': [], 'p': None}
rospy.Subscriber('/mid360_points', PointCloud2, lambda m: cap['c'].append(m), queue_size=2)
rospy.Subscriber('/quad_0/lidar_pose', Odometry, lambda m: cap.__setitem__('p', m), queue_size=2)

try:
    ms = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
    print('模型:', ms.name)
except rospy.ROSException:
    print('模型: model_states 不可用')

t0 = rospy.get_rostime()
while (rospy.get_rostime() - t0).to_sec() < secs and len(cap['c']) < 30:
    rospy.sleep(0.2)

frames = cap['c']
print('帧数:', len(frames))
if not frames:
    sys.exit(0)
# 取中间一帧做逐字段统计（首帧可能撞上插件热身）
msg = frames[len(frames) // 2]
dt = np.dtype({'names': [f.name for f in msg.fields],
               'formats': [np.float32] * len(msg.fields),
               'offsets': [f.offset for f in msg.fields],
               'itemsize': msg.point_step})
arr = np.frombuffer(bytes(msg.data), dtype=dt)
xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
uniq3d = len(np.unique(xyz.round(3), axis=0))
print('点数 %d，3D 去重 %d' % (len(xyz), uniq3d))
if uniq3d > 1 and cap['p'] is not None:
    p = cap['p'].pose.pose.position
    d = np.linalg.norm(xyz[:, :2] - [p.x, p.y], axis=1)
    zspan = xyz[:, 2].max() - xyz[:, 2].min()
    ring = np.abs(xyz[:, 2] - (p.z)) < 0.4   # 与传感器同高度的环带=真「墙」命中
    print('水平距离 min/med/max: %.2f/%.2f/%.2f' % (d.min(), np.median(d), d.max()))
    print('z 跨度 %.2f m；传感器±0.4m 环带命中点数 %d' % (zspan, int(ring.sum())))
else:
    # 全部重合：给出那个点相对传感器的偏移
    if cap['p'] is not None:
        p = cap['p'].pose.pose.position
        off = xyz[0] - np.array([p.x, p.y, p.z])
        print('唯一点相对传感器偏移:', off.round(3).tolist())
