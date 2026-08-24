#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：幻影占据区域里到底有没有雷达点、z 在哪。
对比三块区域的单帧点云 z 直方图：
  P 区 = 北幻影块 x[-7.5,-4] y[9,13]
  Q 区 = 东幻影块 x[-3,-1] y[2,10]
  R 区 = 已正确建图区（对照）x[-8,-6] y[-0.5,3]
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

def parse(msg):
    dt = np.dtype({'names': [f.name for f in msg.fields],
                   'formats': [np.float32]*len(msg.fields),
                   'offsets': [f.offset for f in msg.fields],
                   'itemsize': msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt)

rospy.init_node('region_probe', disable_signals=True)
REGIONS = {'P北幻影': (-7.5,-4.0, 9.0,13.0), 'Q东幻影': (-3.0,-1.0, 2.0,10.0), 'R对照区': (-8.0,-6.0, -0.5,3.0)}
acc = {k: [] for k in REGIONS}
N = 30
for i in range(N):
    msg = rospy.wait_for_message('/mid360_points_clean', PointCloud2, timeout=20)
    pts = parse(msg)
    xy = np.stack([pts['x'], pts['y']], -1).astype(np.float64)
    z = pts['z'].astype(np.float64)
    for k,(x0,x1,y0,y1) in REGIONS.items():
        m = (xy[:,0]>=x0)&(xy[:,0]<=x1)&(xy[:,1]>=y0)&(xy[:,1]<=y1)
        acc[k].append(z[m])
print(f"采样 {N} 帧完成\n")
for k, zs in acc.items():
    z = np.concatenate(zs) if zs else np.array([])
    print(f"== {k} == 点数 {len(z)}")
    if len(z):
        bins = np.arange(0, 2.01, 0.2)
        h, edges = np.histogram(z, bins=bins)
        for c, e in zip(h, edges[:-1]):
            bar = '#' * int(60*c/max(h.max(),1))
            mark = ' <-- 占据带' if abs(e-0.2)<0.01 or abs(e-0.4)<0.01 or abs(e-0.6)<0.01 else ''
            print(f"  z[{e:.1f},{e+0.2:.1f}) {c:7d} {bar}{mark}")
        print(f"  z范围 [{z.min():.2f},{z.max():.2f}]\n")
