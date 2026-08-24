#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""终极探针：订阅 /octomap_point_clouds（octomap 占据体素中心集合），
统计幻影格与真支撑格里的体素高度分布。
用法：python probe_voxel_heights.py > 输出文件"""
import time
import numpy as np, rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

latest = {}
def cb(m): latest['m'] = m

rospy.init_node('voxel_probe', disable_signals=True)

# 当前地图的占据格
sub = rospy.Subscriber('/projected_map', OccupancyGrid, cb, queue_size=2)
t0 = time.time()
while time.time()-t0 < 6 or 'm' not in latest: time.sleep(0.3)
g = np.array(latest['m'].data, dtype=np.int8).reshape(latest['m'].info.height, latest['m'].info.width)
ox, oy, res = latest['m'].info.origin.position.x, latest['m'].info.origin.position.y, latest['m'].info.resolution
sub.unregister()

# 订阅体素云（触发 octomap 开始发布）
latest.clear()
sub2 = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, cb, queue_size=2)
t0 = time.time()
while time.time()-t0 < 15:
    if 'm' in latest and time.time()-t0 > 4: break
    time.sleep(0.3)

if 'm' not in latest:
    print("octomap_point_clouds 无输出"); raise SystemExit
m = latest['m']
dt = np.dtype({'names':[f.name for f in m.fields],'formats':[np.float32]*len(m.fields),'offsets':[f.offset for f in m.fields],'itemsize':m.point_step})
a = np.frombuffer(bytes(m.data), dtype=dt)
vx = a['x'].astype(np.float64); vy = a['y'].astype(np.float64); vz = a['z'].astype(np.float64)
print(f"占据体素总数 {len(vx)}")
print(f"全体素高度直方图:")
h, edges = np.histogram(vz, bins=np.arange(-0.6, 3.01, 0.4))
for c, e in zip(h, edges):
    print(f"  z[{e:+.1f},{e+0.4:+.1f}) {c:6d} {'#'*int(60*c/max(h.max(),1))}")

# 幻影格样例位置（来自 run7_gt：东块 x[-3,-0.6] y[1,4.5]，北块 x[-7.8,-5] y[9,13]）
ZONES = {'东幻影块': (-3.0,-0.6,1.0,4.5), '北幻影块': (-7.8,-5.0,9.0,13.0)}
for name,(x0,x1,y0,y1) in ZONES.items():
    mm = (vx>=x0)&(vx<=x1)&(vy>=y0)&(vy<=y1)
    zz = vz[mm]
    print(f"\n{name}: 体素 {mm.sum()} 个, 高度分布:")
    if len(zz):
        h2, e2 = np.histogram(zz, bins=np.arange(-0.6, 3.01, 0.4))
        for c, e in zip(h2, e2):
            print(f"  z[{e:+.1f},{e+0.4:+.1f}) {c:5d}")
