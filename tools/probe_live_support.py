#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""终极定位：projected_map 的占据格 vs 实时雷达带内点，逐格对照（不依赖GT文件）。
输出：有实时支撑的占据格数 / 零支撑(幽灵)占据格数，及幽灵格的分布图。
用法：python probe_live_support.py > 输出文件"""
import numpy as np, rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

latest = {}
def cb(m): latest['m'] = m

rospy.init_node('live_support', disable_signals=True)

# 1) 取当前地图
sub = rospy.Subscriber('/projected_map', OccupancyGrid, cb, queue_size=2)
t0 = time.time() if False else None
import time
t0 = time.time()
while time.time()-t0 < 6 or 'm' not in latest:
    time.sleep(0.3)
g = np.array(latest['m'].data, dtype=np.int8).reshape(latest['m'].info.height, latest['m'].info.width)
ox, oy, res = latest['m'].info.origin.position.x, latest['m'].info.origin.position.y, latest['m'].info.resolution
occ_cells = np.argwhere(g == 100)
print(f"地图 {g.shape[1]}x{g.shape[0]} 占据格 {len(occ_cells)}")

# 2) 连续采 60 帧 clean 点云，累积带内点的格子直方图
cell_cnt = {}
N = 60
for i in range(N):
    m = rospy.wait_for_message('/mid360_points_clean', PointCloud2, timeout=20)
    dt = np.dtype({'names':[f.name for f in m.fields],'formats':[np.float32]*len(m.fields),'offsets':[f.offset for f in m.fields],'itemsize':m.point_step})
    a = np.frombuffer(bytes(m.data), dtype=dt)
    x = a['x'].astype(np.float64); y = a['y'].astype(np.float64); z = a['z'].astype(np.float64)
    band = (z >= 0.2) & (z <= 0.8)
    cx = ((x[band]-ox)/res).astype(int); cy = ((y[band]-oy)/res).astype(int)
    for cxx, cyy in zip(cx, cy):
        key = (cyy, cxx)
        cell_cnt[key] = cell_cnt.get(key, 0) + 1

# 3) 逐个占据格判定
supported, ghost = [], []
for r, c in occ_cells:
    n = cell_cnt.get((r, c), 0)
    (supported if n >= 15 else ghost).append((r, c, n))
print(f"\n{N}帧累计后：有实时带内支撑的占据格 {len(supported)}，零/微支撑(幽灵) {len(ghost)}")
gs = sorted(ghost, key=lambda t: -t[2])
print("幽灵格样例(r,c,60帧带内点数)：", gs[:15])

# 4) 幽灵分布图
occ_set = {(int(r), int(c)) for r,c,_ in ghost}
sup_set = {(int(r), int(c)) for r,c,_ in supported}
print("\n== 分布图（P=幽灵占据 #=有支撑占据 .=自由 空=未知）==")
for rr in range(g.shape[0])[::-1]:
    line = ''
    for cc in range(g.shape[1]):
        if (rr,cc) in occ_set: line += 'P'
        elif (rr,cc) in sup_set: line += '#'
        else: line += '.' if g[rr,cc]==0 else ' '
    print(line)
