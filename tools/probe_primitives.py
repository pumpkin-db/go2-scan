#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断第二刀：
1. 场景 primitive(unit_*) 的位置/尺寸 vs 雷达点聚集 —— 它们是不是幻影格的真身
2. GT pcd 在每个 primitive 位置有没有点 —— 是不是 GT 缺失导致误判"幻影"
3. P 区带内点的 0.4m 格密度图，与 projected_map 幻影格形状比对
"""
import numpy as np, rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sys
sys.path.insert(0, '/home/pumpkin-db/claude/raicom/go2-scan/tools')
from probe_occ_vs_gt import load_pcd_xyz

PRIMS = {  # 来自 indoor_1.world：<pose> x y z；单位尺寸按 gazebo 原语默认（box 1m 立方、cylinder r0.5 h1、sphere r0.5）
    'unit_sphere':      (-4.28858, 4.61514),
    'unit_box':         (-6.68269, 10.9796),
    'unit_cylinder':    (-5.57918, 7.959),
    'unit_cylinder_0':  (-6.8772, 5.20572),
    'unit_box_0':       (-5.11356, 15.289),
}

def parse(msg):
    dt = np.dtype({'names':[f.name for f in msg.fields],'formats':[np.float32]*len(msg.fields),'offsets':[f.offset for f in msg.fields],'itemsize':msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt)

rospy.init_node('prim_probe', disable_signals=True)
gt = load_pcd_xyz('/home/pumpkin-db/claude/raicom/go2-scan/maps/indoor_1.pcd')

print("== primitive 位置的 GT 覆盖检查（半径0.7m内任意z的点数）==")
for name,(px,py) in PRIMS.items():
    d2 = (gt[:,0]-px)**2 + (gt[:,1]-py)**2
    n_any = int((d2 < 0.7**2).sum())
    m = d2 < 0.7**2
    n_band = int(((gt[m,2]>=0.2)&(gt[m,2]<=0.8)).sum())
    print(f"  {name:18s} ({px:.2f},{py:.2f})  GT任意z: {n_any:6d}  GT带内: {n_band}")

# 采点云画 P 区带内密度
REG = (-8.0, -1.0, 1.0, 13.5)
acc=[]
for i in range(40):
    msg = rospy.wait_for_message('/mid360_points_clean', PointCloud2, timeout=20)
    pts = parse(msg)
    xy = np.stack([pts['x'],pts['y']],-1).astype(np.float64); z=pts['z'].astype(np.float64)
    band = (z>=0.2)&(z<=0.8)
    m = band&(xy[:,0]>=REG[0])&(xy[:,0]<=REG[1])&(xy[:,1]>=REG[2])&(xy[:,1]<=REG[3])
    acc.append(xy[m])
P = np.concatenate(acc)
print(f"\n== P 扩展区带内点 {len(P)} 个，0.4m 格密度图（数字=40帧累计点数对数量级 0-9）==")
x0,x1,y0,y1 = REG
nx, ny = int((x1-x0)/0.4), int((y1-y0)/0.4)
H,_,_ = np.histogram2d(P[:,0], P[:,1], bins=[nx,ny], range=[[x0,x1],[y0,y1]])
for r in range(ny)[::-1]:
    line=''
    for c in range(nx):
        v=H[c,r]
        line += '.' if v<10 else ('%x' % min(int(np.log2(max(v,1))),15)) if v>=10 else ' '
    print(line)
print("列=x从-8.0起每格0.4m；行=y从13.5往下每格0.4m")
