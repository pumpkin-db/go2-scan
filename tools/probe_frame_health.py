#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断第三刀：连续监测原始 /mid360_points 每帧健康度，抓 livox 退化帧现行。
健康帧特征：点数稳定（~万级）、z 分布宽（场景结构）、xy 分布广。
退化帧特征：点数骤降 / 全部贴脸（近距）/ z 塌成单值 / 平环。
"""
import numpy as np, rospy
from sensor_msgs.msg import PointCloud2

def parse(msg):
    dt = np.dtype({'names':[f.name for f in msg.fields],'formats':[np.float32]*len(msg.fields),'offsets':[f.offset for f in msg.fields],'itemsize':msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt)

rospy.init_node('frame_health', disable_signals=True)
print(f"{'帧序':>4} {'点数':>7} {'近距<1m%':>9} {'z_std':>6} {'xy半径P95':>9}  判定")
bad = 0
for i in range(150):
    try:
        msg = rospy.wait_for_message('/mid360_points', PointCloud2, timeout=15)
    except Exception:
        print(f"{i:>4} 超时无数据"); continue
    p = parse(msg)
    n = len(p)
    xy = np.stack([p['x'],p['y']],-1).astype(np.float64)
    z = p['z'].astype(np.float64)
    r = np.hypot(xy[:,0], xy[:,1])
    near = float((r < 1.0).mean()) * 100 if n else 100
    zstd = float(z.std()) if n else 0
    p95 = float(np.percentile(r, 95)) if n else 0
    verdict = 'OK'
    if n < 2000 or near > 50 or zstd < 0.05 or p95 < 1.5:
        verdict = '<<< 疑似退化帧'; bad += 1
    if i % 10 == 0 or verdict != 'OK':
        print(f"{i:>4} {n:>7} {near:>8.1f}% {zstd:>6.2f} {p95:>9.1f}  {verdict}")
print(f"\n共 150 帧，疑似退化 {bad} 帧")
