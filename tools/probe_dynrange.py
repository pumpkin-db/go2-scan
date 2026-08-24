#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时 A/B：octomap max_range 5→20，观察地面幻影是否消失。
用法：python probe_dynrange.py [新max_range]"""
import sys, subprocess, time
import numpy as np, rospy
from nav_msgs.msg import OccupancyGrid

NEW_RANGE = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
ENV = '/home/pumpkin-db/miniconda3/envs/ariadne/bin/python'
DYN = '/opt/ros/noetic/lib/dynamic_reconfigure/dynparam'

def snap(tag):
    rospy.init_node('dyn_probe', disable_signals=True, anonymous=True)
    m = rospy.wait_for_message('/projected_map', OccupancyGrid, timeout=20)
    g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    occ = int((g == 100).sum())
    print(f"[{tag}] 地图 {m.info.width}x{m.info.height} 占据格 {occ} ({occ*m.info.resolution**2:.1f} m²)")
    for row in g[::-1]:
        print(' ' + ''.join('#' if v == 100 else ('.' if v == 0 else ' ') for v in row))
    return occ

rospy.init_node('dyn_probe', disable_signals=True, anonymous=True)
snap('基线 max=5')
subprocess.run([ENV, DYN, 'set', '/octomap', 'sensor_model_max_range', str(NEW_RANGE)], check=True)
print(f"\n>>> 已动态设置 max_range={NEW_RANGE}，等 12 秒累积新观测...\n")
time.sleep(12)
occ_new = snap(f'实验 max={NEW_RANGE:g}')
