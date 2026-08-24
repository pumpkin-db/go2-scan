#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同视场 A/B/A' 切换实验 v2：正确采样（10 秒窗口取最新，绕开 latch 缓存）。"""
import subprocess, time
import numpy as np, rospy
from nav_msgs.msg import OccupancyGrid

ENV = '/home/pumpkin-db/miniconda3/envs/ariadne/bin/python'
DYN = '/opt/ros/noetic/lib/dynamic_reconfigure/dynparam'
latest = {}

def cb(msg):
    latest['msg'] = msg

def set_range(v):
    subprocess.run([ENV, DYN, 'set', '/octomap', 'sensor_model_max_range', str(v)], check=True)

def snap(tag, ascii_on=False, window=12):
    latest.clear()
    sub = rospy.Subscriber('/projected_map', OccupancyGrid, cb, queue_size=2)
    t0 = time.time()
    while time.time() - t0 < window:
        time.sleep(0.5)
    sub.unregister()
    m = latest.get('msg')
    g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    occ = int((g == 100).sum()); fre = int((g == 0).sum())
    print(f"\n[{tag}] 尺寸 {m.info.width}x{m.info.height} 占据 {occ} 格 自由 {fre} 格")
    if ascii_on:
        for row in g[::-1]:
            print(' ' + ''.join('#' if v == 100 else ('.' if v == 0 else ' ') for v in row))
    return occ

rospy.init_node('toggle_probe2', disable_signals=True, anonymous=True)
set_range(5); time.sleep(3)
a1 = snap('max=5', ascii_on=True)
set_range(20); b = snap('max=20', ascii_on=True)
set_range(5); a2 = snap('max=5回切', ascii_on=True)
print(f"\n=== 占据格：5→{a1}  20→{b}  回5→{a2} ===")
