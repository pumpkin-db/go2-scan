#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""同视场 A/B/A' 切换实验：停狗后 max_range 5→20→5，三次快照对比。
判定：若 5 时地面黑块多、20 时显著变干净、再回 5 又变差 → 截断端点机制实锤。"""
import subprocess, time, sys
import numpy as np, rospy
from nav_msgs.msg import OccupancyGrid

ENV = '/home/pumpkin-db/miniconda3/envs/ariadne/bin/python'
DYN = '/opt/ros/noetic/lib/dynamic_reconfigure/dynparam'

def set_range(v):
    subprocess.run([ENV, DYN, 'set', '/octomap', 'sensor_model_max_range', str(v)], check=True)

def snap(tag, ascii_on=False):
    m = rospy.wait_for_message('/projected_map', OccupancyGrid, timeout=25)
    g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    occ = int((g == 100).sum()); fre = int((g == 0).sum())
    print(f"\n[{tag}] 尺寸 {m.info.width}x{m.info.height} 占据 {occ} 格({occ*0.16:.0f}m²) 自由 {fre} 格")
    if ascii_on:
        for row in g[::-1]:
            print(' ' + ''.join('#' if v == 100 else ('.' if v == 0 else ' ') for v in row))
    return occ

rospy.init_node('toggle_probe', disable_signals=True, anonymous=True)
print("== 阶段1: max=5 稳定观测 ==")
set_range(5); time.sleep(15)
a1 = snap('5-第一次', ascii_on=True)
print("\n== 阶段2: 切到 max=20 ==")
set_range(20); time.sleep(15)
b = snap('20', ascii_on=True)
print("\n== 阶段3: 切回 max=5 ==")
set_range(5); time.sleep(15)
a2 = snap('5-第二次', ascii_on=True)
print(f"\n=== 结论数据：5时占据={a1}/{a2} 格，20时占据={b} 格 ===")
