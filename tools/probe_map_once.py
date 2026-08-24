#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对任意 /projected_map 拍一次快照（10 秒窗口取最新，绕开 latch）。
用法：python probe_map_once.py [输出标注]"""
import sys, time
import numpy as np, rospy
from nav_msgs.msg import OccupancyGrid

tag = sys.argv[1] if len(sys.argv) > 1 else 'snapshot'
latest = {}
def cb(m): latest['m'] = m

rospy.init_node('map_once', disable_signals=True, anonymous=True)
sub = rospy.Subscriber('/projected_map', OccupancyGrid, cb, queue_size=2)
t0 = time.time()
while time.time() - t0 < 10:
    time.sleep(0.3)

if 'm' not in latest:
    print(f"[{tag}] 10 秒内无 projected_map"); sys.exit(1)
m = latest['m']
g = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
ox, oy = m.info.origin.position.x, m.info.origin.position.y
occ = int((g == 100).sum()); fre = int((g == 0).sum())
print(f"[{tag}] origin({ox:.1f},{oy:.1f}) 尺寸 {m.info.width}x{m.info.height} 占据 {occ} 格 自由 {fre} 格")
for row in g[::-1]:
    print(' ' + ''.join('#' if v == 100 else ('.' if v == 0 else ' ') for v in row))
