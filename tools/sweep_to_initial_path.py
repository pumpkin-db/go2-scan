#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue：pythonRobotics 弓形覆盖 -> SCAN-Planner mode3 的 /initial_path。
用法（系统 python3，有 rospy）：
  /usr/bin/python3 sweep_to_initial_path.py [x_min x_max y_min y_max] [row_res]
默认矩形对齐 map1 墙场景的内侧自由区。只组合不写算法。
"""
import sys
import importlib.util

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

PYROB_ROOT = "/home/pumpkin-db/claude/raicom/try_algorithm/code/pythonRobotics"
sys.path.insert(0, PYROB_ROOT)

_spec = importlib.util.spec_from_file_location(
    "sweep_cov", PYROB_ROOT + "/PathPlanning/GridBasedSweepCPP/grid_based_sweep_coverage_path_planner.py")
sweep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sweep)


def main():
    xmin, xmax, ymin, ymax = [float(v) for v in sys.argv[1:5]] if len(sys.argv) >= 5 else (-6.0, 24.0, -6.0, 1.0)
    res = float(sys.argv[5]) if len(sys.argv) >= 6 else 1.2

    ox = [xmin, xmax, xmax, xmin]
    oy = [ymin, ymin, ymax, ymax]
    rx, ry = sweep.planning(ox, oy, res)
    print(f"[sweep->initial_path] rect=({xmin},{xmax},{ymin},{ymax}) res={res} pts={len(rx)}")

    rospy.init_node("sweep_to_initial_path")
    pub = rospy.Publisher("/initial_path", Path, latch=True, queue_size=1)
    rospy.sleep(1.0)
    msg = Path()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    for x, y in zip(rx, ry):
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0   # SCAN-Planner pathCallback 会自己加 body_height
        p.pose.orientation.w = 1.0
        msg.poses.append(p)
    pub.publish(msg)
    print(f"[sweep->initial_path] published /initial_path with {len(msg.poses)} poses (latched, node stays alive)")
    rospy.spin()


if __name__ == "__main__":
    main()
