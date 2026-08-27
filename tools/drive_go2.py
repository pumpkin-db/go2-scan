#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""比例导航遥控驱动（B 验收/联调用胶水）：把狗推向目标点后退出。

控制律为经典 P 控制（转向-前进解耦，大角误差先原地转向），非自造算法。
位姿源: /quad_0/body_pose（nav_msgs/Odometry，go2_kinematic_sim 发布）
指令出: /cmd_vel（geometry_msgs/Twist，closed_loop_controller 同款通道）

用法（必须 /usr/bin/python3，env python3 会落到无 numpy 的 miniconda——本脚本仅用 stdlib+rospy，
但统一口径防呆）:
  /usr/bin/python3 tools/drive_go2.py --target 12.87 0.40 --max-v 0.25
退出码: 0=到达 | 1=收不到位姿 | 2=超时
"""
import argparse
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', nargs=2, type=float, required=True, metavar=('X', 'Y'))
    ap.add_argument('--tol', type=float, default=0.35)
    ap.add_argument('--max-v', type=float, default=0.35)
    ap.add_argument('--max-w', type=float, default=0.8)
    ap.add_argument('--kw', type=float, default=1.6)
    ap.add_argument('--slow-dist', type=float, default=1.5, help='距目标小于此值开始线性减速')
    ap.add_argument('--turn-gate', type=float, default=1.2, help='航向误差大于此(rad)时先原地转向')
    ap.add_argument('--timeout', type=float, default=240.0)
    a = ap.parse_args()

    rospy.init_node('drive_go2')
    tx, ty = a.target
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    state = {'msg': None}
    rospy.Subscriber('/quad_0/body_pose', Odometry,
                     lambda m: state.__setitem__('msg', m), queue_size=5)

    rate = rospy.Rate(20)
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        m = state['msg']
        now = rospy.Time.now().to_sec()
        if m is None:
            if now - t0 > 10.0:
                print('ERR: 10s 未收到 /quad_0/body_pose')
                return 1
            rate.sleep()
            continue
        pos = m.pose.pose.position
        yaw = yaw_from_quat(m.pose.pose.orientation)
        dx, dy = tx - pos.x, ty - pos.y
        dist = math.hypot(dx, dy)
        if dist < a.tol:
            print('OK reached (%.2f, %.2f) dist=%.2f' % (pos.x, pos.y, dist))
            break
        err = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                         math.cos(math.atan2(dy, dx) - yaw))
        w = max(-a.max_w, min(a.max_w, a.kw * err))
        if abs(err) > a.turn_gate:
            v = 0.0                                   # 先转向再走，避免画大圈
        else:
            v = a.max_v * min(dist / a.slow_dist, 1.0)
        c = Twist()
        c.linear.x = v
        c.angular.z = w
        cmd_pub.publish(c)
        if now - t0 > a.timeout:
            print('TIMEOUT at (%.2f, %.2f) dist=%.2f' % (pos.x, pos.y, dist))
            cmd_pub.publish(Twist())
            return 2
        rate.sleep()
    cmd_pub.publish(Twist())                          # 收车归零
    return 0


if __name__ == '__main__':
    sys.exit(main())
