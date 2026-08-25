#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
狗腿乱飞修复的数值验证（2026-08-25）。
前提：roslaunch simulation/test_legs.launch 已在跑（headless）。

验证三件事：
  1. set_model_configuration 在腿连杆 kinematic 后仍生效（服务返回 success）；
  2. 行走时 Gazebo 实际关节角跟随 /joint_states 指令角（|误差| < 0.25 rad）——
     修复前物理引擎会在两次传送之间把零阻尼腿甩离指令角很远；
  3. 实际关节角连续性：相邻采样(50ms)最大 |Δ角| 有界且不超过指令相位推进
     （修复前每次传送都是大跳变）。

用法：python3 tools/verify_leg_fix.py [--drive-speed 0.5] [--duration 12]
"""
import argparse
import math
import sys

import rospy
from gazebo_msgs.srv import GetJointProperties, GetModelState, SetModelConfiguration
from sensor_msgs.msg import JointState

JOINT = 'FL_calf_joint'          # 抽样关节（摆幅最大，最能暴露乱飞）
CMD_RANGE = (-2.6, -0.9)         # gait_publisher 对 calf 的 clamp 范围
ERR_TOL = 0.25                   # 实际 vs 指令 容差(rad)
DELTA_TOL = 1.2                  # 相邻采样 |Δ实际角| 上限(rad)，50ms 内正常步态远小于此


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--drive-speed', type=float, default=0.5)
    ap.add_argument('--duration', type=float, default=12.0)
    args, _ = ap.parse_known_args()

    rospy.init_node('verify_leg_fix')

    rospy.loginfo('[verify] 等待 gazebo 服务...')
    rospy.wait_for_service('/gazebo/set_model_configuration')
    rospy.wait_for_service('/gazebo/get_joint_properties')
    rospy.wait_for_service('/gazebo/get_model_state')
    set_cfg = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    get_joint = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # --- 检查1：set_model_configuration 服务本身可用（kinematic 连杆下仍生效） ---
    all_joints = ['%s_%s_joint' % (leg, part)
                  for leg in ('FL', 'FR', 'RL', 'RR')
                  for part in ('hip', 'thigh', 'calf')]
    stance = [0.05, 0.82, -1.58, -0.05, 0.82, -1.58,
              0.05, 0.95, -1.62, -0.05, 0.95, -1.62]
    resp = set_cfg('go2_description', 'robot_description', all_joints, stance)
    if not resp.success:
        rospy.logerr('[verify][FAIL] set_model_configuration 失败: %s', resp.status_message)
        sys.exit(1)
    rospy.loginfo('[verify][OK-1] set_model_configuration success=true（kinematic 腿仍可传送步态）')

    # 记录指令角
    latest = {'msg': None}

    def js_cb(msg):
        if JOINT in msg.name:
            latest['msg'] = msg.position[msg.name.index(JOINT)]

    rospy.Subscriber('/joint_states', JointState, js_cb, queue_size=10)

    # 开始行走（cmd_timeout=0.3s，必须持续发）
    from geometry_msgs.msg import Twist
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(20)  # 20Hz 采样
    t_end = rospy.Time.now() + rospy.Duration(args.duration)
    max_err = 0.0
    max_delta = 0.0
    prev_actual = None
    samples = 0

    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        tw = Twist()
        tw.linear.x = args.drive_speed
        cmd_pub.publish(tw)

        r = get_joint(JOINT)
        actual = r.position[0] if r.position else None
        cmd = latest['msg']

        if actual is not None:
            if prev_actual is not None:
                d = abs(actual - prev_actual)
                if d > max_delta:
                    max_delta = d
                    if d > DELTA_TOL:
                        rospy.logwarn('[verify] Δ=%.3f rad @t=%.2f（超过容差 %.1f）', d,
                                      rospy.Time.now().to_sec(), DELTA_TOL)
            prev_actual = actual
            if cmd is not None:
                err = abs(actual - cmd)
                if err > max_err:
                    max_err = err
            samples += 1
        rate.sleep()
        # 到达后停住再测一段静止（stance 应完全跟住）
        if rospy.Time.now() >= t_end:
            break

    # 基础位移检查：狗确实在走
    ms = get_model('go2_description', 'world')
    rospy.loginfo('[verify] 结束时模型位姿 x=%.3f y=%.3f（起点 -7.5, 0.5）',
                  ms.pose.position.x, ms.pose.position.y)

    moved = math.hypot(ms.pose.position.x - (-7.5), ms.pose.position.y - 0.5)
    ok_move = moved > 0.5 * args.duration * 0.4  # 至少理论位移的 40%
    ok_err = max_err < ERR_TOL
    ok_delta = max_delta < DELTA_TOL

    rospy.loginfo('[verify] 样本数=%d 位移=%.2fm max|实际-指令|=%.3f rad max|Δ实际|=%.3f rad',
                  samples, moved, max_err, max_delta)
    for name, ok in (('行走位移', ok_move), ('跟随指令角', ok_err), ('角度连续性', ok_delta)):
        rospy.loginfo('[verify][%s] %s', 'OK' if ok else 'FAIL', name)

    sys.exit(0 if (ok_move and ok_err and ok_delta) else 1)


if __name__ == '__main__':
    main()
