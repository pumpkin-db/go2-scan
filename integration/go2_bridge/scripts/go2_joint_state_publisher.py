#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""go2_joint_state_publisher — 固定站姿 /joint_states 发布器（Track A1.5 修复）

根因（2026-08-29 A1 诊断）：主栈 gazebo_sim.launch 无任何 /joint_states 发布者
（ModelPlugin 只钉 Gazebo 关节不动 ROS 话题；go2_gait_publisher 只在
test_legs.launch 里），robot_state_publisher 拿不到 12 个 revolute 腿关节角，
导致 RViz RobotModel 全部腿部 link 报 "No transform from [...] to [world]"。

本节点持续发布完整固定站姿（与 ModelPlugin CacheStandingJoints / spawn -J
完全同值），使 RViz=Gazebo 腿姿一致。腿关节在 Gazebo 侧由 ModelPlugin 每
tick 钉死，本话题纯服务 TF/RViz，绝不可被用于驱动 Gazebo 关节
（禁止 set_model_configuration 链路，见 ZCODE 任务 A1.6）。
"""

import xml.etree.ElementTree as ET

import rospy
from sensor_msgs.msg import JointState

# 站姿与 spawn_go2 -J 参数、go2_kinematic_model_plugin CacheStandingJoints 三处同源
STANCE = [
    ("FL_hip_joint", 0.05), ("FL_thigh_joint", 0.82), ("FL_calf_joint", -1.58),
    ("FR_hip_joint", -0.05), ("FR_thigh_joint", 0.82), ("FR_calf_joint", -1.58),
    ("RL_hip_joint", 0.05), ("RL_thigh_joint", 0.95), ("RL_calf_joint", -1.62),
    ("RR_hip_joint", -0.05), ("RR_thigh_joint", 0.95), ("RR_calf_joint", -1.62),
]


def movable_joint_names_from_description():
    """从 /robot_description 解析全部非 fixed joint 名，校验发布名集一致。"""
    try:
        desc = rospy.get_param("/robot_description")
        root = ET.fromstring(desc)
        return {j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"}
    except Exception as exc:  # 描述未就绪或解析失败只降级告警，不阻塞发布
        rospy.logwarn("[go2_joint_state_publisher] robot_description 校验跳过: %s", exc)
        return None


def main():
    rospy.init_node("go2_joint_state_publisher")
    rate_hz = rospy.get_param("~rate", 50.0)
    joint_topic = rospy.get_param("~joint_topic", "/joint_states")

    names = [n for n, _ in STANCE]
    expected = movable_joint_names_from_description()
    if expected is not None:
        if set(names) == expected:
            rospy.loginfo("[go2_joint_state_publisher] 关节名与 robot_description 完全一致 (%d)", len(names))
        else:
            rospy.logwarn("[go2_joint_state_publisher] 关节名不一致! publisher=%s urdf_movable=%s",
                          sorted(set(names) ^ expected) and sorted(set(names)), sorted(expected))

    pub = rospy.Publisher(joint_topic, JointState, queue_size=10)
    rate = rospy.Rate(rate_hz)
    msg = JointState()
    msg.name = names
    msg.position = [p for _, p in STANCE]
    msg.velocity = [0.0] * len(names)
    msg.effort = [0.0] * len(names)
    rospy.loginfo("[go2_joint_state_publisher] 开始 @%.0fHz -> %s (固定站姿, 纯 TF/RViz 用)", rate_hz, joint_topic)
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
