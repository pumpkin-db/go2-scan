#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 桥节点：把 go2_kinematic_sim 的狗位姿同步到 Gazebo（set_model_state），
并发布 LiDAR 的 sensor_pose 给 SCAN-Planner。
- 订阅 /quad_0/body_pose (nav_msgs/Odometry)
- 写 /gazebo/set_model_state（把 Gazebo 里 spawn 的狗模型同步到位，LiDAR 跟着动）
- 发布 /quad_0/lidar_pose (nav_msgs/Odometry) 作为 SCAN-Planner 的 sensor_pose

对照 CMU vehicleSimulator.cpp 里 pubModelState -> /gazebo/set_model_state 的做法。
"""
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelConfiguration
import numpy as np
from tf.transformations import quaternion_matrix


class GazeboBridge:
    def __init__(self):
        self.model_name = rospy.get_param('~model_name', 'go2_description')
        # LiDAR 相对 base 的外参：mount (0.2, 0, 0.17) + MID360 scan joint (0,0,0.0377) = (0.2, 0, 0.2077)
        self.lidar_x = rospy.get_param('~lidar_x', 0.2)
        self.lidar_z = rospy.get_param('~lidar_z', 0.2077)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # 腿关节步态同步（让 Gazebo 狗腿像 RViz 一样动）
        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.set_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.last_joint_sync = 0.0
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_cb, queue_size=1)

        self.sensor_pose_pub = rospy.Publisher('/quad_0/lidar_pose', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('/quad_0/body_pose', Odometry, self.body_cb, queue_size=10)
        rospy.loginfo('[gazebo_bridge] ready: /quad_0/body_pose -> gazebo model %s + /quad_0/lidar_pose + leg gait',
                      self.model_name)

    def joint_cb(self, msg):
        # 30Hz 节流（go2_gait_publisher 发 60Hz，set_model_configuration 是 service 调，别太频）
        now = rospy.Time.now().to_sec()
        if now - self.last_joint_sync < 1.0 / 30.0:
            return
        self.last_joint_sync = now
        # 只取 12 个 revolute 腿关节（hip/thigh/calf），顺序按 joint_states 原样
        names, positions = [], []
        for i, n in enumerate(msg.name):
            if n.endswith('hip_joint') or n.endswith('thigh_joint') or n.endswith('calf_joint'):
                names.append(n)
                positions.append(msg.position[i])
        if not names:
            return
        try:
            self.set_config(self.model_name, 'robot_description', names, positions)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(10, 'set_model_configuration fail: %s', e)

    def body_cb(self, msg):
        # 1) 同步 Gazebo 狗模型位姿
        ms = ModelState()
        ms.model_name = self.model_name
        ms.pose = msg.pose.pose
        ms.twist = msg.twist.twist
        ms.reference_frame = 'world'
        try:
            self.set_state(ms)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(10, 'set_model_state fail: %s', e)

        # 2) 发布 sensor_pose（LiDAR 世界系位姿）
        #    lidar 相对 base 的外参 (lidar_x, 0, lidar_z) 要从 base 系旋转到世界系（不能只加 z）
        sp = Odometry()
        sp.header = msg.header
        sp.child_frame_id = 'mid360'
        sp.pose = msg.pose
        q = msg.pose.pose.orientation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        offset_world = R @ np.array([self.lidar_x, 0.0, self.lidar_z])
        sp.pose.pose.position.x += offset_world[0]
        sp.pose.pose.position.y += offset_world[1]
        sp.pose.pose.position.z += offset_world[2]
        self.sensor_pose_pub.publish(sp)


if __name__ == '__main__':
    rospy.init_node('gazebo_bridge')
    GazeboBridge()
    rospy.spin()
