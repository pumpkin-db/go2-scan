#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时探索评分 → RViz 文本显示（2026-08-27，Depot 交互观察用）。

定位：纯显示胶水，不碰官方评估器 evaluate_exploration.py。可见性模型直接
import 复用（同一份代码，口径保证一致）：ER = 已见 GT 体素 / GT 体素总数，
range+FOV 无遮挡剔除，与评估器同参（VOX=0.4、SENSOR_RANGE=6m、MID360 FOV）。

实现：订阅 /quad_0/body_pose + /map（GT，latched）；位姿按 0.25s 抽帧，
每 2s 把新帧增量过一遍可见性（累积 observed 集，不重算历史）；路径长沿用
评估器 SPEED_MIN 加速度门口径。文本 Marker 发到 /live_score，锚点自动取
GT 包围盒中心上方。

注意：这是「过程仪表盘」，终值与官方评估器结束报告可能有 ±1% 级差异
（抽帧密度不同），正式记录以评估器报告为准。

用法：仿真跑着时 /usr/bin/python3 tools/live_score_rviz.py
"""
import math
import os
import sys
import time

# realpath：本脚本经 go2_bridge/scripts/ 符号链接被 rosrun 启动，
# abspath 不解链接会指向链接目录，找不到 evaluate_exploration
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

from evaluate_exploration import (VOX, SENSOR_RANGE, SPEED_MIN,
                                  pc2_to_xyz, quat_to_yaw, visible_voxels)

POSE_DT = 0.25     # 位姿抽帧间隔（4Hz，够 ER 曲线精度）
TICK = 2.0         # 评分刷新周期
TOPIC = '/live_score'


class LiveScore:
    def __init__(self):
        self.gt_xyz = None
        self.gt_packed = None
        self.n_total = 0
        self.observed = set()
        self.pending = []          # 待过可见性的位姿 [x,y,z,yaw]
        self.last_pose_t = -1e9
        self.last_pose = None      # 路径累计用
        self.path_len = 0.0
        self.t_start = None
        self.last_pub_wall = 0.0

        rospy.Subscriber('/map', PointCloud2, self.gt_cb, queue_size=1)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.odom_cb, queue_size=30)
        self.pub = rospy.Publisher(TOPIC, Marker, queue_size=2)

    def gt_cb(self, msg):
        if self.gt_xyz is not None:
            return                          # latched，收一次
        xyz = pc2_to_xyz(msg)
        if len(xyz) == 0:
            return
        keys = np.floor(xyz / VOX).astype(np.int64)
        keys -= keys.min(axis=0)
        packed = (keys[:, 0] << 42) + (keys[:, 1] << 21) + keys[:, 2]
        uniq, idx = np.unique(packed, return_index=True)
        self.gt_xyz = xyz[np.sort(idx)]     # 每体素留一点，可见性判断等价
        self.gt_packed = uniq
        self.n_total = len(uniq)
        c = self.gt_xyz.mean(axis=0)
        self.anchor = (float(c[0]), float(c[1]), float(self.gt_xyz[:, 2].max()) + 3.0)
        rospy.loginfo('[live_score] GT 就绪：%d 体素，锚点 %.1f,%.1f,%.1f',
                      self.n_total, *self.anchor)

    def odom_cb(self, msg):
        t = msg.header.stamp.to_sec()
        if self.t_start is None:
            self.t_start = t
        p = msg.pose.pose.position
        pose = [p.x, p.y, p.z, quat_to_yaw(msg.pose.pose.orientation)]
        # 路径累计（SPEED_MIN 加速度门，与评估器同口径）
        if self.last_pose is not None:
            dt = t - self.last_pose_t
            if 0 < dt < 1.0:
                step = math.hypot(pose[0] - self.last_pose[0],
                                  pose[1] - self.last_pose[1])
                if step / dt > SPEED_MIN:
                    self.path_len += step
        self.last_pose = pose
        self.last_pose_t = t
        if t - self.last_sample_t >= POSE_DT:
            self.pending.append(pose)
            self.last_sample_t = t

    last_sample_t = -1e9

    def tick(self, _evt):
        now = time.time()
        if now - self.last_pub_wall < TICK:
            return
        self.last_pub_wall = now
        er = None
        if self.gt_xyz is not None and self.pending:
            for pose in self.pending:
                self.observed |= visible_voxels(self.gt_xyz, self.gt_packed,
                                                pose, SENSOR_RANGE)
            self.pending = []
            er = len(self.observed) / self.n_total
        self.publish(er)

    def publish(self, er):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = rospy.Time.now()
        m.ns = 'live_score'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        if self.anchor is not None:
            m.pose.position.x, m.pose.position.y, m.pose.position.z = self.anchor
        else:
            m.pose.position.z = 12.0
        m.scale.z = 1.2
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        if er is None and self.gt_xyz is None:
            m.text = 'live_score: 等 /map (GT)...'
        else:
            t_el = (self.last_pose_t - self.t_start) if self.t_start else 0.0
            m.text = 'ER %.1f%%  |  path %.0fm  |  t %.0fs' % (
                (er if er is not None else self._last_er) * 100,
                self.path_len, t_el)
            if er is not None:
                self._last_er = er
        self.pub.publish(m)

    _last_er = 0.0
    anchor = None


def main():
    rospy.init_node('live_score_rviz', anonymous=True)
    node = LiveScore()
    rospy.Timer(rospy.Duration(0.5), node.tick)
    rospy.loginfo('[live_score] 就绪：发布 %s（周期 %.0fs，抽帧 %.2fs）',
                  TOPIC, TICK, POSE_DT)
    rospy.spin()


if __name__ == '__main__':
    main()
