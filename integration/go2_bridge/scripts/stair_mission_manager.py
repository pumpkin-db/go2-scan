#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""楼梯任务状态机（2026-08-28，P5-P7）。

EXPLORE → APPROACH_STAIR → COMMIT_STAIR → TRAVERSE_STAIR → EXIT_STAIR
→ SWITCH_FLOOR → EXPLORE(二层)

- EXPLORE: ARiADNE 控制（nav_source=ariadne），监听 /stairs_detected，
  机器人距楼梯入口 < detect_radius 时锁定楼梯接管。
- APPROACH: 发 [当前位置 → approach → entry] 给 SCAN；nav_source=stair（bridge 停发）。
- COMMIT: 距 entry < commit_dist → 发布楼梯方向 + ModelPlugin 单调锁 → TRAVERSE。
- TRAVERSE: 发 entry→exit 多点路径；12s 无进度 → 100Hz 前向 fallback（绕过
  closed_loop_controller 的零速稀释）；到达 exit+margin 且 z 达二层 → EXIT。
- EXIT: 解锁；3s 确认 → SWITCH_FLOOR。
- SWITCH_FLOOR: octomap 占据 z 带切到二层 + rosnode kill /octomap（respawn 重启）
  + /floor_reset 广播（rl_planner 新 session、bridge 清 last_sent）→ 交还 ARiADNE。
单次 mission：完成后 mission_done=True，不再触发（本 Demo 只做 floor0→1 一次）。
"""
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Empty, String


class StairMissionManager(object):
    def __init__(self):
        self.state = 'EXPLORE'
        self.floor = 0
        self.mission_done = False
        self.nav_source = 'ariadne'
        self.stairs = []
        self.stair = None
        self.d = None
        self.s_exit = None
        self.pos = None
        self.yaw = 0.0
        self.fallback = False
        self.last_progress_s = None
        self.last_progress_t = None
        self.state_t = time.time()
        self.last_traverse_send = 0.0

        self.detect_radius = rospy.get_param('~detect_radius', 6.0)
        self.approach_dist = rospy.get_param('~approach_dist', 1.5)
        self.commit_dist = rospy.get_param('~commit_dist', 0.9)
        self.exit_margin = rospy.get_param('~exit_margin', 0.5)
        self.stair_index = rospy.get_param('~stair_index', 0)
        self.floor1_z = rospy.get_param('~floor1_z', 2.86)
        self.band_min = rospy.get_param('~floor2_occ_min_z', 3.0)
        self.band_max = rospy.get_param('~floor2_occ_max_z', 3.8)
        self.body_h = rospy.get_param('~scan_body_height', 0.4)
        self.stall_t = rospy.get_param('~fallback_stall_s', 12.0)

        rospy.Subscriber('/stairs_detected', String, self.stairs_cb, queue_size=2)
        rospy.Subscriber('/quad_0/body_pose', Odometry, self.odom_cb, queue_size=1)
        self.path_pub = rospy.Publisher('/initial_path', Path, queue_size=2)
        self.nav_source_pub = rospy.Publisher('/nav_source', String, queue_size=2, latch=True)
        self.lock_pub = rospy.Publisher('/stair_traverse_lock', Bool, queue_size=2)
        self.dir_pub = rospy.Publisher('/stair_traverse_dir', Vector3, queue_size=2)
        self.zprof_pub = rospy.Publisher('/stair_traverse_zprof', Vector3, queue_size=2)
        self.floor_reset_pub = rospy.Publisher('/floor_reset', Empty, queue_size=2)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.nav_source_pub.publish('ariadne')
        rospy.Timer(rospy.Duration(0.5), self.tick)
        rospy.Timer(rospy.Duration(0.01), self.fallback_tick)
        rospy.Timer(rospy.Duration(30.0), self.heartbeat)
        rospy.logwarn('[stair_mission] up state=EXPLORE (detect_radius=%.1f)', self.detect_radius)

    def heartbeat(self, _):
        rospy.loginfo('[stair_mission] state=%s floor=%d pos=%s', self.state, self.floor,
                      '(%0.2f,%0.2f,%0.2f)' % self.pos if self.pos else 'none')

    def stairs_cb(self, msg):
        try:
            self.stairs = json.loads(msg.data)
        except Exception:
            pass

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pos = (p.x, p.y, p.z)
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def send_path(self, pts):
        """pts=[(x,y,world_z)]；SCAN pathCallback 会 +body_height，故发 world_z-body_h。"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'world'
        for x, y, wz in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = max(0.0, wz - self.body_h)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def s_of(self, pos):
        e = self.stair['entry']
        return (pos[0] - e[0]) * self.d[0] + (pos[1] - e[1]) * self.d[1]

    def tick(self, _):
        if self.mission_done:
            return
        pos = self.pos
        if pos is None:
            return
        if self.state == 'EXPLORE':
            if self.stair is None and len(self.stairs) > self.stair_index:
                self.stair = self.stairs[self.stair_index]
                e, x = self.stair['entry'], self.stair['exit']
                dx, dy = x[0] - e[0], x[1] - e[1]
                n = math.hypot(dx, dy)
                self.d = (dx / n, dy / n)
                self.s_exit = (x[0] - e[0]) * self.d[0] + (x[1] - e[1]) * self.d[1]
                rospy.logwarn('[stair_mission] stair locked: %s d=(%.2f,%.2f) s_exit=%.2f',
                              self.stair.get('name'), self.d[0], self.d[1], self.s_exit)
            if self.stair:
                e = self.stair['entry']
                dist = math.hypot(pos[0] - e[0], pos[1] - e[1])
                if dist < self.detect_radius:
                    rospy.logwarn('[stair_mission] EXPLORE->APPROACH_STAIR dist=%.2f', dist)
                    self.set_state('APPROACH_STAIR')
                    self.nav_source_pub.publish('stair')
                    ax = e[0] - self.d[0] * self.approach_dist
                    ay = e[1] - self.d[1] * self.approach_dist
                    self.send_path([(pos[0], pos[1], e[2]), (ax, ay, e[2]), (e[0], e[1], e[2])])
                    rospy.logwarn('[stair_mission] approach: (%.2f,%.2f)→entry (%.2f,%.2f)', ax, ay, e[0], e[1])
        elif self.state == 'APPROACH_STAIR':
            e = self.stair['entry']
            x = self.stair['exit']
            dist = math.hypot(pos[0] - e[0], pos[1] - e[1])
            if dist < self.commit_dist:
                rospy.logwarn('[stair_mission] APPROACH->COMMIT dist=%.2f', dist)
                self.set_state('COMMIT_STAIR')
                self.dir_pub.publish(Vector3(x=self.d[0], y=self.d[1], z=0.0))
                self.zprof_pub.publish(Vector3(x=e[2], y=x[2], z=self.s_exit))
                self.lock_pub.publish(Bool(data=True))
                self.last_progress_s = self.s_of(pos)
                self.last_progress_t = time.time()
                self.send_traverse()
                self.last_traverse_send = time.time()
        elif self.state in ('COMMIT_STAIR', 'TRAVERSE_STAIR', 'EXIT_STAIR'):
            # SCAN respawn/断连兜底：stair 态周期重发 traverse 路径（与 bridge 5s 重发同型）
            if time.time() - self.last_traverse_send > 5.0:
                self.send_traverse()
                self.last_traverse_send = time.time()
            s = self.s_of(pos)
            if self.state == 'COMMIT_STAIR':
                if s >= 0.2 or time.time() - self.state_t > 2.0:
                    rospy.logwarn('[stair_mission] COMMIT->TRAVERSE s=%.2f', s)
                    self.set_state('TRAVERSE_STAIR')
                    if self.last_progress_s is None:
                        self.last_progress_s = s
                    if self.last_progress_t is None:
                        self.last_progress_t = time.time()
            else:
                if s > self.last_progress_s + 0.15:
                    self.last_progress_s = s
                    self.last_progress_t = time.time()
                elif time.time() - self.last_progress_t > self.stall_t and not self.fallback:
                    rospy.logwarn('[stair_mission] stalled %.0fs s=%.2f → forward fallback ON '
                                  '(stays on until EXIT)', time.time() - self.last_progress_t, s)
                    self.fallback = True
                if (self.state == 'TRAVERSE_STAIR'
                        and s >= self.s_exit + self.exit_margin
                        and pos[2] >= self.floor1_z + 0.1):
                    rospy.logwarn('[stair_mission] TRAVERSE->EXIT s=%.2f z=%.2f', s, pos[2])
                    self.set_state('EXIT_STAIR')
                    self.lock_pub.publish(Bool(data=False))
                    self.fallback = False
                if (self.state == 'EXIT_STAIR' and time.time() - self.state_t > 3.0):
                    rospy.logwarn('[stair_mission] EXIT->SWITCH_FLOOR')
                    self.set_state('SWITCH_FLOOR')
                    self.do_floor_switch()
        elif self.state == 'SWITCH_FLOOR':
            if time.time() - self.state_t > 20.0:
                self.floor += 1
                self.nav_source_pub.publish('ariadne')
                self.mission_done = True
                rospy.logwarn('[stair_mission] SWITCH_FLOOR->EXPLORE floor=%d (mission done)', self.floor)
                self.set_state('EXPLORE')

    def set_state(self, s):
        self.state = s
        self.state_t = time.time()

    def send_traverse(self):
        e, x = self.stair['entry'], self.stair['exit']
        pts = [(e[0], e[1], e[2])]
        for k in (1, 2, 3):
            f = k / 4.0
            pts.append((e[0] + (x[0] - e[0]) * f, e[1] + (x[1] - e[1]) * f, e[2] + (x[2] - e[2]) * f))
        pts.append((x[0], x[1], x[2]))
        pts.append((x[0] + self.d[0] * 0.8, x[1] + self.d[1] * 0.8, self.floor1_z))
        self.send_path(pts)
        rospy.logwarn('[stair_mission] traverse path %d pts', len(pts))

    def do_floor_switch(self):
        rospy.set_param('/octomap/occupancy_min_z', self.band_min)
        rospy.set_param('/octomap/occupancy_max_z', self.band_max)
        os.system('rosnode kill /octomap >/dev/null 2>&1')
        self.floor_reset_pub.publish(Empty())
        rospy.logwarn('[stair_mission] floor switch: octomap band→[%.1f,%.1f] + floor_reset sent',
                      self.band_min, self.band_max)

    def fallback_tick(self, _):
        if self.state != 'TRAVERSE_STAIR' or not self.fallback or self.pos is None:
            return
        dvx, dvy = 0.25 * self.d[0], 0.25 * self.d[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        t = Twist()
        t.linear.x = c * dvx + s * dvy
        t.linear.y = -s * dvx + c * dvy
        self.cmd_pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('stair_mission_manager')
    StairMissionManager()
    rospy.spin()
