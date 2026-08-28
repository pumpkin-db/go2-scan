#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""首次 fatal failure 诊断快照节点（配套 [SCAN_DIAG] 埋点，2026-08-28）。

职责：
  1. 缓存关键话题最新帧（body_pose / lidar_pose / initial_path / way_point /
     mid360_points_clean / grid_map 占据可视化云）。
  2. 统计 rosout 关键错误模式的首现/末现/次数。
  3. 全程以 1Hz 记录 pose/cloud 时间戳滞后（任务3 时序证据），
     全程记录每条 /initial_path 几何（任务4 证据）。
  4. 触发条件：收到 FSM 的 /scan_diag/snapshot，或 rosout 首现
     "drone is in obstacle"。把上述全部缓存落盘到 /tmp/go2_diag/snapshot_*。

用法（仿真起来后）：
  PYTHONUNBUFFERED=1 /usr/bin/python3 tools/diag_snapshot_node.py
"""
import json
import os
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs import point_cloud2 as pc2

try:
    from scan_planner.msg import Bspline
    HAS_BSPLINE = True
except Exception:
    HAS_BSPLINE = False  # devel 未 source 时降级运行，不影响其余诊断

OUT_ROOT = '/tmp/go2_diag'
MAX_SNAPS = 8

PATTERNS = [
    'drone is in obstacle',
    'a star error',
    'First 3 control points',
    'escape recovery',
    'EMERGENCY_STOP',
    'final_plan_success=0',
    'Unable to generate global trajectory',
    'emergency stop',
    'A-star path has less than 2 points',
    'SCAN_ANOMALY',
    'STOP_REASON=',
]


def stamp_of(msg):
    return getattr(msg.header.stamp, 'to_sec', lambda: 0.0)()


def cloud_xyz(msg, max_pts=400000):
    try:
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return list(pts)[:max_pts]
    except Exception:
        return []


def write_pcd(path, pts):
    """ascii PCD 足够诊断用。"""
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
                'TYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                'POINTS %d\nDATA ascii\n' % (len(pts), len(pts)))
        for p in pts:
            f.write('%.4f %.4f %.4f\n' % (p[0], p[1], p[2]))


class DiagNode(object):
    def __init__(self):
        self.latest = {}       # topic -> (recv_wall, msg)
        self.snap_n = 0
        self.trig_seen_in_rosout = False
        self.counters = {p: {'n': 0, 'first': None, 'last': None} for p in PATTERNS}
        self.log_tail = []
        self.ip_log = open(os.path.join(OUT_ROOT, 'initial_path_log.csv'), 'a', buffering=1)
        self.tm_log = open(os.path.join(OUT_ROOT, 'timing_log.csv'), 'a', buffering=1)
        self.stop_log = open(os.path.join(OUT_ROOT, 'stop_reason_log.csv'), 'a', buffering=1)
        self.ip_log.write('wall,stamp,n_pts,seg_lens,first,last\n')
        self.tm_log.write('wall,body_stamp,body_recv_lag,lidar_stamp,lidar_recv_lag,'
                          'cloud_stamp,cloud_recv_lag,cloud_n\n')
        rospy.init_node('diag_snapshot_node', anonymous=True, disable_signals=True)

        subs = [
            ('/scan_diag/snapshot', String, self.snapshot_cb, 5),
            ('/rosout_agg', Log, self.rosout_cb, 200),
            ('/quad_0/body_pose', Odometry, self.cache_cb, 30),
            ('/quad_0/lidar_pose', Odometry, self.cache_cb, 30),
            ('/initial_path', Path, self.initial_path_cb, 10),
            ('/way_point', PointStamped, self.cache_cb, 10),
            ('/mid360_points_clean', PointCloud2, self.cache_cb, 5),
            ('/grid_map/occupancy', PointCloud2, self.cache_cb, 3),
            ('/grid_map/occupancy_inflate', PointCloud2, self.cache_cb, 3),
        ]
        if HAS_BSPLINE:
            subs.append(('/planning/bspline', Bspline, self.bspline_cb, 50))
        for topic, typ, cb, q in subs:
            rospy.Subscriber(topic, typ, cb, queue_size=q)

        rospy.Timer(rospy.Duration(1.0), self.timer_cb)
        rospy.loginfo('[diag_snapshot_node] up, out=%s', OUT_ROOT)

    # ---------- collectors ----------

    def cache_cb(self, msg):
        self.latest[msg._connection_header['topic']] = (time.time(), msg)

    def rosout_cb(self, msg):
        text = msg.msg or ''
        line = '%.3f [%s] %s' % (time.time(), msg.name, text)
        self.log_tail.append(line)
        if len(self.log_tail) > 500:
            del self.log_tail[:len(self.log_tail) - 500]
        if 'STOP_REASON=' in text:
            self.stop_log.write('%.3f [%s] %s\n' % (time.time(), msg.name, text))
        for p in PATTERNS:
            if p in text:
                c = self.counters[p]
                c['n'] += 1
                now = time.time()
                if c['first'] is None:
                    c['first'] = now
                c['last'] = now
                if p == 'drone is in obstacle' and not self.trig_seen_in_rosout:
                    self.trig_seen_in_rosout = True
                    self.save_snapshot('rosout_first_drone_in_obstacle')
                if p == 'SCAN_ANOMALY' and not os.path.exists(
                        os.path.join(OUT_ROOT, 'ANOMALY_marker')):
                    # rosout 直达：绕过 roslaunch stdout 块缓冲，附上原文供分类
                    with open(os.path.join(OUT_ROOT, 'ANOMALY_marker'), 'w') as f:
                        f.write(text + '\n')
                    self.save_snapshot('rosout_' + p.replace(' ', '_'))

    def initial_path_cb(self, msg):
        self.cache_cb(msg)
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        segs = [((pts[i][0] - pts[i - 1][0]) ** 2 + (pts[i][1] - pts[i - 1][1]) ** 2 +
                 (pts[i][2] - pts[i - 1][2]) ** 2) ** 0.5 for i in range(1, len(pts))]
        self.ip_log.write('%.3f,%.3f,%d,%s,(%.2f,%.2f),(%.2f,%.2f)\n' % (
            time.time(), stamp_of(msg), len(pts),
            ' '.join('%.2f' % s for s in segs),
            pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]))

    def bspline_cb(self, msg):
        """轨迹发布事件时间线：间隔大 = 没在规划（WAIT_TARGET/急停等）。"""
        if not hasattr(self, 'bsp_log'):
            self.bsp_log = open(os.path.join(OUT_ROOT, 'bspline_log.csv'), 'a', buffering=1)
            self.bsp_log.write('wall,stamp,traj_id\n')
        self.bsp_log.write('%.3f,%.3f,%d\n' % (time.time(), stamp_of(msg), msg.traj_id))
        self.cache_cb(msg)

    def timer_cb(self, _):
        now = time.time()

        def row(topic):
            if topic not in self.latest:
                return ''
            recv, msg = self.latest[topic]
            return '%.3f,%.3f' % (stamp_of(msg), now - recv)

        cloud = self.latest.get('/mid360_points_clean')
        cloud_n = -1
        if cloud:
            cloud_n = cloud[1].width * cloud[1].height
        self.tm_log.write('%.3f,%s,%s,%s,%d\n' % (
            now, row('/quad_0/body_pose'), row('/quad_0/lidar_pose'),
            row('/mid360_points_clean'), cloud_n))

    # ---------- snapshot ----------

    def snapshot_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {'raw': msg.data}
        self.save_snapshot('fsm_' + str(payload.get('reason', 'unknown')).replace(' ', '_'), payload)

    def save_snapshot(self, reason, fsm_payload=None):
        if self.snap_n >= MAX_SNAPS:
            return
        self.snap_n += 1
        d = os.path.join(OUT_ROOT, 'snapshot_%02d' % self.snap_n)
        os.makedirs(d, exist_ok=True)
        meta = {
            'reason': reason,
            'wall': time.time(),
            'fsm': fsm_payload,
            'rosout_counters': self.counters,
            'topics': {},
        }
        for topic, (recv, msg) in list(self.latest.items()):
            entry = {'recv_lag': time.time() - recv, 'stamp': stamp_of(msg),
                     'frame_id': msg.header.frame_id, 'type': msg._type}
            if isinstance(msg, PointCloud2):
                entry['n'] = msg.width * msg.height
            if isinstance(msg, Path):
                entry['n'] = len(msg.poses)
            meta['topics'][topic] = entry
            if isinstance(msg, PointCloud2):
                name = topic.strip('/').replace('/', '_') + '.pcd'
                write_pcd(os.path.join(d, name), cloud_xyz(msg))
        with open(os.path.join(d, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=1, ensure_ascii=False)
        with open(os.path.join(d, 'rosout_tail.txt'), 'w') as f:
            f.write('\n'.join(self.log_tail))
        rospy.logwarn('[diag_snapshot_node] saved %s (%s)', d, reason)


if __name__ == '__main__':
    os.makedirs(OUT_ROOT, exist_ok=True)
    DiagNode()
    rospy.spin()
