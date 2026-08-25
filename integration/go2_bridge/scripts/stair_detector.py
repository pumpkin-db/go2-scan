#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
楼梯检测器（方向④阶段C，2026-08-25）。

输入：/elevation_mapping/elevation_map（GridMap，elevation 层）
输出：/stairs_detected（std_msgs/String，JSON 数组，每项：
        {name, entry:[x,y,z], exit:[x,y,z], yaw, width, rise_total, levels}
      ）+ /stairs_markers（RViz 可视化）

算法（经典方法，无学习组件）：
  1. 取高程层为 numpy 网格，算每格坡度向量 (dh/dx, dh/dy)；
  2. 坡度幅值在 [tan(15°), tan(45°)] 且方向局部一致的连通域 = 候选斜坡带；
  3. 对每个候选带沿主方向采样高程剖面，量化成层级（间距阈值 0.03m），
     相邻层高差 ∈ [0.08, 0.30]m 的级数 ≥3 且总升高 ≥0.8m → 判楼梯；
  4. entry/exit = 剖面最低/最高层的带内质心，yaw = 主方向。

另支持「注册表兜底」：--registry scene.yaml 里预登记的楼梯对直接并入输出
（检测器验收基准 + transit 联调先行）。

用法：rosrun go2_bridge stair_detector.py
      参数：~registry $(find go2_bridge)/../../scenes/depot/scene.yaml
           ~detect_rate 0.5  ~min_levels 3  ~min_rise 0.8
"""
import json
import math
import os

import numpy as np
import rospy
from grid_map_msgs.msg import GridMap
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def gridmap_layer_numpy(msg, layer_name):
    """grid_map_msgs → (H×W numpy[行=x], res, x0, y0)。布局同 go2_kinematic_sim。"""
    try:
        idx = msg.layers.index(layer_name)
    except ValueError:
        return None
    data = msg.data[idx]
    rows = data.layout.dim[0].size
    cols = data.layout.dim[1].size
    if len(data.data) < rows * cols:
        return None
    arr = np.array(data.data[:rows * cols], dtype=np.float64).reshape(cols, rows).T  # col-major→[x,y]
    res = msg.info.resolution
    x0 = msg.info.pose.position.x - rows * res * 0.5
    y0 = msg.info.pose.position.y - cols * res * 0.5
    return arr, res, x0, y0


def detect_stairs(elev, res, x0, y0, min_levels=3, min_rise=0.8,
                  slope_lo=math.tan(math.radians(15)), slope_hi=math.tan(math.radians(45))):
    """返回楼梯对列表。"""
    from scipy import ndimage
    gy, gx = np.gradient(elev, res)
    mag = np.hypot(gx, gy)
    valid = ~np.isnan(elev)
    band = (mag > slope_lo) & (mag < slope_hi) & valid
    lab, n = ndimage.label(band, structure=np.ones((3, 3)))
    out = []
    for i in range(1, n + 1):
        m = lab == i
        cnt = m.sum()
        if cnt < 20:                      # <20 格(0.05m图≈0.05m²) 太小
            continue
        ys, xs = np.where(m)
        # 主方向 = 坡度向量的平均方向
        ang = math.atan2(float(np.mean(gy[m])), float(np.mean(gx[m])))
        # 沿主方向投影采样剖面
        px = xs * res + x0
        py = ys * res + y0
        proj = (px - px.mean()) * math.cos(ang) + (py - py.mean()) * math.sin(ang)
        h = elev[m]
        order = np.argsort(proj)
        proj_s, h_s = proj[order], h[order]
        # 滑窗取每 0.15m 段的中位高程 → 层级序列
        win = max(1, int(0.15 / res))
        levels = []
        for s in range(0, len(proj_s), win):
            seg = h_s[s:s + win]
            levels.append(float(np.nanmedian(seg)))
        # 量化层级（0.03m 内合并）
        qs = []
        for lv in levels:
            if not qs or abs(lv - qs[-1]) > 0.03:
                qs.append(lv)
        rises = np.diff(qs)
        good = [(a, b) for a, b in zip(qs[:-1], qs[1:]) if 0.08 <= (b - a) <= 0.30]
        if len(good) < min_levels:
            continue
        rise_total = sum(b - a for a, b in good)
        if rise_total < min_rise:
            continue
        # entry/exit：剖面两端各取带内质心
        lo_t, hi_t = proj_s[0], proj_s[-1]
        e_m = proj <= lo_t + 0.3
        x_m = proj >= hi_t - 0.3
        entry = (float(px[e_m].mean()), float(py[e_m].mean()), float(np.nanmin(h_s[:max(1, len(h_s) // 4)])))
        exitp = (float(px[x_m].mean()), float(py[x_m].mean()), float(np.nanmax(h_s[-max(1, len(h_s) // 4):])))
        # 带宽：投影垂直方向的展开度
        perp = (px - px.mean()) * -math.sin(ang) + (py - py.mean()) * math.cos(ang)
        width = float(perp.max() - perp.min())
        out.append(dict(entry=list(entry), exit=list(exitp), yaw=math.degrees(ang),
                        width=round(width, 2), rise_total=round(rise_total, 2),
                        levels=len(good)))
    return out


class StairDetector:
    def __init__(self):
        self.registry = []
        reg_path = rospy.get_param('~registry', '')
        if reg_path and os.path.isfile(reg_path):
            import yaml
            with open(reg_path) as f:
                cfg = yaml.safe_load(f)
            for s in (cfg.get('stairs') or []):
                self.registry.append(dict(
                    name=s.get('name', 'reg'),
                    entry=[s['entry']['x'], s['entry']['y'], s['entry']['z']],
                    exit=[s['exit']['x'], s['exit']['y'], s['exit']['z']],
                    yaw=s.get('yaw_deg', 0), width=s.get('width', 1.0),
                    rise_total=s['exit']['z'] - s['entry']['z'], levels=-1,
                    source='registry'))
            rospy.loginfo('[stair_detector] 注册表 %d 对: %s', len(self.registry),
                          [r['name'] for r in self.registry])
        self.min_levels = rospy.get_param('~min_levels', 3)
        self.min_rise = rospy.get_param('~min_rise', 0.8)
        self.rate = rospy.get_param('~detect_rate', 0.5)
        self.last_elev = None
        rospy.Subscriber('/elevation_mapping/elevation_map', GridMap, self.elev_cb, queue_size=1)
        self.pub = rospy.Publisher('/stairs_detected', String, queue_size=2)
        self.mk_pub = rospy.Publisher('/stairs_markers', MarkerArray, queue_size=2)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)

    def elev_cb(self, msg):
        self.last_elev = msg

    def tick(self, _):
        if self.last_elevation_ready():
            parsed = gridmap_layer_numpy(self.last_elev, 'elevation')
            if parsed is not None:
                elev, res, x0, y0 = parsed
                try:
                    found = detect_stairs(elev, res, x0, y0, self.min_levels, self.min_rise)
                except Exception as e:
                    rospy.logwarn_throttle(20, '[stair_detector] 检测异常: %s', e)
                    found = []
                for f in found:
                    f['name'] = 'detected_%.1f_%.1f' % (f['entry'][0], f['entry'][1])
                    f['source'] = 'detector'
                all_pairs = self.registry + found
                self.pub.publish(String(data=json.dumps(all_pairs)))
                self.publish_markers(all_pairs)
                if found:
                    rospy.loginfo_throttle(30, '[stair_detector] 检出 %d 段楼梯', len(found))

    def last_elevation_ready(self):
        return self.last_elev is not None

    def publish_markers(self, pairs):
        ma = MarkerArray()
        for i, p in enumerate(pairs):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'stairs'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = p['entry'][0]
            m.pose.position.y = p['entry'][1]
            m.pose.position.z = p['entry'][2] + 0.3
            yaw = math.radians(p['yaw'])
            m.pose.orientation.z = math.sin(yaw / 2)
            m.pose.orientation.w = math.cos(yaw / 2)
            m.scale.x = max(0.5, math.hypot(p['exit'][0] - p['entry'][0], p['exit'][1] - p['entry'][1]))
            m.scale.y, m.scale.z = 0.15, 0.15
            m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 0.8, 1.0, 0.9) if p['source'] == 'detector' else (1.0, 0.6, 0.0, 0.9)
            ma.markers.append(m)
        self.mk_pub.publish(ma)


if __name__ == '__main__':
    rospy.init_node('stair_detector')
    StairDetector()
    rospy.spin()
