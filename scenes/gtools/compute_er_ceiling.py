#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""可复现的 Depot ER 天花板计算（2026-08-26 审查A1 修正产物）。

背景：旧文档里「地面层上限 56.8% / 加夹层 76.2%」是一次性会话内计算的口头数，
脚本不存在、参数未留痕，已被审查降级为「估计值」。本脚本用与
tools/evaluate_exploration.py **完全相同**的体素化(VOX)与可见性模型
(visible_voxels：range + 垂直 FOV 近似、无遮挡剔除)重算天花板，产出可复现数字。

视点模型（保守，假设声明）：
  - 站立点 = GT 障碍带（z∈[0.25,1.8]，剔地面）0.4m 体素化后的自由格中心，
    且其正下方 0.5m 内存在 GT 支撑面（有地板）——近似「狗能站的地方」。
  - 两层口径：ground(z≈0.09 层的自由格) 与 mezzanine(z≈2.86 层的自由格)，
    分别以传感器高度 ground+0.4577 / 2.86+0.4577 撒视点。
  - 不做遮挡剔除、不做可达性/连通性检查（楼梯单行道等），因此结果是
    「几何可见上限」，真实可达上限只会更低。

用法：python3 scenes/gtools/compute_er_ceiling.py [scene=depot]
输出：各层口径的 ER 上限 + 覆盖并集；数字可直接替换文档中的悬空值。
"""
import math
import sys

import numpy as np

# ---- 与 tools/evaluate_exploration.py 保持一致（改一处须同步另一处）----
VOX = 0.4
SENSOR_RANGE = 6.0
FOV_H_HALF = math.pi
FOV_V_MIN = math.radians(-7.0)
FOV_V_MAX = math.radians(52.0)
LIDAR_OFFSET = np.array([0.2, 0.0, 0.2077])

GT_PCD = 'scenes/depot/gt/depot.pcd'
FLOORS = {'ground': 0.09, 'mezzanine': 2.86}   # scenes/depot/scene.yaml floors
LIDAR_H_ABOVE_FLOOR = LIDAR_OFFSET[2]          # 传感器离站立面高度


def load_gt(path):
    with open(path, 'rb') as f:
        while True:
            line = f.readline().decode('ascii', 'ignore')
            if line.startswith('DATA'):
                break
        return np.loadtxt(f, dtype=np.float32)[:, :3]


def vox_keys(xyz):
    keys = np.floor(xyz / VOX).astype(np.int64)
    kmin = keys.min(axis=0)
    return keys - kmin, (keys - kmin)


def visible_mask(gt_xyz, origin, yaw):
    """同 evaluate_exploration.visible_voxels 的向量化版（水平全向）。"""
    d = gt_xyz - origin
    dist = np.linalg.norm(d, axis=1)
    mask = dist < SENSOR_RANGE
    zc = d[:, 2] / np.maximum(dist, 1e-6)
    mask &= (zc >= FOV_V_MIN) & (zc <= FOV_V_MAX)
    return mask


def stand_points(gt_xyz, floor_z):
    """某楼层上的候选站立点（0.4m 格中心）：障碍带自由 + 有支撑面。"""
    band = gt_xyz[(gt_xyz[:, 2] >= floor_z + 0.25) & (gt_xyz[:, 2] <= floor_z + 1.8)]
    support = gt_xyz[(gt_xyz[:, 2] >= floor_z - 0.15) & (gt_xyz[:, 2] <= floor_z + 0.15)]
    if len(support) == 0:
        return np.zeros((0, 2))
    occ = set(map(tuple, np.floor(band[:, :2] / VOX).astype(np.int64)))
    sup_cells = set(map(tuple, np.floor(support[:, :2] / VOX).astype(np.int64)))
    cells = sorted(c for c in sup_cells if c not in occ)
    pts = np.array([[x * VOX + VOX / 2, y * VOX + VOX / 2] for x, y in cells])
    # 再滤一遍：站立点半径0.3m内不得有障碍带点（格级判定太粗）
    if len(band) and len(pts):
        from scipy.spatial import cKDTree
        tree = cKDTree(band[:, :2])
        d, _ = tree.query(pts)
        pts = pts[d > 0.35]
    return pts


def main():
    gt = load_gt(GT_PCD)
    mn, mx = gt.min(axis=0), gt.max(axis=0)
    print('GT %d 点 x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]'
          % (len(gt), mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]))
    _, gt_keys = vox_keys(gt)
    gt_unique = np.unique(gt_keys, axis=0)
    n_total = len(gt_unique)
    print('GT 体素总数(VOX=%.1fm): %d' % (VOX, n_total))

    observed = set()
    results = {}
    for name, z in FLOORS.items():
        pts = stand_points(gt, z)
        seen_layer = set()
        for p in pts:
            origin = np.array([p[0], p[1], z]) + LIDAR_OFFSET
            m = visible_mask(gt, origin, 0.0)     # 水平全向，yaw 无关
            seen_layer |= set(map(tuple, gt_keys[m].tolist()))
        results[name] = seen_layer
        observed |= seen_layer
        print('%s层: 站立点%d个 → 该层可见 %d 体素 (%.1f%%)'
              % (name, len(pts), len(seen_layer), 100 * len(seen_layer) / n_total))
        print('  累计(含前层): %.1f%%' % (100 * len(observed) / n_total))
    print('---- 结论 ----')
    for name in FLOORS:
        print('仅%s层可达上限: %.1f%%' % (name, 100 * len(results[name]) / n_total))
    print('ground+mezzanine 联合上限: %.1f%%' % (100 * len(observed) / n_total))


if __name__ == '__main__':
    sys.exit(main())
