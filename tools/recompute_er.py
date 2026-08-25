#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线重算 ER：读评估器 JSON 里持久化的 trajectory_5hz + GT PCD，重出 ER 曲线与指标。
用途：evaluate_exploration.py 的 ER 逻辑修 bug 后，对历史 run 补算，不用重跑仿真。

用法：python3 tools/recompute_er.py <run.json> <gt.pcd> [--vox 0.4] [--range 6.0]
"""
import argparse
import json
import math

import numpy as np

LIDAR_OFFSET = np.array([0.2, 0.0, 0.2077])
FOV_V_MIN = math.radians(-7.0)
FOV_V_MAX = math.radians(52.0)


def load_pcd_xyz(path):
    pts = []
    with open(path) as f:
        in_data = False
        for line in f:
            if in_data:
                v = line.split()
                if len(v) >= 3:
                    pts.append([float(v[0]), float(v[1]), float(v[2])])
            elif line.startswith('DATA'):
                in_data = True
    return np.array(pts, dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_json')
    ap.add_argument('gt_pcd')
    ap.add_argument('--vox', type=float, default=0.4)
    ap.add_argument('--range', type=float, default=6.0)
    args = ap.parse_args()

    run = json.load(open(args.run_json))
    traj = np.array(run['trajectory_5hz'], dtype=np.float64)   # [t,x,y,z,yaw]
    gt = load_pcd_xyz(args.gt_pcd)

    keys = np.floor(gt / args.vox).astype(np.int64)
    keys -= keys.min(axis=0)
    packed = (keys[:, 0] << 42) + (keys[:, 1] << 21) + keys[:, 2]
    n_total = len(np.unique(packed))

    observed = set()
    ts, ers = [], []
    for row in traj:
        origin = row[1:4] + LIDAR_OFFSET
        d = gt - origin
        dist = np.linalg.norm(d, axis=1)
        mask = dist < args.range
        zc = d[:, 2] / np.maximum(dist, 1e-6)
        mask &= (zc >= FOV_V_MIN) & (zc <= FOV_V_MAX)
        observed |= set(packed[mask].tolist())
        ts.append(row[0])
        ers.append(len(observed) / n_total)

    er_final = ers[-1] if ers else 0.0
    t90 = next((t for t, e in zip(ts, ers) if e >= 0.9 * er_final), None)
    t95 = next((t for t, e in zip(ts, ers) if e >= 0.95 * er_final), None)
    print('ER_final=%.1f%%  T90(相对)=%s  T95(相对)=%s  样本=%d  GT体素=%d' % (
        er_final * 100,
        ('%.1fs' % t90) if t90 is not None else '未达',
        ('%.1fs' % t95) if t95 is not None else '未达', len(traj), n_total))
    out = args.run_json.replace('.json', '_er_recomputed.csv')
    with open(out, 'w') as f:
        f.write('t_s,er\n')
        for t, e in zip(ts, ers):
            f.write('%.2f,%.4f\n' % (t, e))
    print('曲线 →', out)


if __name__ == '__main__':
    main()
