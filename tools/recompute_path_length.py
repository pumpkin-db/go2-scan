#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线重算历史评估 JSON 的路径长与判废分类（2026-08-26 审查A2/B2 修正）。

背景：evaluate_exploration.py 原 path_length 过滤 (dt<1.0 & step<2.0) 在 ~100Hz
采样下形同虚设，罚站期里程计噪声逐样本累积（Depot D1 站 33min 记 76m）。
本脚本读各 JSON 内嵌的 trajectory_5hz（每50帧≈5Hz降采样），按新口径
（速度>0.05m/s 才累计）重算净路径长、移动时间占比，并用收紧后的判废线
（last_gain_t < 15%×duration 且 er_final<0.95）重新分类。

用法：python3 tools/recompute_path_length.py [results目录]
输出：逐跑对照表（stdout），不改动原始 JSON——原始数据保留，重算值另存
      <原文件名>.recomputed.json。
"""
import json
import os
import sys

import numpy as np

SPEED_MIN = 0.05          # 与 evaluate_exploration.py 保持一致
PLATEAU_GAIN = 0.002


def recompute(path):
    d = json.load(open(path))
    traj = d.get('trajectory_5hz')
    out = {'file': os.path.basename(path), 'algo': d.get('algo'), 'scene': d.get('scene')}
    exp = d.get('exploration', {})
    if not traj or len(traj) < 3:
        out['note'] = '无轨迹数据'
        return out, None
    traj = np.array(traj, dtype=np.float64)   # [t,x,y,z,yaw]
    dt = np.diff(traj[:, 0])
    step = np.hypot(np.diff(traj[:, 1]), np.diff(traj[:, 2]))
    speed = np.where(dt > 0, step / np.maximum(dt, 1e-9), 0.0)
    moving = (dt > 0) & (speed > SPEED_MIN)
    net_len = float(step[moving].sum())
    move_time = float(dt[moving].sum())
    duration = float(traj[-1, 0] - traj[0, 0])
    # 旧口径（原评估器逻辑）对照
    old_valid = (dt > 0) & (dt < 1.0) & (step < 2.0)
    old_len = float(step[old_valid].sum())
    er_final = float(exp.get('er_final', 0.0))
    plateau = float(exp.get('plateau_time_s') or 0.0)
    new_degraded = bool(duration > 0 and plateau < 0.15 * duration and er_final < 0.95)
    out.update({
        'old_path_m': round(old_len, 2),
        'net_path_m': round(net_len, 2),
        'water_ratio': round(old_len / net_len, 2) if net_len > 0.1 else None,
        'moving_ratio': round(move_time / duration, 3) if duration > 0 else None,
        'duration_s': round(duration, 1),
        'plateau_s': round(plateau, 1),
        'er_final': round(er_final, 4),
        'new_degraded': new_degraded,
    })
    # 重算后的 eta（覆盖率效率）
    out['eta_cov_per_m_net'] = round(er_final / net_len, 6) if net_len > 1 else None
    return out, (traj, dt, step, speed, moving)


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), '..', 'evaluation', 'results')
    files = sorted(f for f in os.listdir(results_dir) if f.endswith('.json')
                   and not f.endswith('.recomputed.json'))
    rows = []
    for fn in files:
        try:
            row, _ = recompute(os.path.join(results_dir, fn))
        except Exception as e:                      # 残件 JSON 解析失败也要留痕
            row = {'file': fn, 'note': '解析失败: %s' % e}
        rows.append(row)
        with open(os.path.join(results_dir, fn.replace('.json', '.recomputed.json')), 'w') as f:
            json.dump(row, f, indent=2, ensure_ascii=False)

    print('%-44s %-9s %8s %8s %7s %7s %8s %s' % (
        'file', 'scene', 'old_m', 'net_m', 'water%', 'move%', 'plateau_s', 'degraded'))
    for r in rows:
        print('%-44s %-9s %8s %8s %7s %7s %8s %s' % (
            r.get('file', '?')[:44], r.get('scene', '-'),
            r.get('old_path_m', '-'), r.get('net_path_m', '-'),
            r.get('water_ratio', '-'), r.get('moving_ratio', '-'),
            r.get('plateau_s', '-'),
            ('⚠️%s' % r['new_degraded']) if 'new_degraded' in r else r.get('note', '-')))


if __name__ == '__main__':
    main()
