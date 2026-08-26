#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
探索算法统一评估器（go2-scan，2026-08-25）。

指标体系（来源见 docs/评估标准调研.md，均为公认定义）：
  A. 探索效率（Explore-Bench / MAexp 惯例）
     - ER(t)   探索率：已被传感器覆盖的 GT 表面体素 / GT 表面体素总数
     - T_90 / T_95：ER 达到最终值的 90%/95% 的用时（Explore-Bench 用绝对 90%/99%，
       我们对未知总覆盖的跑分用「相对终值」更稳，另报绝对 ER 终值）
     - L_path  轨迹总长；η_cov = ER_final / L_path（单位路径长度的覆盖率增益）
  B. 地图质量（Cloud_Map_Evaluation / SAMM / Occupency Networks 一系的标准定义）
     - Accuracy    = mean d(map→GT)（重建点到 GT 的最近邻距离）
     - Precision@τ = |d(map→GT)<τ| / N_map
     - Completeness= mean d(GT→map)；Recall@τ = |d(GT→map)<τ| / N_GT
     - F-score@τ = 调和平均(Precision@τ, Recall@τ)，τ∈{0.05,0.10,0.20}m
     - RMSE(inlier)：Accuracy 常伴报告
  简化声明：ER 计算不做遮挡剔除（range+FOV 可见性），是探索评测文献常见近似；
  MID360 为 360°×59°（垂直 -7°~+52°）。

用法：
  # 实时模式（仿真跑着时）
  python3 tools/evaluate_exploration.py --algo ariadne --scene indoor_1 \
      --out evaluation/results/
  # bag 模式（rosbag play 回放同样话题即可离线评）
  python3 tools/evaluate_exploration.py --bag run.bag --algo ariadne ...
输出：<out>/<algo>_<scene>_<时间戳>.json + .md + er_curve.csv
"""
import argparse
import json
import math
import os
import time as wall_time

import numpy as np

VOX = 0.4          # ER 体素边长(m)。GT 表面体素化分辨率——与 octomap res 同量级
TAUS = [0.05, 0.10, 0.20]
SENSOR_RANGE = 6.0          # 与 go2_ariadne.launch 用户终版一致
FOV_H_HALF = math.pi        # MID360 水平 360°
FOV_V_MIN = math.radians(-7.0)
FOV_V_MAX = math.radians(52.0)
LIDAR_OFFSET = np.array([0.2, 0.0, 0.2077])   # base→mid360 外参（同 gazebo_bridge）
PLATEAU_GAIN = 0.002        # ER 增益低于此视为平台期
SPEED_MIN = 0.05            # m/s，低于此视为罚站噪声不累计路径（2026-08-26 审查A2）


def pc2_to_xyz(cloud_msg):
    """PointCloud2 → Nx3 float（只用 xyz 字段）。"""
    from sensor_msgs import point_cloud2 as pc2
    pts = pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.array(list(pts), dtype=np.float32).reshape(-1, 3)


def voxelize(xyz, res):
    """点 → 体素 id 集合。"""
    if len(xyz) == 0:
        return set()
    keys = np.floor(xyz[:, :3] / res).astype(np.int64)
    # 单轴打包成一维 key（场景坐标有限，偏移保证非负）
    kmin = keys.min(axis=0)
    keys -= kmin
    packed = (keys[:, 0] << 42) + (keys[:, 1] << 21) + keys[:, 2]
    return set(packed.tolist())


def visible_voxels(gt_xyz, gt_keys_unique, pose_xy_yaw, max_range):
    """单帧位姿可见的 GT 体素 key 集（range+FOV 近似，无遮挡）。"""
    origin = pose_xy_yaw[:3] + LIDAR_OFFSET
    yaw = pose_xy_yaw[3]
    d = gt_xyz - origin
    dist = np.linalg.norm(d, axis=1)
    mask = dist < max_range
    # FOV 过滤（水平全向时跳过方位角判断）
    if FOV_H_HALF < math.pi:
        ang = np.arctan2(d[:, 1], d[:, 0]) - yaw
        mask &= np.abs(np.arctan2(np.sin(ang), np.cos(ang))) <= FOV_H_HALF
    zc = d[:, 2] / np.maximum(dist, 1e-6)
    mask &= (zc >= FOV_V_MIN) & (zc <= FOV_V_MAX)
    return set(gt_keys_unique[mask].tolist())


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def map_quality(scan_xyz, gt_xyz):
    """标准地图质量指标（KD-tree 双向最近邻）。双方已 0.1 体素下采样。"""
    from scipy.spatial import cKDTree
    out = {'n_scan': int(len(scan_xyz)), 'n_gt': int(len(gt_xyz))}
    if len(scan_xyz) == 0 or len(gt_xyz) == 0:
        out['error'] = 'empty cloud'
        return out
    tree_gt = cKDTree(gt_xyz)
    tree_map = cKDTree(scan_xyz)

    d_map_gt, _ = tree_gt.query(scan_xyz, k=1)      # accuracy 方向
    d_gt_map, _ = tree_map.query(gt_xyz, k=1)       # completeness 方向

    acc = d_map_gt.mean()
    inl = d_map_gt[d_map_gt <= TAUS[1]]
    out['accuracy_mean_m'] = float(acc)
    out['accuracy_median_m'] = float(np.median(d_map_gt))
    out['rmse_inlier_0p10'] = float(np.sqrt((inl ** 2).mean())) if len(inl) else None
    out['chamfer_sym_m'] = float(0.5 * (d_map_gt.mean() + d_gt_map.mean()))
    for tau in TAUS:
        p = float((d_map_gt <= tau).mean())
        r = float((d_gt_map <= tau).mean())
        f = 2 * p * r / (p + r) if (p + r) > 0 else 0.0
        tag = ('%05.2f' % (tau * 100)).replace('.', 'p')
        out['precision@%sm' % tag] = p
        out['recall@%sm' % tag] = r
        out['fscore@%sm' % tag] = f
    out['completeness_mean_m'] = float(d_gt_map.mean())
    return out


class LiveRecorder:
    """实时订阅记录：轨迹 + scan_map 快照 + GT。"""

    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        self.rospy = rospy
        self.traj = []           # [t, x, y, z, yaw]
        self.scan_final = None
        self.gt = None
        self.t_start = None
        rospy.Subscriber('/quad_0/body_pose', Odometry, self._odom_cb, queue_size=30)
        rospy.Subscriber('/scan_map', PointCloud2, self._scan_cb, queue_size=1)
        rospy.Subscriber('/map', PointCloud2, self._gt_cb, queue_size=1)

    def _odom_cb(self, msg):
        t = msg.header.stamp.to_sec()
        if self.t_start is None:
            self.t_start = t
        p = msg.pose.pose.position
        self.traj.append([t - self.t_start, p.x, p.y, p.z, quat_to_yaw(msg.pose.pose.orientation)])

    def _scan_cb(self, msg):
        self.scan_final = msg            # 累积图持续发布，保留最新

    def _gt_cb(self, msg):
        if self.gt is None:
            self.gt = msg                # GT latched，记一次就够


def evaluate(traj, scan_xyz, gt_xyz, algo, scene):
    traj = np.array(traj, dtype=np.float64)
    res = {'algo': algo, 'scene': scene, 'vox_res': VOX,
           'sensor_range': SENSOR_RANGE, 'taus': TAUS,
           'traj_samples': int(len(traj))}

    # ---- A. 探索效率 ----
    if len(traj) < 2:
        res['exploration'] = {'error': 'trajectory missing (<2 samples)，只有地图质量指标'}
        res['map_quality'] = {}
        return res, (np.zeros(0), np.zeros(0))
    dt = np.diff(traj[:, 0])
    step = np.hypot(np.diff(traj[:, 1]), np.diff(traj[:, 2]))
    valid = (dt > 0) & (dt < 1.0) & (step < 2.0)   # 防传送尖刺
    # 2026-08-26 审查A2修正：100Hz 下 dt<1.0 形同虚设，罚站期里程计噪声逐样本
    # 累积（Depot D1 实测站 33min 记 76m）。加速度门：只累计速度>0.05m/s 的位移。
    speed = np.where(dt > 0, step / np.maximum(dt, 1e-9), 0.0)
    moving = valid & (speed > SPEED_MIN)
    path_len = float(step[moving].sum())
    duration = float(traj[-1, 0] - traj[0, 0])
    move_time = float(dt[moving].sum())

    if len(gt_xyz) == 0:
        raise SystemExit('没有收到 /map（GT 点云）——map_pub 是否启动？')

    gt_keys_all = np.floor(gt_xyz / VOX).astype(np.int64)
    gt_keys_all -= gt_keys_all.min(axis=0)
    gt_packed = (gt_keys_all[:, 0] << 42) + (gt_keys_all[:, 1] << 21) + gt_keys_all[:, 2]
    gt_unique, gt_idx = np.unique(gt_packed, return_index=True)
    n_total = len(gt_unique)

    observed = set()
    curve_t, curve_er = [], []
    sample_every = max(1, len(traj) // 600)        # ~600 帧足够平滑
    first_cross = {0.90: None, 0.95: None}
    last_gain_t = 0.0
    prev_er = 0.0
    for i in range(0, len(traj), sample_every):
        row = traj[i]
        # 行结构 [t,x,y,z,yaw]：位姿取 [1:5]，别把 t 当 x（2026-08-25 ER 全零 bug 根因）
        vis = visible_voxels(gt_xyz, gt_packed, row[1:5], SENSOR_RANGE)
        observed |= vis
        er = len(observed) / n_total
        t = row[0]
        curve_t.append(t)
        curve_er.append(er)
        for q in first_cross:
            if first_cross[q] is None and er >= q:
                first_cross[q] = t
        if er - prev_er > PLATEAU_GAIN:
            last_gain_t = t
        prev_er = er
    er_final = prev_er

    # 退化跑自动判废。2026-08-26 审查B2修正：原 0.05×duration(=105s@35min) 被
    # 「~2min 正常探索后规划器失明」的新故障模式擦边逃过（D1-D3 plateau 116-144s），
    # 收紧到 15%×duration 并加 ER 上限条件——完整探索不应在 15% 时长内就停止增长。
    degraded = bool(duration > 0 and last_gain_t < 0.15 * duration and er_final < 0.95)
    res['exploration'] = {
        'duration_s': duration,
        'path_length_m': path_len,
        'move_time_s': move_time,
        'moving_ratio': round(move_time / duration, 3) if duration > 0 else None,
        'er_final': er_final,
        't90_rel_s': first_cross[0.90],
        't95_rel_s': first_cross[0.95],
        'plateau_time_s': last_gain_t,
        'eta_cov_per_m': er_final / path_len if path_len > 1 else None,
        'curve_len': len(curve_t),
        'degraded': degraded,
    }

    # ---- B. 地图质量 ----
    if len(scan_xyz):
        scan_ds = downsample(scan_xyz, 0.1)
        gt_ds = downsample(gt_xyz, 0.1)
        res['map_quality'] = map_quality(scan_ds, gt_ds)
    else:
        res['map_quality'] = {'error': 'no /scan_map received'}
    return res, (np.array(curve_t), np.array(curve_er))


def downsample(xyz, res):
    keys = np.floor(xyz / res).astype(np.int64)
    keys -= keys.min(axis=0)
    packed = (keys[:, 0] << 42) + (keys[:, 1] << 21) + keys[:, 2]
    _, idx = np.unique(packed, return_index=True)
    return xyz[np.sort(idx)]


def write_report(res, curve, out_dir, tag):
    os.makedirs(out_dir, exist_ok=True)
    ct, ce = curve
    with open(os.path.join(out_dir, '%s_er_curve.csv' % tag), 'w') as f:
        f.write('t_s,er\n')
        for a, b in zip(ct, ce):
            f.write('%.2f,%.4f\n' % (a, b))
    with open(os.path.join(out_dir, '%s.json' % tag), 'w') as f:
        # default=float：numpy 标量(np.bool_/np.float64)忘转类型时兜底，不再整份报告炸掉
        json.dump(res, f, indent=2, ensure_ascii=False, default=float)

    e = res['exploration']
    m = res.get('map_quality', {})
    if 'error' in e:
        lines = ['# 探索评估：%s @ %s（无轨迹，仅地图质量）' % (res['algo'], res['scene']), '']
        if 'error' not in m and m:
            lines.append('- Accuracy %.3fm｜Completeness %.3fm｜F@0.10 %.1f%%' % (
                m['accuracy_mean_m'], m['completeness_mean_m'], m['fscore@0p10m'] * 100))
        with open(os.path.join(out_dir, '%s.md' % tag), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print('\n'.join(lines))
        return
    lines = [
        '# 探索评估：%s @ %s' % (res['algo'], res['scene']),
        '',
        '**判废：%s**（degraded=%s，平台期 %s / duration %s）' % (
            '❌ 退化跑——数据不用于 A/B 对比' if e.get('degraded') else '✅ 正常',
            e.get('degraded'),
            ('%.0fs' % e['plateau_time_s']) if e.get('plateau_time_s') else '-',
            ('%.0fs' % e['duration_s']) if e.get('duration_s') else '-'),
        '',
        '## A. 探索效率（Explore-Bench 惯例）',
        '- 最终探索率 ER_final：**%.1f%%**' % (e['er_final'] * 100),
        '- 用时：duration %.1fs｜T90 %.1fs｜T95 %s｜平台期 %s' % (
            e['duration_s'],
            e['t90_rel_s'] if e['t90_rel_s'] is not None else -1,
            ('%.1fs' % e['t95_rel_s']) if e['t95_rel_s'] is not None else '未达',
            ('%.1fs' % e['plateau_time_s']) if e['plateau_time_s'] else '-'),
        '- 轨迹长度 %.1fm｜覆盖率效率 η=%.4f /m' % (e['path_length_m'], e['eta_cov_per_m'] or 0),
        '',
        '## B. 地图质量（Cloud_Map_Evaluation 标准定义，τ 阈值族）',
    ]
    if 'error' not in m:
        lines += [
            '- Accuracy(mean d(map→GT)) **%.3fm**｜median %.3fm｜RMSE(inl@0.1) %s' % (
                m['accuracy_mean_m'], m['accuracy_median_m'],
                ('%.3fm' % m['rmse_inlier_0p10']) if m['rmse_inlier_0p10'] else '-'),
            '- Completeness(mean d(GT→map)) %.3fm' % m['completeness_mean_m'],
            '| τ | Precision(清晰度/精度) | Recall(完整度) | F-score |',
            '|---|---|---|---|',
        ]
        for tau in TAUS:
            tag2 = ('%05.2f' % (tau * 100)).replace('.', 'p')
            lines.append('| %.2fm | %.1f%% | %.1f%% | %.1f%% |' % (
                tau, m['precision@%sm' % tag2] * 100,
                m['recall@%sm' % tag2] * 100, m['fscore@%sm' % tag2] * 100))
        lines.append('- 对称 Chamfer：%.3fm（scan %d pts vs GT %d pts，均 0.1 下采样）' % (
            m['chamfer_sym_m'], m['n_scan'], m['n_gt']))
    else:
        lines.append('- 无 scan_map：%s' % m['error'])
    lines += ['', '> ER 简化：range+FOV 可见性、无遮挡剔除（文献常见近似）；'
                 '体素 %.1fm；量程 %.1fm。' % (VOX, SENSOR_RANGE)]
    with open(os.path.join(out_dir, '%s.md' % tag), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print('\n>>> 输出目录：%s' % out_dir)


def load_pcd_xyz(path):
    """读 ASCII XYZ PCD（与 probe_occ_vs_gt.py 同款轻量实现）。"""
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
    return np.array(pts, dtype=np.float32).reshape(-1, 3)


def fetch_full_scan_map():
    """调 /scan_map/save 拿全量累积图（/scan_map 话题有 1e5 点随机抽稀，不能用于评测）。"""
    import rospy
    try:
        from std_srvs.srv import Trigger, TriggerRequest
        rospy.wait_for_service('/scan_map/save', timeout=5)
        resp = rospy.ServiceProxy('/scan_map/save', Trigger)(TriggerRequest())
        if resp.success:
            path = resp.message.split('to ')[-1].strip()
            xyz = load_pcd_xyz(path)
            rospy.loginfo('[eval] 全量 scan_map %d pts ← %s', len(xyz), path)
            return xyz
    except Exception as e:
        print('全量 scan_map 获取失败(%s)，退回话题快照' % e)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--algo', default='ariadne')
    ap.add_argument('--scene', default='indoor_1')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'evaluation', 'results'))
    ap.add_argument('--bag', default=None, help='rosbag 文件（离线回放模式由 rosbag play 完成，此处仅命名用）')
    ap.add_argument('--live-min', type=float, default=0.0, help='实时模式下最少录制秒数后退出')
    args = ap.parse_args()

    if args.bag:
        raise SystemExit('请用 rosbag play %s 回放话题，再以无 --bag 实时模式运行本脚本' % args.bag)

    import rospy
    rospy.init_node('exploration_evaluator', anonymous=True)
    rec = LiveRecorder()
    rate = getattr(rec, 'rospy').Rate(5)
    print('录制中（Ctrl-C 结束并出报告；GT=/map scan=/scan_map 轨迹=/quad_0/body_pose）...')
    try:
        while not getattr(rec, 'rospy').is_shutdown():
            rate.sleep()
            if len(rec.traj) >= 2 and args.live_min > 0:
                dur = rec.traj[-1][0] - rec.traj[0][0]
                if dur >= args.live_min:
                    break
    except KeyboardInterrupt:
        pass

    traj = rec.traj
    scan_xyz = fetch_full_scan_map()
    if scan_xyz is None or len(scan_xyz) == 0:
        scan_xyz = pc2_to_xyz(rec.scan_final) if rec.scan_final is not None else np.zeros((0, 3))
    gt_xyz = pc2_to_xyz(rec.gt) if rec.gt is not None else np.zeros((0, 3))
    print('轨迹 %d 帧｜scan_map %d pts｜GT %d pts' % (len(traj), len(scan_xyz), len(gt_xyz)))

    res, curve = evaluate(traj, scan_xyz, gt_xyz, args.algo, args.scene)
    # 轨迹降采样持久化（~5Hz）：ER 算法有 bug 时可离线重算，不用重跑仿真
    if len(traj) > 10:
        res['trajectory_5hz'] = np.array(traj)[::50].round(4).tolist()
    stamp = wall_time.strftime('%Y%m%d_%H%M%S')
    tag = '%s_%s_%s' % (args.algo, args.scene, stamp)
    write_report(res, curve, args.out, tag)


if __name__ == '__main__':
    main()
