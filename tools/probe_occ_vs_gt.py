#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：projected_map 占据格 vs GT 点云逐格对照。
对每个占据格中心，查 GT 点云在 [z_min,z_max] 带内、该格水平范围内是否有点。
无 GT 支撑的占据格 = 幻影占据。输出幻影占比与空间分布。"""
import sys, struct
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

Z_MIN, Z_MAX = 0.20, 0.80   # 对齐 octomap 投影带 [0.2,0.8]@res0.2：被投影空间 = 中心0.3层[0.2,0.4] ∪ 0.5层[0.4,0.6] ∪ 0.7层[0.6,0.8]

def load_pcd_xyz(path):
    with open(path, 'rb') as f:
        data = f.read()
    # 解析头部
    fields, sizes, types, counts = [], [], [], []
    data_mode, header_end = None, 0
    pos = 0
    for line in data.split(b'\n'):
        pos += len(line) + 1
        s = line.decode('ascii', 'ignore').strip()
        if s.startswith('FIELDS'): fields = s.split()[1:]
        elif s.startswith('SIZE'): sizes = [int(v) for v in s.split()[1:]]
        elif s.startswith('TYPE'): types = s.split()[1:]
        elif s.startswith('COUNT'): counts = [int(v) for v in s.split()[1:]]
        elif s.startswith('DATA'): data_mode = s.split()[1]; header_end = pos; break
    xi = fields.index('x')
    n_fields = len(fields)
    if data_mode == 'ascii':
        arr = np.loadtxt(data[header_end:].split(b'\n'), comments='#', dtype=np.float32)
        pts = arr.reshape(-1, n_fields)[:, xi:xi+3]
    else:  # binary（暂不支持 compressed）
        dt = np.dtype({'names': fields, 'formats': ['<f4' if t=='F' and sz==4 else ('<f8' if t=='F' else ('<u2' if sz==2 else '<u1')) for t, sz in zip(types, sizes)]})
        raw = np.frombuffer(data[header_end:], dtype=dt, count=-1)
        pts = np.stack([raw['x'], raw['y'], raw['z']], axis=-1).astype(np.float64)
    return pts

def main():
    pcd_path = sys.argv[1] if len(sys.argv) > 1 else '/home/pumpkin-db/claude/raicom/go2-scan/maps/indoor_1.pcd'
    gt = load_pcd_xyz(pcd_path)
    print(f"GT 点数 {len(gt)}, z 范围 [{gt[:,2].min():.2f},{gt[:,2].max():.2f}]")
    band = gt[(gt[:,2] >= Z_MIN) & (gt[:,2] <= Z_MAX)]
    print(f"GT 在 z∈[{Z_MIN},{Z_MAX}] 带内点数 {len(band)}")

    rospy.init_node('occ_vs_gt', disable_signals=True)
    msg = rospy.wait_for_message('/projected_map', OccupancyGrid, timeout=15)
    g = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    ox, oy, res = msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution

    occ_idx = np.argwhere(g == 100)
    phantom, real = [], []
    half = res / 2 + 0.05
    for r, c in occ_idx:
        cx, cy = ox + c*res + res/2, oy + r*res + res/2
        near = band[(np.abs(band[:,0]-cx) <= half) & (np.abs(band[:,1]-cy) <= half)]
        (real if len(near) >= 3 else phantom).append((cx, cy))
    print(f"\n占据格共 {len(occ_idx)}：有 GT 支撑 {len(real)} 格，幻影(带内无点) {len(phantom)} 格")
    if phantom:
        ph = np.array(phantom)
        print(f"幻影分布范围 x[{ph[:,0].min():.1f},{ph[:,0].max():.1f}] y[{ph[:,1].min():.1f},{ph[:,1].max():.1f}]")
        print("=== 幻影占据格 ASCII 图（P=幻影 # =真支撑 .=自由 空格=未知）===")
        pset = set((round(x,1), round(y,1)) for x, y in phantom)
        rset = set((round(x,1), round(y,1)) for x, y in real)
        for rr in range(g.shape[0])[::-1]:
            line = ''
            for cc in range(g.shape[1]):
                key = (round(ox + cc*res + res/2,1), round(oy + rr*res + res/2,1))
                line += 'P' if key in pset else ('#' if key in rset else ('.' if g[rr,cc]==0 else ' '))
            print(line)

if __name__ == '__main__':
    main()
