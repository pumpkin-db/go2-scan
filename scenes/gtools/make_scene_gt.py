#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
场景 GT 点云生成：从 DAE 网格子网格采样 → PCD + 几何报告（bbox/台阶分析）。
依赖：pip install --user trimesh pycollada（纯 python，无需 root）。

用途：
  1. 评估器的 GT（/map 话题源 PCD）；
  2. 报告 STAIRS 子网格的台阶几何（踏步高/深、总升高），验证 Go2 可爬性
     并给楼梯检测层提供真值参数。

用法：
  python3 make_scene_gt.py --model-dir scenes/depot/model/assets \
      --dae meshes/Depot.dae --scale 0.6 \
      --submeshes WALLS,FLOOR,STAIRS,PILLERS,BOXSET,ROOF \
      --out-gt scenes/depot/gt/depot.pcd --n 2000000
"""
import argparse
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-dir', required=True)
    ap.add_argument('--dae', default='meshes/Depot.dae')
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--submeshes', required=True)
    ap.add_argument('--out-gt', required=True)
    ap.add_argument('--n', type=int, default=2_000_000, help='总面积采样点数')
    ap.add_argument('--report', action='store_true')
    args = ap.parse_args()

    import trimesh

    dae_path = os.path.join(args.model_dir, args.dae)
    scene_or_mesh = trimesh.load(dae_path, force='scene',
                                 group_material=False, skip_materials=True)
    # collada 场景的 geometry 名即 submesh 名
    geoms = {}
    for name, g in scene_or_mesh.geometry.items():
        if g is not None and len(g.faces):
            # Collada 导出常带 '-lib' 后缀（如 WALLS-lib），归一化掉；原名也保留
            base = name[:-4] if name.endswith('-lib') else name
            geoms[base] = g
            geoms[name] = g
    print('可用 submesh:', sorted(geoms.keys()))
    wanted = [s.strip() for s in args.submeshes.split(',') if s.strip()]

    meshes = []
    for w in wanted:
        if w not in geoms:
            print('!! 找不到 submesh:', w)
            continue
        m = geoms[w].copy()
        m.apply_scale(args.scale)
        meshes.append((w, m))
        bb = m.bounds
        print('%-10s bbox min=[%.2f %.2f %.2f] max=[%.2f %.2f %.2f] 面积=%.0fm²' % (
            w, *bb[0], *bb[1], m.area))

    # 按面积比例分配采样点
    areas = np.array([m.area for _, m in meshes])
    pts_all = []
    for (name, m), frac in zip(meshes, args.n * areas / areas.sum()):
        if len(m.faces) == 0:
            continue
        pts, _ = trimesh.sample.sample_surface(m, int(frac))
        pts_all.append(pts)
        if name == 'STAIRS' and args.report:
            analyze_stairs(m)
    cloud = np.vstack(pts_all).astype(np.float32)

    os.makedirs(os.path.dirname(args.out_gt), exist_ok=True)
    with open(args.out_gt, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\n')
        f.write('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n' % len(cloud))
        f.write('POINTS %d\nDATA ascii\n' % len(cloud))
        for row in cloud:
            f.write('%.4f %.4f %.4f\n' % (row[0], row[1], row[2]))
    print('GT 点云 %d pts → %s' % (len(cloud), args.out_gt))


def analyze_stairs(m):
    """对 STAIRS 子网格做台阶几何分析：找主爬升方向，统计 z 直方图台阶数。"""
    # 台阶面法向应接近 ±z 的面 = 踏面
    normals = m.face_normals
    up = np.abs(normals[:, 2]) > 0.9
    treads = m.faces[up]
    tz = m.triangles_center[up][:, 2]
    zs = np.sort(np.unique(np.round(tz, 3)))
    if len(zs) < 2:
        print('STAIRS 分析：未找到明显水平踏面')
        return
    # 聚类相近 z
    levels = []
    cur = [zs[0]]
    for z in zs[1:]:
        if z - cur[-1] <= 0.03:
            cur.append(z)
        else:
            levels.append(float(np.mean(cur)))
            cur = [z]
    levels.append(float(np.mean(cur)))
    rises = np.diff(levels)
    print('=== STAIRS 台阶几何 ===')
    print('水平踏面层级数: %d' % len(levels))
    print('层级 z: %s' % np.round(levels, 3))
    if len(rises):
        print('相邻层高差: min=%.3f max=%.3f mean=%.3f (m)' % (
            rises.min(), rises.max(), rises.mean()))
    print('总升高: %.3f m' % (levels[-1] - levels[0]))
    # 踏面深度：沿 x/y 展开看踏面前后缘间距（粗略：踏面中心点间距）
    tc = m.triangles_center[up]
    if len(tc) > 2:
        span = tc.max(axis=0) - tc.min(axis=0)
        print('踏面点云跨度 dx=%.2f dy=%.2f（结合层数可估进深）' % (span[0], span[1]))


if __name__ == '__main__':
    main()
