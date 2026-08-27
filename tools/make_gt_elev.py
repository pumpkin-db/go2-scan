#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""GT 高程网格预计算（terrain_follow 仿真数据源）。

背景（2026-08-26 B 验收发现）：感知高程图（elevation_mapping）在 Depot 楼梯区
不可用——2.5D 每格单值，台阶面与天花板(6-9m)同格融合后报天花板、下段被
visibility_cleanup 清成 NaN；且 go2_kinematic_sim 的 sampleElevation 行列假设
与 grid_map 实际布局相反（dim[0]=y），双重失效。设计文档（阶段B）本就预留
「查场景 GT 高程 f(x,y)」路线——仿真用 GT（干净、无天花板），感知路线留实机。

方法（非算法创新）：GT 表面点云按 res 体素化，每格取 z ∈ [z_min, z_max] 的
最高点为高程（z_max 默认 4.0 滤天花板/夹层），空格用邻域最近有效值填充
（狗贴边走不 NaN）。输出二进制：
    int32 nx, ny | float32 x0, y0, res | float32 h[nx*ny]（row-major, ix+iy*nx）

用法：/usr/bin/python3 tools/make_gt_elev.py <gt_pcd> <out_bin> [z_max] [res]
"""
import struct
import sys

import numpy as np


def load_pcd(path):
    with open(path) as f:
        while True:
            line = f.readline()
            if line.startswith('DATA'):
                data_type = line.split()[1]
                break
        if data_type != 'ascii':
            raise SystemExit('只支持 ascii PCD（当前 GT 生成脚本输出 ascii）')
        a = np.array(f.read().split(), dtype=np.float32).reshape(-1, 3)
    return a


def main():
    gt_pcd, out_bin = sys.argv[1], sys.argv[2]
    z_max = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
    res = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05

    pts = load_pcd(gt_pcd)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = z <= z_max
    x, y, z = x[keep], y[keep], z[keep]
    print('GT 点 %d → z<=%.1f 后 %d' % (len(pts), z_max, len(x)))

    x0, y0 = x.min(), y.min()
    nx = int(np.ceil((x.max() - x0) / res)) + 1
    ny = int(np.ceil((y.max() - y0) / res)) + 1
    ix = ((x - x0) / res).astype(np.int64)
    iy = ((y - y0) / res).astype(np.int64)
    grid = np.full(nx * ny, np.nan, dtype=np.float32)
    # 每格取最高 z：按 h 排序后直接覆盖写
    order = np.argsort(z)
    grid[iy[order] * nx + ix[order]] = z[order].astype(np.float32)

    # 空格最近有效值填充（迭代 3×3 膨胀，够填台阶缝隙）
    g = grid.reshape(ny, nx)
    from scipy import ndimage
    for _ in range(64):
        bad = np.isnan(g)
        if not bad.any():
            break
        filled = ndimage.grey_dilation(g, size=(3, 3))
        g[bad & ~np.isnan(filled)] = filled[bad & ~np.isnan(filled)]
    print('填充后 NaN 剩余: %d' % int(np.isnan(g).sum()))

    with open(out_bin, 'wb') as f:
        f.write(struct.pack('<iiff', nx, ny, x0, y0))
        f.write(struct.pack('<f', res))
        f.write(g.astype('<f4').tobytes())
    print('写出 %s: nx=%d ny=%d res=%.3f 高程范围 [%.3f, %.3f]'
          % (out_bin, nx, ny, res, np.nanmin(g), np.nanmax(g)))


if __name__ == '__main__':
    main()
