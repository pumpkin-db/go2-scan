#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 脚本：2D 占据栅格 PGM (map_server 格式) -> 3D 墙体点云 PCD (PointXYZ)
用途：给 SCAN-Planner 的 map_pub (map_generator) 当地图源，替换默认 mockamap 随机柱。
规则（map_server 约定，negate=0）：occupancy=(255-pixel)/255；> occupied_thresh 为占据。
占据格沿 z 挤出成墙（默认 0~2m，每 0.1m 一层）。未知/空闲格不出点。
用法：python3 pgm_to_pcd.py <map.yaml> <out.pcd> [wall_height] [z_step]
"""
import sys
import numpy as np


def parse_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # P5 头：magic、可能有注释行、width height、maxval，然后单空白符接二进制
    idx = 0

    def next_token():
        nonlocal idx
        while True:
            # 跳空白
            while idx < len(data) and data[idx:idx+1].isspace():
                idx += 1
            if data[idx:idx+1] == b"#":  # 注释到行尾
                while idx < len(data) and data[idx:idx+1] != b"\n":
                    idx += 1
                continue
            start = idx
            while idx < len(data) and not data[idx:idx+1].isspace():
                idx += 1
            return data[start:idx]

    magic = next_token()
    assert magic == b"P5", "只支持 P5 二值 PGM"
    width = int(next_token())
    height = int(next_token())
    maxval = int(next_token())
    assert maxval == 255, f"maxval={maxval} 非 255，未支持"
    idx += 1  # 头与数据之间的单个空白符
    img = np.frombuffer(data[idx:idx + width * height], dtype=np.uint8)
    img = img.reshape(height, width)  # 行 0 = 图像顶部 = 地图最大 y
    return img


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    yaml_path, out_path = sys.argv[1], sys.argv[2]
    wall_height = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    z_step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1

    # 极简 yaml 解析（只取需要的键）
    res = origin = None
    with open(yaml_path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("resolution:"):
                res = float(line.split(":")[1])
            elif line.startswith("origin:"):
                origin = [float(v) for v in line.split(":")[1].strip(" []").split(",")]
    assert res and origin, "yaml 缺 resolution/origin"
    ox, oy = origin[0], origin[1]

    img = parse_pgm(yaml_path.replace(".yaml", ".pgm"))
    h, w = img.shape
    occ = (255.0 - img.astype(np.float32)) / 255.0
    occupied = occ > 0.65  # occupied_thresh
    print(f"地图: {w}x{h} 格, 分辨率 {res}m, 原点 ({ox},{oy})")
    print(f"世界范围: x [{ox:.2f}, {ox + w*res:.2f}], y [{oy:.2f}, {oy + h*res:.2f}]")
    print(f"占据格: {occupied.sum()} / {occupied.size}")

    ys, xs = np.where(occupied)
    # 图像行 0 在顶部 = 最大 y：world_y = oy + (h - 1 - row + 0.5)*res
    wx = ox + (xs + 0.5) * res
    wy = oy + (h - 1 - ys + 0.5) * res
    zs = np.arange(0.0, wall_height + 1e-9, z_step)
    X = np.repeat(wx, len(zs))
    Y = np.repeat(wy, len(zs))
    Z = np.tile(zs, len(wx))

    # 找安全出生点：空闲且周围 5x5 格全空闲的候选
    free = occ < 0.196
    cand = []
    k = 4  # 0.6m 半径
    for r in range(k, h - k, 6):
        for c in range(k, w - k, 6):
            win = ~free[r - k:r + k + 1, c - k:c + k + 1]
            if not win.any():
                cand.append((ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res))
    print(f"安全空闲候选点(世界坐标): 共 {len(cand)} 个")
    for p in cand[:12]:
        print(f"  ({p[0]:.2f}, {p[1]:.2f})")

    # 写 ASCII PCD (PointXYZ)
    n = len(X)
    with open(out_path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        for i in range(n):
            f.write(f"{X[i]:.3f} {Y[i]:.3f} {Z[i]:.3f}\n")
    print(f"已写出 {out_path}: {n} 点")


if __name__ == "__main__":
    main()
