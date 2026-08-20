#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glue 脚本：Gazebo .world -> 3D 障碍物点云 PCD (PointXYZ)
用途：把 HPHS/CMU 的 Gazebo 场景（indoor_1.world 等）转成 SCAN-Planner 的
      map_pub 地图源，替换默认 mockamap 随机柱。
只采样内嵌几何体 box/cylinder/sphere（墙、障碍物）；model:// 外部 mesh 跳过
（那些是家具装饰，mesh 文件缺失，Gazebo 里本来也不显示）。
pose 层级累积：世界位姿 = model_pose ⊕ link_pose ⊕ collision_pose。

用法：python3 world_to_pcd.py <in.world> <out.pcd> [resolution] [min_z] [max_z]
"""
import sys
import xml.etree.ElementTree as ET
import numpy as np


def pose_to_matrix(pose_str):
    """'x y z roll pitch yaw' -> 4x4 变换矩阵（R = Rz*yaw @ Ry*pitch @ Rx*roll）"""
    if pose_str is None:
        return np.eye(4)
    v = [float(x) for x in pose_str.split()]
    if len(v) == 0:
        return np.eye(4)
    x, y, z = v[0], v[1], v[2]
    roll, pitch, yaw = v[3] if len(v) > 3 else 0, v[4] if len(v) > 4 else 0, v[5] if len(v) > 5 else 0
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def sample_box(sx, sy, sz, res):
    """box 六面表面点云（局部坐标，中心原点）"""
    pts = []
    xs, ys, zs = sx / 2, sy / 2, sz / 2
    # ±x 面
    for s in (-xs, xs):
        gy = np.arange(-ys, ys + res, res)
        gz = np.arange(-zs, zs + res, res)
        Y, Z = np.meshgrid(gy, gz)
        pts.append(np.stack([np.full(Y.size, s), Y.ravel(), Z.ravel()], 1))
    # ±y 面
    for s in (-ys, ys):
        gx = np.arange(-xs, xs + res, res)
        gz = np.arange(-zs, zs + res, res)
        X, Z = np.meshgrid(gx, gz)
        pts.append(np.stack([X.ravel(), np.full(X.size, s), Z.ravel()], 1))
    # ±z 面
    for s in (-zs, zs):
        gx = np.arange(-xs, xs + res, res)
        gy = np.arange(-ys, ys + res, res)
        X, Y = np.meshgrid(gx, gy)
        pts.append(np.stack([X.ravel(), Y.ravel(), np.full(X.size, s)], 1))
    return np.vstack(pts) if pts else np.zeros((0, 3))


def sample_cylinder(r, length, res):
    """cylinder 侧面 + 上下底面（轴沿 z，中心原点）"""
    pts = []
    h = length / 2
    n_ang = max(int(2 * np.pi * r / res), 16)
    ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
    zs = np.arange(-h, h + res, res)
    A, Z = np.meshgrid(ang, zs)
    x = r * np.cos(A.ravel())
    y = r * np.sin(A.ravel())
    pts.append(np.stack([x, y, Z.ravel()], 1))
    # 上下底面
    nr = max(int(r / res), 4)
    rr = np.linspace(0, r, nr)
    R, A2 = np.meshgrid(rr, ang)
    for s in (-h, h):
        x = R.ravel() * np.cos(A2.ravel())
        y = R.ravel() * np.sin(A2.ravel())
        pts.append(np.stack([x, y, np.full(x.size, s)], 1))
    return np.vstack(pts)


def sample_sphere(r, res):
    """球面经纬度采样"""
    n_ang = max(int(2 * np.pi * r / res), 12)
    ang = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
    n_el = max(int(np.pi * r / res), 8)
    elev = np.linspace(-np.pi / 2, np.pi / 2, n_el)
    A, E = np.meshgrid(ang, elev)
    x = r * np.cos(E.ravel()) * np.cos(A.ravel())
    y = r * np.cos(E.ravel()) * np.sin(A.ravel())
    z = r * np.sin(E.ravel())
    return np.stack([x, y, z], 1)


def parse_world(path, res, min_z, max_z):
    tree = ET.parse(path)
    root = tree.getroot()
    world = root.find('world')
    if world is None:
        world = root  # 兼容无 <world> 包裹的情况
    # Gazebo 加载时用 <state> 块记录的模型实际位姿覆盖顶层定义 pose（关键！）
    state_poses = {}
    state = world.find('state')
    if state is not None:
        for sm in state.findall('model'):
            nm = sm.get('name')
            sp = sm.findtext('pose')
            if nm and sp:
                state_poses[nm] = pose_to_matrix(sp)
    allpts = []
    n_box = n_cyl = n_sph = n_mesh = 0
    for model in world.findall('model'):  # 只取 world 直接子级 model，跳过 <state>
        nm = model.get('name')
        M = state_poses.get(nm, pose_to_matrix(model.findtext('pose')))
        for link in model.findall('link'):
            L = M @ pose_to_matrix(link.findtext('pose'))
            for coll in link.findall('collision'):
                C = L @ pose_to_matrix(coll.findtext('pose'))
                geo = coll.find('geometry')
                if geo is None:
                    continue
                pts = None
                if geo.find('box') is not None:
                    sz = [float(x) for x in geo.findtext('box/size').split()]
                    pts = sample_box(*sz, res); n_box += 1
                elif geo.find('cylinder') is not None:
                    r = float(geo.findtext('cylinder/radius'))
                    ln = float(geo.findtext('cylinder/length'))
                    pts = sample_cylinder(r, ln, res); n_cyl += 1
                elif geo.find('sphere') is not None:
                    r = float(geo.findtext('sphere/radius'))
                    pts = sample_sphere(r, res); n_sph += 1
                elif geo.find('mesh') is not None:
                    n_mesh += 1
                if pts is not None and len(pts):
                    pts = (C[:3, :3] @ pts.T).T + C[:3, 3]
                    allpts.append(pts)
    pc = np.vstack(allpts) if allpts else np.zeros((0, 3))
    mask = (pc[:, 2] >= min_z) & (pc[:, 2] <= max_z)
    pc = pc[mask]
    print(f"几何体: box={n_box} cylinder={n_cyl} sphere={n_sph} mesh(跳过)={n_mesh}")
    print(f"点云: {len(pc)} 点, z 范围 [{min_z},{max_z}]")
    if len(pc):
        print(f"bbox: x [{pc[:,0].min():.2f}, {pc[:,0].max():.2f}] "
              f"y [{pc[:,1].min():.2f}, {pc[:,1].max():.2f}] "
              f"z [{pc[:,2].min():.2f}, {pc[:,2].max():.2f}]")
    return pc


def find_spawn(pc, z, clearance=1.0, step=0.5):
    """找安全出生点：z 高度附近，xy 平面 clear 半径内无点"""
    if len(pc) == 0:
        return (0.0, 0.0)
    xmin, xmax = pc[:, 0].min(), pc[:, 0].max()
    ymin, ymax = pc[:, 1].min(), pc[:, 1].max()
    zlow = pc[(pc[:, 2] >= z - 0.3) & (pc[:, 2] <= z + 0.3)]
    best = None
    for x in np.arange(xmin, xmax + step, step):
        for y in np.arange(ymin, ymax + step, step):
            d = np.sqrt((zlow[:, 0] - x) ** 2 + (zlow[:, 1] - y) ** 2)
            if d.size and d.min() > clearance:
                return (round(x, 2), round(y, 2))
    return (round((xmin + xmax) / 2, 2), round((ymin + ymax) / 2, 2))


def write_pcd(pc, out_path):
    n = len(pc)
    with open(out_path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        for i in range(n):
            f.write(f"{pc[i,0]:.3f} {pc[i,1]:.3f} {pc[i,2]:.3f}\n")
    print(f"已写出 {out_path}: {n} 点")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    in_path, out_path = sys.argv[1], sys.argv[2]
    res = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    min_z = float(sys.argv[4]) if len(sys.argv) > 4 else -0.5
    max_z = float(sys.argv[5]) if len(sys.argv) > 5 else 5.0

    pc = parse_world(in_path, res, min_z, max_z)
    write_pcd(pc, out_path)

    sp = find_spawn(pc, z=0.25, clearance=1.0)
    print(f"建议出生点 (init_x init_y init_z): {sp[0]} {sp[1]} 0.25")


if __name__ == "__main__":
    main()
