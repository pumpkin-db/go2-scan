#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
把厘米单位的 COLLADA 重导出为「顶点已换算成米」的 STL（无单位歧义），
供碰撞几何使用；visual 保持原 DAE 不动。

背景：Depot.dae 声明 <unit meter="0.01">，trimesh 等按声明换算（GT 点云 ±15.3m 正确），
但若 Gazebo classic 的 ColladaLoader 忽略该声明（待实测裁决），网格会以毫米原值加载
→ 墙在 ±1500m 外，40m 量程雷达全 miss。STL 没有单位元数据，绕开一切歧义。

用法：reexport_mesh_meters.py <in.dae> <out.stl> [合并组名列表，逗号分隔，默认全部]
"""
import sys

import numpy as np
import trimesh


def main():
    src, dst = sys.argv[1], sys.argv[2]
    keep = [s.strip() for s in sys.argv[3].split(',')] if len(sys.argv) > 3 else None

    scene = trimesh.load(src, force='scene', process=False)
    parts = []
    total = 0
    for name, geom in scene.geometry.items():
        if keep and not any(name.startswith(k) or k.startswith(name) for k in keep):
            continue
        m = geom.copy()
        # COLLADA 厘米→米：trimesh 已按 scene.units 处理过？保险起见检查量纲
        extent = m.extents.max()
        if extent > 100:          # 还在百米以上 → 未换算，手动 ×0.01
            m.apply_scale(0.01)
        parts.append(m)
        total += len(m.faces)
        print('%-16s 面 %7d  换算后包围盒 %.2f x %.2f x %.2f m'
              % (name, len(m.faces), *m.extents))
    if not parts:
        print('没有匹配的组'); sys.exit(1)
    merged = trimesh.util.concatenate(parts)
    merged.export(dst)
    print('导出 %d 面 -> %s' % (total, dst))


if __name__ == '__main__':
    main()
