#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
生成「碰撞用 STL」变体模型：collision 的 mesh uri 换成米制 STL（Depot_collision.stl，
顶点已按 cm→m 换算），保留原 scale 0.6（最终尺寸 = raw×0.01×0.6 = GT 尺寸）；visual 不动。

背景：Gazebo classic 疑似无视 COLLADA <unit meter="0.01"> 声明（待本变体实测裁决），
导致网格放大 100 倍、地板变成实心厚板把狗埋掉。
用法：make_stl_collision_variant.py <out_dir>
"""
import re
import shutil
import sys
import os

src_sdf = '/home/pumpkin-db/claude/raicom/go2-scan/scenes/depot/model/assets/model.sdf'
assets = os.path.dirname(src_sdf)
out_root = sys.argv[1] if len(sys.argv) > 1 else '/tmp/depot_variants/stlcol'

os.makedirs(out_root + '/Depot', exist_ok=True)
shutil.copy(assets + '/model.config', out_root + '/Depot/')
for link in ('meshes', 'materials'):
    p = out_root + '/Depot/' + link
    if not os.path.islink(p):
        os.symlink(assets + '/' + link, p)

text = open(src_sdf).read()
n = [0]


def sub_coll(m):
    block = m.group(0)
    new = block.replace('<uri>meshes/Depot.dae</uri>',
                        '<uri>meshes/Depot_collision.stl</uri>')
    if new != block:
        n[0] += 1
    return new


out = re.sub(r'<collision\b.*?</collision>', sub_coll, text, flags=re.S)
open(out_root + '/Depot/model.sdf', 'w').write(out)
print('OK: %d 个 collision 换用 STL -> %s' % (n[0], out_root))
