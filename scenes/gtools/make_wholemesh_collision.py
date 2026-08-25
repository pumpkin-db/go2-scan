#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
把 model.sdf 里 <collision> 内的 <submesh> 块删掉（visual 不动），
使碰撞用整个网格而非命名子网格。

背景（2026-08-25 Depot 排查）：submesh 碰撞疑似导致 ODE 射线空间全局失效
（雷达连自身体/测试盒都打不到，最小世界正常）。本脚本产出对照版 SDF 用于判别与修复。
用法：make_wholemesh_collision.py <in.sdf> <out.sdf>
"""
import re
import sys


def strip_submesh_in_collisions(text):
    out = []
    pos = 0
    n_fixed = 0
    for m in re.finditer(r'<collision\b.*?</collision>', text, flags=re.S):
        out.append(text[pos:m.start()])
        block = m.group(0)
        new_block, k = re.subn(r'<submesh>.*?</submesh>\s*', '', block, flags=re.S)
        # submesh 删掉后，<mesh> 里只剩 scale+uri，合法
        n_fixed += k
        out.append(new_block)
        pos = m.end()
    out.append(text[pos:])
    return ''.join(out), n_fixed


if __name__ == '__main__':
    src, dst = sys.argv[1], sys.argv[2]
    text = open(src).read()
    new_text, n = strip_submesh_in_collisions(text)
    open(dst, 'w').write(new_text)
    print('OK: 移除 %d 个 collision submesh 块 -> %s' % (n, dst))
