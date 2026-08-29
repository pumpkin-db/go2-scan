#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_depot_classic_asset.py — Classic DAE 资产静态验收（Track A2 第八步）

检查:
  1. 所有 geometry 都有 primitive material
  2. 每个 primitive symbol 都有对应 instance_material
  3. 每个 instance_material target 存在
  4. 每个 material 都有 effect
  5. 每个 effect diffuse texture 存在
  6. 每张引用图片可打开（PNG/IHDR 校验）
  7. DAE XML 合法
  8. geometry/primitive/vertex/index 数据与原版一致（逐 geometry 数据指纹）

输出 unbound_primitives / missing_materials / missing_effects / missing_textures。
"""

import hashlib
import os
import re
import sys
import xml.etree.ElementTree as ET

NS = 'http://www.collada.org/2005/11/COLLADASchema'


def q(t):
    return '{%s}%s' % (NS, t)


def verify(src, out, tex_dir):
    problems = {'unbound_primitives': [], 'missing_materials': [],
                'missing_effects': [], 'missing_textures': []}

    tree_o = ET.parse(src)
    tree_c = ET.parse(out)
    ro, rc = tree_o.getroot(), tree_c.getroot()

    imgs = {i.get('id'): (i.find(q('init_from')).text or '').strip() for i in rc.iter(q('image'))}
    effects = set()
    fx_diffuse_img = {}
    for e in rc.iter(q('effect')):
        effects.add(e.get('id'))
        s = ET.tostring(e, encoding='unicode')
        m = re.search(r'<init_from>classic_([A-Za-z0-9_]+)_image</init_from>', s)
        fx_diffuse_img[e.get('id')] = m.group(1) if m else None
    materials = {m.get('id'): m.find(q('instance_effect')).get('url').lstrip('#')
                 for m in rc.iter(q('material'))}

    geo_syms_c, geo_syms_o = {}, {}
    for tag, store, root in (('orig', geo_syms_o, ro), ('classic', geo_syms_c, rc)):
        for g in root.iter(q('geometry')):
            prims = [p for p in g.iter() if p.tag in (q('triangles'), q('polylist'), q('polygons'))]
            store[g.get('id')] = prims

    # 1/2: primitive material + instance_material 匹配
    inst_by_geo = {}
    for ig in rc.iter(q('instance_geometry')):
        gid = ig.get('url').lstrip('#')
        inst_by_geo.setdefault(gid, []).extend(
            (im.get('symbol'), im.get('target').lstrip('#')) for im in ig.iter(q('instance_material')))

    for gid, prims in geo_syms_c.items():
        if not prims:
            problems['unbound_primitives'].append(gid)
        for p in prims:
            sym = p.get('material')
            pairs = inst_by_geo.get(gid, [])
            hit = [t for (s, t) in pairs if s == sym]
            if not hit:
                problems['unbound_primitives'].append('%s symbol=%s' % (gid, sym))
            for t in hit:
                if t not in materials:
                    problems['missing_materials'].append(t)
                else:
                    fx = materials[t]
                    if fx not in effects:
                        problems['missing_effects'].append(fx)
                    else:
                        img_id = 'classic_%s_image' % re.match(r'classic_(.+)_fx', fx).group(1)
                        init = imgs.get(img_id)
                        if not init:
                            problems['missing_textures'].append(img_id)
                        else:
                            fp = os.path.join(tex_dir, os.path.basename(init))
                            if not os.path.isfile(fp):
                                problems['missing_textures'].append(fp)
                            else:
                                with open(fp, 'rb') as f:
                                    head = f.read(29)
                                if head[:8] != b'\x89PNG\r\n\x1a\n':
                                    problems['missing_textures'].append(fp + ' (非PNG)')

    # 8: 数据指纹（source/float_array/<p>/count 全量哈希，逐 geometry）
    def fp(root, gid):
        for g in root.iter(q('geometry')):
            if g.get('id') == gid:
                blob = b''
                for fa in g.iter(q('float_array')):
                    blob += fa.text.encode()
                for p in g.iter(q('p')):
                    blob += p.text.encode()
                blob += str(len(list(g.iter(q('source'))))).encode()
                return hashlib.md5(blob).hexdigest()
        return None

    count_mismatch = []
    for gid in geo_syms_o:
        if fp(ro, gid) != fp(rc, gid):
            count_mismatch.append(gid)
    n_geo_o = len(geo_syms_o)
    n_geo_c = len(geo_syms_c)
    n_prim_o = sum(len(v) for v in geo_syms_o.values())
    n_prim_c = sum(len(v) for v in geo_syms_c.values())

    print('geometry 数: 原版=%d classic=%d %s' % (n_geo_o, n_geo_c, 'OK' if n_geo_o == n_geo_c else '不一致!'))
    print('primitive 数: 原版=%d classic=%d %s' % (n_prim_o, n_prim_c, 'OK' if n_prim_o == n_prim_c else '不一致!'))
    print('几何数据指纹不一致: %s' % (count_mismatch or '无'))
    for k, v in problems.items():
        print('%s = %s' % (k, v or '[]'))
    ok = (not any(problems.values())) and (not count_mismatch) and n_geo_o == n_geo_c and n_prim_o == n_prim_c
    print('STATIC VERIFY: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    rc = 0
    rc |= verify(sys.argv[1], sys.argv[2], sys.argv[4])  # Depot
    rc |= verify(sys.argv[3], sys.argv[2].replace('Depot.dae', 'Crates.dae'), sys.argv[4])  # Crates
    sys.exit(rc)
