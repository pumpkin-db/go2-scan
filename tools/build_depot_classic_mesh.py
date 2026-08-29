#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""build_depot_classic_mesh.py — 生成 Gazebo Classic 11 专用的简化 Depot DAE（Track A2）

读取官方 Collada（FBX 导出，含 Maya extra/emission/blend_mode/多通道），输出一份
结构最简的 Classic DAE：
  - library_images / library_effects / library_materials 全部重建：
    每个材质只保留 lambert diffuse = 官方 Albedo 纹理（CHANNEL0）
  - primitive material symbol 与 instance_material symbol 显式重命名为 CLASSIC_XXX，
    一一对应，不依赖 FBX 隐式映射
  - 纹理复制到 meshes_classic/textures/，DAE 内用同目录相对路径 textures/XXX.png
  - geometry / vertices / normals / UV / <p> 索引 / transforms / collision 一律不修改

用法:
  python3 tools/build_depot_classic_mesh.py --src <官方.dae> --out <meshes_classic/Depot.dae> \
      --tex-out <meshes_classic/textures> --tex-src <官方textures目录>
"""

import argparse
import os
import re
import shutil
import xml.etree.ElementTree as ET

NS = 'http://www.collada.org/2005/11/COLLADASchema'


def q(t):
    return '{%s}%s' % (NS, t)


def classic_name(albedo_basename):
    stem = re.sub(r'\.(png|jpg|jpeg)$', '', albedo_basename, flags=re.I)
    stem = re.sub(r'_(Albedo|BaseColor)$', '', stem, flags=re.I)
    return re.sub(r'[^A-Za-z0-9]', '_', stem).upper()


def diffuse_chain(root, symbol):
    """material symbol -> (effect id, diffuse texture image id, init_from 相对路径)"""
    target = None
    for m in root.iter(q('material')):
        if m.get('id') == symbol:
            target = m.find(q('instance_effect')).get('url').lstrip('#')
    if target is None:
        return None
    for e in root.iter(q('effect')):
        if e.get('id') != target:
            continue
        s = ET.tostring(e, encoding='unicode').replace('ns0:', '')
        m = re.search(r'<diffuse>\s*<texture texture="([^"]+)"', s)
        return (target, m.group(1) if m else None)
    return None


def build(src, out, tex_out, tex_src):
    ET.register_namespace('', NS)
    tree = ET.parse(src)
    root = tree.getroot()

    # image id -> init_from
    imgs = {i.get('id'): (i.find(q('init_from')).text or '').strip()
            for i in root.iter(q('image'))}

    # 1) 收集每个 primitive symbol 的 diffuse 贴图 -> classic 命名
    sym_map = {}   # 原symbol -> classic 名
    albedo_of = {}  # classic 名 -> albedo 文件名
    for g in root.iter(q('geometry')):
        for prim in g.iter():
            if prim.tag not in (q('triangles'), q('polylist'), q('polygons')):
                continue
            sym = prim.get('material')
            if not sym or sym in sym_map:
                continue
            chain = diffuse_chain(root, sym)
            albedo = None
            if chain:
                _, tex_id = chain
                init = imgs.get(tex_id, '')
                albedo = os.path.basename(init.replace('\\', '/')) if init else None
            name = classic_name(albedo) if albedo else classic_name(sym)
            sym_map[sym] = name
            albedo_of[name] = albedo

    # 2) 重建 library_images / effects / materials
    for lib in (q('library_images'), q('library_effects'), q('library_materials')):
        el = root.find(lib)
        if el is not None:
            root.remove(el)

    lib_images = ET.Element(q('library_images'))
    lib_effects = ET.Element(q('library_effects'))
    lib_materials = ET.Element(q('library_materials'))

    for name, albedo in sorted(albedo_of.items()):
        img = ET.SubElement(lib_images, q('image'), {
            'id': 'classic_%s_image' % name, 'name': 'classic_%s_image' % name})
        ET.SubElement(img, q('init_from')).text = 'textures/%s' % albedo

        eff = ET.SubElement(lib_effects, q('effect'), {'id': 'classic_%s_fx' % name})
        prof = ET.SubElement(eff, q('profile_COMMON'))
        np1 = ET.SubElement(prof, q('newparam'), {'sid': 'classic_%s_surface' % name})
        surf = ET.SubElement(np1, q('surface'), {'type': '2D'})
        ET.SubElement(surf, q('init_from')).text = 'classic_%s_image' % name
        np2 = ET.SubElement(prof, q('newparam'), {'sid': 'classic_%s_sampler' % name})
        samp = ET.SubElement(np2, q('sampler2D'))
        ET.SubElement(samp, q('source')).text = 'classic_%s_surface' % name
        tech = ET.SubElement(prof, q('technique'), {'sid': 'common'})
        lam = ET.SubElement(tech, q('lambert'))
        diff = ET.SubElement(lam, q('diffuse'))
        ET.SubElement(diff, q('texture'), {
            'texture': 'classic_%s_sampler' % name, 'texcoord': 'CHANNEL0'})

        mat = ET.SubElement(lib_materials, q('material'), {
            'id': 'classic_%s_material' % name, 'name': 'CLASSIC_%s' % name})
        ET.SubElement(mat, q('instance_effect'), {'url': '#classic_%s_fx' % name})

    # schema 顺序: images -> effects -> materials -> geometries ...
    geo_idx = list(root).index(root.find(q('library_geometries')))
    root.insert(geo_idx, lib_materials)
    root.insert(geo_idx, lib_effects)
    root.insert(geo_idx, lib_images)

    # 3) primitive symbol 显式重命名
    for g in root.iter(q('geometry')):
        for prim in g.iter():
            if prim.tag in (q('triangles'), q('polylist'), q('polygons')):
                sym = prim.get('material')
                if sym in sym_map:
                    prim.set('material', 'CLASSIC_%s' % sym_map[sym])

    # 4) 每个 instance_geometry 强制单一显式 instance_material
    geo_syms = {}
    for g in root.iter(q('geometry')):
        syms = [p.get('material') for p in g.iter()
                if p.tag in (q('triangles'), q('polylist'), q('polygons')) and p.get('material')]
        geo_syms[g.get('id')] = syms

    for ig in root.iter(q('instance_geometry')):
        gid = ig.get('url').lstrip('#')
        syms = geo_syms.get(gid, [])
        bm = ig.find(q('bind_material'))
        if bm is None:
            bm = ET.SubElement(ig, q('bind_material'))
        tc = bm.find(q('technique_common'))
        if tc is not None:
            bm.remove(tc)
        tc = ET.SubElement(bm, q('technique_common'))
        for s in syms:
            name = s.replace('CLASSIC_', '')
            ET.SubElement(tc, q('instance_material'), {
                'symbol': s, 'target': '#classic_%s_material' % name})

    # 5) 写出
    os.makedirs(os.path.dirname(out), exist_ok=True)
    tree.write(out, encoding='utf-8', xml_declaration=True)

    # 6) 复制 diffuse 纹理
    os.makedirs(tex_out, exist_ok=True)
    copied = []
    for name, albedo in sorted(albedo_of.items()):
        src = os.path.join(tex_src, albedo)
        dst = os.path.join(tex_out, albedo)
        shutil.copyfile(src, dst)
        copied.append(albedo)
    return sym_map, copied


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--tex-out', required=True)
    ap.add_argument('--tex-src', required=True)
    args = ap.parse_args()
    sym_map, copied = build(args.src, args.out, args.tex_out, args.tex_src)
    print('symbol 重命名映射:')
    for k, v in sorted(sym_map.items()):
        print('  %s -> CLASSIC_%s' % (k, v))
    print('已复制 diffuse 纹理: %s' % ', '.join(sorted(copied)))
    print('输出: %s' % args.out)


if __name__ == '__main__':
    main()
