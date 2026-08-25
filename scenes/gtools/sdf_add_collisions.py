#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把「只有 visual 的 Ignition 模型」转成经典 Gazebo9 可感知的模型：
  - 为白名单 submesh 追加同名 <collision>（雷达射线只与 collision 相交！）
  - static 设 true（省物理开销；风扇等装饰不动）
  - 删除 ignition-gazebo 插件块与 revolute 关节（gazebo9 无此插件）
用法：
  python3 sdf_add_collisions.py <in.sdf> <out.sdf> WALLS,FLOOR,STAIRS,...
"""
import re
import sys


def main():
    src, dst = sys.argv[1], sys.argv[2]
    whitelist = [s.strip() for s in sys.argv[3].split(',') if s.strip()]
    text = open(src).read()

    # 1. static -> true
    text = re.sub(r'<static>false</static>', '<static>true</static>', text)

    # 2. 删 ignition 插件块
    text = re.sub(r'\s*<plugin filename="ignition-[^"]*".*?</plugin>', '', text, flags=re.S)

    # 3. 删 revolute joint 块（风扇等）
    text = re.sub(r'\s*<joint name="[^"]+" type="revolute">.*?</joint>', '', text, flags=re.S)

    # 4. 对白名单 visual，在其后插入同几何 collision。
    #    匹配每个 <visual ...>...</visual>，抓 submesh 名与 scale。
    vis_pat = re.compile(r'( *)<visual name="([^"]+)">(.*?)</visual>', re.S)

    def make_collision(indent, name, body):
        m_sub = re.search(r'<submesh>\s*<name>([^<]+)</name>', body)
        if not m_sub:
            return ''
        sub = m_sub.group(1)
        m_scale = re.search(r'<scale>([^<]+)</scale>', body)
        scale = m_scale.group(1) if m_scale else '1 1 1'
        m_uri = re.search(r'<uri>([^<]+\.dae)</uri>', body)
        uri = m_uri.group(1) if m_uri else None
        if uri is None:
            return ''
        i = indent
        return f'''{i}<collision name="{name}_collision">
{i}  <geometry>
{i}    <mesh>
{i}      <scale>{scale}</scale>
{i}      <uri>{uri}</uri>
{i}      <submesh>
{i}        <name>{sub}</name>
{i}        <center>false</center>
{i}      </submesh>
{i}    </mesh>
{i}  </geometry>
{i}</collision>'''

    out_parts = []
    last = 0
    added = []
    for m in vis_pat.finditer(text):
        out_parts.append(text[last:m.end()])
        last = m.end()
        name, body = m.group(2), m.group(3)
        # 该 visual 的 submesh 在白名单里才加 collision
        ms = re.search(r'<submesh>\s*<name>([^<]+)</name>', body)
        if ms and ms.group(1) in whitelist:
            col = make_collision(' ' * len(m.group(1)), name, body)
            if col:
                out_parts.append('\n' + col.rstrip('\n'))
                added.append(ms.group(1))
    out_parts.append(text[last:])
    open(dst, 'w').write(''.join(out_parts))
    print('added collisions:', sorted(set(added)))
    print('missing from whitelist:', sorted(set(whitelist) - set(added)))


if __name__ == '__main__':
    main()
