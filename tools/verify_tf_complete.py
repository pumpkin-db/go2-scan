#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_tf_complete — Track A1.8 自动验收

从 /robot_description 解析全部 link，逐个验证 world→link 的 TF 可达。
要求 missing_tf_links == []。另检查 /joint_states 发布者与活动关节覆盖。
"""

import sys
import xml.etree.ElementTree as ET

import rosgraph
import rospy
import tf


def main():
    rospy.init_node("verify_tf_complete")
    root_frame = rospy.get_param("~root_frame", "world")
    settle = rospy.Duration(rospy.get_param("~settle_seconds", 15.0))

    desc = rospy.get_param("/robot_description", None)
    if not desc:
        print("FATAL: /robot_description 不存在")
        sys.exit(2)
    root = ET.fromstring(desc)
    links = [l.get("name") for l in root.findall("link")]
    movable = [j.get("name") for j in root.findall("joint") if j.get("type") != "fixed"]
    print("URDF links: %d 个; movable joints: %d 个" % (len(links), len(movable)))

    master = rosgraph.Master("/verify_tf_complete")
    try:
        pubs, _, _ = master.getSystemState()
        js_pubs = [t for t, _ in pubs if t == "/joint_states"]
        print("/joint_states publishers: %s" % (js_pubs if js_pubs else "无"))
    except Exception as exc:
        print("/joint_states publishers: 查询失败 %s" % exc)

    listener = tf.TransformListener()
    rospy.sleep(settle)  # 等 TF buffer 填充（含 /tf_static）

    missing, ok = [], []
    deadline = rospy.Time.now() + rospy.Duration(20.0)
    for link in links:
        found = False
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            try:
                if listener.canTransform(root_frame, link, rospy.Time(0)):
                    found = True
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            rospy.sleep(0.2)
        (ok if found else missing).append(link)

    legs = [n for n in links if any(n.startswith(p) for p in ("FL_", "FR_", "RL_", "RR_"))]
    rotors = [n for n in legs if "rotor" in n]
    print("=" * 60)
    print("TF 完整性: %d/%d link 可从 %s 到达" % (len(ok), len(links), root_frame))
    print("missing_tf_links = %s" % missing)
    print("腿部 link: %d (rotor %d) — 缺失 %s" % (
        len(legs), len(rotors), [l for l in legs if l in missing] or "无"))
    print("=" * 60)
    if not missing:
        print("VERIFY PASS: missing_tf_links == []")
        sys.exit(0)
    print("VERIFY FAIL")
    sys.exit(1)


if __name__ == "__main__":
    main()
