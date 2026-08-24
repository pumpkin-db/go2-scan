#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决定性实验：验证 世界系→传感器系→世界系 往返是否把 z 抬高了。
同时取同 stamp 的 /mid360_points_clean 与 /sensor_scan，
用当前 TF 把 /sensor_scan 变回世界系，与原世界系点做最近邻比对。
若变换后地板点 z≈0 → 变换无罪，问题在 octomap 内部；
若 z 被抬到 0.2~0.8 → 抓到真凶。"""
import numpy as np, rospy, tf2_ros
from sensor_msgs.msg import PointCloud2

def parse(msg, want_xyz=True):
    names = [f.name for f in msg.fields]
    dt = np.dtype({'names': names, 'formats': [np.float32]*len(names),
                   'offsets': [f.offset for f in msg.fields], 'itemsize': msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dt)

rospy.init_node('roundtrip_probe', disable_signals=True)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
import time; time.sleep(2)

clean = rospy.wait_for_message('/mid360_points_clean', PointCloud2, timeout=20)
t = clean.header.stamp
scan = None
for _ in range(10):  # 找 stamp 最接近的 sensor_scan
    m = rospy.wait_for_message('/sensor_scan', PointCloud2, timeout=10)
    if abs((m.header.stamp - t).to_sec()) < 0.02:
        scan = m; break
if scan is None:
    print("没配到同拍 sensor_scan"); sys.exit(1)

# 当前 TF: map -> sensor_at_scan
tr = buf.lookup_transform('map', 'sensor_at_scan', rospy.Time(0))
q = tr.transform.rotation; tt = tr.transform.translation
def q_rot(qx,qy,qz,qw, v):
    # R(q)·v
    x,y,z = v[:,0],v[:,1],v[:,2]
    tx = (1-2*(qy*qy+qz*qz))*x + 2*(qx*qy-qz*qw)*y + 2*(qx*qz+qy*qw)*z
    ty = 2*(qx*qy+qz*qw)*x + (1-2*(qx*qx+qz*qz))*y + 2*(qy*qz-qx*qw)*z
    tz = 2*(qx*qz-qy*qw)*x + 2*(qy*qz+qx*qw)*y + (1-2*(qx*qx+qy*qy))*z
    return np.stack([tx,ty,tz],axis=-1)

pc = parse(clean); pworld = np.stack([pc['x'],pc['y'],pc['z']],-1).astype(np.float64)
sc = parse(scan); psensor = np.stack([sc['x'],sc['y'],sc['z']],-1).astype(np.float64)
print(f"clean帧 {len(pworld)} 点 @stamp{t.to_sec():.2f}；sensor_scan帧 {len(psensor)} 点")

# 手工重放变换链：用矩阵法
def quat_to_R(qx,qy,qz,qw):
    return np.array([
      [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
      [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
      [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])
R = quat_to_R(q.x,q.y,q.z,q.w); T = np.array([tt.x,tt.y,tt.z])
p_recon_sensor = (R @ pworld.T + T.reshape(3,1)).T   # sensorScanGeneration 应得到的结果
back = (R.T @ (psensor - T).T).T                     # octomap 视角：sensor -> world

print("\n== 关键统计 ==")
print(f"TF平移: ({tt.x:.3f},{tt.y:.3f},{tt.z:.3f})  四元数: ({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
print(f"原始世界点 z 分布: min={pworld[:,2].min():.3f} max={pworld[:,2].max():.3f} 均值={pworld[:,2].mean():.3f}")
print(f"重放转换后 sensor 系 z: min={p_recon_sensor[:,2].min():.3f} max={p_recon_sensor[:,2].max():.3f}")
print(f"sensor_scan 实际 z:     min={psensor[:,2].min():.3f} max={psensor[:,2].max():.3f}")
print(f"经 TF 变回世界后 z:     min={back[:,2].min():.3f} max={back[:,2].max():.3f} 均值={back[:,2].mean():.3f}")
zb = ((back[:,2]>=0.2)&(back[:,2]<=0.9)).mean()*100
zo = ((pworld[:,2]>=0.2)&(pworld[:,2]<=0.9)).mean()*100
print(f"落入占据带比例：原始 {zo:.1f}%  →  变回后 {zb:.1f}%")
