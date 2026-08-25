#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Go2 spawn 托管脚本：查重 → 删旧 → 服务化 spawn → 点云健康门。

为什么存在（2026-08-25 Depot 事故复盘）：
旧 run_benchmark.sh 用 `rosrun spawn_model` CLI + 60s timeout，首 spawn 超时但请求
已入队，重试造成多只 Go2 叠在同一出生点；活狗雷达被埋在别的狗身体里，10000 个点
全部距离传感器 0.00m（实测复现），近距滤波器全丢 → octomap 空 → 狗站桩。
且 "entity already exists" 被当成功，静默带病跑完 35min 出废报告。

参考 gazebo_ros_pkgs#347 与 ROS2 spawn_entity.py 的标准做法：
1. spawn 前 get_world_properties 查重，存在则 delete_model + 轮询到消失
   （delete 是异步的，服务返回≠物理引擎删完，必须轮询）；
2. 直接调 /gazebo/spawn_urdf_model 服务（有明确 success 字段，无 CLI 超时歧义）；
3. spawn 后健康门：采 5s /mid360_points 与 /quad_0/lidar_pose 算水平距离，
   中位数 < --min-median 判「狗被埋/雷达退化」→ 返回非零让上层清理重来。

用法：spawn_go2.py [--model-name go2_description] [--min-median 0.5]
返回码：0 成功；1 gazebo 服务不可用；2 多次删除仍冲突；3 健康门不过；4 雷达无数据
"""
import argparse
import sys
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, GetWorldProperties, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler


def wait_service(name, timeout):
    try:
        rospy.wait_for_service(name, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def model_exists(name):
    """优先用 /gazebo/model_states（持续发布、无服务往返），退化用 world_properties。"""
    try:
        states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=3)
        return name in states.name
    except rospy.ROSException:
        pass
    if not wait_service('/gazebo/get_world_properties', 5):
        return False
    props = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    return name in props.model_names


def delete_and_wait(name, settle=30.0):
    """delete_model 是异步的：轮询直到模型真消失。返回是否删干净。"""
    if not wait_service('/gazebo/delete_model', 5):
        return False
    try:
        r = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(name)
        rospy.loginfo('[spawn_go2] delete %s: %s', name, r.status_message)
    except rospy.ServiceException as e:
        rospy.logwarn('[spawn_go2] delete 服务异常: %s', e)
    t0 = time.time()
    while time.time() - t0 < settle:
        if not model_exists(name):
            return True
        time.sleep(1.0)
    return not model_exists(name)


def do_spawn(name, x, y, z, yaw):
    urdf = rospy.get_param('robot_description')
    pose = Pose()
    pose.position = Vector3(x, y, z)
    q = quaternion_from_euler(0, 0, yaw)
    pose.orientation = Quaternion(*q)
    if not wait_service('/gazebo/spawn_urdf_model', 10):
        return False, 'spawn service unavailable'
    r = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)(
        model_name=name, model_xml=urdf, robot_namespace='/', initial_pose=pose,
        reference_frame='world')
    return bool(r.success), r.status_message


def lidar_health(min_median, window=5.0, min_msgs=10, first_msg_timeout=60.0):
    """采 window 秒点云，与雷达世界位姿算水平距离。返回 (ok, median, n_msgs)。
    大世界加载后插件热身慢（实测 sim time 个位数时仍无数据），先等首帧到来再采样。"""
    box = {'pts': [], 'pose': None}

    def cloud_cb(m):
        dt = np.dtype({'names': [f.name for f in m.fields],
                       'formats': [np.float32] * len(m.fields),
                       'offsets': [f.offset for f in m.fields],
                       'itemsize': m.point_step})
        arr = np.frombuffer(bytes(m.data), dtype=dt)
        box['pts'].append(np.stack([arr['x'], arr['y']], axis=-1).astype(np.float64))

    def pose_cb(m):
        p = m.pose.pose.position
        box['pose'] = np.array([p.x, p.y])

    sub_c = rospy.Subscriber('/mid360_points', PointCloud2, cloud_cb, queue_size=2)
    sub_p = rospy.Subscriber('/quad_0/lidar_pose', Odometry, pose_cb, queue_size=2)
    t0 = time.time()
    # 阶段1：等首帧点云（大世界插件热身可长达 60s）
    while time.time() - t0 < first_msg_timeout:
        if box['pts']:
            break
        time.sleep(0.3)
    if not box['pts']:
        sub_c.unregister(); sub_p.unregister()
        return False, float('nan'), 0
    # 阶段2：凑够 min_msgs 帧再判定（最多再等 window*4）
    t1 = time.time()
    while time.time() - t1 < window * 4:
        if len(box['pts']) >= min_msgs and box['pose'] is not None:
            break
        time.sleep(0.2)
    sub_c.unregister()
    sub_p.unregister()
    n = len(box['pts'])
    if box['pose'] is None or n == 0:
        return False, float('nan'), 0
    pts = np.concatenate(box['pts'], axis=0)
    d = np.linalg.norm(pts - box['pose'][None, :], axis=1)
    med = float(np.median(d))
    return med >= min_median, med, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model-name', default='go2_description')
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, required=True)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--retries', type=int, default=3)
    ap.add_argument('--min-median', type=float, default=0.5)
    args = ap.parse_args()

    rospy.init_node('spawn_go2', anonymous=True, disable_signals=True)

    # Depot 等 25MB world 加载实测需 4min+（旧脚本也是第3次重试才成功），轮询等就绪并打心跳
    deadline = time.time() + 420
    ready = False
    while time.time() < deadline:
        try:
            rospy.wait_for_service('/gazebo/get_world_properties', timeout=15)
            ready = True
            break
        except rospy.ROSException:
            print('[spawn_go2] ... 等待 gazebo 服务（已等 %ds/420s）'
                  % int(time.time() - (deadline - 420)), flush=True)
    if not ready:
        print('[spawn_go2] FAIL: gazebo 服务 420s 未就绪', flush=True)
        return 1

    for i in range(1, args.retries + 1):
        # 健康优先：launch 会自己 spawn 狗（gazebo_sim.launch L39），若已存在且体检
        # 通过就直接放行，不做无谓的删+重发（那正是历史上多狗叠加的来源）
        if model_exists(args.model_name):
            print('[spawn_go2] 第%d次: %s 已在世界中，直接体检...' % (i, args.model_name),
                  flush=True)
        else:
            ok, msg = do_spawn(args.model_name, args.x, args.y, args.z, args.yaw)
            print('[spawn_go2] spawn: success=%s (%s)' % (ok, msg), flush=True)
            if not ok:
                time.sleep(5)
                continue
        good, med, n = lidar_health(args.min_median)
        if good:
            print('[spawn_go2] OK: 健康门通过（%d 帧，水平距离中位 %.2fm ≥ %.2fm）'
                  % (n, med, args.min_median), flush=True)
            return 0
        if n == 0:
            print('[spawn_go2] FAIL: 雷达持续无数据（第%d次）' % i, flush=True)
            rc = 4
        else:
            print('[spawn_go2] FAIL: 疑似被埋/雷达退化（%d 帧，中位 %.3fm < %.2fm），'
                  '删除重试' % (n, med, args.min_median), flush=True)
            rc = 3
        delete_and_wait(args.model_name)
    print('[spawn_go2] FATAL: %d 次尝试全部失败' % args.retries, flush=True)
    return rc


if __name__ == '__main__':
    sys.exit(main())
