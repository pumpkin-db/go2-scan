#!/usr/bin/env python3
"""Monitor/verify the FAST-LIO2-style integration of scan_planner.

Collects for a fixed duration:
  - message rates of /grid_map/occupancy, /cloud_registered, /Odometry
  - occupied point counts over time
  - robot trajectory from /Odometry
  - alignment of occupied voxels vs the (static) mock global map:
    nearest-neighbor distances via KDTree (drift/mis-registration detector)
  - temporal stability of the occupancy cloud (centroid drift between frames)

Usage: fastlio_test_monitor.py <label> <duration_sec>
"""
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

OCC = "/grid_map/occupancy"
INF = "/grid_map/occupancy_inflate"
CLOUD = "/cloud_registered"
ODOM = "/Odometry"
GLOBAL = "/map_generator/global_cloud"

counts = {}
occ_points = []          # list of (t, np.array Nx3)
odom_traj = []           # list of (t, x, y, z)
global_tree = [None]
global_n = [0]
cloud_sizes = []


def xyz_from_pc2(msg):
    # assumes float32 x,y,z fields, which is what scan_planner/mockamap publish
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    usable = (buf.size // msg.point_step) * msg.point_step
    buf = buf[:usable].reshape(-1, msg.point_step)
    n = min(n, buf.shape[0])
    arr = np.zeros((n, 3), dtype=np.float32)
    for i, f in enumerate(msg.fields[:3]):
        col = buf[:n, f.offset:f.offset + 4].tobytes()
        arr[:, i] = np.frombuffer(col, dtype=np.float32)
    return arr


def mk_count_cb(name):
    def cb(msg):
        counts[name] = counts.get(name, 0) + 1
    return cb


def occ_cb(msg):
    counts[OCC] = counts.get(OCC, 0) + 1
    occ_points.append((time.time(), xyz_from_pc2(msg)))


def inf_cb(msg):
    counts[INF] = counts.get(INF, 0) + 1


def cloud_cb(msg):
    counts[CLOUD] = counts.get(CLOUD, 0) + 1
    cloud_sizes.append(msg.width * msg.height)


def odom_cb(msg):
    counts[ODOM] = counts.get(ODOM, 0) + 1
    p = msg.pose.pose.position
    odom_traj.append((msg.header.stamp.to_sec(), p.x, p.y, p.z))


def global_cb(msg):
    counts[GLOBAL] = counts.get(GLOBAL, 0) + 1
    pts = xyz_from_pc2(msg)
    if pts.shape[0] > 0:
        global_tree[0] = cKDTree(pts)
        global_n[0] = pts.shape[0]


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

    rospy.init_node("fastlio_test_monitor", anonymous=True, disable_signals=True)
    rospy.Subscriber(OCC, PointCloud2, occ_cb, queue_size=5)
    rospy.Subscriber(INF, PointCloud2, inf_cb, queue_size=5)
    rospy.Subscriber(CLOUD, PointCloud2, cloud_cb, queue_size=5)
    rospy.Subscriber(ODOM, Odometry, odom_cb, queue_size=50)
    rospy.Subscriber(GLOBAL, PointCloud2, global_cb, queue_size=2)

    t0 = time.time()
    while time.time() - t0 < duration and not rospy.is_shutdown():
        time.sleep(0.2)
    dt = time.time() - t0

    print("== REPORT [%s] window=%.1fs ==" % (label, dt))
    for t in (OCC, INF, CLOUD, ODOM, GLOBAL):
        c = counts.get(t, 0)
        print("RATE %-28s %5d msgs  %6.1f Hz" % (t, c, c / dt))

    if occ_points:
        sizes = [p.shape[0] for _, p in occ_points]
        print("OCC_POINTS n_frames=%d min=%d max=%d mean=%.0f" %
              (len(sizes), min(sizes), max(sizes), np.mean(sizes)))
        # temporal stability: centroid difference between consecutive frames
        centroids = np.array([p.mean(axis=0) if p.shape[0] else [np.nan] * 3
                              for _, p in occ_points])
        valid = ~np.isnan(centroids[:, 0])
        if valid.sum() > 2:
            dc = np.linalg.norm(np.diff(centroids[valid], axis=0), axis=1)
            print("OCC_CENTROID_STEP median=%.4f max=%.4f m/frame" %
                  (np.median(dc), dc.max()))
        # alignment vs global map
        if global_tree[0] is not None:
            all_d = []
            for _, p in occ_points:
                if p.shape[0] == 0:
                    continue
                d, _ = global_tree[0].query(p.astype(np.float64), k=1,
                                            workers=1, distance_upper_bound=1.0)
                all_d.append(d[np.isfinite(d)])
            if all_d:
                all_d = np.concatenate(all_d)
                print("ALIGN occ-vs-globalmap: n=%d d_median=%.3f d90=%.3f d99=%.3f max=%.3f (global pts=%d)"
                      % (all_d.size, np.median(all_d),
                         np.percentile(all_d, 90), np.percentile(all_d, 99),
                         all_d.max(), global_n[0]))
        else:
            print("ALIGN no global map received")
    else:
        print("OCC_POINTS no /grid_map/occupancy frames received!")

    if odom_traj:
        traj = np.array(odom_traj)
        dist = np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1).sum()
        args = tuple(traj[0, 1:4]) + tuple(traj[-1, 1:4]) + (dist,)
        print("ODOM_TRAJ start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f) path_len=%.2f m"
              % args)
    else:
        print("ODOM_TRAJ no /Odometry received!")

    if cloud_sizes:
        print("CLOUD_SIZES min=%d max=%d mean=%.0f" %
              (min(cloud_sizes), max(cloud_sizes), np.mean(cloud_sizes)))


if __name__ == "__main__":
    main()
