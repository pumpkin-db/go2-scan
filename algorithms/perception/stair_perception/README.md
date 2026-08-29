# Stair Perception

LiDAR-only stair candidate detection and multi-frame tracking. The core contract is a
gravity-aligned, world-frame `sensor_msgs/PointCloud2` plus `nav_msgs/Odometry`; it has no
Gazebo, Depot registry, elevation-map, or robot-control dependency.

## Build and test

```bash
source /opt/ros/noetic/setup.bash
cd algorithms/perception/stair_perception
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make run_tests_stair_perception_gtest_test_detector
catkin_test_results --verbose
```

## Inputs

- Simulation: `/mid360_points` + `/quad_0/body_pose`
- Real Go2/FAST-LIO: `/cloud_registered` + `/Odometry`

Both headers must use the same gravity-aligned world frame. The node rejects mismatched
frames instead of silently transforming data.

## Outputs

- `/stair_perception/observations`: per-frame geometry observations
- `/stair_perception/tracks`: temporally associated tracks; confirmed after three observations
- `/stair_perception/debug/support`: RANSAC support points
- `/stair_perception/debug/candidates`, `/stair_perception/debug/tracks`: RViz markers

The legacy `go2_bridge/stair_detector.py` remains a simulation GT/elevation backend and must
not run concurrently with this package under the same node name.
