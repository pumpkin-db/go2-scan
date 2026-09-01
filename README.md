# go2-scan_2D

Go2 的 ROS1 Noetic / Gazebo Classic 11 纯二维、单层自主探索基线。
当前算法链为 ARiADNE + SCAN-Planner，默认场景是 `indoor_1`。

## 当前仿真链

```text
indoor_1
  → MID360 simulation
  → sensor_scan_generation
  → OctoMap /projected_map
  → ARiADNE
  → /way_point
  → ariadne_goal_bridge
  → SCAN-Planner
  → /cmd_vel
  → go2_kinematic_model_plugin
  → Go2
```

二维 Gazebo 模型是运动学 body + 视觉步态；`go2_kinematic_model_plugin`
是 body pose 的唯一写入者，`go2_gait_publisher` 是 `/joint_states` 的唯一发布者。

本仓库当前不包含或不启动：TARE、楼梯/多楼层、`elevation_mapping`、
`terrain_analysis`、physical hotel backend。SCAN、ARiADNE、MID360 仿真和
OctoMap 是当前基线的核心组件。

## 启动

```bash
cd ~/claude/raicom/go2-scan_2D
bash simulation/kill_all_sim.sh
bash simulation/launch_gazebo_sim.sh scene:=indoor_1
```

启动器从自身位置推导仓库根目录，并打印实际 workspace 路径。启动前应确认没有
旧的 `gzserver`/ROS 进程。首次编译：

```bash
cd algorithms/local_planning/scan_planner && catkin_make
cd ../../../simulation/cmu_env && catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"
```

## 外部 LIO 接口

未来真机链路为：

```text
external LIO
  → Pose/Odometry + registered/world cloud + TF
  → 2D mapping
  → ARiADNE
  → SCAN-Planner
  → /cmd_vel
  → existing real Go2 bridge
```

项目不绑定 FAST-LIO、Point-LIO 或其它具体 LIO。`/Odometry` 和
`/cloud_registered` 只是默认/示例 topic，`fastlio_integration.launch` 是可参数化的
外部 LIO 接口模板，不包含 FAST-LIO 实现。

## 目录要点

- `algorithms/global_planning/ariadne/`：ARiADNE 全局二维探索。
- `algorithms/local_planning/scan_planner/`：SCAN-Planner、Go2 URDF、视觉步态及 Gazebo plugin。
- `simulation/cmu_env/`：Velodyne/MID360 仿真插件和 `sensor_scan_generation`。
- `integration/go2_bridge/`：点云预处理、扫描累积、ARiADNE waypoint bridge。
- `scenes/indoor_1/`：默认场景启动参数。
- `tools/evaluate_exploration.py`：二维探索评估工具（离线/在线）。

## 真机前置检查

真机仍使用已有 `cmd_vel_bridge → unitree_sdk2 / CycloneDDS → SportClient`，本仓库不
新增 ROS2 控制栈。接入前必须填写真实 base_link↔MID360 六自由度外参，并检查 NX 与
MID360 时钟同步、Odometry/registered cloud 频率、TF、命令 watchdog/STOP 及人工接管。

## Runtime provenance

修改 Gazebo C++ plugin 后必须在当前 checkout 重编译，停止旧 `gzserver`，并用
`/proc/<gzserver-pid>/maps` 与 `sha256sum` 确认实际加载的 `.so` 来自当前 workspace。

## 状态记录

当前有效状态和历史决策见 [PROGRESS.md](PROGRESS.md)；第三方来源见
[third_party.md](third_party.md)。
