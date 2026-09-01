# 第三方组件与依赖

本文件只记录当前仓库实际保留、二维基线需要或未来接口有用的组件。

## 算法

| 位置 | 上游 | 用途 |
|---|---|---|
| `algorithms/global_planning/ariadne/src/rl_planner/` | [ARiADNE-ROS-Planner](https://github.com/marmotlab/ARiADNE-ROS-Planner) | 二维全局探索与 waypoint |
| `algorithms/local_planning/scan_planner/` | [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) | 局部轨迹规划、Go2 仿真模型 |

## 仿真与地图

| 位置/包 | 上游或来源 | 用途 |
|---|---|---|
| `simulation/cmu_env/src/velodyne_simulator/` | CMU exploration environment | Gazebo Velodyne 插件/描述包 |
| `simulation/cmu_env/src/livox_laser_simulation/` | Mid360 simulation plugin | Go2 URDF 使用的 MID360 mesh 与仿真资源 |
| `simulation/cmu_env/src/sensor_scan_generation/` | CMU/本项目接线 | registered cloud 与 pose 的扫描时序转换 |
| `octomap_server` | ROS 系统包 | `/projected_map` 占据地图 |

## ROS 接口

`integration/go2_bridge/` 是 Python 胶水包，当前包含点云范围过滤、扫描累积和
ARiADNE waypoint/path 转发；它由启动器通过 `ROS_PACKAGE_PATH` 找到，不绑定具体 LIO。

`fastlio_integration.launch` 仅是外部 LIO 接口模板。外部实现需要提供 pose/odometry、
registered/world cloud 和 TF；默认 topic `/Odometry`、`/cloud_registered` 可通过 launch
参数替换。

## 编译

```bash
cd algorithms/local_planning/scan_planner && catkin_make
cd simulation/cmu_env && catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"
```

ROS Noetic、Gazebo Classic 11、系统 PCL/OctoMap 等为外部系统依赖。
TARE、elevation_mapping、terrain_analysis、stair/multifloor、physical Go2 workspace
已从本二维仓库移除，不应作为当前构建步骤或运行依赖。
