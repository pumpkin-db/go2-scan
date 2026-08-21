# 第三方算法与依赖清单

> 本仓库**自包含全部第三方源码**（vendor 方式：源码直接进仓库、删掉各自 `.git`、用下表的上游 commit 溯源）。
> 除 ROS / 系统标准库外，仿真 + 算法所需的一切第三方代码都在 `algorithms/` 和 `simulation/` 里，**无需再 clone**。
> 更新上游：从下表 URL 重新 clone 对应 commit，覆盖本仓库对应目录即可（注意我们已在上游基础上做了四足化改造，覆盖前先 diff）。

## 算法层（algorithms/）

| 仓库内位置 | 上游仓库 | commit | 作用 |
|-----------|---------|--------|------|
| `algorithms/global_planning/tare/` | https://github.com/caochao39/tare_planner | `44500592b861` | 全局探索决策（TARE，CMU） |
| `algorithms/local_planning/scan_planner/` | https://github.com/wuyi2121/SCAN-Planner | `348e8a590a50` | 局部轨迹规划（SCAN-Planner，SJTU） |
| `algorithms/mapping/elevation_mapping/` | 见下表（多包 workspace） | — | 2.5D 高程图 |

### elevation_mapping workspace 内各包

| 包 | 上游仓库 | commit |
|----|---------|--------|
| elevation_mapping | https://github.com/anybotics/elevation_mapping | `f4b082c64a3e` |
| kindr | https://github.com/anybotics/kindr | `32800890d546` |
| kindr_ros | https://github.com/anybotics/kindr_ros | `8d60e3f8df5d` |
| message_logger | https://github.com/anybotics/message_logger | `bd99bd663bc6` |

## 仿真层（simulation/）

| 仓库内位置 | 上游仓库 | commit | 作用 |
|-----------|---------|--------|------|
| `simulation/cmu_env/` | https://github.com/HongbiaoZ/autonomous_exploration_development_environment | `bf0cba713652` | 仿真底座（velodyne/livox 插件 + go2 模型） |
| `simulation/cmu_env/src/livox_laser_simulation/` | https://github.com/fratopa/Mid360_simulation_plugin | `aae8ee3a0f16` | MID360 仿真插件（拷入 cmu_env 编译） |

> 注：cmu_env 上游含 `vehicle_simulator`（TARE 原作者的**无人车**模拟器），四足 Go2 用不到且含 458MB mesh zip，**已删除**。白名单编译只保留 velodyne 插件 + livox 插件。

## TARE 自带 vendor 依赖

- **or-tools**（`algorithms/global_planning/tare/src/tare_planner/or-tools/`）：Google OR-Tools v9.8 预编译 C++ 库（含 `include/` + `lib/`），用于 TARE 的 TSP 求解。随 TARE 打包，约 137MB 二进制已进仓库。

## 已评估但未采用的算法（未进本仓库）

以下仓库在选型阶段评估过、当前**未采用**，保留在 `~/claude/raicom/new_algorithm/`（未迁入 go2-scan），后续若启用再按上面规范迁入：

- ARiADNE、FAEL、FUEL、UFEP-Released、TravExplorer、gbplanner_ros、ego-planner、stc_ws、LKH-3.0.14、nlopt（gbplanner 依赖）、HPHS（场景制作工具）

## 编译

```bash
# 1. 局部规划 workspace（scan_planner + plan_env 等 + go2_description + map_generator）
cd ~/claude/raicom/go2-scan/algorithms/local_planning/scan_planner && catkin_make

# 2. 全局探索 workspace（tare_planner，自带 or-tools）
cd ~/claude/raicom/go2-scan/algorithms/global_planning/tare && catkin_make

# 3. 高程图 workspace（elevation_mapping + kindr 等）
cd ~/claude/raicom/go2-scan/algorithms/mapping/elevation_mapping && catkin_make

# 4. 仿真底座 workspace（velodyne/livox 插件）
cd ~/claude/raicom/go2-scan/simulation/cmu_env && catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation"
```

> go2_bridge（`integration/go2_bridge/`）是纯 Python 包，不编译，靠 `launch_gazebo_sim.sh` 里 `ROS_PACKAGE_PATH` 指向 `integration/` 被 rospack 找到。

## 启动

```bash
bash ~/claude/raicom/go2-scan/simulation/launch_gazebo_sim.sh
```
