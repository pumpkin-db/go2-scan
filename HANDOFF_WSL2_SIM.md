# go2-scan_2D：WSL2/Gazebo 仿真交接

> 面向下一位负责 WSL2 / Gazebo Classic 11 的 AI。标签含义：`[VERIFIED]` 已由当前代码或用户验收确认；`[CURRENT]` 当前实现；`[TARGET]` 目标语义；`[TODO]` 尚未完成；`[HISTORICAL]` 历史资料，不代表当前 main。

## 1. 定位

`[CURRENT]` 本仓库是 Go2 **纯二维、单层**自主探索基线：`indoor_1 + ARiADNE + SCAN-Planner`。当前 main 不包含或不启动 TARE、stair/multi-floor、`hotel_stairs`、Depot、`elevation_mapping`、`terrain_analysis` 或 physical Go2 backend。V2 多楼层研发在另一仓库/分支路线中，不能据历史 `PROGRESS.md` 条目恢复这些功能。

## 2. Git 与路径

`[VERIFIED]` 当前仓库：`/home/pumpkin-db/claude/raicom/go2-scan_2D`；branch=`main`；HEAD=`e5c8dd718f063a36b67bb8170a26508f9c451bd5`；origin fetch=`https://github.com/pumpkin-db/go2-scan.git`，push 使用 SSH `git@github.com:pumpkin-db/go2-scan.git`。当前 main 已推送且工作区应保持 clean。

```bash
cd ~/claude/raicom/go2-scan_2D
git status
git rev-parse HEAD
```

## 3. 当前二维运行链

```text
indoor_1
  → Gazebo simulated MID360 (/mid360_points)
  → cloud_range_filter (/mid360_points_clean)
  → sensor_scan_generation (/sensor_scan)
  → octomap_server (/projected_map)
  → ARiADNE (rl_planner, /way_point)
  → ariadne_goal_bridge (/initial_path)
  → SCAN-Planner (local trajectory)
  → closed_loop_controller (/cmd_vel)
  → go2_kinematic_model_plugin
  → simulated Go2
```

主要接口（均为当前二维必需）：

| 组件 | 输入 | 输出/语义 |
|---|---|---|
| Go2 ModelPlugin | `/cmd_vel`、`/joint_states` | `/quad_0/body_pose`（`world`→`base`）、`/quad_0/lidar_pose`（`world`→`mid360`），唯一 body pose writer |
| Gazebo LiDAR | Go2 URDF 内 RaySensor | `/mid360_points`，当前 Velodyne 插件配置为 `frameName=world`、`worldFrame=true` |
| `cloud_range_filter.py` | `/mid360_points`、`/quad_0/lidar_pose` | `/mid360_points_clean`，仍为 world frame；剔除传感器原点附近无效点 |
| `sensorScanGeneration` | `/mid360_points_clean`、`/quad_0/lidar_pose` | `/sensor_scan`、`/state_estimation_at_scan`，将 world cloud 转到扫描瞬时 sensor frame；同步窗口为 ApproximateTime |
| `octomap_server` | `/sensor_scan` | `/projected_map`，frame=`map`；`map→world` 有恒等 static TF |
| ARiADNE `rl_planner` | `/projected_map`、`/quad_0/body_pose` | `/way_point`；当前网络/权重不改，TARE 已删除 |
| `ariadne_goal_bridge.py` | `/way_point`、`/quad_0/body_pose` | `/initial_path`（`nav_msgs/Path`） |
| SCAN `scan_planner_node` | `/initial_path`、cloud、body/sensor pose | 局部轨迹；`closed_loop_controller` 输出最终 `/cmd_vel` |
| `go2_gait_publisher` | `/quad_0/body_pose` | `/joint_states`，视觉步态唯一发布者；ModelPlugin 跟随关节目标 |

## 4. 标准启动

```bash
cd ~/claude/raicom/go2-scan_2D
bash simulation/kill_all_sim.sh
bash simulation/launch_gazebo_sim.sh scene:=indoor_1
```

`[CURRENT]` planner 固定为 ARiADNE；无需再传 `global_planner:=ariadne`。场景脚本 `scenes/indoor_1/env.sh` 给出 `world=simulation/worlds/indoor_1.world`、spawn=`(-7.5, 0.5, 0.32, yaw=0)`。`SPAWN_Z=0.32` 是用户已验收的平地高度；历史 `0.25` 会使腿插地，禁止回退。默认启动 Gazebo GUI、RViz、ARiADNE、SCAN、MID360、OctoMap。

停止：优先 `bash simulation/kill_all_sim.sh`，并复核 `pgrep -x gzserver`、`rosnode list`。启动器会拒绝已有 `gzserver`，避免旧 `/clock`/点云污染。

## 5. Go2 仿真实现

`[CURRENT]` URDF：`algorithms/local_planning/scan_planner/src/simulator/Utils/go2_description/urdf/go2_description.urdf`。body link 名是 `base`（不是 `base_link`）；插件为 `libgo2_kinematic_model_plugin.so`。插件在 Gazebo `WorldUpdateEnd` 用 `/cmd_vel` 按 simulation `dt` 积分 x/y/yaw，并 `SetWorldPose`；默认 body z 保持 spawn z（仅保留历史 `/sim/body_z_target` 兼容路径，当前二维主链不使用）。body 消息 `child_frame_id=base`，lidar 消息 `child_frame_id=mid360`，插件内默认 lidar 偏移为 x=`0.2`、z=`0.2077` m。

12 个腿关节在 URDF Gazebo 标签中设为 `kinematic=true`，避免 ODE 与直接视觉关节写入竞争；站姿由插件缓存，步态目标来自 `go2_gait_publisher`。当前用户已目视验收：四腿完整、站姿/视觉步态正常、RViz 与 Gazebo 一致。不要把此模型当楼梯物理证据。

## 6. Runtime provenance（强制）

任何 C++/Gazebo plugin 修改后：停止旧进程，重编译当前 checkout，检查环境与实际加载库：

```bash
echo "$ROS_PACKAGE_PATH"
echo "$CMAKE_PREFIX_PATH"
echo "$LD_LIBRARY_PATH"
pgrep -x gzserver
grep -F 'libgo2_kinematic_model_plugin.so' /proc/<gzserver_pid>/maps
sha256sum <实际加载的.so>
```

`[VERIFIED]` 最近验证的库应来自 `~/claude/raicom/go2-scan_2D/.../devel/lib/libgo2_kinematic_model_plugin.so`。严禁加载旧 `~/claude/raicom/go2-scan/...` 的 build/devel/install 或 plugin。启动器自身路径推导 `GO2_ROOT`，但用户环境仍可能污染 ROS/LD 路径，必须检查 `rospack find scan_planner`、`rospack find go2_description`。

## 7. ARiADNE

`[CURRENT]` launch：`algorithms/global_planning/ariadne/src/rl_planner/launch/go2_ariadne.launch`。输入 `/projected_map`（`map` frame）和 `/quad_0/body_pose`；`map_resolution=0.2`、`node_resolution=2.0`、planner/sensor range=6 m、`utility_range_factor=0.5`、`frontier_downsample_factor=1`、replanning=2.5 Hz；save/D*Lite/escape 均关闭。输出 `/way_point`。ARiADNE 只负责当前层二维探索，当前 main 已删除 TARE。不要未经实验修改网络、checkpoint 或决策语义。

## 8. SCAN-Planner

SCAN 是局部规划/轨迹跟踪，不负责探索、SLAM 或 SDK。当前 `advanced_param.xml`：`navi_mode=3`（由 Gazebo launch 传入）、grid resolution=`0.05`、body height=`0.4`、规划 horizon=`3.5`。官方 Go2 double-cylinder footprint 已确认：radius=`0.25` m、offset=`0.18` m，见 `advanced_param.xml` 的 `grid_map/double_cylinder_*`；中心按 body pose（当前为 `base` 语义）使用。当前二维标准输入是 world cloud 经 `sensorScanGeneration` 得到的 `/sensor_scan`，同时传 body/sensor pose；`cloud_is_world` 在标准 Gazebo launch 为 `true` 的 SCAN 配置路径。

## 9. 点云、地图与参数

当前 Go2 URDF 的 Velodyne RaySensor 话题为 `/mid360_points`，`frameName=world`、`worldFrame=true`。插件源码：`simulation/cmu_env/src/velodyne_simulator/velodyne_gazebo_plugins/src/GazeboRosVelodyneLaser.cpp`；`worldFrame` 读取扫描消息自带 `scan.world_pose()`，将 XYZ 显式转成 world，避免实时 `WorldPose()` 时序误差。点云 header 实际应为 `world`；启动后仍可用 `rostopic echo -n1 /mid360_points/header` 复核。

OctoMap/ARiADNE 配置在 `go2_ariadne.launch`：resolution=`0.2` m，frame=`map`，`base_frame_id=world`，`filter_ground=true`，`occupancy_min_z=0.2`、`occupancy_max_z=0.8`，`sensor_model/max_range=6`，hit/miss/max/min=`0.70/0.40/0.97/0.12`。`map→world` 是恒等 TF；ARiADNE 自己按 `map_resolution` 解释投影网格。

重要边界：当前代码已有 z slice `[0.2,0.8]`（世界/地图语义），但**尚未证明它等价于局部地面相对高度**。`[TARGET]` 目标二维障碍定义为 `local_ground+0.20m ~ local_ground+0.90m`；`[TODO]` 应审计点云 z filtering 与 OctoMap 投影，并令 ARiADNE/SCAN 使用一致障碍语义。不要把目标规则写成已完成功能。

## 10. 已知 warning 与已删除模块

`Spawn service failed`、部分 TF warning 在 Go2 已存在、body pose/点云/`/cmd_vel` 正常且用户已 GUI 验收时可作为非致命 warning 记录；先查进程/库来源，不要因此重写狗模型。

当前 main 已删除/不再运行：TARE、stair/multifloor、hotel/Depot 场景、`elevation_mapping`、`terrain_analysis`、旧 `local_planner`、`go2_kinematic_sim`、`loam_interface`、physical Go2 workspace 及历史 benchmark 资产。`tools/evaluate_exploration.py` 仍保留作评估工具；`fastlio_integration.launch` 只是外部 LIO 接口模板。

## 11. 下一阶段（仅建议，不自动执行）

1. `[TODO]` 查清二维障碍高度是否应改为 local-ground-relative；
2. `[TARGET]` 统一 ARiADNE 与 SCAN 的障碍语义并验证；
3. `[TODO]` 以外部 LIO 的 pose/registered cloud/TF 接口做仿真接线验证；
4. `[TARGET]` 真机前完成低速、安全、时钟和 provenance 检查。

## 给下一位 AI 的第一条任务

不要立刻改代码。先启动当前标准 `indoor_1` 仿真，确认当前 main 与本文一致，再继续二维障碍语义工作。
