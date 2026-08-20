# 11 · SCAN-Planner 话题/参数/命令速查（仿真态）

> 全部来自代码核实（文件:行号见《调研与交接》§3），2026-08-18 实跑复核。

## 数据流（仿真闭环，navi_mode=1）

```
mockamap_node ─/map_generator/global_cloud─▶ pcl_render_node(CPU,10Hz)
                                                ├─/pcl_render_node/cloud(世界系)─▶ grid_map(在scan_planner_node内)
                                                └─/quad_0/lidar_pose─▶ grid_map.sensor_pose
/move_base_simple/goal ─▶ scan_replan_fsm（A*+B样条）─/planning/bspline─▶ closed_loop_controller(100Hz)
                                                                            └─/cmd_vel─▶ go2_kinematic_sim(纯积分,z不动)
                                                                                        └─/quad_0/body_pose─▶（回到渲染/FSM/控制器）
RViz 狗模型：/joint_states←go2_gait_publisher；world→base TF←odom_visualization(tf45默认true)
```

## 话题表

| 话题 | 类型 | Hz | 说明 |
|---|---|---|---|
| `/move_base_simple/goal` | PoseStamped | 事件 | mode1 目标入口（RViz 2D Nav Goal；goal z 会被覆盖为出生点 z） |
| `/planning/bspline` | scan_planner/Bspline | 事件 | 规划输出，traj_id 递增 |
| `/cmd_vel` | Twist | 100 | closed_loop 输出；**实机时接 cmd_vel_bridge**（在 NX） |
| `/planning/go2_execution_frozen` | Bool | 100 | true=偏航差>0.8rad 原地对航向（正常行为，不是故障） |
| `/quad_0/body_pose` | Odometry | 100 | 仿真=kinematic_sim 积分；实机=FAST-LIO `/Odometry` remap |
| `/quad_0/lidar_pose` | Odometry | 100 | 渲染节点由 body_pose 加 lidar_pitch(45°) 算出 |
| `/pcl_render_node/cloud` | PointCloud2 | 10 | 渲染的传感器点云（世界系） |
| `/grid_map/occupancy` `/occupancy_inflate` `/sliding_map_bbox` | PointCloud2/Marker | 20 | **有订阅者才发布**（getNumSubscribers 门控） |
| `/initial_path` | Path | — | mode3 参考路径（覆盖规划挂点，未测） |

## 关键参数（advanced_param.xml 默认值，✅）

```
max_vel 0.75 / max_acc 0.5 / planning_horizon 3.5
栅格 0.05m；滑窗 10×10×5m（map_sliding_thresh 0.2）
双圆柱足迹 radius 0.25 / offset 0.18 / z膨胀±0.1；body_height 0.4
闭环 kp_pos 0.8 / kp_yaw 1.5 / max_vy 0.35 / max_vyaw 1.0 / finish_dist 0.15 / heading_error_threshold 0.8
仿真出生点 mode1: (-19, 1, 0.25)（simulator.xml）；mockamap seed=127 type=2 500障碍 40×40×5m
```
⚠️ README 说默认参数按作者 Go2 调过，但我们的安装/场地不同，**实机必须重调**。

## 启动命令

```bash
# 默认随机柱场景
roslaunch scan_planner run.launch [navi_mode:=1] [controller_mode:=closed_loop]
# PCD 墙场景（wrapper 在本目录）
roslaunch ~/claude/raicom/仿真迁移/local_sim_pcd.launch
# RViz
roslaunch scan_planner rviz.launch
# 程序化发目标（不等 RViz 手点）
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
'{header: {stamp: now, frame_id: "world"}, pose: {position: {x: -9.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
```

## 就绪门（发第一个目标前必须全过，防"盲走"）

1. run.log 出现 `Normal compute finished..`（渲染节点首次建图法线估计完成，秒~十秒级）；
2. run.log 出现 `Set RViz goal height from initial body_pose z`（FSM 就绪）；
3. `/pcl_render_node/cloud` ≈10Hz；`/grid_map/occupancy` 有数据。
在此之前地图未融合，未知区当空闲，点了目标狗会盲走。

## RViz 显示项说明（首跑用户提问记录）

- **Simulation 组的 `self_inflation`**：规划器发布的**双圆柱碰撞体可视化**（`scan_replan_fsm.cpp:485-520`）：两个半透明蓝圆柱，半径=self_double_cylinder_radius(0.25)、沿狗头向 ±offset(0.18)、高=z_up+z_down(0.2)。即 SCAN-Planner 核心碰撞模型「yaw 感知双圆柱足迹」的实时包络，跟着狗走。
- **go2_robot 的 `Visual Enabled`**：URDF `<visual>` 外观网格开关；`Collision Enabled`：`<collision>` 简化体开关。
- **🐧W mesh 渲染问题（2026-08-18 已确诊并解决）**：本 WSLg 环境下 RobotModel 的 dae 外观网格与 MESH_RESOURCE Marker 均「Status Ok 但不显示」，collision 简化体/普通 Marker/点云正常。根因 = WSLg 的 D3D12-GL 包装对 Ogre mesh 实体渲染缺陷（推断，但解法已验证）。**解法（✅ 用户确认有效）**：`LIBGL_ALWAYS_SOFTWARE=1 roslaunch scan_planner rviz.launch`（llvmpipe 软渲染，慢一点但完整外观正常）。以后本机开 RViz 一律带这个环境变量。诊断工具：`tools/assimp_test.cpp`（隔离 assimp 解析层）。迁原生 Linux 后复验硬件渲染是否可用。

## 已知限制（接线/魔改前必看）

- **run.launch 不转发** simulator.xml 的参数（use_pcd_map / pcd_map_file / init_x/y/z / map_size_*）和 advanced_param.xml 的参数（max_vel 等）——CLI 覆盖报 unused arg；要覆盖用 wrapper launch（参考本目录 local_sim_pcd.launch）。
- FSM 只等 body_pose 不等点云；goal 在首帧 body_pose 前发会被丢弃（日志有提示）。
- world→base TF 唯一来源是 odom_visualization（tf45）；kinematic_sim 的 publish_tf 默认 false。若 RViz 狗模型不动：给 kinematic_sim 加 `<param name="publish_tf" value="true"/>`（child_frame_id 默认 `base`=URDF 根）。
- mockamap 的 update_freq=0.5 是**重发布频率不是重新随机**，地图启动后不变。
- 仿真无物理引擎：z 恒为出生高度、无摔倒概念；运动能力/摔倒风险仿真验证不了（实机红线）。
- 死胡同不会倒车绕行（见 10 篇 run2 结论）——局部规划器定位决定，全局回路由上层做。

## 实机话题对照（接线时查，《Linux 侧交接文档》§2.2 已核实）

| 仿真 | 实机（NX FAST-LIO） |
|---|---|
| /quad_0/body_pose | `/Odometry`（大写 O，~10Hz） |
| /quad_0/lidar_pose | `/LIO/odom_imu` 或同样 remap `/Odometry`（上轮仿真验证过双 remap 可行） |
| /pcl_render_node/cloud | `/cloud_registered`（世界系，cloud_is_world:=true，用 fastlio_integration.launch） |
