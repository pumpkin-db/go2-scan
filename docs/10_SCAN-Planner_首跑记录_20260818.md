# 10 · SCAN-Planner WSL2 首跑记录（2026-08-18）

> 仓库：`~/claude/raicom/new_algorithm/SCAN-Planner`（commit 348e8a5 "add: sensor layout"）。
> 前提：上一会话（08-16）已编译通过、话题级验证过默认仿真；本轮补上**端到端可视化运行 + PCD 地图源**。

---

## Run 1 · 默认场景（mockamap 随机柱）✅

**启动**：
```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch scan_planner run.launch        # navi_mode=1 closed_loop CPU渲染
roslaunch scan_planner rviz.launch       # WSLg 弹窗
```
日志：`logs/20260818_run1/`。

**实测（✅）**：
- 节点 10 个全起：scan_planner_node / closed_loop_controller / go2_kinematic_sim / go2_gait_publisher / go2_robot_state_publisher / mockamap_node / pcl_render_node / odom_visualization / rviz / rosout。
- 频率：`/pcl_render_node/cloud` 10.0Hz、`/quad_0/body_pose` 100Hz、`/cmd_vel` 100Hz。
- 就绪日志顺序：`[closed_loop_controller] ready` → `[Go2 kinematic sim] ready` → `Set RViz goal height from initial body_pose z: 0.250` → `Global Pointcloud received..` → `Normal compute finished..`（首次建图法线估计 ~3s，mapsize 223015）。
- 自动发目标 (-19,1)→(-9,1)：FSM 进 EXEC_TRAJ，`/cmd_vel` linear.x≈0.54 m/s，狗实际移动；随后改为用户在 RViz 手动点目标测试。
- **用户结论**：路径规划、局部避障可以。
- 自动化到达监视被用户测试打断，未记录完整到达数据 ⚠️；到达能力以用户 RViz 目视为准。

**已知良性告警（✅ 确认无害，勿排查）**：
- `Failed to find match for field 'intensity'`——RViz sim_map 显示要 intensity 通道，点云是 PointXYZ，纯外观问题。
- KDL `root link base has an inertia` 警告——robot_state_publisher 的已知提示，不影响。
- `Local target in collision, adjusted to a nearby collision-free point`——目标点落在障碍里时自动挪到邻近空闲点，设计行为。

## Run 2 · PCD 墙场景（map_pub 地图源）✅

**动机**：换掉随机柱，用结构化场景测穿行；同时验证 `map_pub` 这条地图源通路。

**关键事实（✅）**：仓库**不带任何 PCD**——`simulator.xml` 引用的 `$(find map_generator)/resource/building.pcd` 不存在（resource/ 目录整个缺失），直接 `use_pcd_map:=true` 会秒退。本机也没有其它 .pcd。

**解法**：用旧项目 2D 地图挤出 3D 墙——
1. `tools/pgm_to_pcd.py`（glue 脚本）读 `~/catkin_ws/src/map/map1.pgm`（map_saver 产物，0.15m/格，234×69，世界范围 x[-9,26.1]×y[-7.8,2.55]），占据格按 occupancy>0.65 判定，沿 z 挤出 0~2m、每 0.1m 一层 → `maps/map1_walls.pcd`（44436 点，ASCII PCD v0.7）。
2. `local_sim_pcd.launch`（wrapper，结构照抄 run.launch + 向 simulator.xml 转发 `use_pcd_map/pcd_map_file/init_x/y/z`——run.launch 本身不转发这些参数，CLI 覆盖会报 unused arg）。出生点 (3.38, 0.98)（脚本扫描出的周围 0.6m 全空闲点）。

**启动**：
```bash
roslaunch ~/claude/raicom/仿真迁移/local_sim_pcd.launch
roslaunch scan_planner rviz.launch
```
日志：`logs/20260818_run2_pcd/`。

**实测（✅）**：`Normal compute finished.., mapsize = 44436`（与 PCD 点数一致，证明 map_pub 加载正确）；cloud 10Hz；用户 RViz 多点测试，共收到 **62 条 bspline 轨迹**。

**用户结论**：
1. 总体还行，**规划速度明显快于之前的 move_base+TEB**（旧 2D 管线的全局 global_planner + 局部 TEB）。
2. **死胡同问题**：走进死胡同后卡住，不会倒车回去找另一条路。

**死胡同行为的日志证据（run2，✅）**：9 次 `[SAFETY]: from EXEC_TRAJ to EMERGENCY_STOP`（"Suddenly discovered obstacles. emergency stop!"），多数能 replan 恢复；但有一次 `Exiting EMERGENCY_STOP. Switching to WAIT_TARGET. Need a new target point.`——规划器放弃当前目标、等待新目标，即用户看到的「卡住」。
**结论**：这是局部规划器的**定位决定的**——SCAN-Planner 只对给定目标做局部重规划，没有全局路由/回退换路能力。死胡同恢复属于上层（覆盖/全局路径层，mode3 `/initial_path` 挂点）的职责。⚠️ 推断：滑窗地图只有 10×10m，深入死胡同后出口可能已滑出窗口，也是 replan 找不回出路的原因之一。

## Run 3 · arena 空房间 + 弓形→mode3 闭环 ✅（探索层第①步）

**场景**：`maps/arena_walls.pcd`（30×20m 空房间仅四周墙，5100 点，脚本即时生成）。map1 墙场景因内部隔墙不连通、狗卡死不动，**弃作覆盖测试场景**（用户判断）。
**链路**：`tools/sweep_to_initial_path.py`（pythonRobotics GridBasedSweepCPP 弓形 → nav_msgs/Path latch 发 `/initial_path`）→ SCAN-Planner `navi_mode:=3` 跟踪。启动：`local_sim_pcd.launch navi_mode:=3 pcd_map_file:=...arena_walls.pcd init_x:=2 init_y:=2`。
**实测（✅）**：弓形 2m 行距；轨迹采样确认逐行往返（y≈7 行右→左，换行 y≈9 左→右…直到 y≈15）；FSM EXEC_TRAJ/REPLAN 正常。
**坑记录**：
- RViz 若先于仿真启动，`robot_description` 参数未就位 → go2_robot Status Error；重启 RViz 即愈 🐧W（启动顺序）。
- shell 一条命令里 `A && B & C` 会把 source 并进后台子任务，后续 roslaunch 丢环境——分两条命令写。
- 长驻进程用 run_in_background 起，shell `&` 在本工具下偶被回收。
- **同名节点坑（重要）**：脏重启会让旧节点占名，新节点被拒（"new node registered with same name"），表现为 go2_gait_publisher/odom_visualization 缺失→joint_states 断+world→base TF 断→RViz go2_robot Status Error。处置：pkill 全部 ROS 进程→按序重启。重启前 `rosnode list` 核对 9 节点齐全。

## Run 4 · Spiral-STC → mode3（探索层第②步）✅ 集成通，⚠️ 走得不顺

**集成**：`new_algorithm/stc_ws/src/stc_glue/`（复制 nobleo spiral_stc/common/full_coverage_path_planner 三 cpp，Apache-2.0；自写 glue 节点 ~70 行：bool 栅格文本→`SpiralSTC::spiral_stc()`→nav_msgs/Path latch→/initial_path）。绕开其 nav_core/mbf 依赖（本机缺 mbf）。先验栅格 `maps/arena_grid.txt`（1m 格、arena 边界占 1 圈）。编译坑：新 ws 需 `-DPYTHON_EXECUTABLE=/usr/bin/python3`；spiral_stc.o 依赖父类实现，须连 full_coverage_path_planner.cpp 一起编。
**实测**：螺旋覆盖确在推进（左列 y 4→15 逐段爬升）；但**每个 90° 转角停 20-30s**（局部 replan 等待），10 次 EXEC_TRAJ 重规划；对比弓形（Run3）直行段流畅。
**结论**：STC 的强项是「连通区单遍完备」，弱项是路径碎、转角多，与四足局部规划器匹配差；若要用需加胶水平滑/抽稀（共线合并+转角圆化，属允许的组合胶水）。弓形在「执行顺畅度」上胜出；STC 在「完备性保证」上胜出。二者可互补：STC 出序、弓形化简执行。

## 迁移注意（🐧L）

- run1/run2 的启动方式与命令在纯 Linux 完全相同（无 WSL2 特有步骤）；RViz 换原生显示。
- `local_sim_pcd.launch` 用 `$(dirname)` 定位 PCD，整目录拷走即可复用。
- build/devel 不要拷，重新 catkin_make。

## 未测清单（下一步候选）

- mode2 keypoint 巡航（`tools/keypoint.yaml` 录制/回放）。
- mode3 `/initial_path` 参考路径跟踪（覆盖规划挂点；先查 GitHub issue #7 现状 ⚠️）。
- `fastlio_integration.launch` 接真实 FAST-LIO2 输出（或 bag 回放）复测——上轮只用了仿真真值模拟其话题语义。
- GPU 渲染 `use_gpu:=true`（有独显时）。
- PCD 源 + 更大/更接近真实场地的地图（可用真实 rosbag 建图后导出 PCD 回放）。
