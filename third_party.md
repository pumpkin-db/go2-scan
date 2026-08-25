# 第三方算法与依赖清单

> 本仓库**自包含全部第三方源码**（vendor 方式：源码直接进仓库、删掉各自 `.git`、用下表的上游 commit 溯源）。
> 除 ROS / 系统标准库外，仿真 + 算法所需的一切第三方代码都在 `algorithms/` 和 `simulation/` 里，**无需再 clone**。
> 更新上游：从下表 URL 重新 clone 对应 commit，覆盖本仓库对应目录即可（注意我们已在上游基础上做了四足化改造，覆盖前先 diff）。

## 算法层（algorithms/）

| 仓库内位置 | 上游仓库 | commit | 作用 |
|-----------|---------|--------|------|
| `algorithms/global_planning/tare/` | https://github.com/caochao39/tare_planner | `44500592b861` | 全局探索决策（TARE，CMU） |
| `algorithms/global_planning/ariadne/src/rl_planner/` | https://github.com/marmotlab/ARiADNE-ROS-Planner | `773ebcf` | 全局探索决策（ARiADNE，RL+图注意力；scripts 为官方原版零改动） |
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

- FAEL、FUEL、UFEP-Released、TravExplorer、gbplanner_ros、ego-planner、stc_ws、LKH-3.0.14、nlopt（gbplanner 依赖）、HPHS（场景制作工具）
- **ARiADNE2-ROS-Planner**（marmotlab，2026-08-25 克隆到 `try_algorithm/code/`）：HEADER 论文
  （arXiv 2510.15679）的 ROS1 Noetic 官方包，社区检测全局图 + 注意力决策，是 ARiADNE v1 同组续作。
  候选升级：编译冒烟后用我们评估器跑 indoor_1 对照再定。笔记见 `try_algorithm/notes/HEADER与HDPlanner精读.md`。

## 编译

```bash
# 1. 局部规划 workspace（scan_planner + plan_env 等 + go2_description + map_generator）
cd ~/claude/raicom/go2-scan/algorithms/local_planning/scan_planner && catkin_make

# 2. 全局探索 workspace（tare_planner，自带 or-tools）
cd ~/claude/raicom/go2-scan/algorithms/global_planning/tare && catkin_make

# 3. 高程图 workspace（elevation_mapping + kindr 等）
cd ~/claude/raicom/go2-scan/algorithms/mapping/elevation_mapping && catkin_make

# 4. 仿真底座 workspace（velodyne/livox 插件 + sensor_scan_generation，后者供 ARiADNE 链用）
cd ~/claude/raicom/go2-scan/simulation/cmu_env && catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"
```

> go2_bridge（`integration/go2_bridge/`）是纯 Python 包，不编译，靠 `launch_gazebo_sim.sh` 里 `ROS_PACKAGE_PATH` 指向 `integration/` 被 rospack 找到。
> rl_planner（`algorithms/global_planning/ariadne/src/rl_planner/`）同为纯 Python 包不编译，同样靠 `ROS_PACKAGE_PATH` 直指 `ariadne/src`；运行依赖 conda env `ariadne`（py3.8 + torch cpu），入口 `run_planner.sh`。

## ARiADNE（go2-scan 全局探索决策层）

### 官方数据契约（源码级核实，2026-08-24）

ARiADNE 是纯 2D 决策层：吃 2D 占据栅格 + 机器人 x/y，吐航点。官方 README 明确它需要三个外部伙伴：Lidar SLAM（里程计+点云）、建图（octomap）、waypoint follower。

| 话题 | 类型 | frame | 频率 | 语义 |
|---|---|---|---|---|
| /state_estimation（入） | Odometry | header=map, child=sensor | CMU 仿真 200Hz | **传感器(lidar)原点位姿**，非机体中心 |
| /projected_map（入） | OccupancyGrid | map | latched | octomap 投影；值 −1/0/100；origin=八叉树包围盒角点动态长大 |
| /way_point（出） | PointStamped | map | ≤2.5Hz | 下游裸读 x/y |

硬性机制：
- rl_planner **零 TF 查询、零 frame 校验**、零时间同步——把 map.origin 与位姿当裸数字混算（utils.py 断言机器人在图包围盒内）。数值一致性由接入方单方面保证。
- octomap_server 的射线起点 = TF(frame_id → 点云header.frame) 平移；有 tf::MessageFilter 门控（点云 stamp 时刻 TF 不可解则扣帧 → 地图空/滞后）。
- 官方链路单一全局系：CMU 把仿真器世界坐标直接命名 map；**map≡world 数值恒等是构造出来的**。
- sensorScanGeneration（CMU 包）：ApproximateTime(100) 同步 registered_scan+state_estimation → /sensor_scan(frame=sensor_at_scan) + 点云 stamp 时刻广播 TF map→sensor_at_scan。
- 分辨率两处一致：octomap resolution = rl_planner map_resolution（原 0.4，2026-08-24 用户决策改 0.2）——rl_planner 不读消息头只信自己的 param，两处不同步=全图错位。

### 第一次 A2 接入失败解剖（2026-08-24，完整快照在 archive/ariadne-a2-attempt1 分支）

症状：RViz 里 projected_map 悬空错位「完全不对」；vendored py 打满补丁成屎山。根因三层：

1. **frame 断裂（主因）**：octomap 设 frame_id=map，但全系统（RViz Fixed Frame、SCAN、里程计）都在 world 系，无任何 TF 桥 → 地图悬空。修法 = 一条恒等 static TF map→world（官方有真 map 帧所以没这问题，我们没有）。
2. **state_estimation 错接 body_pose**：官方该输入语义是雷达原点位姿；接机体中心使 octomap 射线原点系统性偏 ~0.4m（x 0.2 + z 0.21）。应接 /quad_0/lidar_pose。
3. **对抗性补丁病**：miss 0.45→0.35、utility_factor 0.5→1.0、min_utility 3→0、los_ignore_unknown、停滞判定、恢复导航……全是给「过早完成」打的标补丁，而病根之一正是上面两条地图错位。教训：**先修架构再调参数；参数偏离官方必须有当轮证据**。

另：scan_map 漂移与该批改动无关（逐行比对未动其链路）。幻影墙根因已实锤并修复（2026-08-24，run9 验证幻影 0 格）：

**幻影墙根因链（源码级+探针实证）**
1. 我们的场景有无限大 ground_plane，官方 CMU 场景没有（模型列表对比实证）。
2. more_complex_env_0 的薄墙/窗户会泄漏雷达射线打到室外地面；octomap_server 源码（kinetic-devel OctomapServer.cpp insertScan）：filter_ground=false 时所有端点一律标占据，无 z 带检查。
3. 地面端点 z≈0 → 体素中心 0.2（分辨率 0.4），投影条件「中心±半格与带相交」使 [0.2,0.8]/[0,1.2] 都包含它 → 建筑外围成片黑格。
4. 黑环封住自由空间边界 → rl_planner 效用归零 → 过早完成（8~10 航点即停）。
5. 曾误诊「livox 退化帧」（z_std≈0 帧）——实为 MID360 非重复扫描的正常单仰角环形态；据此加的帧级门控会误杀合法观测，已撤除。

**修复配方（run9 验证：幻影 0/428 格，自由:占据=1433:428≈3.4:1 达官方级，狗 54 航点未假完成）**
- 世界文件 indoor_1.world：无限 ground_plane → 有限地板 box（58×38m 覆盖建筑足迹）
- octomap：`occupancy_min_z=0.4`（关键一刀：地面端点的体素中心 0.2 投不进带，≥0.4m 真障碍经中心 0.6 体素照常投影）；max_z=1.2、收发耦合 7m 回本机跑通现场值(envB，非官方配方)
- 辅助：`filter_ground=true` + `base_frame_id=world`（RANSAC 在地面点占比仅 4.7% 时不可靠，只作辅助层）
- 教训：诊断时「z_std≈0 帧」「截断端点」等假设均被实验否决——每一步都必须用探针数据背书

诊断工具：tools/probe_occ_vs_gt.py（GT对照）、probe_map_once.py（快照）、probe_voxel_heights.py（体素高度）、probe_dynrange.py / probe_toggle2.py（动态参数 A/B）、probe_roundtrip.py（变换往返验证）、probe_live_support.py、probe_cloud_regions.py、probe_primitives.py、probe_frame_health.py。

### 正确架构（重做版，2026-08-24）

```
/mid360_points ─cloud_range_filter(0.7m)─> /mid360_points_clean ─┐
                                                                 ├ ApproximateTime(100)
/quad_0/lidar_pose(≈官方 state_estimation 语义) ─────────────────┘  ↓ sensorScanGeneration
                                    ├ /sensor_scan(sensor_at_scan) + TF map→sensor_at_scan
static TF: map→world 恒等 ──┐        ↓
                            └──> octomap_server[frame_id=map, res0.4, z∈0.2-0.8, max_range5, miss0.45]
                                      ↓ /projected_map(header=map)
/projected_map + /quad_0/body_pose(remap 成 /state_estimation) ─> rl_planner（官方原版一字不改）
                                      ↓ /way_point(frame=map)
ariadne_goal_bridge（唯一自研胶水，去重>1m） ─> /initial_path(Path[robot_xy,wp], world)
                                      ↓
SCAN navi_mode=3（替代官方 waypoint follower；同样裸读坐标）
```

参数基准 = 官方 indoor launch 全套（factor 0.5 / min_utility 3 / replanning 2.5 / node_resolution 2.0）。2026-08-24~25 用户决策三项偏离官方（A/B 判据：tools/probe_occ_vs_gt.py 幻影占比<5% + 航点数/用时）。**术语勘误（2026-08-25，用户强调）：官方配方 = 上游原版默认 sensor_range 20m，本机从未完整跑通；envB.log 的 7m 是「本机自设参数跑通官方 demo」的现场记录，不是官方配方**。①收发量程耦合 **6m（2026-08-25 终版：用户自行多轮实测后拍板，明确不再改动）**（沿革：官方配方 20 → 本机跑通记录 envB 7 → 用户 5 → 3 → 4 → 10 → 6.5 → **6 终版**；有效效用环 3.0m；黄点消失=SCAN optimal_traj 无更新=断供信号）；②**map_resolution 0.2**——octomap resolution 与 rl_planner map_resolution 必须同步改（rl_planner 只信自己的 param 不读消息头，不一致=全图几何错位），octomap 负载相对原版 ≈8×、决策图规模 ≈4×；③z 切片 **[0.2,0.8]**——免疫规则=min_z ≥ 一个整格（res=0.2 体素中心层 0.1/0.3/0.5/0.7…，地面端点落 0.1 层永不投影），用户原始四足障碍带偏好在 0.2 格下完整达成。链路配置在包内 `launch/go2_ariadne.launch`。

### 保留思路库（第一次接入的遗产，按需启用，不再预埋）

1. **停滞式完成判定**：「效用全零 且 地图 N 秒无增长」才算完成——真实投影下门洞前沿可能不可见，纯 utility 判完成会假停。若重做版再现假 Completed/假停滞，这是第一杠杆。
2. **目标消毒/恢复导航**：粗分辨率上游目标 vs 精细下游规划器的接口契约问题（is_free_with_margin/find_approach_point 思路），与新算法无关，应做成独立桥模块。
3. **等图忙等让出 GIL**：`while None: pass` 会饿死回调线程自锁，改 sleep。
4. **Timer 回调兜底**：rospy Timer 线程异常只上 stderr 且缓冲，死掉后日志无痕；try/except + PYTHONUNBUFFERED 是可观测性底线（run_planner.sh 已带 UNBUFFERED）。
5. **效用视野三杠杆**（出现「全图零效用/过早冻结」时依次查）：sensor_range 与 octomap max_range 解耦、utility_range_factor、min_utility 的 `<=` 语义。

### 环境坑清单（运行依赖）

- conda activate 本机崩 → run_planner.sh 用绝对路径 python
- 系统 python3 无 torch、conda base py3.13 进不了 ROS → ariadne env(py3.8 + torch 2.3.1cpu + scikit-image + rospkg)
- PYTHONPATH 只挂 /opt/ros/noetic/lib/python3/dist-packages，禁挂系统 dist-packages（numpy ABI 冲突）
- **包装脚本 exec python 时必须带 `"$@"`**：roslaunch 把 remap 与 __name 放 argv；漏掉则节点订阅字面量话题、launch 私有参数全落空（redo_run1 实录：节点卡死等图循环、sensor_range 走默认 20 而非 5）。上游 init_node 是 anonymous=True，私有参数命名空间依赖 argv 正确透传
- 权重定位走 rospkg（rl_planner.py:165）→ ROS_PACKAGE_PATH 必须含 ariadne/src
- kill_all_sim.sh 的复核 pgrep 会匹配到「命令行文本里含同款字面量」的调用方自身——不要把含进程名的 grep/pgrep 写在同一命令行里跑它

## 启动

```bash
bash ~/claude/raicom/go2-scan/simulation/launch_gazebo_sim.sh
```
