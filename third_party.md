# 第三方算法与依赖清单

> 本仓库**自包含全部第三方源码**（vendor 方式：源码直接进仓库、删掉各自 `.git`、用下表的上游 commit 溯源）。
> 除 ROS / 系统标准库外，仿真 + 算法所需的一切第三方代码都在 `algorithms/` 和 `simulation/` 里，**无需再 clone**。
> 更新上游：从下表 URL 重新 clone 对应 commit，覆盖本仓库对应目录即可（注意我们已在上游基础上做了四足化改造，覆盖前先 diff）。

## 算法层（algorithms/）

| 仓库内位置 | 上游仓库 | commit | 作用 |
|-----------|---------|--------|------|
| `algorithms/global_planning/tare/` | https://github.com/caochao39/tare_planner | `44500592b861` | 全局探索决策（TARE，CMU） |
| `algorithms/global_planning/ariadne/` | https://github.com/marmotlab/ARiADNE-ROS-Planner | `773ebcf60334b2a16a865f1207938f53fee92031` | 全局探索决策（ARiADNE，RL 注意力，ICRA23；**main 分支=ROS1 Noetic**，别用 humble=ROS2） |
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

## ARiADNE 运行时依赖（非 vendor，需在机器上装一次）

ARiADNE 是纯 Python 包（`rl_planner`），**不用 catkin_make**，靠 `launch_gazebo_sim.sh` 里 `ROS_PACKAGE_PATH=$ARIADNE/src` 被 rospack 找到。但推理需要装以下库：

- **octomap**（ROS 标准，已装）：`ros-noetic-octomap`，把 3D 点云投影成 2D `/projected_map`。

> **⚠️ 坑：ARiADNE「过早完成」**（`key_utility==0` 即 `self.done=True` 永久冻结）。四足场景常出现「局部无可见前沿（多被墙挡）但全场远未扫完」，ARiADNE 会判"Exploration Completed"并停，狗只走一两步。**已做四足适配**（`rl_planner.py:238`）：去掉 `self.done=True` 和 `return`，utility==0 也继续 `select_next_waypoint` 发 waypoint，让狗持续尝试绕墙。代价：真扫完时会继续发相邻节点 waypoint（可能原地徘徊），以 2D 图铺满为准判断完成。这是对"尝试算法"的取舍，非原版行为。

> **⚠️ 坑：四足自占据 → ARiADNE 误判"探索完成"，狗不动**。MID360 装在 Go2 顶部 0.46m，360° 会扫到机器人自己的腿/身体（z≈0.25-0.46）。octomap 若用默认 `occupancy_min_z=0.0` 投影，机器人自己被投影成**占据格**，导致 ARiADNE 的 `get_updating_node_coords` 在机器人处找不到自由节点、`check_collision` 从相邻节点到任何前沿都先撞这团"机器人占据"→ `key_utility` 恒 0 → `rospy.loginfo("Exploration Completed")`，狗原地不动。**修法：octomap `occupancy_min_z` 设 0.5**（剔除 z<0.46 的自身，墙高>0.5 不受影响）。判断办法：`/projected_map` 里机器人所在格值是 100（占据）。注意 `_min_utility:=1` 也救不了（节点看清前沿=0 不是阈值问题）。

> **⚠️ 坑：开机起点断言死循环 + Timer 线程静默死亡 + 忙等自锁**（2026-08-21 实测，三个叠加表现为"狗一动不动、RViz 无图无前沿"）。上游三处脆断点：
> ① `get_loc_callback` 只在 NODE_RESOLUTION 网格 **4 个角点**里找起点，开机时 octomap 还没把机器人周围清成 free → 4 个全不是 free → `assert self.start is not None` 崩在回调里；rospy 回调异常**不杀节点** → 每条 body_pose 消息崩一次、无限循环刷爆 rosout（10 分钟 18MB）。
> ② `run()` 里 `self.start[0]` 在 start=None 时抛 TypeError 把 **Timer 线程打死**；且该异常走 stderr 只上屏、不进 ~/.ros/log 的节点日志文件——日志看起来"安静"，实际永远不发 waypoint。
> ③ `__init__` 里 `while map_info is None: pass` 纯忙等霸占 GIL，会饿死恰恰负责置位这两个标志的回调线程。
> **已做四足适配**（rl_planner.py）：搜索扩到 ±2×NODE_RESOLUTION（25 候选）+ 自带越界保护（`is_free`→`get_cell_position_from_coords` 对地图外候选会 assert 崩）；找不到起点不崩、throttle 告警等下一帧；`run()` 加 start None 早退；忙等改 `rospy.sleep(0.05)`。诊断手段：`~/.ros/log/<run_id>/rl_planner-N.log` 看 `can not find valid start point` 刷屏 = 中招①；有 /clock 有 body_pose 但无 waypoint 且日志安静 = 中招②。

> **⚠️ 坑：ARiADNE×SCAN-Planner 目标死锁（站桩）**（2026-08-22 实测）。ARiADNE 的 0.4m 投影图分辨率粗，RL 选出的节点可能落在**离墙脸仅 0.1~0.3m 的格子里**（本格 free、邻格全是墙；ground truth 验证：目标(-5.5,0.5)距墙脸(-5.6)仅 0.1m）。下游 SCAN-Planner 精细地图+膨胀判该目标 "occupied"（`adjustGlobalTargetIfOccupied` 刷屏），把目标**回退到机器人自身位置** → 「已到达」→ cmd_vel=0 站桩；ARiADNE 收不到任何拒收反馈，utility 全 0 下每拍重发同一目标 → 永久死锁。上游 save_mode 兜底在此场景**无效**：`path_to_nearest_frontier` 只对 utility>0 的节点计算（`node_manager.get_rarefied_graph`），utility 全 0 时恒 None。**已做四足适配**（rl_planner.py + utils.py）：发 waypoint 前「目标消毒」——目标及其 3×3 邻域（≈±0.4m 净空）全 free 才放行；否则改发「沿线最后一个带净空的逼近点」（贴近障碍物让雷达看清绕行通道）；连逼近点都没有则本拍不发。

> **⚠️ 坑：gazebo_bridge 腿态服务风暴 → gzserver 服务连接重置 → livox 输出退化**（2026-08-22 实测）。gazebo_bridge 曾以 30Hz 调 `set_model_configuration`（service），高负载下 gzserver 报 `ConnectionResetError: [Errno 104]`，随后 **livox 插件整帧输出退化成传感器原点一团点**（10000 点全部 <0.55m、z=传感器高度 0.46，雷达实际"失明"，但话题频率正常、/clock 正常走）。降到 10Hz 降压。诊断手法：`rostopic echo /mid360_points` 一帧算水平距离分布——全部 <1m 即中招。疑与上次「Gazebo 物理冻结」同族，未根查插件本身。

> **⚠️ 待查：仿真 RTF 异常**。indoor_1.world 配置 `real_time_factor=1` + `real_time_update_rate=100`，实测 RTF≈1.5~4（sim 时间跑得比 wall 快）。不影响功能正确性但会放大服务压力、让 wall-clock 类延迟（如 tare_goal_bridge start_delay=15s）在 sim 语义下失真。未修。

> **⚠️ 坑（传感器层）：livox 插件零程点 + 间歇性平环退化帧 + 自体回波，三重污染下游地图**（2026-08-22 实测）。
> ① 插件把未击中射线的 range 置 0 后**仍按 `range*axis` 发布** → 鬼影点精确落在传感器原点；② 间歇性「平环」退化帧（整帧 z 恒等于传感器高度、距离集中 5-10m；时好时坏，机制未根查，疑与 gzserver 服务压力/RTF 异常同族）；③ MID360(装高 0.46m) 正常扫描也持续打到 Go2 自身身体。
> octomap 的 `occupancy_min_z=0.5` 按【体素】过滤挡不住——z=0.46 的点落在 [0.4,0.8) 体素层、体素中心 0.6 > 0.5 照样通过；SCAN grid_map 则完全没有近距过滤。结果：两边地图的机器人周边全被染成占据 → ARiADNE 与 SCAN 对同一格子判断相反 → 目标被拒/误判 → 狗站桩。
> **修法：新增 `integration/go2_bridge/scripts/cloud_range_filter.py`**——剔除与传感器（/quad_0/lidar_pose）水平距离 <0.7m 的点，按原始字段布局重打包发 `/mid360_points_clean`；gazebo_sim.launch 里 octomap 与 SCAN 均改吃净化话题，octomap `sensor_model/miss` 0.45→0.35 加速洗掉陈旧污染格。注意：**elevation.launch 仍吃原始 /mid360_points**，做楼梯检测前需一并切换。诊断手法：连续采几帧算 z 的 unique 值数量（健康帧数千个、平环帧只有 1 个）。

> **⚠️ 坑（2026-08-22 最终定位）：起点偏离生成格栅 → 图每拍被清空 → 效用恒零 → waypoint=自身位置站桩。** `get_updating_node_coords` 生成的候选节点对齐全局 NODE_RESOLUTION 格栅（整数倍坐标）；我们改过的起点搜索若以机器人为中心取候选，机器人坐标 (-7.5,0.5) 是半格 → start 永远不在格栅上、连不上图 → `remove_unconnected_nodes(start)` 每拍把其余节点**全部清光**（离线复现实测：19 节点建好、连边正常、18 个效用>0，清除后仅剩 1）→ 图退化孤点 → RL 只能选自身位置。**修法：起点直接取全局格栅点**（机器人位置四舍五入到 NODE_RESOLUTION 整数倍 ±6 格，按距离排序逐个验 free）。诊断手法：离线复刻 add_node→连边→remove 三步看节点数是否在第三步崩塌；注意 `min_utility` 的语义是 `utility <= MIN_UTILITY` 即清零（=1 表示需要 ≥2 个可见前沿）。另：`visualize_graph` 三个可视化消息的 frame_id 硬编码 'map'，本系统无 map→world TF、RViz Fixed Frame=world，已改 'world'。

> **⚠️ 坑：ARiADNE `min_utility=3`（上游默认）在真实传感器投影下把全部节点效用清零**（2026-08-22 实测）。语义：节点可见前沿数 ≤ MIN_UTILITY 就视为零效用（`node_manager.py` 两处 `if self.utility <= MIN_UTILITY: utility=0`）。上游按其完美锥形传感器模型调的默认 3；我们的 octomap 投影保守（`check_collision` 把 UNKNOWN 也当碰撞，utils.py:255），单节点可见前沿常只有 1~2 个 → 全部清零 → RL 只会选当前节点 → goal=狗自身位置 → 「已到达」站桩。**修法：launch `min_utility` 3→1**。离线复现手法：订阅 `/projected_map`，用包内 utils.check_collision 对最近 20 个前沿逐条测视线（free 才通），统计可达数。

> **⚠️ 坑（低级但极耗时，2026-08-22 双中招）**：① go2_bridge 新增 Python 脚本必须 `chmod +x`——否则 roslaunch **静默跳过**该节点（无任何报错、rosnode 列表里直接没有它），下游全部饿死等话题；② RViz1（Noetic）的占据栅格显示类是 **`rviz/Map`**，不存在 `rviz/OccupancyGrid`（那是 ROS2 rviz2 的类名），default.rviz 里写错只会显示红叉「class could not be loaded」。

> **⚠️ 排除项：ariadne 模式下 `/registered_scan` 没人发布是正常的**——tare_bridge 在 TARE 专属 group 里不启动，octomap 直接吃净化前的话题。别把它当 bug 查。
- **conda 环境 `ariadne`**（`/home/pumpkin-db/miniconda3/envs/ariadne`，Python 3.8，装以下全部）：
  - `torch==2.3.1+cpu`（**锁 2.3.1**：系统 python 是 3.8，torch≥2.4 不认）
  - `scikit-image matplotlib`（rl_planner 的 `utils.py` 用 `skimage.morphology.label`；**缺它会直接 `ModuleNotFoundError` 起不来**）
  - `rospkg catkin_pkg pyyaml`（这三样在系统 `/usr/lib/python3/dist-packages` 里，但**不能挂那个目录**——见下）
- **为什么放 conda 而非系统 python**：ROS 节点跑在清掉 conda 后的系统 python3.8 上，无 torch；conda base 是 3.13，装不了 ROS（rospy 编译组件是 python3.8 ABI）。故单独建 `python=3.8` 的 conda 环境，经 `run_ariadne.sh` 包装运行。

> **⚠️ 关键坑（numpy ABI）**：`run_ariadne.sh` 里 `PYTHONPATH` **只能挂 `/opt/ros/noetic/lib/python3/dist-packages`，绝不能加 `/usr/lib/python3/dist-packages`**。后者含系统 numpy 1.17，会抢在 conda numpy 1.24 前被 skimage/torch 用到，报 `ValueError: numpy.ndarray size changed ... Expected 88 got 80`（skimage 是 C 扩展、对 numpy ABI 严格）。torch 宽容能忍，skimage 不行。已把 rospkg/catkin_pkg/yaml 装进 conda 解耦，不必再靠 `/usr/lib`。

> 若系统 python 能直接 pip 装 torch，可用更简方案：`sudo /usr/bin/python3 -m pip install --index-url https://download.pytorch.org/whl/cpu torch==2.3.1`（但动系统环境，故默认走 conda）。

## ARiADNE A2 重做（2026-08-24，架构级变更记录）

官方仿真（new_algorithm/ARiADNE-ROS-Planner + CMU env）端到端跑通后回看旧集成，确认**架构错误**并重做。本次改动全部在配置/胶水层，上游算法逻辑未动（除下述两处带注释的四足适配）。

### ① octomap 射线原点 bug（旧版地图穿墙的根源）
- 旧接线：世界系 `/mid360_points_clean` 直喂 octomap + `frame_id=base_frame_id=world`。
- 机制：octomap_server 用 `lookupTransform(frame_id, cloud_header.frame_id)` 定位**射线起点**；两 frame 相同时 TF 恒等 → 射线从世界 (0,0,0) 发出 → 自由空间沿"原点→点"乱标、穿墙、边界失真。
- 正确模式（CMU 官方链路实测验证）：世界系点云+位姿 → **sensorScanGeneration** 转回传感器系（frame=sensor_at_scan）并广播 TF map→sensor_at_scan → octomap 设 `frame_id=map`。map 系数值 ≡ world 系（里程计即世界坐标），rl_planner 直接可用。

### ② 接口从 navi_mode=1 改为 navi_mode=3（A2）
- 旧：`tare_goal_bridge` 把 /way_point 转 `/move_base_simple/goal`，SCAN 追单点直线走（ARiADNE 选的节点可能隔墙）。
- 新：`ariadne_goal_bridge.py`（integration/go2_bridge/scripts/）把 /way_point 包装成 **2 点 Path**（机器人当前位置→航点，z 全发 0，SCAN 自动加 body_height 0.4）发 `/initial_path`；SCAN navi_mode=3 以此为全局参考做 A*初值+B样条优化，会绕障。
- 桥内置去重门控（航点移动 >repub_dist=1.0m 才转发）：ARiADNE 以 2.5Hz 重发同一目标，不去重会触发同频全量重规划。废弃 tare_goal_bridge 的墙钟 start_delay 设计（PROGRESS.md 六-5 缺陷）。
- launch 里 `global_planner:=ariadne` 时 navi_mode 自动切 3（roslaunch eval 实现）。

### ③ 「过早探索完成」三连修（run2/run4/run5 实录，离线复现定位）
现象：探索 ~16%（GT 场景 53×34m 只探一角）即宣布 Completed 并永久冻结。逐层根因：
1. **效用视野塌缩**：sensor_range 16→5 后 `UTILITY_RANGE=0.5×5=2.5m`，前沿环在 ~5m 外全够不着 → `utility_range_factor` 提到 1.0。
2. **min_utility 的 <= 语义**：node_manager.py 用 `utility <= MIN_UTILITY → 清零`，设 1 时"只看到 1 个前沿"的节点也归零 → 取 0。
3. **unknown 阻挡视线（核心）**：门洞前沿只有 1~2 格且紧邻门框锯齿格，`check_collision` 把 unknown 当碰撞 → 46 对节点-前沿视线 46 挡 → 全图效用恒零。新增参数门控的 `check_collision_ignore_unknown`（utils.py，只挡 OCCUPIED）+ `_utility_los`（node_manager.py），launch 开关 `los_ignore_unknown`。仅影响能见度评估；路径有效性检查仍用原版。
4. **结构性兜底——停滞式完成判定**：以上都是概率性缓解，最终改为「效用全零 **且** 地图连续 stagnant_done_sec(20)s 无增长」才 done；期间继续正常规划。地图增长监测在 get_map_callback 里统计 free 格数。这样过早冻结在结构上不可能发生，比赛需要的完成信号依然真实。

### ④ 其他实录坑
- **孤儿 gzserver 发布陈旧 /clock**：清理不彻底时，旧 gzserver 在新 master 起来后重连，新栈一启动 sim time 就 >1000、spawn_go2 报 "Spawn service failed"、全链路假死。对策：重启前跑 `simulation/kill_all_sim.sh` 并以 pgrep 复核为空。
- **Timer 线程异常只上 stderr 且被缓冲**：回调一抛异常线程即死、日志无痕（run3）。已在 rl_planner.py 给 run() 套 try/except 写 rosout，并在 run_ariadne.sh 加 `PYTHONUNBUFFERED=1`。
- **geometry_msgs 没有 Path**：`nav_msgs.msg import Path`（ariadne_goal_bridge 首跑 ImportError）。
- vendored 包内 5 个 launch（rl_planner.launch 等）是上游模板留作参照，实际生效的是 gazebo_sim.launch 的 ariadne 分支。

## 编译

```bash
# 1. 局部规划 workspace（scan_planner + plan_env 等 + go2_description + map_generator）
cd ~/claude/raicom/go2-scan/algorithms/local_planning/scan_planner && catkin_make

# 2. 全局探索 workspace（tare_planner，自带 or-tools）
cd ~/claude/raicom/go2-scan/algorithms/global_planning/tare && catkin_make

# 3. 高程图 workspace（elevation_mapping + kindr 等）
cd ~/claude/raicom/go2-scan/algorithms/mapping/elevation_mapping && catkin_make

# 4. 仿真底座 workspace（velodyne/livox 插件 + ARiADNE 链用的 sensorScanGeneration）
cd ~/claude/raicom/go2-scan/simulation/cmu_env && catkin_make --force-cmake -DCATKIN_WHITELIST_PACKAGES="sensor_scan_generation" && catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation"
```

> go2_bridge（`integration/go2_bridge/`）是纯 Python 包，不编译，靠 `launch_gazebo_sim.sh` 里 `ROS_PACKAGE_PATH` 指向 `integration/` 被 rospack 找到。

## 启动

```bash
bash ~/claude/raicom/go2-scan/simulation/launch_gazebo_sim.sh
```
