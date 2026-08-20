# SCAN-Planner 调研与集成手册（交接文档）

> 目的：交接给在纯 Linux 环境工作的 AI/开发者。后续仿真与实机验证都在纯 Linux 上跑（可能先在 WSL2 跑仿真），不在 WSL 里做实机。
> 整理日期：2026-08-16。所有结论都标注了证据来源；**未核实的内容一律标为「待验证」，不要当成事实**。
> 本工作区根目录（本机）：`~/claude/raicom/`，仓库在 `new_algorithm/SCAN-Planner/`。

---

## 0. 一分钟摘要

SCAN-Planner（上海交大 Tong Qin 组，arXiv 2606.19555，Apache-2.0）是一个 **ROS1 Noetic 的四足局部规划器**：给定目标点/参考路径后，用 3D 碰撞感知的 B 样条重规划把狗安全送过去。它**不是探索系统、不含 SLAM、不含赋色、不含宇树 SDK 桥**，输出只到 `/cmd_vel`。环境与我们的硬约束完全一致（Ubuntu 20.04 + Noetic + catkin，Go2 EDU + MID360 + Orin NX 实机验证过），已在本机编译通过并完成 FAST-LIO2 接入的仿真验证。**定位：L2 局部规划/执行层的候选**，探索决策（去哪）必须由上层另做。没有楼梯专用算法，爬楼靠宇树自身运动控制。

---

## 1. 比赛背景与硬约束（不变的前提）

**比赛**：RAICOM 2026 北斗时空应用赛四足 SLAM 组。任务 = 10 分钟内自主扫描完全未知的场地，产出**带颜色的 3D 点云**。评分 = 20% 用时 + 80% 质量（完整度 20 / 清晰度 10 / 细节 10 / 精度 10，手动/自主二选一 20，自主奖励 10）；碰撞 -5；摔倒或人为触碰 = 取消资格。原文见根目录 `北斗时空应用赛题.pdf`。

**硬件**：Unitree Go2（EDU）+ Livox MID360 + 前置相机 + NVIDIA Orin NX 16GB。

**硬约束**（用户多次强调，不可违反）：
1. 算法禁止从零自己写——用现成/已验证的实现，只做修改+组合+胶水（SCAN-Planner 符合这条）。
2. ROS1 Noetic + Ubuntu 20.04，不升 ROS2/22.04（JetPack 锁死、D435i 兼容、SDK2 DDS 版本等原因）。
3. Orin NX 16GB 内存上限；VLM 只能 Int4 低频跑。
4. 摔倒 = 取消资格 → 任何实机动作先想安全兜底。
5. 赛中禁止人为触碰 → 语言控制只用于赛前任务解析/答辩，不做赛中实时指挥。

**总体架构方向**（详见原机器上的 `/home/pumpkin-db/.claude/plans/quiet-floating-neumann.md`）：
自主全覆盖扫描为核心（弓形为主、前沿补漏），分层 = FAST-LIO2 SLAM → 稀疏体素地图+相机投影赋色 → 覆盖规划 → 局部规划+跟踪 → SDK2 执行 + 安全层。SCAN-Planner 对应「局部规划+跟踪」这一层。

**已明确排除的方向（前期深度调研后否决，别再重新提议）**：
- QUART / QUAR-VLA（西湖+浙大）：代码未开源、只支持 Go1，只可当步态参数空间的概念参考；
- VLM 做赛中实时执行：Orin 上 15-30s 延迟扫不完。NaVILA 8B（NVIDIA，开源 Apache-2.0）**只作辅助层**——赛前任务解析 + 答辩演示，不做赛中实时指挥（比赛禁止人为触碰机器狗）；
- PB-NBV 原版 C++ 代码：机械臂+转台场景，不适用（其投影评分的数学思想可借鉴）；
- Nav2 / OctoMap / ros1-bridge 路线：配置成本和失控风险高于自写 A*+Pure Pursuit + Python 稀疏体素地图（注意：局部规划层现已被 SCAN-Planner 这一更强候选替代）；
- ROS2 / Ubuntu 22.04：见硬约束 2。

---

## 2. SCAN-Planner 基本信息（全部已核实）

| 项 | 内容 | 证据 |
|---|---|---|
| 全称 | Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation | README 标题 |
| 论文 | arXiv:2606.19555 [cs.RO]，2026-06-17 提交，**无期刊/会议录用信息**（abs 页无 comments/journal-ref；摘要 "This letter presents..." 疑似 RA-L 体例，仅推断） | arXiv abs 页 |
| 作者 | Han Zheng, Zhe Chen, Yiwen Fu, Ming Yang, Tong Qin*（通讯），**全部上海交通大学**，上海市自然科学基金 24ZR1435600 | 论文 affiliation 逐字 |
| 仓库 | github.com/wuyi2121/SCAN-Planner，Apache-2.0，调查时 452★/44 fork；创建 2026-05-30，主算法 2026-07-09 首发，2026-07-13 社区 ROS2 分支（ros2-community，非官方），2026-07-29 传感器 CAD | GitHub API |
| 实机平台 | Go2 EDU（论文只写 "Unitree Go2"，但传感器 CAD 仓库 zhechen003/GO2-EDU-sensor_layout 首段逐字 "for the Unitree GO2 EDU"，owner=二作 Zhe Chen；MID360+RealSense 头戴）| README:32 + CAD 仓库 README |
| 机上配置 | 拆原装 LiDAR 换 MID360；改适配 FAST-LIO2 做状态估计；全部模块 Jetson Orin NX 实时；实机限速 1.0 m/s、加速度 0.25 m/s² | 论文 §VI-D 逐字 |
| 定位模块 | README 致谢称基于 Elevator-LIO（FAST-LIO2 多楼层扩展），与论文 "adapted FAST-LIO2" 说法并存；两者都不在本仓库内 | README:128、论文 |
| 技术血缘 | 框架基于 EGO-Planner（浙大 FAST-Lab）、地图思想来自 ROG-Map（港大）、仿真器改编自 MARSIM、地图生成 Mockamap、trot 动画来自 Leg-KILO | README 致谢 |
| 构建环境 | **Ubuntu 20.04 + ROS Noetic + catkin_make**（README 原文）；依赖 Eigen3、PCL 1.7、OpenCV、cv_bridge、tf、libarmadillo-dev（仿真需要）；GPU 渲染可选（GLEW/GLFW，`-DUSE_GPU=ON`）| README:60-117、CMakeLists |
| 许可证 | Apache-2.0 | LICENSE |

**注意**：`package.xml` 里出现的 zju.edu.cn 邮箱是上游 EGO-Planner 包的遗留 maintainer，**不是**本项目作者，别被误导。

### 2.1 效果（论文数据，诚实转述）

仿真（MARSIM，i9-14900K，每场景 50 次，密集障碍 40×20m、100~500 障碍）：

| 方法 | 成功率↑ | 碰撞率↓ | SPL↑ |
|---|---|---|---|
| ART-Planner | 0.44 | 0.56 | 0.31 |
| EGO-Planner-3D | 0.76 | 0.24 | 0.75 |
| CMU-Planner | 0.92 | 0.08 | 0.86 |
| EGO-Planner-2D | 0.96 | 0.04 | 0.88 |
| **SCAN-Planner** | **1.00** | **0.00** | **0.95** |

能力对比表 6 项（地形穿越/悬空障碍/贴地跟随/平滑/长距离/yaw 感知机身）唯一全勾。

实机：三层办公楼跨层巡检 45×15×15 m³，251 s / 149 m（穿楼梯）；校园末端配送 278×48×5 m³，589 s / 367 m（避行人车辆，全局路由来自商业导航地图）。

**如实的短板**：论文没有规划耗时/频率数字、没有消融、实机没有成功率；README 只有 4 个 GIF 零定量；仓库很年轻（16 次提交）。已知 issue：#5「爬楼梯时楼梯被当障碍物」→ 作者答复调大 `body_height`；#7「mode3 容易撞墙」；作者承认 mode3 指向全局路径终点是代码 bug（调查时点，可能已变）。

### 2.2 它是什么 / 不是什么

- **是**：反应式局部规划器。链路 = 3D 占据栅格（0.05m、10×10×5m 滑动窗口、log-odds 更新）→ 2.5D 投影 A* → B 样条 rebound 优化（L-BFGS，代价=平滑+碰撞+可行性+贴合）→ FSM 重规划 → 轨迹跟踪。
- **不是**：探索/覆盖系统（全 src/planner grep `frontier|exploration|coverage` 零命中）；不含 SLAM；不含点云赋色/保存（点云被强转 `pcl::PointXYZ`，RGB 丢弃，无 PCD 保存代码，地图只是局部滑窗不累积全局）；不含宇树 SDK（grep 零命中）。
- 类比：**好司机，不是领航员**。给目的地能安全开到、路上随机应变；「接下来去哪」一概不管。

---

## 3. 架构与接口（代码级，接线时查这张表）

### 3.1 节点拓扑（仿真闭环时）

```
mockamap/map_generator → /map_generator/global_cloud（全局地图）
    → pcl_render_node(local_sensing) → /pcl_render_node/cloud（渲染的传感器点云）+ /quad_0/lidar_pose
go2_kinematic_sim ⇄ /cmd_vel ⇄ closed_loop_controller
FSM: scan_planner_node（含 grid_map / A* / bspline_opt）
    发布 /planning/bspline（scan_planner/Bspline.msg）
closed_loop_controller：100Hz 跟踪 → /cmd_vel（geometry_msgs::Twist）+ /planning/go2_execution_frozen
go2_gait_publisher + robot_state_publisher：仅 URDF 可视化动画（不是控制）
```

### 3.2 话题表

| 用途 | 仿真话题 | 实机话题（run.launch is_real_world=true） |
|---|---|---|
| 机体内位姿 body_pose | /quad_0/body_pose | **/LIO/odom_vehicle** |
| 传感器位姿 sensor_pose | /quad_0/lidar_pose（或 camera_pose） | **/LIO/odom_imu** |
| 点云 cloud | /pcl_render_node/cloud（世界系） | **/LIO/clouds_lidar**（传感器系，cloud_is_world=false） |
| 深度图（depth 模式） | /pcl_render_node/depth | /camera/aligned_depth_to_color/image_raw（内参硬编码在 run.launch） |
| 轨迹 | /planning/bspline | 同 |
| 速度输出 | /cmd_vel | 同（**需要外部桥转给狗**） |
| 目标（mode1） | /move_base_simple/goal（RViz 2D Nav Goal） | 同 |
| 参考路径（mode3） | /initial_path（nav_msgs/Path） | 同 |

grid_map 内部：sensor_pose 是决定性输入（射线投射起点+滑窗中心+融合门控，`odomValid()` 只看它）；body_pose 在栅格内只用于可视化 TF。FSM 和闭环控制器各自再订阅一次 body_pose_topic（全局参数）。

### 3.3 关键参数（`plan_manage/launch/advanced_param.xml` 默认值）

```
max_vel 0.75 / max_acc 0.5 / max_jerk 4 / planning_horizon 3.5
栅格 resolution 0.05；sliding_map 10×10×5m；map_sliding_thresh 0.2
双圆柱碰撞体：radius 0.25、offset 0.18、z 膨胀 ±0.1；body_height 0.4
FSM：thresh_replan 1.0、thresh_no_replan 0.1、fail_safe true、max_replan_fail_count 1000
闭环：time_forward 0.8、heading_error_threshold 0.8、kp_pos 0.8、kp_yaw 1.5、max_vy 0.35、max_vyaw 1.0、finish_dist 0.15
```
README:101 说默认参数按 Go2 调过，但换安装/换场地仍需重调——**不要默认这套参数就是我们的最终参数**。

### 3.4 导航模式（navi_mode，run.launch）

1. RViz 手动点目标；
2. `tools/keypoint.yaml` 预录航点（`tools/keypoint_recorder.py` 录制，只含 x/y/z，无步态标注）；
3. 订阅 `/initial_path` 参考路径跟踪+局部避障（作者上层是 TravExplorer，未开源）。**这个模式是覆盖规划器的天然挂点**；注意 mode3 有已知 issue（见 2.1），用前先看 GitHub issue 现状。

### 3.5 「三维还是二维」的准确说法：3D 感知、3D 碰撞、2.5D 规划、2D 执行

- 地图/碰撞是真 3D（能看到楼梯、桌面、悬空障碍）；
- A* 只在 xy 扩展，z 沿起终点线性插值（`dyn_a_star.cpp` `interpolateZIndexOnSearchPlane`）；
- 优化器显式清零 z 梯度（`bspline_optimizer.cpp:1154/1182` `grad_3D.row(2).setZero()`）——垂直轮廓由航点/参考路径给定，优化器只做水平变形（论文原话 z-gradient suppression / ground-following surface）；
- 真机输出只有 2D Twist，z 从不被跟踪。
- 含义：它不会「自己决定爬上某个高台」，z 轮廓要由上层给。纯 2D 的探索算法（如 rrt_exploration）出的目标可以直接给它，但三维地形收益就浪费了。

### 3.6 楼梯问题（用户重点关注，结论已多路核实）

**没有楼梯检测、地形分类、步态切换的任何算法。** 爬楼机制 = 人工录制带 z 的航点（或给 3D 参考路径）+ 必要时调高 `body_height`（默认 0.4，issue #5 作者唯一答复）+ **宇树狗自身运动控制把楼爬上去**（论文原话轨迹 "executed by the quadruped locomotion module"；全文 gait/sport/SDK 零提及）。`go2_gait_publisher` 只是 RViz 关节动画节点，不是控制代码。作者真机用什么 sport mode 爬楼，所有公开渠道（论文/README/B站/issue）都没有记录——**留待实机自己定**。

### 3.7 仿真是什么仿真（MARSIM 风格，无动力学）

- 开源在 `src/simulator/`：mockamap（程序化地图）或 map_generator（PCD 地图）→ local_sensing 的 pcl_render_node（CPU 默认）/opengl_render_node（GPU）按位姿渲染传感器视图；
- **没有物理引擎**：closed_loop+go2_kinematic_sim 是把 cmd_vel 纯积分成假里程计（z 不动）；多楼层仿真用 open_loop_controller 把规划轨迹（含 z）原样回放成里程计；
- Go2 URDF 在仓库里（go2_description），RViz 里有 trot 关节动画，但**只是可视化，不是物理模型**；
- `go2_description/launch/gazebo.launch` 是 URDF 包自带样板，SCAN 管线没用它；**想要带动力学的仿真（腿-楼梯接触、摔倒）要自己接 Gazebo/Isaac，仓库不提供**。
- 仿真能验证：规划逻辑、话题接线、避障/重规划；不能验证：狗的运动能力、摔倒风险。

---

## 4. 已验证结论清单（2026-08-16，本机 WSL2 Ubuntu 20.04）

1. **编译通过**：`catkin_make` 全量成功（5 个可执行文件齐全）。坑：conda 环境的 python 缺 empy 会失败，用 `-DPYTHON_EXECUTABLE=/usr/bin/python3` 绕过。纯 Linux 上未必有此问题，以实测为准。
2. **默认仿真跑通**：`roslaunch scan_planner run.launch`，`/grid_map/occupancy` 稳定 20Hz（每帧约 1400-1500 占据点），点云 10Hz、pose 100Hz。
3. **FAST-LIO2 接线方案仿真验证通过**：
   - 方案：`/grid_map/body_pose` 与 `/grid_map/sensor_pose` 双 remap 到同一条 `/Odometry`；`/grid_map/cloud` remap 到 `/cloud_registered`；`cloud_is_world:=true`；
   - 代码依据：两个 pose 是独立 Subscriber 无冲突；cloud_is_world=true 时点云直接取世界坐标，pose 只影响射线起点，5Hz 低频 pose 压力测试也正常；
   - 结果：occupancy 20Hz 无漂移（与真值地图最近邻中位数 0.034-0.035m ≈ 0.7 体素），闭环行驶 8-9m 到达目标；
   - **坑**：`run.launch` 把 `cloud_is_world` 与 `is_real_world` 硬绑定（真机时强制 false），接 FAST-LIO2 必须绕过 run.launch 直接 include `advanced_param.xml`——已写好可用 launch：`src/planner/plan_manage/launch/fastlio_integration.launch`（本次新建，在仓库里）；
   - 保留声明：测试是用仿真真值模拟 FAST-LIO2 的话题语义，不是真 FAST-LIO2/真 bag，真机需复验。
4. **rrt_exploration 是纯 2D**（用户已决定大概率弃用）：地图类型写死 `nav_msgs::OccupancyGrid`，栅格索引/采样/目标全部只有 xy（`functions.cpp:88`、`global_rrt_detector.cpp:198-221`），无 3D 通道；参考工程靠 FAST-LIO2 点云投影成 `/projected_map` 喂它。它只有「前沿检测」这个决策思路可借鉴，执行链路整体 2D。

---

## 5. 待验证/实机相关的开放问题（不要写死，实机为准）

以下事项**本工作区无法确认**，接手后按实际环境决策，别照抄任何人的断言：

1. **cmd_vel → 狗的桥怎么做**：仓库只发 `geometry_msgs::Twist`。候选：(a) unitree_sdk2_python 的 sport_client.Move() 自写桥节点；(b) 现有社区桥（ROS1 生态里有哪些、哪个靠谱，需要调研）；(c) 作者的桥实现完全未知。取决于实机 SDK 版本与网络配置（DDS 参数等），实机说了算。
2. **实机 SLAM 选型**：作者用 Elevator-LIO（跨楼层）或 "adapted FAST-LIO2"（论文）。比赛场地若为单层，现有 FAST-LIO2 大概率够，但话题细节（`/Odometry` 频率、`/cloud_registered` 的 frame_id 与坐标系语义）要在实机用 rostopic 实测后再定接线，别直接套用仿真结论。
3. **传感器外参**：`grid_map.cpp:57-67` 硬编码了作者的 lidar/depth 外参。我们的 MID360/相机安装位置不同，需先实测标定，再参数化这 11 行（改动点已定位，约 15 行改法）；改法与数值都以标定结果为准。
4. **body_height / 双圆柱参数**：默认 0.4 是作者安装的值；我们的挂载（CAD 见 GO2-EDU-sensor_layout，可参考但不保证相同）需要按实测机身包络重调。
5. **爬楼/台阶的步态策略**：作者未公开。需要实机试验确定（Go2 SDK 内置步态/运动模式 + 限速），并与规划层的 z 轮廓、body_height 配合。这是安全红线相关，慢慢调。
6. **规划频率/时延**：论文未报告，Orin NX 上的实际耗时需实测。
7. **mode3 的成熟度**：issue #5/#7 与「指向全局路径终点的 bug」作者自认；用 mode3 前先查 GitHub 最新 issue/commit，或先用 mode1/2 打基础。
8. **WSL2 特有的坑**（仅影响前期仿真）：OpenGL 渲染节点（GPU 版）在 WSL2 的表现未知，默认 CPU 版应该没问题；性能与纯 Linux 有差异属正常。

---

## 6. 已知陷阱（我们核实过的，别重复踩）

1. **`go2_ros2_sdk`（abizovnuralem，约 1000★）是非官方、且是 2D 建图**：管线 MID360→`lidar_processor`→`/scan`(2D)→slam_toolbox(2D 栅格)→Nav2(2D)，不产出彩色 3D 点云；宇树官方名下并不存在 go2_ros2_sdk 仓库（`unitreerobotics/go2_ros2_sdk` 返回 404）。调研 cmd_vel 桥或「官方 ROS 接口」时别把它当官方 SDK。官方 SDK（unitree_sdk2）是纯通信层，没有 SLAM/探索/赋色代码。
2. **qwen 整理的《宇树Go2_EDU封闭场景自主探索方案.docx》（若在流传）是「骨架真、血肉假」**：explore_lite/nav2/slam_toolbox 这些名字是真的，但 `exploration_demo.cpp`、`perception/point_cloud_colorize/`、`save_colored_pcd` 服务、所有具体参数名（`frontier_travel_point`、`enable_rotation_recovery`、`climbing_height_threshold` 等）、「无需人工标定」「覆盖≥95%自动完成」全部是编造的，不要当事实依据。
3. **explore_lite 与 rrt_exploration 都是 2D 前沿探索，都不赋色**：explore_lite = hrnr/m-explore 的 BFS 栅格扫描；rrt_exploration（Hassan Umari 硕士论文，美沙迦美国大学）启动要人工点 5 个点（前 4 点定区域、第 5 点是 RRT 起点，`global_rrt_detector.cpp:132`）。纯 2D 探索已弃用（见 §4 第 4 条），它们的价值只剩「前沿检测」这个决策思路。
4. 引申纪律：这些坑全是「名字真、功能假」模式——引用任何第三方仓库/作者/能力前，先下载代码验证，别把搜索到的名字当事实。

---

## 7. 演进路线（用户既定策略：先把定点移动做好，再逐步叠合，不追求一步到位）

每个阶段独立验收、可停可弃；上一阶段不稳不进下一阶段。

**Phase 0 · 纯 Linux 环境复现**
- 内容：装依赖 → catkin_make → 默认仿真 run.launch 跑通 → （可选 WSL2 先跑一遍对照）。
- 验收：occupancy 20Hz、RViz 里能点目标走完。

**Phase 1 · 实机定点移动（mode 1）**
- 内容：SLAM（FAST-LIO2）→ 按第 4 节验证过的接线喂 SCAN-Planner → cmd_vel 桥（第 5.1 节候选方案择一实测）→ RViz 点哪走哪。
- 安全：先人持遥控器/急停跟随，限速（建议比默认 0.75 再低），IMU/姿态看门狗可先做最简版（姿态超限发 stop）。
- 验收：多次定点移动零碰撞、无摔倒；记录实际规划频率与延迟。
- 这一阶段本身不产出比赛能力，但它是后面一切的底座，**值得花时间调稳**。

**Phase 2 · 预录航点巡航（mode 2）**
- 内容：keypoint_recorder 在已知场地录航点 → 回放执行；试验台阶/门槛场景下 body_height 与 z 轮廓的效果。
- 验收：固定场地可重复跑完；把「哪段路要什么参数」记成笔记（实机调参经验是后面自主探索的种子）。

**Phase 3 · 覆盖规划层接入（mode 3 /initial_path）**
- 内容：上层覆盖策略（弓形/墙跟随/前沿，来源 PythonRobotics 等现成实现）产出全局路径 → /initial_path → SCAN-Planner 跟踪+避障。先在仿真/已知场地调，再上未知场地。
- 注意：前沿检测必须是 3D 体素版（纯 2D 已弃用，见 §4 第 4 条与 §6 第 3 条）；覆盖层与规划层的频率、路径点间距等接口参数需要联调，不要指望一次对。
- 验收：指定区域覆盖率可量化（对照我们自己的体素地图统计）。

**Phase 4 · 比赛闭环**
- 内容：赋色（相机投影）+ 全局彩色地图累积与导出 + 10 分钟流程编排 + 安全层硬化 + NaVILA 赛前任务解析（辅助）。
- 这部分与 SCAN-Planner 基本无关，按总体计划（plan 文件）推进。

---

## 8. 文件索引

- 仓库：`new_algorithm/SCAN-Planner/`（main 分支；`build/`、`devel/` 是本机 WSL2 编译产物，纯 Linux 上需重新编译）
- 核心代码：`src/planner/plan_manage/`（FSM、planner_manager、两个控制器、launch/参数）、`src/planner/plan_env/src/grid_map.cpp`（地图+外参硬编码 57-67 行）、`src/planner/bspline_opt/src/bspline_optimizer.cpp`（1154/1182 行 z 梯度清零）、`src/planner/path_searching/src/dyn_a_star.cpp`
- 接线测试 launch：`src/planner/plan_manage/launch/fastlio_integration.launch`（本工作区新建）
- 仿真：`src/simulator/`；工具：`tools/keypoint_recorder.py` + `tools/README.md`
- 本工作区相关文档：根目录 `CLAUDE.md`（项目总纪律）、`算法调研报告.md`、`try_algorithm/notes/`（历次决策笔记）、plan 文件 `/home/pumpkin-db/.claude/plans/quiet-floating-neumann.md`
- 2D 参考工程（大概率弃用，仅借鉴前沿检测思路）：`Go2_frontier_based_exploration/`

## 9. 工作纪律（给接手的 AI）

1. **区分「已核实的事实」与「推断」**：引用仓库/论文/能力时给证据（文件:行号或原文引文）；未下载验证前不要断言路径/功能存在。本项目历史上被「名字真、功能假」的二手材料坑过多次（qwen 的 docx 方案大半是编造的，见 CLAUDE.md「已知陷阱」）。
2. **不要把东西写死**：实机环境未知，本文档第 5 节列的开放问题以实机实测为准；参数、选型、桥法都是候选，不是命令。
3. 算法不许从零写：改+组合+胶水可以，发明新算法不行。
4. 摔倒=取消资格：任何实机改动先想安全兜底。
5. 文档与沟通用中文（技术名词保留英文）。
