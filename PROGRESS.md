# go2-scan 进度记录

> 实时进度流水账（写给未来的 AI / 自己接续用）。按日期倒序。
> 指令、规则、硬约束见 `CLAUDE.md`（那是规则层，不是进度层）。
> 第三方来源/commit/编译见 `third_party.md`。

## 2026-08-29：主线交接与 P0 资料审计

### P2 Landing Next-Flight Reacquisition（Phase A PASS）

- 保存并离线重放 landing 实云（`/mid360_points + body pose + tracks/state`）；新增 detector reject
  diagnostics 与 `detector_replay` PCD 工具，不改变 P1 检测参数。
- 根因确认：真实第二 flight（entry≈(14.2,1.5~1.8,3.4)、heading≈+Y、rise≈1.0~1.4m）在
  landing 前已被多帧观测；旧 traverser 却按最近距离选中陈旧、与第一 flight 同向的 −Y track。
- `LANDING_SCAN` 改为专用 switchback reacquire：只接受与上一 flight 折返≥120°、连接当前 landing、
  观测新鲜且≥2帧/高置信的 track；NORMAL_SEARCH 仍要求 tracker `CONFIRMED`（3帧），行为不变。
- 两次独立 Depot 回归均稳定选中 +Y 第二 flight（4帧与2帧），旧 −Y track 未再误选，均运行到当前
  `COMPLETE`。Phase A PASS；P2 仍为 PARTIAL，因为下一步需加入独立 `EXIT_VERIFY` 多条件上层确认，
  不能仅以 flight 数量 + landing timer 宣布完成。

### P2 Standalone Stair Episode（Navigation-Level PASS）

- 新增独立 `EXIT_VERIFY`：第二 flight 后同时验证 perceived-track 累计高度增益、末段几何进度、
  1s 的 z/xy 停稳窗口和持续定位；8s 内证据不足则 fail-closed，不再按 flight 数+定时器假完成。
- active episode 增加 0.5s localization timeout；丢失 pose 立即 FAILED/零速。速度所有权仍只有
  `stair_traverser → motion_arbiter → /cmd_vel`，ModelPlugin 仍为唯一 pose writer。
- 两次独立 Depot 全流程回归均通过：`first flight → LANDING_SCAN → +Y second flight →
  LANDING_SCAN → EXIT_VERIFY → COMPLETE`，无 FAIL；第二次 EXIT_VERIFY 70.55~72.55s，最终
  pose≈(14.43,2.78,4.22)。裁决：P2 Navigation-Level PASS；不代表 Real Go2 Stair Locomotion PASS。
- 下一步进入 P3：用 SCAN 抵达 perceived stair staging pose，停止后 reverify，再经 arbiter 交权给
  已验证的完整 stair episode；不修改 SCAN planner core。

### ARiADNE stable-baseline guard

- Escape recovery / blocked-node 代码保留供实验，但稳定 e438-derived baseline 默认关闭：
  `rl_planner.py` fallback 与 `go2_ariadne.launch` 均为 `enable_escape_recovery=false`。
- `utility_range_factor=0.5`；`policy_blocked_nodes` 初始为空，且仅在 escape 显式启用后写入。
  Depot/indoor 无额外覆盖；P2 stair-only 不启动 ARiADNE。

- 完整阅读 `CODEX-MASTER-HANDOFF.md`、本文件、`ZCODE_RUNBOOK.md`、真机交接资料及附件；
  当前开发分支切换为 `refactor/multifloor-realrobot-v2`（基于稳定 tip `7e94cad`）。
- 真机契约确认：定位 `/Odometry`、世界系点云 `/cloud_registered`、最终运动后端复用已验证的
  `cmd_vel_bridge → SportClient::Move/StopMove`；核心算法不得依赖 Gazebo topic/frame/GT。
- KAIST 资料与冻结架构无结构性冲突：论文/解读支持 LiDAR+VoxelGrid+RANSAC 坡面检测、楼梯延后通过、
  单一状态管理器和墙面引导；其“pitch≈0 即到新楼层”过弱，项目继续采用 landing scan + 多证据退出判定。
- 现有 V2 审计确认 SCAN mode3 `/initial_path` 保留动态 XYZ，适合作为普通地面/楼梯接近接口；真正
  stair traversal 第一版仍由独立 traverser 接管，不把楼梯逻辑塞入 SCAN 或 ModelPlugin。
- 本轮未改功能代码、未运行仿真。下一步：完成当前 topic/TF/adaptor 的最小运行时审计后，进入 P1
  `stair_detector + stair_tracker + RViz debug`，输入只用规范化点云与位姿，禁止 GT。

### P0 运行时接口复核

- Depot headless 短跑确认：`/mid360_points`=`sensor_msgs/PointCloud2`、`frame_id=world`，唯一发布者
  `/gazebo`；`/quad_0/body_pose`=`nav_msgs/Odometry`、`frame_id=world`、`child_frame_id=base`，唯一发布者
  `/gazebo`。因此 P1 核心可统一接收“世界系 PointCloud2 + Odometry”；真机仅把输入换成
  `/cloud_registered + /Odometry`，无需依赖 Gazebo TF/消息。
- 现有 `integration/go2_bridge/scripts/stair_detector.py` 依赖 2.5D elevation map、JSON 输出、无多帧
  tracker，且可混入 Depot registry GT；裁决为 simulation GT backend，不能继续充当正式 Stair Perception。
- 运行时发现两个待隔离风险：Depot `env.sh` 的附加参数会覆盖命令行 `stair_detect:=false`，强制启动旧
  registry detector；TF 出现大量 `TF_REPEATED_DATA`，短测中 `tf_echo world base` 未稳定建立。
  P1 不依赖该 TF，但正式 launch 接入前需修清所有权/开关。
- 下一步唯一动作：建立独立、无 GT 的 `stair_perception` 包，先交付 PointCloud+Odometry candidate detector、
  多帧 tracker、标准消息与 RViz evidence；用合成点云单测后再接 Depot。

### P1 Stair Perception tracer bullet（PARTIAL）

- 新建独立 workspace `algorithms/perception/stair_perception`，核心只接收同一重力对齐世界帧下的
  PointCloud2 + Odometry；默认仿真 `/mid360_points + /quad_0/body_pose`，真机可直接切换
  `/cloud_registered + /Odometry`，无 Gazebo/Depot/elevation map/控制依赖。
- detector 已实现：局部 ROI、VoxelGrid、扇区 corridor、PCL RANSAC 平面、15°~45°坡度、宽度/长度/
  rise/支持点与置信度门限、候选去重；发布标准 `StairObservationArray`、support cloud 与 RViz arrow。
- tracker 已实现：entry/heading/slope 关联、几何滑动融合、三帧确认、超时清理；发布
  `StairTrackArray` 与确认状态 RViz marker。旧 JSON/registry 脚本未修改、未接入新链。
- 验证：全包 `catkin_make` 通过；`roslaunch --nodes` 正确解析 detector+tracker；合成测试
  `RejectsFlatGround` 与 `DetectsSteppedFlight` 均通过（0 failures）。
- 当前限制：尚未接主 launch、尚未用 Depot 实际点云验证误报/漏报与 track 稳定性，故 P1 仍为 PARTIAL。
  下一步只做新旧 detector 隔离接线，并在 Depot 楼梯附近采集/验证 observation 与 RViz evidence。

### P1 Depot 接线与实云验证（PASS）

- 主 launch 的 `stair_detect:=true` 已切到无 GT `stair_perception` detector+tracker；旧
  registry/elevation 脚本改为显式 `stair_gt_backend:=true`，默认关闭。RViz 已接 candidate、track、support。
- Depot 主楼梯前（12.87, 4.5）实云稳定检出唯一主候选：entry≈(12.78,2.97,0.69)、
  heading≈(−0.10,−1.00)、rise≈2.79m、confidence≈0.95；21帧后 track confirmed。
- 首跑发现一个入口高于当前层约1.95m的稳定高层结构误检。根因：坡度/尺寸门限未验证候选连接当前
  支撑层。新增“两端至少一端接近当前地面层”门限；回归测试先失败后通过，实云候选由2降为1。
- Depot 出生点负样本连续帧0候选；全包编译、3项 detector gtest、launch解析均通过。
- P1 验收补齐：水平地面/垂直墙/悬空斜面拒绝测试均通过；RViz 已显示 supporting points、
  fitted plane、entry、heading arrow、confirmed ID。实云 entry/heading/slope 多帧稳定，完全未使用 GT。
- 二层下降侧实验确认物理可见性限制：MID360 向下视场仅约7°，主楼梯 ROI 沿真实下降坡面的回波为0；
  点云只能看到护栏/高层结构。尝试扩大为双向 RANSAC 只产生假候选，已完整撤回。
- 架构结论：P1 按既定验收完成；下楼不能依赖“到二层后重新直检坡面”，后续 manager 必须持久保存
  上楼时确认的 stair landmark/transition，供返程复用。下一步进入 P2 Standalone Stair Episode。

### P2 Standalone Stair Episode tracer bullet（PARTIAL）

- 新增 `stair_navigation`：bounded traverser（ACQUIRE/ALIGN/ASCEND/LANDING_SCAN/COMPLETE/FAIL）、
  perception centerline follower、fail-closed motion arbiter、仅仿真的 terrain z adapter；全部输入来自
  `StairTrack + body pose`，无 Depot registry/坐标/GT。`scan_enabled:=false` 可确保 P2 不启动 SCAN。
- ModelPlugin 新增 `/sim/body_z_target` 输入及0.5m/s z slew，仍为唯一 pose writer；无新 target 时保持高度。
  motion arbiter 对 nav/stair source 独占选择、0.3s timeout，失联输出零速。
- 构建/测试：perception workspace 2包编译通过，共11项测试0失败；SCAN workspace ModelPlugin 编译通过；
  P2 launch 节点表确认无 `scan_planner_node` 和 closed-loop controller。
- Depot 实云短测：自动完成 `IDLE→ACQUIRE→ALIGN→ASCEND→LANDING_SCAN`，body z 从0.35升至2.97m，
  第一 flight navigation-level PASS；landing 后无 confirmed next track，12s bounded timeout 后进入 FAILED 并停车。
- 当前阻塞仅为 landing 的下一 flight 感知/重捕获，非 arbiter、运动或 z adapter。P2 尚未完成；下一步重放
  landing 点云，确认是 current-floor reference 被前一段楼梯污染，还是下一 flight 实际不可见，再做单因修复。

## 2026-08-28（晚）：V2 重构启动——Real-Robot-First 审计（ZCode）

- **P0 完成**：确认 `origin/debug/ariadne-baseline-20260828`（tip=7e94cad，含 37325bf e438 语义恢复）；v1 锚点 5797b4e 不在 baseline 内。本分支 `refactor/multifloor-realrobot-v2` 自 baseline 新建，零 v1 代码（v1 冻结在 feature 分支）。
- **SCAN 审计**（与上游 348e8a5 blob 级 diff）：我们的 vendored 改动=纯诊断+仿真 plant，规划核心零偏离。mode2=keypoint.yaml init 固定序列（官方机制，无运行时接口）；mode3=/initial_path 运行时动态 3D 多点路线（官方 TravExplorer 用法）；楼梯无专用逻辑=抬高参考路径 z。**裁决：mode3 为 V2 正式接口**，Test A/B 待跑。
- **Unitree 审计**（sdk2 main clone）：SportClient 无楼梯/步态切换公开接口（Move/StopMove/SpeedLevel 即全部；gait_type 仅在底层 IDL 未文档化）；cmd_vel→Move 映射天然匹配。
- **v1 处置**：ModelPlugin 楼梯锁/zprof/位置钳制 + stair_mission_manager.py 全删；STOP_REASON/floor_reset 思想选择性移植；新增 StairTransition.msg / multifloor_manager / transition_planner / motion_safety_filter / floor_session 抽象。
- 详细报告：`docs/V2_ARCHITECTURE_AUDIT.md`。等 GPT 对架构裁决后开写 V2。

## 2026-08-28：fatal 未复现 + 逐 replan 监测落地（ZCode 执行）

- **结论：073402 报告的 fatal（436s 起点判障碍急停）在干净重建二进制上未复现**。
  监测跑 1201s（mission），ER 29.6% / 轨迹 92.6m，判废原因=平台期 166s（新故障形态，非 drone-in-obstacle）。
  `obst=0`、78 次 a-star error 全部自愈、escape recovery 6 次均成功、SCAN 全程存活（PID 零变化）。
- **逐 replan 监测（SCAN_RDIAG，161 次）**：start_pt 与前 3 控制点在 replan 入口处始终 in-map 且 free
  → “起点滑出 10×10 滑窗 / 被判占据”（分类 C/D）在 replan 入口未发生；a-star 失败源于**轨迹中段**
  碰撞段（当前监测盲区，[SCAN_DIAG-OPT] 埋点未触发）。滑窗越界假设未被证实。
- **e438e07 静态对比**：GridMap 全部参数与滑窗/out-of-map 语义完全一致 → 排除地图层回归。
- **诊断埋点（未提交，工作区改动）**：FSM `dumpDiagSnapshot`（EMERGENCY_STOP/失败计数触发，
  发布 /scan_diag/snapshot）+ optimizer 两失败点控制点占据倾倒（[SCAN_DIAG-OPT]）+
  grid_map 时戳 getter + `tools/diag_snapshot_node.py`（rosout 直达 + JSON/PCD 落盘 /tmp/go2_diag）。
- **事故复盘**：① UNC 编辑+增量编译 mtime 陷阱 → bspline_optimizer.cpp 静默未重编 + Eigen 二义性
  编译错误从未暴露 → 新旧 .o 混链 → exit -11 循环；全量重编+修复后消除。
  ② logReplanDiag 对未初始化轨迹调 evaluateDeBoorT 段错误，已加 has_traj 守卫。
  ③ WSL 互通守则沉淀至 ZCode 工作区 `ZCODE_RUNBOOK.md`（不入库）。
- **新挂账**：respawn=true + bridge 去重门（repub_dist=1.0）→ 节点重启后 FSM 可能永久等不到重发
  航点（解释 093831 基线跑轨迹 0.0m）；roslaunch stdout 块缓冲会吞错误行，观测必须走 rosout/PID。
- 待办：fatal 复现性实验（同场景多跑 + 确认 073402 当时实际运行的二进制）；中段碰撞段监测扩展。
- **系统全盘对比（e438e07 vs 当前）→ ER 98.6%→29.6% 根因聚焦**：
  - 完全一致（排除）：checkpoint.pth、parameter.py、planner_manager、dyn_a_star、closed_loop_controller、
    ariadne_goal_bridge、simulator.xml、indoor_1.world、GRID_MAP 全参数。
  - **ER 曲线分段斜率（决定性）**：e438 前 300s 冲到 87.9%（+24.6/+33.7/+24.3 每百秒）；
    当前跑 0-100s 仅 +13.1，**200s 起 ER 冻结在 29.6% 直到 1200s**；073402 同样 100s 起冻在 15%。
  - **航点节奏（决定性）**：t=0-200s 发 64 个航点（间隔中位 2.0s、指令跳距合计 213.8m，实际仅走 ~65m
    → 策略目标振荡）；**t=200-400s 仅 5 个航点，400s 后归零 → ARiADNE 停止发新目标是 ER 冻结直接原因**
    （SCAN 侧 161 replans 全健康、运动链正常）。
  - escape 路径合计 64.9m ≈ 全程轨迹 92.6m 的 70%（4-6 次集中在前 200s）。
  - 当前独有差异清单：① escape recovery + policy_blocked_nodes(≤32) + stalled_complete(20s) 三连机制；
    ② utility_range_factor 0.5→1.0；③ 运动层 kinematic_sim+gazebo_bridge → ModelPlugin
    （cmdTimeout 0.3/maxVx 0.8/maxVy 0.5/maxVyaw 1.0）；④ livox→velodyne ray 传感器（360°×59.7°,10Hz,40m）
    + cloud_range_filter 接线调整；⑤ respawn=true。
  - **嫌疑排序**：①escape 三连机制（70% 轨迹耗在 escape + blocked 扭曲策略 + 航点枯竭）>
    ②utility_range 1.0（策略效用分布偏移，0.5 下 98.6%）> ③运动层架构（0-200s 斜率减半的部分原因）>
    ④雷达换装（二阶，经地图/前沿间接）> ⑤respawn（无关，仅次生冻结风险）。
  - **最小 A-B 顺序**：A-B-1 escape:=false；A-B-2 utility_range_factor:=0.5；A-B-3 回退 e438 运动层（成本最高）。
- **AB-0 定案（当前架构 + e438 决策语义）→ ER 恢复历史水平 ✅**（报告 114647）：
  - 配置：utility_range_factor=0.5 + enable_escape_recovery=false + stalled_complete_seconds=0.0
    （仅改 go2_ariadne.launch），ModelPlugin/Velodyne/SCAN/地图链全部保持当前。
  - **ER_final 95.1% @ 961s，degraded=False**；轨迹 264.5m（e438: 251.9m）；T95=486s（e438: 460s）；
    ER 曲线全程健康（+20/+16.5/+18.8/+28.4 每百秒），~475s 效用归零自然完成；
    obst=0、astar=64 全自愈、escape=0；STOP_REASON 全程仅 6 次 oscillation_break（AVOID_OSCILLATION 兜住，未成环）。
  - **裁决：ER 98.6→29.6 的回归来源 = ARiADNE 新增决策机制组合（escape/blocked/stalled/utility 1.0），
    当前架构（ModelPlugin/Velodyne/SCAN/地图链）无罪。** ARiADNE 调试结束，保留历史决策行为。
  - 遗留注意：Depot 曾因 utility 0.5 停摆改 1.0（D2/D3 复盘）——本次回 0.5 后 Depot 需复测，
    楼梯/Depot 场景按需再调该参数；escape/blocked/stalled 代码保留但默认关，若他场景复现 A-B-A-B 再议。
- **projected_map"墙外 unknown"排查（2026-08-28，短测试，未改代码）**：
  - projected_map 窗口随观测生长（183s: 149×170 覆盖 x≤21.4；422s: 268×170 覆盖全建筑 x≤45.2），
    octomap_server 无 bbox 限制；"建筑外 unknown"=窗口内未观测格（27208），属 OccupancyGrid 正常语义。
  - **frontier 出界=0/75**、**utility>0 节点出界=0/4**（GT 室内掩膜判据；
    连通域判据下 2 个"不连通"点经 GT 判据确认是未打通邻室）。
  - 墙投影基本闭合：仅 (9.5,19.2) 附近 2 格孤立 free + 3 格被夹墙格的微小泄漏，
    未演化出任何出界目标，无需处理。
  - **结论：不需要 exploration_boundary mask。** 抓取/分析脚本在 /tmp/cap_*.py（未入库）。

## 2026-08-27：CHAMP 物理接入后回归问题（待处理）

- 好转：腿不再乱飞，12 个关节角已能保持有界，模型、CHAMP、ros_control 均能启动。
- 新回归：初始站立时腿自动动、机身晃动；实跐时狗不能正常移动。
- 新回归：ARiADNE 蓝色砖块生成错位，高可能是物理姿态/高度与 LiDAR、body pose 和 world/map 契约不一致。

### 快速可用方案（已实施）

已恢复「身体运动学积分 + 腿视觉步态」；身体 z 可跟随楼梯 GT 高程上升，并由单一 body pose 同时计算 LiDAR pose。保留物理组件仅作显示，不参与运动。

## 2026-08-27：恢复运动学后新回归（待处理）

- 用户实跑：scan_map 开始漂移，ARiADNE 蓝色砖块同步漂移；腿不乱飞但姿态扭曲，且没有走路动态。
- 高优先疑点：源码恢复后未重新 catkin_make，devel/lib 可能仍在运行旧的物理版二进制；这可同时解释“静态腿”和异常姿态。
- 次要疑点：点云 world/sensor frame 与 scan_cloud_accumulator 假设不一致，或 body_pose/lidar_pose 时间不同步。重编译后应先静止检查两者，再检查点云 frame_id 与位姿外参。

## 2026-08-26：Depot 三轮对照定稿（D2=49.2% 基准确立）→ 任务#3 收官 ✅

| 跑 | 效用环 | ER_final | 判废 | 轨迹 | 备注 |
|---|---|---|---|---|---|
| D1 | 3m | 43.5% | （旧逻辑误判完成） | ~8m 冻结 | 假完成数据点 |
| **D2** | 3m+停滞门 | **49.2%** | ✅ 正常 | 67.4m | **基准跑**（P@0.2=98.9%/Chamfer 0.103m） |
| D3 | 6m+停滞门 | 48.0% | ✅ 正常 | 79.2m | 扩环无增益 → factor 维持官方 0.5 |

**定稿结论**：
1. ARiADNE @ Depot 稳定基线 ER≈48-49%（双跑复现），为地面层理论天花板（56.8%）的 ~85%。
2. 地图质量优秀且稳定（P@0.2≈97-99%、R@0.2≈82-83%、Chamfer≈0.103m）。
3. 抬升 ER 的唯一路径 = 楼梯上夹层（地面+夹层上限 76.2%）——任务#4 的战略价值定量坐实。
4. utility_range_factor 维持官方值 0.5（少一个偏离上游的补丁）；停滞式完成判定保留。

## 2026-08-25（深夜）：Phase 3 楼梯检测器注册表模式实测打通 ✅

- `/stairs_detected` 实测发布 main_east：entry[12.87,0.40,0.44]→exit[12.87,3.20,2.76]、
  yaw=90°、宽 1.2m、总升 2.32m，与 scene.yaml 注册表逐字段一致。
- 踩坑记录：rospy 脚本私有参数命令行是 `_registry:=`（带下划线），`~registry:=` 无效且无报错；
  nohup 下 python 日志缓冲不可见（rosout 里反而有）。
- **遗留谜团**：elevation_mapping 进程存活、订阅正常、首次回调成功（"mapping started"）、
  参数核对无误，但 fused/raw 高程图持续零发布（16 字节布局版）。与 CMU 节点段错误同族待查，
  已并入任务 #8 的 gdb 排查。几何法楼梯检测的实景验证被此阻塞；
  transit 联调可先用注册表模式推进。

## 2026-08-25（晚）：TARE 对照首跑即崩（CMU 节点段错误）→ 记录阻塞转 Phase 3

- **TARE @ Depot 首次端到端运行失败**：`terrainAnalysisExt` 与 `tare_planner_node` 均 exit -11
  （段错误），required=true 拖垮整个仿真。16 字节布局修复后**仍崩** → 排除 PC2 格式因素。
- **集成债曝光**：此前 TARE 分支 launch 从未启动 terrain_analysis(+ext)——/terrain_map 无发布者，
  tare_planner 收不到地形层永不规划（狗站桩的 TARE 特有机制，与 ARiADNE 的假完成不同）。
  已补 include 接线（CMU 默认输入恰为 /state_estimation+/registered_scan，tare_bridge 已产出）。
  注意 terrain_analysis_ext.launch 与主 launch 有 navigation_boundary_publisher 重名冲突，勿重复。
- **结论**：TARE 对照阻塞（任务#8），ARiADNE 为当前唯一可用全局层。velodyne 插件已回退到
  与旧 livox 完全一致的 16 字节 x/y/z/intensity 布局（治本尝试，虽未解决此崩但消除了格式风险）。
- elevation_mapping 的 -11 与 CMU 节点同病相怜，一并 gdb 排查（任务#8 扩展）。

## 2026-08-25（傍晚）：Depot 基线确立 D2=ER49.2%（地面层天花板的 87%）+ 理论上限标尺

### Depot 跑分（velodyne 换装 + spawn 托管后）

| 跑 | ER_final | 说明 |
|---|---|---|
| D1 124318 | 43.5% | 「效用全零即判完成」假完成，~8m 后冻结；JSON 序列化 bug 致 md 缺失（已修 bool()） |
| **D2 133232** | **49.2%** ✅ degraded=False | 停滞式判定版：轨迹 67.4m；P@0.2=98.9%/R@0.2=81.5%/Chamfer 0.103m |

### 关键认知（评估体系级）

1. **Depot ER 物理天花板标定**（同款可见性模型离线撒点）：仅地面层 ≈56.8%，地面+夹层 ≈76.2%
   （GT 里 57% 是 z>4.5m 屋顶高层，6m 量程永远不可及）。**单层跑分 ~50% 已接近物理极限，
   想再抬必须靠楼梯上夹层（任务#4 的战略价值被定量证实）**。
2. **假完成第一杠杆已落地并验证**：rl_planner 完成判定改为「效用全零 AND 地图静止 ≥20s」
   （~stalled_complete_seconds 可调）；D1→D2 ER +5.7pt、轨迹 8m→67m。
3. **剩余停滞机制已定位未修**：效用环=0.5×量程=3m 内无前沿节点就不发航点（楼梯口远端前沿
   够不着）。D3 杠杆已备：utility_range_factor 0.5→1.0（launch 已改，待跑）。
   若 D3 仍停摆 → 上「全局最近前沿恢复航点」胶水（run10 丢失的思路重写）。

### 正在进行

- TARE @ Depot 对照基线（Phase 2）：后台运行中，日志 /tmp/bench_depot_tare.log。

## 2026-08-25（下午）：Depot 全盲根因终局 = livox 自制射线插件；换装 velodyne 后全链路打通 ✅

### 终局结论

**雷达全盲（点云坍缩为传感器原点单点）的根因 = livox_laser_simulation 的自制 ODE 射线路径**
在多模型世界里完全失效。排除法穷尽了：spawn 竞态✗、model://Depot 解析✗、submesh 命名✗
（libsuffix 同样失败）、整网格碰撞✗、STL 米制网格✗、物理配置✗（默认与 0.002/500Hz 同败）、
RTF/预热✗（实测 0.84）、ODE 递归修复✗（无条件递归后依旧全盲）。帧指纹证明插件在复读死缓冲
（stamp 前进、数据 md5 不变）。**换装标准 RaySensor + velodyne 插件后一次点亮**：
9801/9847 去重点、中位距离 8.36m、环带命中 953、projected_map 出图、way_point 恢复、狗开始探索。

### 落地的修改

1. **URDF 换装**：`go2_description.urdf` 用标准 `<ray>` 传感器块 + `libgazebo_ros_velodyne_laser.so`
   （CMU vendor），视场角对齐 MID360（360°×-7°~52°，200×50=10000 点 @10Hz，量程 40m）。
   插件增补 `<worldFrame>true</worldFrame>` 参数：点云直接转世界系输出，下游 cloud_is_world
   管线零改动。旧 livox 块注释保留在 URDF 尾部备查。注意 point_step 16→22（多了 ring/time 字段），
   动态字段解析的节点兼容。
2. **velodyne 插件增补**（cmu_env）：`GazeboRosVelodyneLaser` 加 world_frame_/sensor_world_rot_/pos_
   ——照搬 livox 魔改版的「父实体位姿 × 传感器位姿」世界系变换，参数化开关。
3. **livox 插件递归修复保留但未采用**：UpdateCallback 改无条件递归子空间（理论上更正确，
   但对本故障不致命）；ps3joy 加 CATKIN_IGNORE（缺 libusb）；cmu_env 构建树推倒重建过一次。
4. **spawn 托管 + 单实例守卫**：见上一节。

### 工具沉淀（scenes/gtools/）

- `probe_cloud_diversity.py`：点云取证探针（帧数/3D去重数/环带命中一次输出）
- `reexport_mesh_meters.py` / `make_stl_collision_variant.py` / `make_wholemesh_collision.py`：
  网格单位与碰撞变体工具链（本次未成为解法，留作资产）
- `test_depot_raycast.sh`：分变体自动实测框架（含清场复核）

### 教训（写给自己）

1. **自制物理交互代码是最大风险源**——livox 插件绕开 gazebo SensorManager 自己玩 ODE space，
   平时能跑、换个世界就全盲，且失败模式静默（有帧、有点、就是全 miss）。标准组件优先。
2. **实验纪律**：每次起仿真前 kill_all + pgrep 复核；不用 timeout 包 launch 脚本；
   探针先看「多样性指标」（3D 去重数）而不是单一距离统计。
3. **对照实验要控制变量**：下午曾因实例污染误判「indoor_1 也瞎了」，浪费两小时。

## 2026-08-25（凌晨）：用户终版定参 + 五方向开工

### 三重根因（按发现顺序）

1. **spawn CLI 竞态**：旧 run_benchmark.sh 用 `rosrun spawn_model`+60s timeout，首 spawn 超时但请求入队，重试叠出多狗；"entity already exists" 被当成功。→ 已换 `simulation/spawn_go2.py` 服务化托管 + 点云健康门。
2. **`model://Depot` 解析失败 → 世界无几何**：模型本体在 `scenes/depot/model/assets/`，但 include 要找 `model/Depot/model.sdf`。→ 已修：符号链接 `model/Depot -> assets`。
3. **Depot submesh 碰撞疑似令 ODE 射线空间失效**：单实例取证（无污染窗口）显示模型加载成功但雷达连测试盒都打不到，全部点=传感器世界位姿（livox 插件魔改版把 miss 点收缩到传感器原点）。待验证修复：`scenes/gtools/make_wholemesh_collision.py` 把 collision 内 submesh 换成整网格。

### 进程卫生纪律（血泪教训，以后每次启动前必须执行）

- bench_depot3/4 用旧版脚本 FATAL 退出时**没有清理仿真** → 孤儿 gzserver 泄漏 → 后续所有 roslaunch 复用 master、孤儿重连发陈旧 /clock → **今天下午「indoor_1 也瞎了」是误判**（探针听到的是僵尸世界的帧，插件加载时 sim time 已 1119s 是铁证）。
- **第二个泄漏源（更隐蔽）**：`timeout N bash launch_gazebo_sim.sh &` 这种实验写法，timeout 只杀外层 bash，gzserver 孤儿化继续跑。下午用 timeout 包实验反复触发。**已修：launch_gazebo_sim.sh 开头加单实例守卫**——检测到已有 gzserver 立即 exit 42 拒启；做实验一律不用 timeout 包 launch 脚本。
- `kill_all_sim.sh` 头注释早就记载了此故障模式（2026-08-24），但我一直 `>/dev/null` 吞掉它的输出、不复核。**正确姿势：跑完 kill_all 必须显式 pgrep 复核为空再启动新栈。**
- livox 插件（魔改世界系输出版）的全 miss 病征 = 点云所有点精确重合在传感器世界位姿上（3D 去重后 1 个点）。取证探针：`scenes/gtools/probe_cloud_diversity.py`（帧数/去重数/环带命中一次输出）。

### 其他进展

- UFEP 定位澄清（用户指示）：配 UFEP 是为**参考思想**，复杂度超收益就不采用。pplanner_simulator 是作者自家仿真器（不用），重心改为 vrmapping 冒烟+思想评估（GSOM+LSOM 双层采样拓扑图，见 Sampler.h）。
- 论文笔记产出：`try_algorithm/notes/HEADER与HDPlanner精读.md`（ARiADNE2/HEADER=升级候选，ROS1 Noetic 原生）、`try_algorithm/notes/LITE多楼层探索精读.md`（楼层-楼梯拓扑+FSM，与我们方向④设计互证）。
- ARiADNE2-ROS-Planner 已克隆到 `try_algorithm/code/`（约 1050 行 C++，含 terrain_segmentation/map_handler/collision_checker）。

## 2026-08-25（凌晨）：用户终版定参 + 五方向开工

- **参数终版**：收发量程耦合 **6m**——用户自行多轮实测后拍板，「不用再改了」。此前文档里的 4.5/6.5 均为过程值，已在 launch 注释与 third_party.md 勘误。res=0.2、z∈[0.2,0.8] 维持。
- **用户下达 5 个工作方向**（顺序自定）：①修仿真狗腿乱飞（TARE 时期没有）②阅读配置 `new_algorithm/UFEP-Released` ③找公认评估标准给所有算法统一评分 ④ARiADNE 加楼梯识别+爬楼，验证场景 `~/claude/raicom/Scene_Gazebo/Depot`，go2-scan 新建场景文件夹统一管理场景参数 ⑤论文收集阅读 → `new_algorithm/论文/`（示例：HDPlanner、KAIST UR2025 多楼层窄室内探索）。
- 开工前按要求先推 GitHub 备份。

### 方向③ 评估标准——评估器已建+实测中 ✅（基准跑分进行时）

- 调研结论见 `docs/评估标准调研.md`：探索效率用 Explore-Bench 惯例（ER(t)/T90/T95/路径长），
  地图质量用 Cloud_Map_Evaluation 标准定义（Accuracy/Completeness/Precision@τ/Recall@τ/F-score，τ∈{5,10,20}cm）。
  ER 可见性=range+FOV 无遮挡剔除（文献常见近似，已在报告声明）。
- 评估器 `tools/evaluate_exploration.py`：实时订阅 /quad_0/body_pose+/scan_map+/map，
  结束时经 `/scan_map/save` 服务取**全量**累积图（话题有 1e5 随机抽稀不能用于评测），输出 JSON+MD+ER曲线。
- 一键基准脚本 `simulation/run_benchmark.sh`（无头启动→spawn 竞态重试→挂评估器→出报告→清理），
  gazebo_sim.launch 新增 gui/rviz 开关与 gt_pcd 参数（默认行为不变）。
- **顺带修了潜伏 bug**：scan_cloud_accumulator 的 /scan_map/save 返回 Trigger(...)（服务类型）而非
  TriggerResponse(...)，首次真实调用即崩——已修。
- 教训记录：后台跑 python 必须显式 `/usr/bin/python3`（nohup 环境的 python3 无 numpy，第一次评估因此报废）；
  rospy 脚本别忘了 init_node。
- ARiADNE @ indoor_1 基准跑分正在后台进行，结果将落在 evaluation/results/。

### 方向④ 楼梯/多层——设计定稿 + Depot 场景接入（阶段A完成大半）

- 设计文档 `docs/楼梯与多层探索设计.md`：官方代码零改动原则下做「逐层会话」架构
  （stair_detector / floor_session_manager / stair_transit / way_point_mux 四组件 +
  kinematic_sim 地形跟随 z）。关键前提已验证：**octomap_server 的 occupancy_min_z/max_z
  支持动态重配**（dynparam get /octomap 实测），z 带可运行时按层切换。
- 核心事实（决定架构）：rl_planner 官方版连 z 都没有（waypoint 纯 x/y）；go2_kinematic_sim
  只积分 x/y/yaw、z 恒 0.25 → 现仿真狗永远不会爬升；octomap z 带是世界系绝对高度。
- **Depot 场景接入**：
  - 原模型是 Ignition 版且除地面外全无 collision（雷达打不到墙）→ `scenes/gtools/sdf_add_collisions.py`
    给 WALLS/FLOOR/STAIRS/PILLERS/BOXSET/ROOF 补 collision、转 static、去 ignition 插件。
  - GT 点云 `scenes/depot/gt/depot.pcd`（150万点，trimesh+pycollada 网格采样，工具
    gtools/make_scene_gt.py 可复用于任何 DAE 场景；注意 py3.8 需 pycollada==0.7.2、子网格名带 -lib 后缀）。
  - 台阶几何分析：踏步高≈0.161m、主楼梯段在东侧 x[12.5,13.25] y[0→3.5]，z 0.44→2.76（升高2.32m）。
    已注册进 `scenes/depot/scene.yaml`（含楼层高度/边界/出生点）。
  - 场景参数化：`scenes/<name>/env.sh`（indoor_1 与 depot 各一）+ launch_gazebo_sim.sh/run_benchmark.sh
    支持 `scene:=<name>`。用法：`bash simulation/run_benchmark.sh global_planner:=ariadne scene:=depot`。
- 下一步：Depot 平地基线跑分（阶段A验收）→ kinematic_sim 地形跟随 z（B）→ detector（C）→ transit+会话管理（D）。

### 方向② UFEP——阅读完成，编译待做

- 调研笔记 `new_algorithm/UFEP-Released_接入调研.md`（在 go2-scan 外）：接口契约（吃 grid_map 高程图、
  吐 /my_cmd_vel 直接兼容我们运动学仿真）、依赖清单（vrmapping_msgs+CUDA+glog）、坑（作者家目录硬编码日志路径、Debug 硬编码）。
- 主要缺口：其所需 grid_map 层来自作者的 elevation_mapping_cupy 魔改分支（CUDA+pybind）。
- 注意：UFEP 吃高程图+可通行性层，是楼梯/多层场景的备选全局层。

### 方向③④ 补充（2026-08-25 晚）：run3 基线 + Depot 基线跑 + UFEP 编译通过

- **run3（成功跑基线）**：ER 99.6%、T90=476s/T95=678s/平台期 685s、路径 318m、
  Recall@0.2=95.2%、Chamfer 0.168m。ER 与地图 Recall 差 4.4pt，指标体系自洽 ✅。
  三跑对照（530m/11.5m/318m）定量证实 6m/res0.2 组合成功率 2/3；退化跑特征 =
  ER 极早平台期 + 路径异常短（可作自动判废信号）。详见 evaluation/results/README.md。
- **UFEP 编译通过**（Release, CUDA 12.9, WSL2）：两个坑——①package.xml 漏声明
  vrmapping_msgs 依赖→catkin 拓扑排序把主包排在 msgs 前→配置必炸（已补声明）；
  ②首次失败的 CMakeCache 缓存了坏 PYTHON 结果（rm build/devel 重来）。
  catkin_make 不跟符号链接→ufep_ws 用实体拷贝。产物：vrmapping_node + libCellsIntegratorGPU.so。
- **Depot 基线跑启动**（阶段A验收）：`run_benchmark.sh scene:=depot`（地形跟随自动开启）。
- 论文库新增（均经 arXiv API 核实，凭记忆写的编号两次全错已记录为教训）：
  ARiADNE 原文 2301.11575（决策核正源）、CMU 开发环境论文 2110.14573（cmu_env 正源，
  TARE 四层车库 1839m/1907s 跨楼层实证；**其 terrain_analysis 把楼梯当可通行代价而非障碍**
  ——stair_transit 可借路线）、FAR Planner 2110.09460（可见性路由，多层候选方案）。

### 方向⑤ 论文——目录建立 + 两篇示例落位

- `new_algorithm/论文/`：HDPlanner（arXiv PDF+MIT 开源代码已下载；代码框架与 ARiADNE 同构）
  + KAIST UR2025 多楼层 MCDM（IEEE 付费墙无开源，需用户机构通道下载原文；已下载两篇开放替代文献）。
  索引与收录规范见 `new_algorithm/论文/README.md`。

### 方向① 狗腿乱飞——已修复并数值验证 ✅

**根因（两个独立缺陷叠加，均为「外观动画」误入「物理仿真」）**：
1. URDF 的 12 根腿连杆是动力学体，但 revolute 关节零阻尼、无任何控制器插件（URDF 里唯一插件是 livox 雷达）。狗本体是运动学传送模型（base 由 set_model_state 传送），腿却参与物理：gazebo_bridge 以 ~30Hz 用 set_model_configuration 把关节角瞬移到步态角度，两次传送之间物理引擎在重力+足底接触下自由甩腿，每次传送注入能量且速度残留 → 走动时接触摩擦加剧 → 乱飞。TARE→ARiADNE 期间此机制没变过；变的是负载（octomap res=0.2 等）压低 RTF，暴露并放大了缺陷 2：
2. go2_gait_publisher 步态相位用 event.current_real（墙钟），而 gazebo_bridge 的应用节流用 rospy.Time.now()（仿真时间）。RTF<1 时墙钟相位狂飙、应用按仿真节流 → 每次应用都是大角度跳变 = 肉眼乱踢。

**修法**：
1. URDF：12 根腿连杆全加 `<gazebo reference="…"><kinematic>true</kinematic></gazebo>`——物理引擎跳过其动力学积分，腿变纯外观动画（传感器挂 base 不受影响）。
2. gait_publisher：相位改用 ros::Time::now()（仿真时间），与应用节流同源；RTF 低时步态整体慢放（物理上合理的慢动作）。顺带修掉 joint_states header 墙钟戳会污染 TF 时序的隐患。

**验证**（`simulation/test_legs.launch` 最小环境 + `tools/verify_leg_fix.py`）：set_model_configuration 在 kinematic 腿下 success=true；行走时 Gazebo 实际关节角全程跟随指令（max|实际-指令|=0.180 rad < 容差0.25）；相邻50ms采样 max|Δ实际|=0.232 rad 无跳变；狗 0.49m/s 正常行走。三项 OK。

**遗留**：spawn 竞态（gzserver 加载 world 时 spawn 服务超时但请求已入队执行）再次复现——首启后若报 "model does not exist"，等几秒重试 spawn 会报 "entity already exists"=实际已成功。已知无害，未改。

## 2026-08-24：ARiADNE 推倒重来（官方契约版）

### 为什么推倒

第一次 A2 接入（run1-run11）被判定失败：projected_map 悬空错位、补丁层层叠加成屎山。取证结论：
- **frame 断裂**：octomap frame_id=map 与全系统的 world 无 TF 桥 → 地图悬空（主因）
- **state_estimation 错接 body_pose**：官方语义是雷达原点位姿（vehicleSimulator 源码核实），射线原点系统性偏 ~0.4m
- **对抗性参数补丁病**：miss/factor/min_utility/LOS/停滞判定全是治标；vendored py 打了 10 处补丁
- scan_map 链路逐行比对未被动过；漂移嫌疑是 livox 退化帧+add-only 累积器（另案取证）

处置：完整快照存 `archive/ariadne-a2-attempt1` 分支（151a42a）→ main 清场回 2bf137d 行为 → 官方契约版重建。

### 重建要点（架构与契约细节见 third_party.md「ARiADNE」段）

- rl_planner scripts 从官方副本**原样拷贝一字不改**——map→world 恒等桥修好后上游代码本来就能工作
- 新增恒等 static TF map→world（一行修 frame 断裂）；链路配置独立成包内 launch/go2_ariadne.launch
- sensorScanGeneration 的 state_estimation 输入改接 /quad_0/lidar_pose（对齐官方「传感器位姿」语义）；rl_planner 仍接 body_pose（机器人所在位置）
- 参数全回官方 indoor 基准（factor 0.5 / min_utility 3 / miss 0.45），仅用户决策两项覆盖：max_range=5、z∈[0.2,0.8]
- goal_bridge 重写 ~70 行（唯一自研胶水）；cloud_range_filter 移进 ariadne 组内（TARE 路径保持基线行为）

### 门禁实录

| 门禁 | 结果 |
|---|---|
| G1 链路活性 | ✅ run2 起（sensor_scan/projected_map/way_point 全通） |
| G2 地图正确性 | ✅ TF 恒等桥+雷达位姿精确；幻影墙经三轮排查后 run9 幻影 0 格 |
| G3 决策 | ✅ run9：54 航点零假完成（此前 8~10 航点即停） |
| G4 执行 | ✅ 狗出西楼块至 (7.9,14.6)，位移 >15m |
| G5 端到端 | ✅ run9：81 航点合法完成；地图 105×85 格(42×34m)覆盖全场；狗终点 (26.0,18.0) 距出生点 33m——「出不了西楼块」一并解决 |

**幻影墙插曲（2026-08-24 下午）**：projected_map 大量地面被判黑 → 三轮排查（帧门控[误诊已撤]/截断理论[否决]/官方对表）→ 根因=无限地面接住穿墙泄漏射线+octomap 无带检查的端点占据 → 修复配方见 third_party.md「幻影墙根因链」。run9 幻影 0 格，commit dbf74a5。

### 遗留

- 用户偏好参数回收（2026-08-24~25 多轮连调，用户拍板同轮生效）：①收发量程耦合 **6.5m**（沿革：官方配方 20 → 本机跑通记录 envB 7 → 用户 5 → 3 → 4 → 10 → 6.5。**术语勘误：官方配方=上游原版默认 20m 且本机从未跑通，envB 的 7m 是本机自设参数的现场记录而非官方配方**）；②**map_resolution 0.4→0.2**（octomap 相对原版负载 ≈8×）；③z 带 **[0.2,0.8]**——用户原始偏好在 res=0.2 下完整达成且幻影墙免疫成立（min_z≥一个整格规则：地面端点落中心 0.1 层，0.1+0.1=0.2>0.2 恒假）。**当前组合无任何跑通记录**，观察信号：卡顿 / 过早 Completed / 黄点(optimal_traj 低速段)消失=断供。probe_occ_vs_gt.py 的 GT 对照带已同步 [0.20,0.80]。RViz 显示组已按官方 rviz.rviz 对齐（node 效用蓝→橙渐变、Waypoint 紫球 0.3m、ObstacleVoxel 体素层常开、frontier 0.3m；黄色小球=SCAN optimal_traj 速度着色低速段）。**穿地事故排查结论**：spawn(-7.5,0.5,0.25)/地板/URDF 均未被动过且几何正确，头号嫌疑=res=0.2 负载挤垮 WSL2 物理步进；二分法=命令行 arg 覆盖回本机跑通档（map_resolution:=0.4 sensor_range:=7.0）
- 「路径穿障」观感说明（2026-08-24 用户报告 SCAN 路径穿障碍物）：occ_map/grid_map 链路未被动过；goal_bridge 发的 /initial_path 是两点直线（目标指示非轨迹，设计上无碰撞检查），ARiADNE 航点天生贴障碍表面故直线视觉穿墙属预期；真轨迹是 optimal_traj（B 样条），穿实体才算 bug。待用户确认具体显示项
- TARE 分支回归冒烟未做（本轮全程 ariadne；launch 改动已保证 tare 路径参数不变）

### 楼梯 × 地图分辨率分层结论（2026-08-24 讨论定音）

- **map_resolution=0.4 不阻碍楼梯识别**：占据栅格本来就不负责台阶感知。查过 TARE 全部官方场景 config：indoor/campus/tunnel 0.3m、matterport 0.2m——没有任何一层的地图分辨率能装下单级台阶（15~20cm 高/28cm 进深），行业共识是地形感知独立成层。
- TARE 跨楼层的真机制（本机 vendor 源码 config 核实）：`terrain_analysis` 的 cm 级高程+可通行性网格 + `kUseTerrainHeight:true` + `kCheckTerrainCollision:true`。我们的 go2.yaml 里 `kCheckTerrainCollision: false`，且视点 z 冻结问题在 08-23 审查报告 #8 记录过。
- **真正的风险不是「识别」而是「全局层敢不敢往楼梯走」**：楼梯累加高度超过 z 带上限(1.2)后，整段楼梯在 ARiADNE 的 2D 图上投影成黑墙 → 无前沿无效用 → 上层永远探索盲区。降分辨率治不了（纯 2D 投影天然如此）。
- 选型两条路（未拍板）：①全局层换 TARE——其 3D keypose_graph 天生跨层连接不同高度节点（campus 多层 demo 即此路），代价是丢掉刚调通的 ARiADNE 链；②ARiADNE+楼梯口注册胶水——terrain_analysis 发现楼梯后把上下两端注册为一对连通点塞给全局层，贴合 SCAN 实机人工航点爬楼思路，改动量小。
- 前置条件：决赛场地有无楼梯/允不允许上下，决定此事排期优先级。

## 2026-08-24：ARiADNE A2 重做（官方仿真跑通后回炉）【已废弃——被上面「推倒重来」取代，快照在 archive 分支】

### 背景与决策

- 先在 `new_algorithm/ARiADNE-ROS-Planner + CMU env` 把**官方仿真端到端跑通**（含场景包 479MB 下载、GAZEBO_MODEL_PATH、terrain_analysis 补编译；详见 third_party.md「A2 重做」段），确认了算法的正确数据形态，再回 go2-scan 重做集成。
- 用户决策：删旧架构接线重做；接口用 **A2**（way_point→Path→SCAN navi_mode=3）；octomap 切片 z∈[0.2,0.8]；max_range=5m。路线：纯2D → 楼梯检测 → 赋色。

### 已完成的改动

1. **gazebo_sim.launch ariadne 分支重写**：sensorScanGeneration(转传感器系+TF) → octomap(frame_id=map, z∈[0.2,0.8], max_range=5) → rl_planner → ariadne_goal_bridge(新, /way_point→2点Path去重>1m→/initial_path)。旧的世界系直喂 octomap 有射线原点=(0,0,0) bug（穿墙地图根源），已废。navi_mode 随 global_planner 自动联动为 3。
2. **vendored rl_planner 三处带注释的四足适配**（其余上游代码未动）：
   - run() 兜底 try/except：Timer 线程异常只上 stderr 且缓冲，线程死掉后日志无痕
   - LOS 归因新增 ignore_unknown 变体（utils.check_collision_ignore_unknown + node_manager._utility_los，launch 开关 los_ignore_unknown）
   - 停滞式完成判定：「效用全零 且 地图 stagnant_done_sec 秒无增长」才 done（get_map_callback 里监测 free 格数）
3. **ariadne_goal_bridge.py 新胶水**（integration/go2_bridge/scripts/，已 chmod+x）。
4. cmu_env 补编译 sensorScanGeneration；simulation/kill_all_sim.sh 清理脚本（防孤儿 gzserver 发陈旧 /clock）。

### 调试实录（7 轮迭代的核心发现）

| 轮次 | 现象 | 根因 | 对策 |
|---|---|---|---|
| run1-2 | ~16% 覆盖即判 Completed | utility 视野=0.5×5=2.5m 够不着前沿环；min_utility 的 <= 语义 | factor→1.0；min_utility→0 |
| run4-5 | 仍过早完成 | 门洞前沿紧邻门框锯齿格，unknown 挡视线，46 对视线 46 挡（离线复现证实） | los_ignore_unknown=true |
| run5-6 | 还是停 | 节点只长在轨迹周边 sensor_range 内，前沿 >5m 外永远零效用 | **rl_planner sensor_range 与 octomap max_range 解耦**：5m 只管标图，决策视野提到 12m |
| run6-7 | spawn 报错但模型实际在 | gzserver 加载大 world 时 spawn 服务超时，客户端退出但请求已入队执行 | 瞬态竞态，可忽略（模型最终插入成功） |

### 待验证（下一步）

- [x] run8：发现 `remove_unconnected_nodes` 起点节点清理后 `find()→None` 崩 Timer 线程——**此前多轮无声停摆的真凶**，已兜底回退最近现存节点
- [x] run9：sensor_range=12 后效用不再全零，但狗卡 (-7.8,7.2)：策略反复选贴障碍节点、消毒器逐拍抑制。新增**恢复导航**胶水（rl_planner._find_recovery_waypoint：效用全零时朝最近前沿的可达自由格发航点）
- [x] run10（2026-08-24 收官）：恢复导航触发 9 次正常工作；run() 异常 0；最终按停滞判定（效用全零+地图 20s 无增长）宣布 Completed——完成信号已是真实语义。**遗留**：狗始终出不了起始西楼块（地图 22×33 @(-8.4,-0.8)，GT 场景 53×34m），门洞在 0.4m 切片分辨率下疑似过窄或被 SCAN 局部规划拒绝穿越 → 下一轮调优方向：octomap/rl_planner 分辨率 0.4→0.2、miss、z 窗口微调、SCAN 门洞穿越
- [ ] 集成验证结论：A2 架构端到端打通（G1-G6 全过），G7 的"覆盖持续增长"依赖上述调优 → 任务 #54 完成，覆盖率调优另开任务

## 2026-08-21：标准化迁移 + 点云漂移 + TARE AND 低占据

### 一、仓库标准化（已推 GitHub `pumpkin-db/go2-scan`）

- 源码从 `new_algorithm/` 迁入 `go2-scan/`，按层分目录，每算法独立 workspace：
  - `algorithms/global_planning/tare/` —— TARE 探索决策（CMU）
  - `algorithms/local_planning/scan_planner/` —— SCAN-Planner（SJTU，含 go2_description/map_generator）
  - `algorithms/mapping/elevation_mapping/` —— 高程图（ANYbotics + kindr）
  - `simulation/cmu_env/` —— 仿真底座（velodyne/livox 插件）
  - `integration/go2_bridge/` —— 自研胶水（5 个桥脚本，独立 catkin package）
- vendor 方式：删第三方 `.git`，上游 commit 记入 `third_party.md`。
- 删 `simulation/` 旧副本、`cmu_env/vehicle_simulator`（无人车模拟器，458MB，未用）。
- commit `a845c4a`，4 workspace 重新编译通过，rospack 8 包全命中新路径。

### 二、点云漂移解决（根因 = 两套位姿源不一致）

- **根源**：原 `/mid360_points` 是传感器系（livox 插件 `point=range*axis`），下游再用 `kinematic_sim` 积分的**无抖动平滑 pose** 变换成世界系。狗步态抖动时，点云真实原点在抖、变换 pose 不抖 → 错位漂移。
- **修复**：改 livox 插件 `livox_points_plugin.cpp` 的 `PublishPointCloud2XYZ`，直接输出**世界系真实点云**：
  ```cpp
  ignition::math::Pose3d sensor_world_pose = parentEntity->WorldPose() * raySensor->Pose();
  pt_world = sensor_world_pose.Rot() * point + sensor_world_pose.Pos();  // frame="world"
  ```
- 下游**全部免变换**：
  - `scan_cloud_accumulator.py`：删 lidar_pose 坐标变换，直接用世界系点云（lidar_pose 仅保留作距离过滤基准）。
  - `tare_bridge.py`：删变换，原样转发 `/mid360_points(world)` → `/registered_scan`。
  - `gazebo_sim.launch`：`cloud_is_world: false→true`（grid_map 直接用世界系点云）。
- 结果：点云基本不漂（偶尔边缘出墙一点点，非主导）。
- **注意**：URDF 里还加过 `<gazebo reference="base"><kinematic>true</kinematic></gazebo>`（曾试给 base 设 kinematic 消步态抖动），**未解决问题点云仍飘，已确认非主因**，可保留可回退。

### 三、TARE AND 低占据逻辑（挡墙外目标）

- **问题**：TARE 会发墙外目标点 → SCAN-Planner 拒绝走 → 狗来回逛死循环。
- **根因**：TARE 的碰撞云 = 垂直面提取器的产物（`ExtractVerticalSurface`，`kZDiffMin=0.3`），墙点被滤掉，墙外视点被当可探索。
- **修复（AND 逻辑）**：碰撞云 = 垂直面云(TARE原) ∪ z∈[ground, ground+height] 低占据云(SCAN-Planner式)。改在 `planning_env`：
  - `UpdateKeyposeCloud`：从原始点云剪 z∈[ground, ground+height]，存 `low_occupy_cloud_`（`planning_env.h` 177-190）。
  - `UpdateCollisionCloud`：`collision_cloud_ += low_occupy_cloud_`（`planning_env.cpp`）。
  - 参数（`go2.yaml`）：`low_occupy_ground_z=0.2`、`low_occupy_height_z=0.6`（即 z∈[0.2, 0.8]）。
- **效果**：墙外目标明显减少、事故高发区不再徘徊，**但仍有零星墙外目标**（未根治）。
- **防楼梯误判**：`CheckViewPointCollision` 用"单格累计 ≥ kCollisionPointThr(3) 点才判碰撞"，楼梯台阶是逐级可站立面，不形成连续低占据柱 → 原理上不误判。**未实测验证。**
- **"5帧一发"**：`keypose_cloud_` 每 5 帧（%5==0）才被灌入 + 置 `keypose_cloud_update_`，低占据云约 0.5s 一拍（雷达 10Hz）。静态障碍够用。
- **z 边界**：`CheckViewPointCollision` 判碰撞要求 `point.z - 视点高度 ∈ [-0.5, +0.5]`，而 `kUseTerrainHeight=false` 时视点高度 = `robot_position.z()`(≈0.46)。故低占据点实际生效带 ≈ [0.2, 0.96]，z 上沿>视点+0.5 部分失效。已把 height 调成 0.6 让 z∈[0.2,0.8] 最稳妥。

### 四、RViz / 话题清理

- 删 `tare_scan_stack`（后续接的 TARE 累积全图接口 `registered_scan_accum`，用户不再看）。
- 删 `sim_clouds` 里的**无名点云**残缺显示项（只有 Channel/Class、无 Name/Topic 的坏残片）。
- `sim_clouds`(→`/pcl_render_node/cloud`) 和 `real_clouds`(→`/LIO/clouds_lidar`) 是仿真无用残留（前者没跑 local_sensing，后者 FAST-LIO 实机才用），**暂留**。

### 五、待办 / 开放问题

- **TARE 仍零星空墙外目标**：AND 低占据未 100% 根治。可试收紧（`kCollisionPointThr` 调低、或 `low_occupy_height_z` 微调、或让 `CheckViewPointCollision` 的 z 判定适配视点高度）。
- **楼梯误判未实测**：当前室内场景无楼梯，将来实体机爬梯需拿真实楼梯点云验证 `low_occupy` 会不会误判，必要时对楼梯区域豁免。
- **`scan_map` 纯累积版**：动态障碍旧点不清除（log-odds 版失败已回退），待重实现。
- **`elevation_map` 高程偏移**：ANYbotics 输出位置偏移，疑 TF 外参或 frame 配置，待查。
- **赋色层（比赛 80% 分值）**：相机投影→彩色点云，**完全未做**。
- **安全层**（高程地形 + IMU 看门狗 + LiDAR 接近检测）：未做。
- **NaVILA 语言层**（赛前区域解析 + 答辩演示）：未做。
- **FAST-LIO2**：实机才有，仿真用 `go2_kinematic_sim` 顶替（`fastlio_integration.launch` 已单独验证过）。

### 六、TARE 集成审查发现（2026-08-23，待修复）

> 完整报告在 `docs/TARE集成审查报告.md`（7 维度并行审查 + 对抗复核；33 findings → confirmed 29 / refuted 3 / uncertain 1 → 归并 22 条；high 2 / medium 9 / low 18）。下面按严重度列全，**细节与修法见报告，本处只做索引**。

**高危**
1. **探索完成判定块 vendor 时整块丢失**：`exploration_finished_/near_home_/at_home_/stopped_` 只初始化不置位；上游 execute() 完成检测+回家逻辑在 a845c4a 迁移时缺。全场扫完**不宣布完成、狗沿环路绕圈**、`/exploration_finish` 不发布、比赛收尾/存点云挂空。`kRushHome/kNoExplorationReturnHome/kRushHomeDist` 全死配置。→ 从上游 44500592b861 补回三段逻辑。

**中危**
2. `low_occupy` 是纯世界系 z 带过滤，「楼梯不误伤」注释与代码相反 → 误杀楼梯/低台旁视点。
3. 两套碰撞判定契约割裂：`AddEdge` 用的 InCollision 完全看不到 low_occupy。
4. `/way_point` 外推到 8m 不受 navigation_boundary/低占据约束，胶水原样转发 → 墙外 goal。
5. `start_delay` 用 wall clock 且从桥节点启动起算，Gazebo 慢启动时防穿墙静止期被吞。
6. `low_occupy_ground_z=0.2` 造成 <20cm 障碍盲区，三条碰撞通道全看不见矮障碍。
7. 覆盖记账 LiDAR 模型硬编码 ±12° 垂直窗，MID360 上半球回波永不进覆盖 → 高处漏扫（影响完整度）。
8. `number_z=1`+`kUseTerrainHeight=false` 下视点 z 冻结：高差场景先「碰撞失明」再「连通误断」。

**低危**
9. `low_occupy` 只用最近一拍 keypose 云、不进 5 帧栈 → 记忆只及垂直面的 1/5。
10. `low_occupy` 是世界系绝对 z[0.2,0.8]，上高层后功能静默失效。
11. livox 插件把 miss 射线写成传感器原点点，经原始 `/mid360_points` + low_occupy 污染近机视点。
12. 一组死参数：kViewPointHeightFromTerrain / kTerrainCollisionThreshold / kAddNonKeyposeNodeMinDist / kCollisionGridZScale 配了不生效。
13. GridWorld 固定 121×121×121 格（z 向 363m），启动常驻约 480MB。
14. `/way_point` 残留 publisher 无防护：新旧全局规划器交替驱动狗。
15. `/initial_path` 只 advertise 从不 publish：navi_mode=3 覆盖规划挂点静默失效。
16. 双短路径轮次沿用 stale lookahead 且被 8m 外推放大：目标每秒 180° 反转。
17. LOS 检查 else 分支从射线远端反向遍历：两个分离障碍之间的格子被误标可见。
18. `kCollisionPointThr=1`+碰撞标志粘滞：单个瞬时点永久判死 0.35m 内视点直到栅格滚动。
19. `kCollisionPointThr=1` 与 go2.yaml/PROGRESS 的「≥3 点才判碰撞、不误伤楼梯」论据矛盾。
20. TSP 距离矩阵把「不可达」记成 0 代价：潜伏伪路径缺口（上游既有，三层保护挡住）。
21. 补 viapoint 时误用下标 i 读 poses[i]（应为 j）：字段当前无人读，潜伏。
22. A* 用 in_pg 门控阻止重入队但不更新旧队列键：可构造次优路径。

**存疑（uncertain）**：integration-wiring 的 boundary/TF 交互 1 条（见报告）。
**排除（refuted）**：3 条（config-yaml ×2、integration-wiring ×1，见报告）。

---

## 2026-08-26 用户独立审查回应 + 勘误（本节为权威版本，与旧节冲突处以本节为准）

用户对本仓全部结论做了独立审查，逐条以仓库实物复核后：**A1/A2/A3/B1/B2/B3/C1-C4 全部属实，认账**。勘误如下：

### 撤回的表述
1. **撤回「轨迹 8m→67.4m」杠杆对比**：D1 JSON 白纸黑字 path_length_m=76.04，「真实~8m」是人工口径不可比；D1 轨迹已随残件丢失无法复核。
2. **撤回「统计等价→效用环不是瓶颈」「维持官方 0.5 定稿」**：n=1 对照撑不起该结论。indoor_1 自记同参数三跑 530/11.5/318m（成功率 2/3），run-to-run 方差下 D2/D3 各一跑不叫等价。
3. **天花板 56.8%/76.2%/57.1%(z>4.5) 降级为估计值**：计算脚本不存在、参数未留痕（全仓 grep 零命中），违反「引用前必须核实」纪律。「唯一路径=上楼」归因同步悬置，待 gtools 撒点脚本复现。
4. **修正「站桩 33 分钟」的粗粒度说法** → 精确版：plateau 前 ~116-144s 连续狂奔 65-79m @0.56m/s，之后纯站桩（D2 后段仅 2.3m / D3 仅 0.1m）。

### 重算发现（tools/recompute_path_length.py，2026-08-26）
- **path_length 注水说不成立**：trajectory_5hz 口径含水率仅 ~1%（67.24→67.22）。真正指标是 **moving_ratio**：D2=5.7%、D3=6.7% ——35 分钟里只动 ~2 分钟。
- **新故障模式确认（实锤）**：rl_planner 日志 `0c5b2cee*/rl_planner-18.log`——开跑 216s 墙钟后「utility all-zero + 地图静止20s」→ 停滞式判定宣布 Completed → 站桩。三跑 plateau 136.6/115.6/143.6s 全部擦边逃过旧判废线(<105s)。**我的停滞判定补丁把「规划器失明」翻译成了「合法完成」，两个补丁合谋掩盖核心故障。**
- 评估器已修：SPEED_MIN=0.05 加速度门 + moving_ratio 字段 + 判废线收紧至 15%×duration 且 er<0.95 + json.dump default=float 兜底。重算判定 Depot 三跑均 degraded=True。

### 任务账目更正
- #3 重开（in_progress）：results/ 目录 21 个文件全为 ariadne_*，「评一遍现有算法」实际覆盖 1 个算法；SCAN-Planner 从未进评估器。
- #4 进度自评 40%→33%：打通的只是最易的注册表发布档；#4 与 #8 大概率同根（elevation 零发布 ↔ CMU 崩溃族）。
- #2 表述修正：「腿部物理缺陷已修（最小环境验证）；完整基准跑动稳定性仍有 1/3 失败率，未闭环」。

### 当前第一优先级
任务#11「为什么 ~2 分钟后 ARiADNE 效用全零」——ER 仅 48% 时规划器无候选目标，这比上楼和天花板都重要。elevation_mapping 当晚又死一次（exit code 1，00:30:54，非段错误，日志文件缺失待查 stderr 去向）。

## 2026-08-26 效用全零根因破案 + 修复（任务#11 里程碑）

### 尸检链（全部活体实测证据）
1. rl_planner 开跑 216s 墙钟宣布 Completed：效用全零+地图静止20s → 停滞判定放行。
2. /projected_map 解剖（站桩现场）：unknown 仅 5.6%、free 81.1%、occ 13.3% —— 2D 图自认为探完了。
3. GT 裁决（剔地面、障碍带 z∈[0.25,1.8]）：GT 障碍格 2287 个中 ~42% 被标 free；octree 全场仅 2623 占据叶、z∈[0.2,0.7]、GT 障碍表面点最近邻中位 0.556m —— **墙从来没被建进 octree**。
4. 排除清单：lidar_pose/body_pose/Gazebo物理真值三者一致(14.18,6.08)/(14,6)/(14,6) 无罪；/mid360_points 到 GT 中位差 0.030m 完全真实。（自查教训：曾把 worldFrame 点云再变换一次制造出 15.5m「假漂移」假信号，已识别废弃。）
5. 病灶定性（口径错位，非几何 bug）：
   - Gazebo ray 物理量程 **40m**（URDF），一帧扫全场（sensor_scan P95=13.75m 最大 31.7m）；
   - octomap `sensor_model/max_range=6`：>6m 端点只清空沿途 free、永不产生占据；
   - z 带 [0.2,0.8]（indoor_1 小房间遗产）把墙身/楼梯主体排除在插入之外；
   - 净效果：看得见的远处=free 而非 unknown → frontier 枯竭 → 效用全零。
   - indoor_1 为何幸免：小房间墙总在 6m 内，语义破坏不暴露。

### 修复（最小改动，恢复「看不见=unknown」语义）
- integration/go2_bridge/scripts/cloud_range_filter.py：新增 `~max_range`（默认 0=禁用），按传感器水平距离裁远点。
- gazebo_sim.launch ariadne 分支：max_range=planner_sensor_range(默认 6) 与 octomap/评估器同口径。
- 待办：Depot 重跑验证（预期 frontier 复活、plateau 大幅推迟）；indoor_1 回归一次防基线破坏。

### 附带修正
- utility_range_factor 澄清：launch 文件实际为 1.0（D2 复盘后改的），README 旧结论「维持官方 0.5」与文件不符——已在勘误节撤回该定稿表述。D3(n=1) 不构成扩环无增益的证据，待根因修复后重做 A/B。

## 2026-08-26 续：ASAN 遗留破案 + 建图链恢复 + 修复后首验

### #8 elevation_mapping exit code 1 破案
- 前台裸跑复现：`AddressSanitizer:DEADLYSIGNAL` 死循环——**二进制是上次 ASAN 调试的遗留插桩版**（nm -D 可见 28 个 asan 符号），ASAN 默认 exitcode=1 即历次「exit code 1」死亡的真因。三字段修复是否有效曾被它掩盖。
- 已 rm -rf build/devel 全量重编（需 -DPYTHON_EXECUTABLE=/usr/bin/python3 绕开 conda empy 干扰），asan 符号归零。cmu_env terrainAnalysisExt 与 tare ws 的 tare_planner_node 检查均 asan=0（TARE -11 另查）。
- 修复后首跑中 /elevation_mapping/elevation_map 话题有发布者注册（待长稳观察）。

### cloud_range_filter max_range=6 修复首验（bench_fix1，15min 短窗）
- ✅ 核心修复生效：plateau 849s/902s（修复前 116-144s 即死），移动 246m、moving_ratio 57.1%，degraded=False 且为真正常收敛。
- ⚠️ ER_final 29.0%（15min vs 历史 35min 不同窗，不可直接比）；R@0.2=30.1% 暴露新问题：**建图累加器被 max_range 裁剪连坐**，全量程建图能力丢失。
- 行为模式变化如实记录：修复前「假 free 引导直线狂奔」单位路径覆盖虚高（η 0.0073）；修复后 frontier 引导细致探索 η 0.0012——两者口径不同，比较无意义。

### 架构修正：规划/建图分层供云
- scan_cloud_accumulator 改回两分支统一吃原始 /mid360_points + 自身过滤（max_range=40）：比赛完整度评分依赖全量程建图；cloud_range_filter 的 6m 裁剪只服务 octomap→rl_planner 链。「_clean 门控退化帧」是 livox 时代措施，velodyne+健康门后不再必要。
- SCAN 局部规划器与 elevation_mapping 保持吃 ≤6m clean 云（工作区本就局部）。

### 进行中
- bench_fix2：Depot 35min 同窗对照跑（pid 3177742），与 D1-D3 同窗可比，验证修复后真实 ER 水平。

## 2026-08-26 续2：row_step 一字节破三案（#8 收官在望）

### 段错误根因链（PCL 源码级实锤）
- elevation_mapping 稳定段错误（干净版 SIGSEGV -11；ASAN 版 DEADLYSIGNAL exitcode=1，两者同源）。
- PCL1.10 fromPCLPointCloud2 会把 x/y/z 相邻字段合并为一次 size=12 的 memcpy（ASAN「READ of size 12」吻合），
  且按行遍历时用 msg.row_step 寻址：`&msg.data[row * row_step]`。
- **velodyne 插件上游 bug**：非 organize 分支把 row_step 写成整个点云字节数 `msg.data.size()` 而非
  width×point_step(=16)。height>1 时第 1 行即从 data 末尾外开始读 → 堆越界 → 崩。
- 受害者：elevation_mapping(fromPCLPointCloud2)、terrainAnalysisExt(fromROSMsg 同路径，即 TARE 链 -11)；
  octomap/自研探针按 point_step 迭代不受影响——解释了为何只有 PCL 系消费者崩。
- indoor_1 时代幸免：当时是 livox 插件；换装 velodyne 后才暴露。
- 修复：GazeboRosVelodyneLaser.cpp `msg.row_step = POINT_STEP * msg.width;`（已注释完整因果）。
  重编遇 /mnt/e Anaconda protobuf 污染 + cmake 缓存 → 清缓存 + 剔 /mnt 路径 + 强制系统 protobuf 后 exit=0。
- 待验证：重启仿真后 elevation_map 出帧、terrainAnalysisExt 存活 → #4 几何检测解锁。

### 其他
- em 复现实验教训：裸跑 elevation_mapping 不带 launch rosparam 是空转（input_sources 默认话题不对），
  「合成消息不崩」的第一次结论作废；正确做法是挂同名 __name:=elevation_mapping 吃参数服务器配置。
- bench_fix2（Depot 35min 同窗对照）仍在跑，出数后填对照表。

## 2026-08-26 续3：fix2 出数（ER 新高+建图链翻车）→ 修复 → fix3 在跑

### bench_fix2（35min 同窗对照，max_range 裁剪生效版）结果
- ✅ 规划侧：**ER_final 51.4%**（同窗历史最高，> D1-D3 的 43.5/49.2/48.0）；plateau 1767s/2101s（84% 时长）；轨迹 578.5m；degraded=False 真正常。「效用全零」根因修复在同窗下确认有效。
- ❌ 建图侧崩：scan 仅 4877 点（fix1 为 15.7万）、R@0.2=2.4%、Chamfer 3.95m。
- 根因：accumulator 用的 noetic `pc2.read_points` 对本链路点云有 unpack_from 越界(struct.error)，
  逐帧异常 → 几乎零积累（4877 点为残余）。此前探针也踩过同款报错。
- 修复：scan_cloud_accumulator.py 改 numpy 直析（width×height×point_step 校验 + frombuffer）。

### row_step 一字节破三案（详见上节）
- elevation_mapping 段错误(-11/-1 双形态)与 terrainAnalysisExt -11 同根：
  PCL fromPCLPointCloud2/fromROSMsg 按行(row_step)遍历，velodyne 插件上游把 row_step
  写成整云字节数 → 第 1 行即越界读 12 字节(x/y/z 合并 memcpy)。插件已修+重编(exit=0)。
- 编译坑留档：cmu_env 重编需清 build/CMakeCache.txt + PATH/CMAKE_PREFIX_PATH/LD_LIBRARY_PATH
  剔除 miniconda 与 /mnt/e(Windows Anaconda protobuf 污染) + -Dprotobuf_DIR=系统路径。

### bench_fix3（进行中，全套修复叠加）
- 内容：row_step 修复版插件 + accumulator 直析 + 规划6m/建图40m 分层供云。
- 预期：ER ≥51% 且 scan_map 点数恢复 ≥10万级、R@0.2 回升；elevation_mapping 应存活出帧。
- 启动健康门通过（中位 7.36m）。35min 后出数。

### 论文阅读（#6 本轮 3 篇，笔记待落盘）
- Asymptotically-Bounded 3D Frontier (RA-L26 Lima组)：frontier 并入 raycast 前向模型 O(|F|)；
  Alg.3「frontier 离机器人过近即删」；GP 回归估增益。借鉴点：其 SLAM子图→Octomap 架构与我们一致；
  3D 直接维护 frontier 可避免我们刚发现的「2D 投影口径失真」，但违背 ARiADNE 官方原版原则，仅记录。
- Becoy Go2 Coverage CPP (Frontiers25 TU Delft)：先验 2D 图→形态学骨架→叶节点贪心+Dijkstra→FSM。
  **前提是有先验地图，与比赛未知场地不符**；真正价值=探索完成后扫尾覆盖级（比 frontier 更保完整度），
  开源 ROS2 可参考算法自写 Python 胶水(scipy.morphology+networkx)。
- MA-SLAM (25.11)：2D DRL active SLAM，结构化张量表示。环境 400-520m²、Gmapping 2D——与比赛
  3D 赋色需求不匹配，DRL 训练成本高，价值低。

## 2026-08-26 续4：fix3 出数——全修复栈验证通过，#11 关闭

### bench_fix3（35min，ariadne @ depot）结果 ✅ 三项预期全部达成
| 指标 | fix2（带病） | **fix3（全修复）** |
|---|---|---|
| ER_final | 51.4% | **53.8%**（历史新高）|
| scan_map 点数 | 4,877 | **882,656** |
| R@0.2 完整度 | 2.4% | **96.0%** |
| P@0.2 精度 | 31.9% | 81.6% |
| Chamfer 对称 | 3.946m | **0.110m** |
| Acc/Comp mean | 0.388/7.504m | 0.140/**0.080m** |
- 判废正常（degraded=False）；plateau 862s/2121s（40% 时长，较 fix2 的 84% 大幅改善但仍在）。
- 报告：evaluation/results/ariadne_depot_20260826_033632.md
- 结论：①row_step 修复 + ②accumulator numpy 直析 + ③规划6m/建图40m 分层供云，
  三项修复叠加后规划与建图两侧同时达标。**「效用全零」问题闭环（#11 完成）。**

### 附带清理
- 发现并击毙残留 5.5h 的旧 TARE 评估器进程（昨晚22:04，订阅同话题），临死自写了一份
  tare_depot_20260826_033733.md（自判退化跑，无污染风险，忽略即可）。

### 进行中：bench_tare（35min，tare @ depot）
- 目的：补齐「评一遍现有算法」的第二个算法对照（审查 A3）+ 验证 row_step 修复是否治愈
  TARE 三连段错误（elevation_mapping 已实证复活 52帧/15s，剩 terrainAnalysisExt/tare_planner 待验证 = #8）。
- 启动健康门通过（中位 8.62m）。监视器盯崩溃签名与出报告事件。

### stair_detector 实测受阻（已排障）
- 第一次启动即崩：`#!/usr/bin/env python3` 解析到无 numpy 的 miniconda python → ModuleNotFoundError。
- 排障结论：须用 `/usr/bin/python3`（系统 python3.8 带 numpy 1.24.4）显式启动；等下一个 ariadne 跑时挂载实测（#4）。

## 2026-08-26 续5：TARE 站桩根因破案（/state_estimation 饿死）→ 修桥重跑

### bench_tare 第一跑作废（站桩实证）
- spawn 健康门过（中位 8.62m），CMU 三节点全部存活无段错误（**row_step 修复对 TARE 链生效，
  #8 段错误部分可结**：terrainAnalysisExt/tare_planner 不再 -11）。但狗 0 位移（出生点 -12.60, 0.14 采样 6s 无动）。
- 决策链逐级探针：/registered_scan ✅、/state_estimation_at_scan ✅（tare_bridge 工作正常）、
  **/terrain_cloud ❌、/way_point ❌** → 断点在 terrain 层。
- **根因（源码级证据）**：terrainAnalysis.cpp:227 与 terrainAnalysisExt.cpp:193 硬编码订阅
  `/state_estimation`，而 tare_bridge 只发 `/state_estimation_at_scan`；go2.yaml 只配了
  tare_planner 自己的位姿话题（_at_scan 版本，正确）。两个地形节点收不到位姿 → 静默空转
  （C++ 节点零日志输出连 .log 文件都没建）→ 无 /terrain_map(_ext) → TARE 无法规划。
  launch 注释「CMU 默认输入恰为 /state_estimation + /registered_scan（tare_bridge 已产出）」
  与源码不符，系当时误判未验证。
- **修复**：tare_bridge.py 增加 /state_estimation 双发（胶水层一行发布器）；launch 注释更正。
- scan_planner_node exit -6（SIGABRT）：TARE 分支控制由 closed_loop_controller 承担
  （cmd_vel 发布者实查确认），该死亡不影响本分支主链，暂不追。

### 当前动作
- 杀掉站桩的 bench_tare（pid 3669563），清环境后带修复重启第二跑。
- 论文笔记已落盘：try_algorithm/notes/论文笔记_3D_frontier与覆盖扫尾.md（#6 三篇）。

## 2026-08-26 续6：第二重根因（A* 自回波阻塞）+ launch 统一过滤云 → 待重启验证

### scan_planner_node exit -6 并非无关（推翻「不影响主链」初判）
- rosout 时间线重建：scan_planner_node 前 ~86s 活着且**持续收到 TARE 目标**
  （bspline_optimizer.cpp:146 "a star error, force return!" 反复刷屏 = TARE 在发 /way_point、
  SCAN 的 A* 全部失败），03:43:24 崩 -6。控制链实查：closed_loop_controller 只吃
  `planning/bspline`（SCAN 优化器输出）→ **SCAN 死 = 控制链断**，光修地形层狗也不会动。
- A* 全败根因假设（分支差异对照）：grid_map 点源 tare=原始 /mid360_points（含 velodyne
  360° 下视打到狗体的自回波）、ariadne=/mid360_points_clean（min_range=0.7 已滤）。
  自回波占据起点周围 → 膨胀后 A* 出发即被围死。「tare 吃原云」的基线结论是 livox
  时代（indoor_1 run3 ER=99.6%）产物，velodyne 换装后未复测，今日实测推翻。

### launch 改动（gazebo_sim.launch）
1. scan_cloud 两分支统一 `/mid360_points_clean`（删分支切换）。
2. cloud_range_filter 从 ariadne 组上提为两分支共用，max_range 参数化：
   ariadne=6（保 octomap unknown 语义）/ tare=40（只滤近场，远场供建图全量程）。
3. 注释同步更新（含作废原因与证据行号）。

### 下一步
- 清环境重启 bench_tare 第二跑。观察点：①/terrain_map(_ext) 是否出流；
  ②A* 是否还报错；③scan_planner_node 是否存活；④狗是否移动。
- 若 A* 修复后 scan_planner_node 仍崩 -6 → 查 Eigen/优化器对退化输入的断言（上游 bug）。

## 2026-08-26 续7：TARE 第二重根因破案（NaN 目标 → vector length_error）→ 第三跑在跑

### 第二跑结果：地形层修复生效 ✅，但规划器又崩 -6 ❌
- spawn 健康门过（8.21m）；**/terrain_map 11843 点、/terrain_map_ext 14361 点出流**
  （首跑全死）→ tare_bridge 双发位姿修复验证成功，#8 的「CMU 节点饿死」闭环。
- 但 scan_planner_node 于 wall 04:16:47 再次 exit -6，狗全程未动。
- 排障插曲：pgrep -f 自匹配陷阱——查询命令本身含模式串导致每次匹配到 1 秒大的
  bash 假进程，「节点存活」为误报。此后用 `ps -eo pid,comm | grep -w` 精确查。

### 崩溃根因（bench_sim.log + 源码级证据）
```
Triggered!                                    ← waypointCallback 接受了目标
terminate called after throwing 'std::length_error'
  what():  cannot create std::vector larger than max_size()
```
- 因果链：TARE 偶发发布含 NaN 的前瞻点 → tare_goal_bridge 原样转发 → FSM
  planGlobalTraj 各段时间 = dist/max_vel 全 NaN → global_duration_ NaN →
  scan_replan_fsm.cpp:189 `int i_end = floor(duration/0.1)` 转 int 溢出 →
  :190 `vector<Vector3d> gloabl_traj(i_end)` 抛 length_error → SIGABRT。
  此前 74-89s 的 "a star error" 刷屏是同批目标的另一失败形态（A* 对坏目标失败但没崩），
  崩在第一个触发 NaN 时长的目标上。
- 注：ariadne 分支不受此害——rl_planner+goal_bridge 有去重门控且其目标经过
  octomap 可达性筛选，从不产出 NaN。

### 三层修复
1. **tare_goal_bridge.py NaN 门**：x/y/z 任一为 NaN/Inf 即丢弃（logwarn_throttle 提示）。
2. **advanced_param.xml respawn**：scan_planner_node 加 respawn="true" respawn_delay=2.0，
   单次崩溃自愈不再永久停摆。
3. 清理孤儿进程教训：kill_all_sim.sh 杀不死旧 roslaunch 的 C++ 孤儿（两跑各残留一对
   terrainAnalysis 打架），重启前需 `ps -eo pid,comm | grep` 复核并手动补刀。

### 当前动作
- bench_tare 第三跑（pid 3952143）已启动，监视器盯 Triggered!/length_error/died 签名。
- 观察点：NaN 门是否拦截（logwarn 出现）、scan_planner_node 若再崩是否 2s 内复活、狗是否动起来。

### stair_detector 进展（#4）
- 注册表半链路实测通过：/stairs_detected 正常发布 main_east 条目
  （entry{12.87,0.40,0.44}→exit{12.87,3.20,2.76}, source=registry），/stairs_markers 同发。
- 几何检测半链路待狗走近楼梯区（高程图就绪后 tick 自动升级为几何检出）。
- 教训：rospy 节点 nohup 后 stdout 块缓冲，日志空≠进程死；探针消息类型要先查源码。

## 2026-08-26 续8：TARE 第三跑早期信号——狗动起来了（分支首次）

### 三层修复全部生效的实证
- spawn 健康门过（8.34m）；**狗已离开出生点**（-10.64，前进 ~2m，TARE 分支历史首次移动）；
- /planning/bspline 22 点轨迹正常产出（SCAN FSM 规划链恢复）；
- NaN 门本跑拦截 0 次（NaN 为偶发，门是保险丝）；
- run2 的 elevation_mapping TF 报错风暴在 run3 消失（0 次）——疑似 run2 孤儿
  terrainAnalysis 双实例打架所致，佐证「重启前必须复核孤儿」教训；
- 高程图健康：36×22m res0.05、12 层齐全、13.8 万有效格、高程范围 [-0.05,9.57]m。
- 备份：git 6d1b357 已推送 GitHub（fix3 数据 + TARE 双根因修复 + 文档勘误）。

## 2026-08-26 续9（例行维护）：第三跑中期检查 + 交接前状态快照

### 已验证的运行状态（本小节时间点的实测）
- bench_tare 第三跑进程链全活：run_benchmark(pid 3952143)、gzserver、scan_planner_node、
  tare_planner_node 均在；监视器曾因会话边界断线一次，已重挂。
- **狗仍在出生点附近徘徊**：先后实测 -10.64/-10.54/-11.33（出生点 -12.74），
  最大位移约 2.3m，出现过去而复返。bspline 在产出（22 点）、无 length_error、NaN 门 0 拦截。
  结论：控制链全通、狗会动，但**尚未进入长距离探索**——这是当前卡点（见交接说明）。
- 高程图健康：36×22m res0.05、12 层、13.8 万有效格；run2 的 TF 报错风暴在 run3 为 0 次。
- stair_detector（pid 见 /tmp/stair_det2.log）在跑，注册表条目持续发布。

### 论文（#6）
- 读毕 Sim2Real_Coverage_2024.pdf（实为 IEEE Access 2025，Jonnarth 组，Linköping）：
  2D 在线覆盖的 DRL，多尺度自中心 frontier 地图 + total-variation 覆盖奖励 +
  两步 sim2real；代码开源（github.com/arvij/nl-cpp）。**结论（推断）**：与比赛 3D 赋色
  需求不匹配且训练成本高，不作主线；TV 奖励思想可作扫尾覆盖参考。要点已记，独立笔记待补。

## 2026-08-26 续10：TARE 封存定论 + 双算法对照落盘；UFEP 编译通过；B 验收材料就绪

> 本节起工作主线按用户指示重排：**方向④（ARiADNE 楼梯识别+爬楼）为绝对主线**，
> 方向② UFEP 为楼梯线备选全局层，方向③ 评分扩容、⑤ 论文为辅助填充。
> 五大原始方向进度盘点：①狗腿✅（残留注记：基准跑稳定性 1/3 失败率未闭环）
> ②UFEP 编译通过待跑通 ③标准落地、覆盖不全 ④场景机制✅+楼梯半链路
> ⑤骨架好、KAIST 原文付费墙需用户机构通道下载。

### 方向③：双算法对照表落盘（results/README.md 新增「算法对照表」节）
- TARE run3（052638）：分支首个有效跑（degraded=False）ER 38.3%、轨迹 412.3m、
  R@0.2=93.8%、P@0.2=73.1%、Chamfer 0.134m。
- 对照 ARiADNE fix3 ER 53.8%：ARiADNE 显著占优（ER +15.5pt、轨迹效率 ~2.4×，
  294m vs 412m）。TARE 完整度不差但清晰度明显低（建图噪点多，疑其地形层供云口径）。
- 决策：TARE 封存不再排查（run2「出生点徘徊」不再追因），资源全转方向④。
- 环境遗留：terrainAnalysis ×2 孤儿未补刀（kill 类命令被权限分类器持续挡，
  恢复后先杀再起仿真——run2 TF 报错风暴=孤儿双实例打架的教训）。

### 方向②：UFEP catkin_make 通过 ✅（build_ufep.sh 一把过）
- 100% Built target：vrmapping_msgs/ui + CellsIntegratorGPU(CUDA nvcc) + vrmapping_node；
  devel/lib 出 libCellsIntegratorGPU.so。此前两天被权限分类器挡住的编译命令一次成功。
- 下一步：elevation 层来源决策（作者 cupy 分支 vs 自建 step_cutter/traversability/
  cum_prob/normal 层）→ 最小探索 → 接统一评估器（任务#2/#5）。

### 方向④：B 验收材料全部就绪（等分类器恢复即可一键执行）
- gazebo_sim.launch 加 `global_planner:=none` 纯底座分支：无任何全局决策核，
  FSM 由 navi_mode 直驱（=1 外部单点 / =2 keypoint.yaml 预设序列）；同步改
  navi_mode_eff（仅 ariadne 强制 3）与 planner_sensor_range（仅 ariadne 6m）、
  决策核 group 条件。用途：B 验收/传感链调试/D 组件 climb_mode 单测的干净底座。
- 新建 scan_planner/tools/keypoint.yaml：navi_mode=2 预设航点（集结 11.2,0.4→
  entry 12.87,0.40→exit 12.90,3.60），坐标取 scene.yaml 注册表 GT；FSM 硬编码
  路径 `$(rospack find scan_planner)/../../../tools/` 已核实吻合本文件。
- 新建 tools/drive_go2.py：P 控制遥控（body_pose 闭环→/cmd_vel），转向-前进解耦。
- 新建 simulation/test_stair_climb.sh 一键 B 验收：清场复核→none 底座→spawn 健康门→
  stair_detector→双记录器（pos.log/stairs.log）→三段推进→z 曲线+geometry 检出摘要。
- 验收标准：狗 z 从 0.25 沿台阶连续升至 >1.0m（理论二层 ~3.0）；/stairs_detected
  从 registry 兜底升级 source=geometry。

## 2026-08-26 续11：B 验收二跑（狗横穿成功、terrain_follow 失效破案）、GT 高程路线落地、注册表方向疑案

### B 验收二跑结果（修复 remote_drive + 绕行航线后）
- 五段遥控推进：段1/2/5 rc=0，段3/4 超时但最终段5 把狗送到 (12.66, 3.05)
  ——楼梯 exit (12.87,3.20) 旁。绕行路线（南通道 y=-5.5 穿三道隔断墙）实测可行。
- cmd_vel 稀释修复实证：段2 南通道 18.5m 用时约 1min（≈0.3m/s），remote_drive
  开关生效（一跑 0.014m/s → 二跑 0.3m/s）。
- **但 terrain_follow 未生效**：狗水平穿越楼梯正上方，楼梯区 (x>11.5,y>2.0)
  25149 帧 z 恒 0.412 —— 狗是从台阶几何里「平飘」穿过去的（运动学模型无碰撞）。

### terrain_follow 失效双层根因（代码级+实测级）
1. **go2_kinematic_sim.cpp sampleElevation 行列对调 bug**：grid_map 实际布局
   dim[0]=y 方向（440×0.05=22m=length_y）、dim[1]=x，C++ 注释与代码假设
   「row↔x」相反。探针实证：按 C++ 公式采样楼梯区全「越界」、平地报夹层值 5.3m。
   → terrain_follow 自编译起从未正确工作过（一跑 z 的 0.35→0.58→0.64→0.25 乱跳
   全是错位采样值恰在跳变限幅内被跟了）。
2. **感知高程图在楼梯区不可用**：2.5D 每格单值，台阶面与天花板(6-9m)同格融合后
   报上层表面（实测楼梯中线报 6.6-7.6），下段被 visibility_cleanup 清成 NaN。
   → 即使修好行列，感知高程也带不动爬升。且 Depot 配置无 z 过滤。

### GT 高程路线落地（设计文档阶段 B 预留的「场景 GT 高程」分支）
- 新工具 tools/make_gt_elev.py：GT pcd → 0.05m 2.5D 高程二进制
  （z≤4.0 滤天花板/夹层，空格 3×3 膨胀填充）。
- 产物 scenes/depot/gt/elev_gt.bin：613×314，z∈[0.09,4.0]，平地/出生点 0.093 ✓。
- 未完成接线：kinematic_sim 读 GT 高程文件（参数化数据源）→ 重编译 → 重跑 B。

### ⚠ 新发现：楼梯注册表 entry/exit 疑写反（待仲裁）
- GT 高程实测楼梯中线 x=12.87：y=0.0-0.8 处 2.69m（二层平台），y=3.2 处 0.51m
  （地面）——沿 +y 是**下降**。而 scene.yaml 注册表 entry{y=0.4,z=0.44}→
  exit{y=3.2,z=2.76}「沿 +y 爬升」方向相反。判断：y=0.4 侧是二层平台端、
  y=3.2 侧接地，注册表 entry/exit 字段互换。影响：D 阶段 stair_transit 爬向；
  stair_detector 几何检出的 yaw 恰可修正注册表（待几何验证后回填）。
  （仲裁被分类器中断，GT 原始点两端 z 对照未跑完。）

### 遗留清单（下次开工）
1. kinematic_sim 接 GT 高程查表源 + 重编译 + 重跑 B 验收（z 应沿 0.41→2.9 爬升）
2. stair_detector 几何检出验证——本次又栽在启动坑：rosrun 走 shebang env python3
   → miniconda 无 numpy。必须 /usr/bin/python3 <全路径> 直接起（续9 教训复现）
3. test_stair_climb.sh 两处修正：摘要解析器（rostopic echo 是单行 key: value，
   解析器按「键独行」假设写错）、stair_detector 启动命令
4. ~~UFEP 下一步：elevation 层来源路线决策~~ → **已作废，见续12 用户决定**
   （cupy fork 已克隆至 new_algorithm/elevation_mapping_cupy_EleForUFEP，原地封存）
5. MapExRL 论文未读（其余两篇笔记已落盘）

## 2026-08-27 续12：UFEP 冻结决定 + 性质核实收官（短记录）

### 用户决定（2026-08-27 会话，口头指示）
- **UFEP 先不管（冻结）**：本体不跑通、不接管线；编译产物 ufep_ws 原地保留备查。
  续11 遗留第 4 条（elevation 层来源路线决策）作废。
- 与既有结论（try_algorithm/notes/UFEP思想评估.md）一致：不采用本体，
  仅采纳两思想（沿线采样边检查→楼梯 transit 可达性验证；frontier 即图节点）。

### UFEP 性质核实（本次会话源码级，补充思想评估）
- **vrmapping 规划器本体 = 纯传统方法**：全源码无 torch/tensorflow/onnx 任何引用；
  核心链路 = 采样撒点 + 几何边检查（沿线查高程差/可通行性）+ 图搜索。
  CUDA 只用于 CellsIntegrator.cu 点云→高程图融合加速，非神经网络。
- 上游 elevation_mapping_cupy fork 的 traversability 层**可选**挂小 CNN
  （3 层 dilated conv，torch/chainer），但：权重走外部文件加载（load_weights），
  克隆的 fork 里**无任何 .npz/.pth/.npy 权重**→ 开箱不可启用；且有纯经典替代
  （cupy 滤波、traversability_polygon 几何规则）。→ 整条链可零深度学习运行。
- 对照记忆：**ARiADNE（主力）才是深度学习**（MARMoT 系 RL 策略网络），
  UFEP（已降级参考）反而是经典方法。两者均为现成开源实现，不违反禁自造约束。

### 用户指示（2026-08-27，第二件事之前）
- **评分系统的评分概念，用户之后会亲自具体了解**——评分扩容（任务#5）与指标细化
  全部挂起，等用户看完概念再定；近期不动评分系统。
- 工作重心：**楼梯检测 + 正常上楼梯**（方向④），按续11 遗留 1-3 开工。

### 当前挂起事项（等用户下指令）
- 楼梯主线按续11 遗留 1-3 待开工：GT 高程接 kinematic_sim → 重编译 → 重跑 B 验收；
  stair_detector 几何验证；test_stair_climb.sh 两处修正。

## 2026-08-27 续13：楼梯线遗留 1-3 代码全部改完（未编译未验证，等用户合跑）

### 楼梯注册表方向仲裁——结案：写反了，已修
- GT 点云直查（3608 点，楼梯带 |x-12.87|<0.3，z<4）：
  - y<0.8（原 entry 端）：中位高 **2.53m**、最高 3.48 → 二层平台端
  - y>2.5（原 exit 端）：中位高 **0.44m**、最高 1.24 → 接地端
  - y 1-2.5 中段中位 1.73（过渡台阶）→ 完整单调坡面，方向唯一
- 修正 `scenes/depot/scene.yaml`：entry=(12.87, **3.20**, 0.44 接地)、
  exit=(12.87, **0.40**, 2.76 平台)、yaw_deg 90→**-90（沿 -y 爬升）**

### go2_kinematic_sim.cpp 双修（scan_planner 包，**未重编译**）
1. **行列对调 bug 修复**（感知路线，实机用）：dim[0]=y/dim[1]=x，
   `data[iy + ix*rows]`，越界判断同步对调。
2. **GT 高程查表源新增**：参数 `terrain_source`（gt_file/elevation，默认 gt_file）
   + `gt_elev_file`；读 make_gt_elev.py 二进制格式（int32 nx,ny | float32 x0,y0,res
   | float32 h[iy*nx+ix]），失败自动回退感知话题订阅。
- 配套：`gazebo_sim.launch` 透传两参数；`scenes/depot/env.sh` SCENE_EXTRA_ARGS
  追加 terrain_source:=gt_file + gt_elev_file:=…/elev_gt.bin。

### test_stair_climb.sh 三修 + 航点改道
1. stair_detector：rosrun → `$PY 全路径直起`（避 miniconda 无 numpy）
2. 记录器：stdbuf → `PYTHONUNBUFFERED=1`（rostopic echo 是 Python 进程）
3. 摘要解析器：单行 `x: 7.518` 正则版（截断科学计数法 try/except 跳过）
4. 航点按修正后方向改道：段3 (11.2,3.5)、段4 (12.85,3.55 接地端前)、
   段5 (12.87,0.15 沿 -y 爬，0.13m/s，timeout 300)——从楼梯北侧进场

### 待合跑（用户指令后）
- catkin_make 重编译 scan_planner → bash simulation/test_stair_climb.sh
- 预期：GT 高程载入日志 + z 从 0.41 沿台阶爬至 ~2.9（段5）

### 追加（同日）：重编译完成 + Depot 交互首跑准备就绪
- **重编译 ✅**（04:05，含 GT 高程+行列修复）；自查堵漏一处：launch 声明了
  terrain_source/gt_elev_file arg 却没传进节点 param，已补。
- **实时评分进 RViz**：新工具 `tools/live_score_rviz.py`（增量 ER+路径+用时 →
  /live_score 文本 Marker；import 复用评估器可见性函数，口径一致，官方评估器
  未动）；经 go2_bridge/scripts/ 符号链接挂进 launch（修 realpath 解链接坑）。
  默认跟随 rviz 开关 → 无头跑分零行为变化。
- **default.rviz 新显示组 Score_Stairs**：/live_score、/stairs_markers、/initial_path。
- **stair_detector 接进 launch**（arg stair_detect）；Depot env.sh 自动开——
  注册表兜底橙箭头无条件发（已验证代码逻辑），几何检出蓝箭头依赖高程图。
- **话题核查**：/map（GT 全场）、/scan_map（累积）、/frontier、/projected_map、
  /way_point、octomap 系、grid_map 系、elevation_map、his_path、机器狗模型
  原显示组全在；新加 3 个。/stairs_detected 是 String 只能终端看。
- **dry-run 通过**：`roslaunch --nodes` 全参解析，22 节点无报错。
- **启动命令（等用户指令）**：
  `bash simulation/launch_gazebo_sim.sh scene:=depot global_planner:=ariadne`
  （gui/rviz 默认 true；env.sh 自动带地形跟随+GT高程+楼梯检测）

## 2026-08-27 续14：狗模型分体根治（刚体化，社区标准做法）+ 纹理官方格式修正

### 狗分体问题（用户实测发现，此前验证有盲区）
- 症状：狗走动时 12 条腿留在原地、身体传送走（link_states 实测：腿距身体
  1.1-1.5m、抖动 2m+），站定后才归位。
- 根因：腿连杆 <kinematic>true</kinematic> 后被物理引擎完全跳过，
  set_model_state 传送 base 时关节约束不带它们走。此前方向①验证只测了
  「关节角跟随指令」，从未测「连杆位置跟随身体」——验证盲区。
- **修法（对齐社区/参考工程标准：机器人按刚体 + 官方驱动插件，不做传送式
  关节动画）**：URDF 12 个腿关节 revolute→fixed，Gazebo 自动把 13 连杆合并为
  单一刚体——物理上不可能再分体。teleport 机制保留（管线需要 Gazebo 狗位姿与
  运动学里程计精确一致供雷达射线）。
- 实测验证（Depot 世界）：腿连杆不再以独立体出现于 /gazebo/link_states（已合并）；
  狗正常行走（-12→-10.35）；雷达健康门通过；零报错。
- 代价：步态动画消失（腿固定零位站姿）。日后若要动画，走官方
  gazebo_ros_control + position_controllers 全套（本机已装齐），不再自造。
- 配套：gazebo_bridge 关节同步改为失败一次即永久关闭（刚体无活动关节），防刷错。

### 纹理修正（对齐官方模型用法，砖箱模型为证）
- 根因：SubT Depot 模型转经典 Gazebo 时 script 兜底 URI 写成相对路径，
  经典 Gazebo 不认 → 材质永远找不到 → 白模。
- 修法：16 处 <uri> 改官方格式 model://Depot/materials/scripts|textures；
  4 个纯 PBR visual（货箱/风扇×3）补 script 兜底 + 新增 2 个材质定义；
  撤销此前绕路的 media/ 符号链接与 GAZEBO_RESOURCE_PATH hack（官方用法只需
  GAZEBO_MODEL_PATH）。ogre.log 实证 15 张贴图加载成功（余 3 张为 Emissive
  自发光层，经典 Gazebo 不支持，不影响外观）。

## 2026-08-27 续15：ARiADNE 相对 e438e07 差异定档 + 三个未决问题挂账（用户口述）

### ARiADNE 链相对 e438e07 的全部差异（git 逐文件核实）
决策包仅 3 个文件动过，网络结构/权重/效用计算/节点图/ariadne_goal_bridge/TF桥 逐字节未变：
1. **算法级仅 1 处**：rl_planner.py 停滞式完成判定（效用全零 且 地图已知格静止满
   20s 才判完成，~stalled_complete_seconds 可调；9500ef5，修 Depot 假完成）
2. **参数 1 处**：utility_range_factor 0.5→1.0（效用环 3m→6m；⚠偏离官方值，
   为 Depot 大场景规划停摆加的）
3. **供云 1 处**：cloud_range_filter 加 max_range=6 裁剪（671e3f3，修「40m 量程
   vs 6m 更新半径」口径错位致前沿枯竭）
关键事实：三处全是 Depot 排障产物；用户的 indoor 高分跑（99.6%）发生在这些改动
之前，当时状态≈e438e07。要复现 indoor 老行为需全退这三处；保 Depot 53.8% 则保留。

### 狗模型折腾全程与当前状态（重要交接）
- 用户实测发现：kinematic 腿版本走动时**分体**（腿留原地、身体传送走）——
  此前方向①验证只测关节角未测连杆位置，验证盲区。
- 之后多轮尝试（刚体化/阻尼/回退）均未让用户满意；**当前状态 = 腿完全等同
  e438e07 原版（revolute 零阻尼带碰撞）+ base kinematic（必须保留：删了之后
  乱甩的腿反踢身体→雷达抖→点云飞，实测复现过一次，已恢复）+ velodyne 雷达**。
- 官方件备查：gazebo_ros_control + position/joint_state controllers 本机已装齐，
  若日后要让 Gazebo 里的腿连续受控动画（不再靠传送摆腿），走那条标准路。
  RViz 的 RobotModel 狗一直是干净动画（TF+ /joint_states 驱动，无物理参与）。

### ⚠ 四个未决问题（用户 2026-08-27 实测口述，挂账待查）
1. **狗腿还是乱飞，用户怀疑是高度问题**（当前 e438e07 腿=零阻尼物理关节，
   乱飞是该版固有症状；「高度问题」为用户猜想，未验证——可查出生点 z/体高/
   足底接触与甩腿的相关性）。
2. **Depot 上探索很小，不能正常探索**（与本次会话航点日志实测吻合：航点长期
   在出生点 ±2m 震荡；对比室内跑前 60s 即走出 23m。历史所有 Depot 跑均此慢启动，
   嫌疑：大开阔场景前沿/效用区分度低，rl_planner 在近点间震荡）。
3. **楼梯检测似乎无效**（用户 Depot 实测观感；未核实是注册表橙箭头没显示、
   还是几何检出没触发。下次先问清现象再查）。
4. **Depot 纹理很多加载失败**（用户实测确认仍大面积白模；已做过 script URI
   官方格式化 + 4 个 PBR-only visual 兜底，ogre.log 显示 15 张贴图加载成功
   但未除根。详见 临时问题.md 第 6 条。纯视觉问题，不影响雷达/探索/评分）。

## 2026-08-28：Gazebo ModelPlugin 运动迁移阶段 A

- 新增 Go2 ModelPlugin：ROS 回调仅缓存 `/cmd_vel`，Gazebo WorldUpdate 负责积分
  `x/y/yaw`、写入 model pose，并发布 `/quad_0/body_pose` 与 `/quad_0/lidar_pose`；
  `z` 暂时固定，腿锁定为站立姿态。
- 新模式已关闭旧 `go2_kinematic_sim → gazebo_bridge → /gazebo/set_model_state`
  运动链及 gait publisher，Gazebo 中只保留一个模型位姿写入者。
- 阶段 A 基本跑通：`/cmd_vel`、body/lidar pose、LiDAR、scan_map、SCAN-Planner
  与 ARiADNE 基础链均可运行。
- 正式 10 ms / 100 Hz 配置下约 0.262 m/s 法向运动墙测（100 帧）：60% 帧约
  0 mm，40% 帧约 +2.7 mm；旧架构的 ±20 ms 离散档消失。
- 剩余约 10 ms 离散误差暂时接受，不再继续调试点云时序。下一步进入阶段 B
  前先完成楼梯与 SCAN-Planner 既有 3D 接口审计。

### 2026-08-28：ARiADNE ping-pong 现象固化

- Depot 与 indoor_1 均曾出现相邻 2m waypoint 往返；当前不再继续 utility/点云 A/B。
- Depot 观测确认：action candidates 很多、utility 非零，greedy policy 偏向
  `(-12,-2)` 与 `(-10,-2)`；同时 projected_map known/free 长时间不变，地图停滞。
- 下一步改为对照官方 ARiADNE 系统审计，暂不改参数、ModelPlugin 或点云链。

### 2026-08-28：阶段性收尾（当前基线）

- 分支 `debug/ariadne-baseline-20260828`，基线 commit `c33cedd`。
- Gazebo ModelPlugin 阶段 A 基本跑通；旧 `go2_kinematic_sim → gazebo_bridge →
  set_model_state` 运动链已关闭，Velodyne worldFrame 使用 `scan.world_pose()`。
- `/mid360_points` 已确认是真 world 坐标；剩余毫米级误差属于 10 ms 离散相位抖动，暂不处理。
- Depot 与 indoor_1 均观察到 ARiADNE 相邻 2m waypoint ping-pong；候选节点多、graph
  connectivity 正常，policy 使用 greedy/argmax。官方 Save Mode 会触发但不能脱困。
- `path_to_nearest_frontier` 实际是最近可达 `utility>0` graph node 的路径；frontier
  遮挡检查存在，occupancy 正确时墙后 frontier 不会穿墙贡献。
- 未解决：ARiADNE 局部 ping-pong/地图停滞；视觉腿未恢复；terrain-follow/body z 未迁入
  ModelPlugin；楼梯状态机/楼层切换未实现；Depot 纹理白模未处理。
- 下一步：先继续定位 ARiADNE ping-pong 的可恢复路径，再进入楼梯和腿。

### 2026-08-28：ARiADNE ping-pong 根因审计与 escape recovery

- 系统审计确认：当前核心网络、graph 代码和 checkpoint 与 indoor_1 99.6% / Depot
  53.8% 基线一致。`UPDATING_MAP_SIZE=88m` 未随运行时 6m 量程重算，但官方 ROS fork
  和历史好基线同样如此，保留其训练输入尺度，不作为本轮根因修复。
- ping-pong 机制已实测闭环：机器人跨过相邻 2m 节点中点后 current node 切换；current
  action 被 mask，greedy policy 在新 current 上以 0.70～0.9999 概率重新选择上一节点，
  形成确定性 A↔B 环。官方 ROS checkpoint 对全零 guidepost 为既有契约；尝试恢复原版
  `visited=1` 后循环仍存在且转移到其他节点，已撤回。
- 决策归类为 **B：官方 policy 在部分局部图状态存在稳定循环，需要外部 recovery**。
  新增 escape recovery：确认循环后屏蔽循环节点作为 RL 目标（仍允许作为图路径中间点），
  选择距局部区域较远、A* 可达且确实关联真实 frontier 的 graph node，沿现有路径脱困；
  达到地图增长和净位移门槛后恢复 RL。tabu 超过32节点时仅保留最新循环，防止无限增长。
- 短测：Depot 120s 内连续打破两组局部环，航点推进到 y=6m 等新区域，known cells
  约 1900→3793；indoor_1 同样完成 6.3m escape、地图增长1004格并继续推进到新区域。
  Python 编译/静态检查通过，无临时调试日志残留。
- 风险/下一步：尚未跑完整 ER；短测中 SCAN 偶发 A* error 但节点存活且后续继续规划。
  下一步先跑 Depot/indoor_1 中时长回归确认覆盖率，再进入 terrain-follow/body z。
