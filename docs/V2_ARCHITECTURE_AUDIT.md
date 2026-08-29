# V2 架构审计报告（Real-Robot-First）

> 2026-08-28 晚 / 分支 `refactor/multifloor-realrobot-v2`（自 baseline 新建）/ 执行：ZCode
> 任务来源：GPT《go2-scan 多楼层自主探索 V2：Real-Robot-First 重构任务》第 21 节。
> 本轮只审计不改代码。审计方法：源码精读 + 与官方上游 `wuyi2121/SCAN-Planner@348e8a5` blob 级 diff + unitree_sdk2 main 分支 clone 精读。

---

## 0. P0：稳定基线确认 ✅

- `origin/debug/ariadne-baseline-20260828` 存在，tip = `7e94cad`（chore(diag): exploration diagnostics and runbook），其下含 `37325bf`（fix(ariadne): restore e438 exploration decision semantics）。✓ 与任务要求一致。
- v1 冻结锚点 `5797b4e` **不在** baseline 内；baseline 是 v1 的祖先。→ 从 baseline 开 V2 分支天然零 v1 代码。
- 新分支 `refactor/multifloor-realrobot-v2` 已从 baseline tip 创建。
- baseline 自带 B 阶段资产（非 v1，可直接继承）：
  - `integration/go2_bridge/scripts/stair_detector.py`（Depot registry 检测后端）
  - `scenes/depot/scene.yaml` 楼梯注册表（main_east：entry(12.87,3.20,0.44)→exit(12.87,0.40,2.76)，yaw −90，ground 0.09 / mezzanine 2.86）
  - `algorithms/local_planning/scan_planner/tools/keypoint.yaml`（已配 Depot 楼梯序列）+ 官方 `keypoint_recorder.py`
  - `go2_kinematic_sim.cpp` 内 GT/感知高程跟随（standalone 仿真 plant）

---

## 1. SCAN 官方能力审计（源码级）

### 1.1 我们相对上游改了什么（diff @348e8a5，plan_manage/src）

| 文件 | 差异 | 性质 |
|---|---|---|
| `scan_replan_fsm.cpp` | 仅诊断：`dumpDiagSnapshot`/`logReplanDiag`/`odom_stamp_`/`/scan_diag/snapshot`/initial_path 缓存 | **零行为改动** |
| `go2_kinematic_model_plugin.cpp` | 上游无此文件，我们新增（Gazebo 执行器） | 仿真 plant |
| `go2_kinematic_sim.cpp` | 高程跟随（GT 文件/感知高程图双源）、限速、跳变钳制 | 仿真 plant |
| `go2_gait_publisher.cpp` | 仅步态相位改用 ROS 仿真时间（防 RTF<1 时腿乱踢） | 仿真 plant |
| `closed_loop_controller.cpp` / `open_loop_controller.cpp` / `planner_manager.cpp` / bspline_opt / plan_env / path_searching | **与上游完全一致** | — |

结论：**规划算法核心与上游零偏离**。历史"楼梯 B 阶段"全部落在仿真 plant 与桥接层。

### 1.2 navi_mode=2（PRESET_TARGET，官方"multi-floor keypoint"）

- init 时 `std::system("rosparam load .../tools/keypoint.yaml")` 一次性读入 `waypoint_num` + `waypoint{i}_{x,y,z}`（**官方原版机制，一字未改**）。
- 推进：`EXEC_TRAJ` 中距 `end_pt_` < 0.5m 或当前段轨迹时间耗尽 → `current_wp_++` → `planNextWaypoint()`。
- 每段 = `planGlobalTraj(start→end)` **直线 3D 多项式** + `adjustGlobalTargetIfOccupied()`（末端在栅格判占据 → 沿全局轨迹回退采样到最近无碰撞点，截短全局轨迹）。
- **keypoint 列表 init 后固定，无任何运行时注入接口**。上游 tools/README 语义 = `keypoint_recorder.py` 从实机 odom 人工录制 → 回放。

### 1.3 navi_mode=3（REFERENCE_PATH，官方"reference-path tracking"）

- 订阅 `/initial_path`（nav_msgs/Path），逐点取 **XYZ 完整保留** + `body_height`（z 抬高到体心），≥0.5m 去重。
- `planGlobalTrajWaypoints()` 生成**穿过全部路径点**的一条 3D 全局多项式轨迹；`adjustGlobalTargetIfOccupied()` 同样会做末端回拉。
- replan（`planFromCurrentTraj` REFERENCE_PATH 分支）：从当前局部轨迹状态直接 `callReboundReplan`（三次降级：普通→真 restart→随机初始化），**全局轨迹不被重算**；B-spline 控制点初始化用 3D A*（100³ cell）穿越路径兜底。
- 收到新 Path：WAIT_TARGET→GEN_NEW_TRAJ / EXEC_TRAJ→REPLAN_TRAJ。**这就是官方给 TravExplorer 式动态 3D route 用的接口**（README 原文："3: reference-path tracking with local obstacle avoidance; see TravExplorer"）。
- 楼梯语义（README 原话）："**If the robot cannot climb stairs, increase the z height of body, keypoints or initial path.**" —— SCAN 无任何楼梯专用逻辑；跨层 = 把参考路径 z 抬高，让 3D optimizer 走台阶上方的自由通道。

### 1.4 is_real_world 机制

- 纯 launch 接线：body_pose `/LIO/odom_vehicle`、sensor_pose `/LIO/odom_imu`、cloud `/LIO/clouds_lidar`、`cloud_is_world=false` → `grid_map/need_extrinsic=true`（cloud 在传感器系，按外参+sensor_pose 变到世界系）。仿真侧 cloud 已在世界系、无需外参。
- `fsm/is_real_world` 参数在 C++ 中**无人读取**（死参数）。
- **FSM/优化器/闭环 controller 无任何仿真专用代码**；仿真专用件全部是独立节点（open_loop_controller、go2_kinematic_sim、local_sensing、ModelPlugin），真机直接不启动它们。
- 官方真机定位配套 = **Elevator-LIO**（FAST-LIO2 多楼层扩展）——与任务书 §12 参考一致。

### 1.5 裁决：mode3 作为 V2 正式接口

1. **接口本质**：Manager 检测到楼梯后要给 planner 的正是"运行时动态 3D 多点路线"。`/initial_path` 已经就是它；mode2 的 keypoint 表 init 固定（YAML+rosparam load），改造它必须动 FSM 代码，改完能力与 mode3 现有接口等价——重复造轮子。
2. **几何表达**：mode3 全局轨迹穿过全部路径点（楼梯这种 z 连续变化的地形用 3~4 个中间点即可锚定通道）；mode2 每段是 start→end 直线，需要密到台阶级的 keypoint 才能近似。
3. **官方站位**：SCAN 自我定位 = 低层规划器，上层动态 route 用法官方就是 mode3+TravExplorer。
4. mode2 保留一个真机价值：`keypoint_recorder.py` 录制回放可作**实机楼梯人工示教基准**（Test A 顺带验证，也为 §11 的实机步态对照提供参照轨迹）。

### 1.6 Test A/B 最小实验设计（P2，无 fallback/teleport/位置强改）

- **起点**：Depot main_east 楼梯前（entry 北侧 ~2m，地面 z≈0.09+body_height）。
- **Test A（mode2）**：`tools/keypoint.yaml` 已是 Depot 序列（基线资产，微调 z 抬高即可）→ 零代码，`navi_mode:=2` 直接跑。
- **Test B（mode3）**：最小过渡脚本按 scene.yaml entry/exit 几何插值发一条 `pre-approach → entry → 3 个楼梯中点（z 按台阶插值+抬高）→ exit → post-exit` 的 3D Path 到 `/initial_path`。
- **统一观察项**：A* 失败率、全局轨迹生成成功率、cmd_vel 连续性（无反向/无抖动）、是否真的沿 3D 路径 z 推进、`adjustGlobalTargetIfOccupied` 是否把出口回拉（2.5D 歧义格风险点）、是否依赖任何仿真 hack。
- 若两者都失败 → 按任务书 §18 止损，报告 SCAN 具体失败层（global/A*/rebound/controller 四选）再议替换。

---

## 2. 真机接口审计（Unitree Go2 / unitree_sdk2 main）

- `SportClient`（go2）公开方法全集：`Damp / BalanceStand / **StopMove** / StandUp / StandDown / RecoveryStand / Euler / **Move(vx,vy,vyaw)** / Sit / RiseSit / **SpeedLevel(int)** / Hello / Stretch / Pose / 翻滚跳舞类 / **FreeWalk / ClassicWalk(flag) / StaticWalk / TrotRun / EconomicGait / SwitchAvoidMode**。
- **没有** `SwitchGait` / `SwitchLocomotionMode` / 楼梯步态接口（B2/A2 的 sport_client 有 gait 切换，Go2 公开头文件没有）。
- 底层 DDS IDL `SportModeCmd_` 有 `mode / gait_type / speed_level / foot_raise_height / body_height / path_point[30]` 字段——社区所谓"stair gait 3 / forwardDownStair 4"属于**未文档化的底层字段/逆向**，公开 SDK 不承诺。实机验证前不得假设。
- 实际含义：Go2 官方上楼方式 = 普通运动模式 `Move()` 慢速直行（官方内置步态自带上楼梯能力）；SCAN `closed_loop_controller` 输出 `cmd_vel(Twist)` → 适配器直接映射 `Move(vx, vy, vyaw)` / 停止时 `StopMove()`，**接口天然匹配**。
- 架构落点：`Locomotion Mode Adapter` 枚举预留 `NORMAL / STAIR_UP / STAIR_DOWN`；第一版只实现 NORMAL（Move/StopMove/SpeedLevel），STAIR_* 的映射留待实机审计（届时验证 SportModeCmd 低层字段或 EDU 私有服务），**该差异被 adapter 吸收，不污染上层**。

---

## 3. v1 处置清单（baseline..5797b4e，13 commits / 38 files）

### 删除（v1 瞬移根源，禁止移植）
- ModelPlugin 楼梯控制系统全部：`/stair_traverse_lock`、`/stair_traverse_dir`、`/stair_traverse_zprof` 三话题事务、单调锁位置钳制（s 回退时重写 pose）、zprof 几何 z 直写。
- `stair_mission_manager.py` 整文件（含 100Hz cmd_vel fallback、直接 kill octomap、COMMIT/TRAVERSE tick 合并等全部低层控制）。
- ariadne_goal_bridge 5s resend_tick（v1 特有 workaround，V2 重审是否还需要）。

### 选择性移植（逐项审查后）
- `rl_planner.py` STOP_REASON 全路径日志（纯诊断，高价值）。
- `rl_planner.py` floor_reset 的**思想**：网格对齐重锚 + 30s 完成宽限；接口改挂 `/floor_session/reset` 抽象。
- bridge 的 nav_source 门控**思想**（过渡期暂停航点下发），实现重写。

### 原样继承（baseline 资产）
- stair_detector.py（降级定义为 Simulation GT Backend）、scene.yaml 注册表、keypoint.yaml、diagnostics 全套、ModelPlugin 的 cmd_vel 积分骨架 + GT 高程跟随（参数门控，标注 SIMULATION ONLY）。

### 全新实现（V2）
- `StairTransition.msg`（header/stair_id/from_floor/to_floor/direction/entry/exit/width/rise/confidence/centerline path —— 原子消息替代跨话题事务）。
- `multifloor_exploration_manager.py`（高层状态机，只发 route+状态，禁发速度）。
- `transition_planner.py`（entry/exit 几何 → 动态 3D route，发 `/initial_path` 语义的 Path）。
- `motion_safety_filter`（`/cmd_vel` → `/cmd_vel_safe`：楼梯 COMMIT 后切前进分量、实测 s 倒退>容差 → StopMove+fault+重新规划，**绝不改 pose**）。
- `floor_session` 抽象（sim backend 暂时内部实现 octomap 重启，manager 不写死任何后端操作）。
- `go2_hw_adapter` stub + `docs/REAL_ROBOT_PORTING.md`。

---

## 4. V2 实施顺序（对齐任务书 P0-P10）

P0 分支 ✅ → P1 审计 ✅（本报告）→ **P2 Test A/B**（设计见 §1.6）→ P3 go2 model error 快速取证（gzserver/spawn/URDF 解析日志 + gazebo_bridge set_model_configuration 时序；止损 1h；纯视觉/腿问题进 backlog，collision 未加载才阻塞）→ P4 msg/接口 → P5 safety filter → P6 插件清理为纯 adapter → P7 manager 重写状态机 → P8 floor_session 抽象 → P9 Depot 人工 GUI 验收（全程单速度链 SCAN→filter→adapter）→ P10 移植文档。

## 5. 风险与观察点

1. SCAN 自己的 3D voxel 滑窗图吃的是 mid360 原始点云（非 octomap 2.5D 投影），v1 时代的"楼梯下方格报 0.09"风险对 SCAN 不直接适用；A/B 实验直接实测回答。
2. `adjustGlobalTargetIfOccupied` 末端回拉可能截断楼梯出口段（出口点被判占据时）——Test A/B 重点观察项，必要时 route 末尾追加 post-exit 冗余点对冲。
3. `/initial_path` z 语义约定必须唯一：**route 点 z = 地面/踏步面 z，SCAN 内部统一 +body_height**（pathCallback 现状即此语义，bridge/planner 不得两边都加）。
4. grid_map 滑窗 z 半径 5m 对层高 2.86m 够用；楼梯全程在窗内。
5. Test B 中 route 重发频率要低（v1 的 5s resend 是 workaround）；mode3 下每条新 Path 都触发 REPLAN，重发风暴本身是干扰源。
