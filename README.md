# go2-scan

RAICOM 2026 北斗时空应用赛 **四足 SLAM 组** —— 宇树 Go2 + Livox MID360 自主全覆盖彩色扫描系统。

**核心任务**：10 分钟内自主扫描未知场地，产出**带颜色的 3D 点云**。
评分 = 20% 用时 + 80% 质量（完整度/清晰度/细节/精度），自主模式 +10，碰撞 -5，摔倒取消资格。

## 本仓库是什么

**算法源码 + 自研胶水 + 仿真环境 + 文档**，全部自包含（第三方源码 vendor 进仓库，见 [`third_party.md`](third_party.md)）。
核心算法一律用现成/已验证的开源实现，自己只做「修改 + 组合 + 胶水」。

```
┌─ 赛前：NaVILA 8B VLM（Int4，低频）→ 区域语义解析 + 答辩演示（辅）
│
实机主链路（仿真中 FAST-LIO2 用 go2_kinematic_sim 无误差积分替代）：
  FAST-LIO2 ──► TARE ──► SCAN-Planner ──► Go2 SDK2 Move()
  (SLAM定位)   (全局探索)  (局部避障轨迹)    (执行)
                            ▲
                 相机投影赋色 ──► scan_map（最终产出，80%分值）
```

## 目录结构

```
go2-scan/
├── algorithms/                        # ★ 算法源码（vendor，按层分，每算法独立 workspace）
│   ├── global_planning/
│   │   └── tare/                      #   全局探索决策（TARE，CMU）
│   ├── local_planning/
│   │   └── scan_planner/              #   局部轨迹规划（SCAN-Planner，SJTU，含 go2_description/map_generator）
│   └── mapping/
│       └── elevation_mapping/         #   高程图（ANYbotics + kindr 等依赖）
├── integration/
│   └── go2_bridge/                    # ★ 自研胶水（5 个桥脚本，纯 Python）
├── simulation/                        # 仿真环境
│   ├── cmu_env/                       #   仿真底座（velodyne/livox 插件 + go2 模型）
│   ├── launch_gazebo_sim.sh           #   一键启动脚本
│   ├── worlds/                        #   indoor_1.world（HPHS 场景，已修关节撞名）
│   └── elevation/                     #   elevation_mapping 配置（launch + yaml）
├── tools/                             # 工具脚本（world→PCD、PGM→PCD、弓形→initial_path）
├── maps/                              # 场景 ground-truth 点云（indoor_1.pcd）
├── docs/                              # 调研 + 流程文档
└── third_party.md                     # 第三方来源 + commit + 编译步骤
```

## 仿真配置

仿真 = **Gazebo 场景 + Go2 狗 + Livox MID360 传感器 → 实时点云 → SCAN-Planner → 狗动起来**。
核心思路：Gazebo 当**传感器模拟器**（狗身上的 MID360 边走边感知识别）。

### 组件（一条链路上的节点）

| 组件 | 作用 |
|------|------|
| **Gazebo** | 加载 `indoor_1.world` 场景 + Go2 狗 URDF（含 Livox MID360 插件），产出 `/mid360_points` |
| **go2_kinematic_sim** | 积分 `/cmd_vel` → `/quad_0/body_pose`（纯运动学，不是 Gazebo 物理，仿真替代 FAST-LIO2） |
| **closed_loop_controller** | 闭环控制器，下发 `/cmd_vel` |
| **gazebo_bridge** | 胶水：把 `body_pose` 同步回 Gazebo 狗模型（`set_model_state`）+ 发 `/quad_0/lidar_pose` |
| **scan_cloud_accumulator** | 胶水：把每帧 `/mid360_points` 变换到世界系体素累积 → `/scan_map` |
| **tare_bridge / tare_planner_node / tare_goal_bridge** | TARE 全局探索决策层（配准喂入 → 决策 → 前瞻航点转发给 SCAN-Planner） |
| **SCAN-Planner** | 局部规划核心（2.5D A* + B样条 + FSM），吃点云+位姿，出路径 |
| **robot_state_publisher / go2_gait_publisher** | 狗模型 TF + 腿关节步态（让 Gazebo 狗腿动起来） |
| **map_pub** | 发布 `indoor_1.pcd` → `/map`（场景完整点云 ground truth，对照用） |
| **elevation_mapping** | 2.5D 高程图（预留，供地形安全决策） |

### 话题数据流

```
                    ┌── /mid360_points（传感器坐标，frame=mid360）
Gazebo MID360 ──────┤
                    └── /quad_0/body_pose（Odometry，go2_kinematic_sim 积分）
                              │
                              ▼
                     gazebo_bridge（胶水）
                    ┌─────────┴─────────┐
                    │ set_model_state   │  同步 Gazebo 狗模型位姿
                    │ /quad_0/lidar_pose│  MID360 世界系位姿 = body_pose + lidar_z
                    └─────────┬─────────┘
                              ▼
        ┌─ tare_bridge → /registered_scan → tare_planner_node → /way_point
        │                                                    → tare_goal_bridge → /move_base_simple/goal
        ▼
  SCAN-Planner（cloud_is_world=false，用 lidar_pose 变换点云到世界系）
        │
        ▼ /cmd_vel
  go2_kinematic_sim ──→ /quad_0/body_pose ──→ 回到开头（闭环）
```

三条地图对照线（RViz 里同时看）：

| 名字 | 话题 | 来源 | 颜色 |
|------|------|------|------|
| `map` | `/map` | `indoor_1.pcd`（完整场景 ground truth） | 白 |
| `scan_map` | `/scan_map` | accumulator 累积的真实扫描 | 红 |
| `occ_map` | `/grid_map/occupancy` | SCAN-Planner 占据地图 | 灰 |

### 关键参数

| 参数 | 值 | 位置 | 说明 |
|------|-----|------|------|
| `lidar_z` | 0.2077 | launch → gazebo_bridge | MID360 相对 base 的 z 偏移（mount 0.17 + scan joint 0.0377） |
| `cloud_is_world` | `false` | launch → advanced_param | 点云是传感器坐标，用 `lidar_pose` 变换 |
| `downsample` | 2 | URDF 插件 | 每帧 10000 点（20000 采样降一半），密度/性能平衡 |
| `real_time_update_rate` | 100Hz | indoor_1.world | 物理频率（不影响点云和运动学，只影响接触检测） |
| `navi_mode` | 1 | launch | 1=闭环朝目标走（TARE 前瞻单点）；3=订阅 `/initial_path` |
| `start_delay` | 15s | tare_goal_bridge | 启动后静止 15s 只扫描，避免初始地图不全导致穿墙 |

### 环境变量（`launch_gazebo_sim.sh` 已封装）

启动前必须：
1. **清理 PATH 里的 conda/anaconda** —— 否则 cmake 找错 protobuf、rospy import yaml 失败。
2. `source /opt/ros/noetic/setup.bash` + SCAN-Planner 的 `devel/setup.bash`。
3. **手动补各 workspace 环境**（不 source setup，避免 catkin 挤掉 SCAN 的 CMAKE_PREFIX_PATH）：CMU / elevation / TARE 的 `devel` + `ROS_PACKAGE_PATH`，以及 go2_bridge 的 `integration/`。

## 仿真流程

### 首次搭建（一次）

按 [`third_party.md`](third_party.md) 编译仓库内 4 个 workspace（scan_planner / tare / elevation / cmu_env），无需 clone 第三方、无需复制胶水。

### 日常启动

```bash
bash ~/claude/raicom/go2-scan/simulation/launch_gazebo_sim.sh
```

会自动弹出 **Gazebo**（场景 + 狗 + MID360）和 **RViz**（三个地图 + 狗模型）两个窗口。

### 运行时的现象（对照预期）

1. Gazebo 里狗有腿步态摆动，MID360 是 mesh 在狗前方。
2. RViz 里 `scan_map`（红）随狗走逐步扩展，慢慢逼近 `map`（白，完整场景）。
3. 启动后狗**静止 15 秒**只扫描建图，之后 TARE 给前瞻航点、SCAN-Planner 朝目标走。

### 换场景

照 [`docs/40_Gazebo场景仿真_替换场景标准流程.md`](docs/40_Gazebo场景仿真_替换场景标准流程.md) 七步走：改 world → 编插件 → URDF 加传感器 → 写 bridge → 写 launch → world转PCD → 调参。

> ⚠️ `launch_gazebo_sim.sh` 和 `gazebo_sim.launch` 里有 `$HOME/claude/raicom/go2-scan/...` 硬编码路径，换机器需改成实际路径。

## 当前进度

- ✅ Gazebo 场景仿真（indoor_1.world + Go2 + Livox MID360 → 实时点云 → SCAN-Planner → 狗动起来）
- ✅ 三个地图对照（map 完整场景 / scan_map 累积扫描 / occ_map 占据图）
- ✅ TARE 全局探索决策层接入（前瞻航点 → SCAN-Planner）
- ⏳ 赋色层（相机投影 → 彩色点云，80% 分值）、安全层（高程地形 + 看门狗）、NaVILA 语言层 —— 待做
