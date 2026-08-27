# go2-scan

RAICOM 2026 北斗时空应用赛 **四足 SLAM 组** —— 宇树 Go2（EDU）+ Livox MID360 + Orin NX 16GB，
自主全覆盖扫描系统。**核心任务**：10 分钟内自主扫描完全未知的场地，产出**带颜色的 3D 点云**。

评分 = 20% 用时 + 80% 质量（完整度20/清晰度10/细节10/精度10），自主模式 +10，碰撞 -5，摔倒取消资格。

## 本仓库是什么

**算法源码 + 自研胶水 + 仿真环境 + 评估体系 + 文档**，全部自包含
（第三方源码 vendor 进仓库，来源/commit/编译见 [`third_party.md`](third_party.md)）。
硬规则：核心算法一律用现成/已验证的开源实现，自己只做「修改 + 组合 + 胶水」。

- **实时进度流水账**：[`PROGRESS.md`](PROGRESS.md)（写给接续者，按日期倒序，含全部已核实的坑与结论）
- **规则与硬约束**：见上级目录 `CLAUDE.md`
- **实机迁移准备**：[`docs/实机迁移准备.md`](docs/实机迁移准备.md)（FAST-LIO2/Point-LIO 接入 + 运动链路）

## 系统架构（当前主线）

```
Gazebo（传感器模拟器：MID360 点云 /mid360_points，世界系）
   │
   ├─ 建图/决策链（ARiADNE，全局「往哪探索」）：
   │    cloud_range_filter(6m) → sensor_scan_generation → octomap(res0.2)
   │    → /projected_map → rl_planner（MARMoT 系 RL 策略，官方权重未动）
   │    → /way_point → ariadne_goal_bridge → /initial_path
   │
   ├─ 局部规划（SCAN-Planner，SJTU，navi_mode=3 跟踪 /initial_path）：
   │    2.5D 投影 A* + B 样条 → closed_loop_controller → /cmd_vel
   │
   └─ 运动学仿真：/cmd_vel → go2_kinematic_sim → /quad_0/body_pose（仿真里替代 SLAM 里程计）
                （实机将换 FAST-LIO2 / Point-LIO 的 /Odometry，见实机迁移文档）
```

- 全局决策：**ARiADNE 为主力**（Depot ER 53.8%）；TARE 已封存（对照 ER 38.3%）。
- 仿真里程计 = `go2_kinematic_sim`（积分 /cmd_vel，无误差），实机由真实 SLAM 替换。

## 目录结构

```
go2-scan/
├── algorithms/                        # ★ 算法源码（vendor，每算法独立 workspace）
│   ├── global_planning/
│   │   ├── ariadne/                   #   ★主力：ARiADNE RL 全局探索（官方脚本+权重）
│   │   └── tare/                      #   TARE（CMU，已封存，仅对照）
│   ├── local_planning/
│   │   └── scan_planner/              #   ★SCAN-Planner 局部规划 + go2_kinematic_sim 仿真
│   │                                  #   + go2_description（狗模型）+ map_generator
│   └── mapping/
│       └── elevation_mapping/         #   ANYbotics 高程图（楼梯/地形用）
├── integration/
│   └── go2_bridge/                    # ★ 自研胶水包（桥脚本，纯 Python）：
│                                      #   gazebo_bridge / cloud_range_filter /
│                                      #   ariadne_goal_bridge / scan_cloud_accumulator /
│                                      #   stair_detector 等
├── simulation/                        # 仿真环境：launch 脚本、indoor_1 世界、cmu_env 底座
│   ├── launch_gazebo_sim.sh           #   ★一键启动（scene:=depot / 默认 indoor_1）
│   ├── run_benchmark.sh               #   一键无头跑分
│   └── elevation/                     #   elevation_mapping 配置
├── scenes/                            # ★ 场景统一管理（2026-08 起）
│   ├── indoor_1/                      #   env.sh（世界/出生点）
│   ├── depot/                         #   多层仓库（楼梯）：env.sh/scene.yaml/楼梯注册表/世界
│   └── gtools/                        #   场景工具（GT 生成、碰撞补丁等）
├── evaluation/                        # ★ 统一评估体系（Explore-Bench 式 ER + 地图质量 P/R/F/Chamfer）
│   └── results/                       #   历次跑分报告 + 算法对照表（README 内）
├── tools/                             # 评估器、探针、遥控驱动、GT 高程生成等
├── maps/                              # 场景 GT 点云
├── docs/                              # 调研 + 设计 + 流程文档
└── try_algorithm/                     # 评估工作区（历史调研笔记/论文笔记/候选代码）
```

## 仿真配置（从零到跑起来）

### 0. 系统要求

| 项 | 要求 | 说明 |
|---|---|---|
| 系统 | Ubuntu 20.04 + **ROS1 Noetic** | 本机为 WSL2（WSLg + NVIDIA GPU 直通，Gazebo/RViz 走 D3D12 硬件渲染） |
| 仿真器 | Gazebo 11 classic（`ros-noetic-gazebo-*`） | 不是新版 Gazebo/Ignition |
| Python | 系统 `/usr/bin/python3`（ROS ABI） | **conda base 不可用**（py3.13 进不了 ROS）；节点脚本统一显式 `/usr/bin/python3` |
| ARiADNE 决策层 | 专用 conda env `ariadne`：py3.8 + torch 2.3.1 cpu + scikit-image + rospkg | 已建好；`run_planner.sh` 用绝对路径调它（本机 `conda activate` 会段错误，勿改回） |
| CUDA | 编译 elevation_mapping 需要（本机 12.9） | 只编译用，运行时不依赖 |
| 其他 apt | grid_map 全套、glog、ros-noetic-gazebo-ros-control 系（备查） | 见 `third_party.md` |

### 1. 编译（首次，4 个 workspace）

`build/`、`devel/` 不入仓库（.gitignore），**新机器必须全部重编**。顺序不敏感：

```bash
# ① 局部规划（SCAN-Planner + go2_kinematic_sim + go2_description + map_generator）
cd ~/claude/raicom/go2-scan/algorithms/local_planning/scan_planner && catkin_make

# ② 全局探索 TARE（已封存，但 launch 仍会补它的环境，建议编）
cd ~/claude/raicom/go2-scan/algorithms/global_planning/tare && catkin_make

# ③ 高程图（CUDA）
cd ~/claude/raicom/go2-scan/algorithms/mapping/elevation_mapping && catkin_make

# ④ 仿真底座（velodyne 插件等，白名单编译）
cd ~/claude/raicom/go2-scan/simulation/cmu_env && catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"
```

注意：
- 编译前把 PATH 里的 conda/anaconda 剔掉（`launch_gazebo_sim.sh` 运行时有同款清理）。
- ARiADNE（`algorithms/global_planning/ariadne`）是纯 Python 包，**不需要编译**，
  靠 ROS_PACKAGE_PATH 找到即可。
- 改过 `go2_kinematic_sim.cpp` 等 C++ 后只需在 ① 里重跑 `catkin_make`。

### 2. 环境机制（启动脚本已封装，排障时需要知道原理）

`launch_gazebo_sim.sh` 自动完成三件事，手动起栈时照抄：
1. **剔 conda**：PATH 去掉 conda/anaconda/miniconda（否则 cmake 找错 protobuf、rospy 挂）。
2. **多 workspace 手动补环境**：只 `source` ROS Noetic + scan_planner 的 devel，
   其余（cmu_env / elevation / tare / ariadne / go2_bridge）**不 source devel**，
   手动往 `CMAKE_PREFIX_PATH` / `LD_LIBRARY_PATH` / `ROS_PACKAGE_PATH` 里加——
   因为 catkin 的 setup 会互相挤掉 CMAKE_PREFIX_PATH。
3. **单实例守卫**：检测到已有 gzserver 直接 exit 42 拒启（孤儿 gzserver 会连上
   新 master 发陈旧 /clock 与点云，污染整条链——历史两小时误诊的根源）。

### 3. 启动与参数

```bash
# 交互观察（Gazebo GUI + RViz）
bash simulation/launch_gazebo_sim.sh global_planner:=ariadne               # indoor_1（默认）
bash simulation/launch_gazebo_sim.sh scene:=depot global_planner:=ariadne  # Depot 多层仓库
```

参数开关（都是 `key:=value` 追加）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `scene` | 无（=indoor_1） | `depot` 时自动 source `scenes/depot/env.sh`：世界文件/出生点(-12,0,0.35)/GT点云，并**自动附加** `terrain_follow:=true terrain_source:=gt_file gt_elev_file:=…elev_gt.bin stair_detect:=true` |
| `global_planner` | `tare` | `ariadne`=主力；`tare`=封存对照；`none`=纯底盘+传感链（遥控/验收用） |
| `gui` / `rviz` | `true`/`true` | 无头跑分设 `gui:=false rviz:=false` |
| `remote_drive` | `false` | `true` 时跳过 closed_loop_controller（它无轨迹时 100Hz 发零速会稀释遥控指令）——遥控/直驱场景必开 |
| `live_score` | 跟随 rviz | RViz 实时评分文本（/live_score），官方评估器口径的轻量镜像 |
| `stair_detect` | `false`（depot 自动 true） | 楼梯检测节点：注册表橙箭头 + 高程图几何检出蓝箭头 |
| `terrain_follow` | `false`（depot 自动 true） | kinematic_sim 地形跟随 z（数据源 `terrain_source`：gt_file/elevation） |

### 4. 场景系统（`scenes/`）

每个场景一个目录 + `env.sh`（世界文件/出生点/附加参数），新增场景照
`docs/40_Gazebo场景仿真_替换场景标准流程.md` 走：

- `indoor_1`：单层，出生点 (-7.5, 0.5, 0.25)，GT = `maps/indoor_1.pcd`
- `depot`：多层仓库带楼梯，`scene.yaml` 内有**楼梯注册表**（entry/exit/yaw，
  2026-08-27 已按 GT 点云仲裁修正方向：沿 -y 爬升）；`gt/elev_gt.bin` 为
  地形跟随的 GT 高程源（`tools/make_gt_elev.py` 生成）

### 5. 观察与评分

RViz 显示组（`default.rviz`）：Planning（goal/optimal_traj/his_path）、
Mapping（projected_map/frontier/node/edge/occupancy/elevation_map）、
**Score_Stairs（/live_score 实时评分、/stairs_markers 楼梯箭头、/initial_path 目标路径）**。
三条地图对照：`/map`（GT 全场，白）、`/scan_map`（累积扫描，红）、
`/grid_map/occupancy`（占据栅格，灰）。

```bash
# 无头一键跑分：启动→spawn健康门→挂评估器→出报告→清理
bash simulation/run_benchmark.sh
# 报告落 evaluation/results/（JSON+MD+ER曲线）；指标定义见 docs/评估标准调研.md
# 楼梯专项验收（B 阶段）：
bash simulation/test_stair_climb.sh
```

### 6. 纪律（血泪教训，照做）

1. 起仿真前：`bash simulation/kill_all_sim.sh` **且** `pgrep -x gzserver` 复核为空。
2. 不用 `timeout` 包 launch 脚本（只杀外层，gzserver 孤儿化）。
3. 后台记录器加 `PYTHONUNBUFFERED=1`（rostopic echo 是 Python，杀进程丢缓冲）。
4. 脚本里写死的路径是 `$HOME/claude/raicom/go2-scan`，换机器要改。
5. 详细坑与事故复盘全在 `PROGRESS.md` / `临时问题.md`（本地），接手前先读。

## 常用命令

```bash
# 交互观察（Gazebo GUI + RViz 全显示组，含实时评分 /live_score）
bash simulation/launch_gazebo_sim.sh global_planner:=ariadne            # indoor_1（默认）
bash simulation/launch_gazebo_sim.sh scene:=depot global_planner:=ariadne  # Depot 多层场景

# 无头跑分（自动出报告到 evaluation/results/）
bash simulation/run_benchmark.sh

# 楼梯专项验收（B 阶段）
bash simulation/test_stair_climb.sh
```

启动脚本已封装：conda 清理、多 workspace 手动补环境、单实例守卫、spawn 健康门。

## 当前状态速览（详见 PROGRESS.md）

- ✅ ARiADNE 全链路跑通：indoor_1 ER 99.6%、Depot ER 53.8%（地面层天花板 57.5%）
- ✅ 统一评估体系落地（ER/路径长/判废线 + 地图质量 + 实时评分进 RViz）
- ✅ Depot 场景接入 + 楼梯五阶段计划的 A 阶段、GT 高程数据源
- ⏳ 楼梯爬升执行链（阶段 D）、赋色层（相机投影→彩色点云）、安全层（看门狗/限速）、实机迁移
- ⚠ 未决问题挂账见 PROGRESS.md 续15（狗腿乱飞 / Depot 探索慢启动 / 楼梯检测无效）

> ⚠ 脚本与 launch 里有 `$HOME/claude/raicom/go2-scan/...` 硬编码路径，换机器需改。
