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
