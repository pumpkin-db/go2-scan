# go2-scan

RAICOM 2026 北斗时空应用赛 **四足 SLAM 组** —— 宇树 Go2 + Livox MID360 自主全覆盖彩色扫描系统。

**核心任务**：10 分钟内自主扫描未知场地，产出**带颜色的 3D 点云**。
评分 = 20% 用时 + 80% 质量（完整度/清晰度/细节/精度），自主模式 +10，碰撞 -5，摔倒取消资格。

## 本仓库是什么

**自研胶水层 + 文档 + 仿真配置**（不是第三方源码，第三方见 [`third_party.md`](third_party.md)）。
核心算法一律用现成/已验证的开源实现，自己只做「修改 + 组合 + 胶水」。

```
┌─ 赛前：任务描述 ────────────┐
│  NaVILA 8B VLM（Int4，低频）│   ← 区域语义解析 + 答辩演示（辅）
└──────────┬─────────────────┘
           ▼
┌──────────────────────────────┐
│ L0  SLAM: FAST-LIO2 + MID360 │   ← 实时位姿 + 点云
│ L1  地图: Python VoxelMap    │   ← 稀疏字典 + 相机投影赋色
│ L2  规划: 探索 + A* + PP │   ← SCAN-Planner 局部规划
│ L3  执行: Go2 SDK2 Move()    │
│ L4  安全: 高程地形 + 看门狗  │   ← 摔倒=取消资格
└──────────────────────────────┘
```

## 目录结构

```
go2-scan/
├── docs/           # 调研 + 流程文档（含换场景标准流程）
├── simulation/     # 仿真胶水（自研）：launch / scripts / urdf / worlds
│   ├── launch_gazebo_sim.sh    # 一键启动脚本
│   ├── launch/                 # gazebo_sim.launch + default.rviz
│   ├── scripts/                # gazebo_bridge.py + scan_cloud_accumulator.py
│   ├── urdf/                   # go2_description.urdf（加 MID360）
│   └── worlds/                 # indoor_1.world（HPHS 场景，已修关节撞名）
├── tools/          # 工具脚本（world→PCD 转换、PGM→PCD、弓形→initial_path）
├── maps/           # 场景 ground-truth 点云（indoor_1.pcd）
└── third_party.md  # 第三方依赖清单 + 编译步骤
```

## 快速开始

1. 按 [`third_party.md`](third_party.md) clone + 编译 4 个第三方依赖。
2. 把 `simulation/` 下的胶水文件复制回 SCAN-Planner 对应位置（清单见 third_party.md）。
3. `bash simulation/launch_gazebo_sim.sh` —— 打开 Gazebo + RViz，Go2 狗 + MID360 实时扫描。
4. 换场景照 [`docs/40_Gazebo场景仿真_替换场景标准流程.md`](docs/40_Gazebo场景仿真_替换场景标准流程.md)。

## 当前进度

- ✅ Gazebo 场景仿真（indoor_1.world + Go2 + Livox MID360 → 实时点云 → SCAN-Planner → 狗动起来）
- ✅ 三个地图对照（map 完整场景 / scan_map 累积扫描 / occ_map 占据图）
- ⏳ 覆盖规划（尝试已知的探索算法）、赋色层、安全层、NaVILA 语言层 —— 待做

## 硬约束

1. 算法不自己写，只用现成/已验证实现（修改+组合+胶水）。
2. **ROS1 Noetic + Ubuntu 20.04**，不是 ROS2（JetPack 5.1.1 锁死）。
3. Orin NX 16GB 内存，VLM 只能 Int4 低频。
4. 摔倒=取消资格，地形安全是硬约束。
5. 赛中不能人为触碰机器狗，语言控制只用于赛前解析+答辩。
