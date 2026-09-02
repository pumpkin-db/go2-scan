# go2-scan_2D

Go2 的 ROS1 Noetic / Gazebo Classic 11 纯二维、单层自主探索基线——**从 clone 到 indoor_1 能跑的简洁复现说明**。

**当前项目定位：**

```text
Pure 2D
Single-floor
ROS1 Noetic
Gazebo Classic 11
ARiADNE + SCAN-Planner
默认场景 indoor_1
```

## 当前仿真链

```text
indoor_1
→ MID360 simulation
→ cloud_range_filter
→ sensor_scan_generation
→ OctoMap /projected_map
→ ARiADNE
→ /way_point
→ ariadne_goal_bridge
→ /initial_path
→ SCAN-Planner
→ /cmd_vel
→ go2_kinematic_model_plugin
→ Go2
```

当前**不包含**：

```text
TARE
楼梯/多楼层
elevation_mapping
terrain_analysis
physical hotel backend
```

## 当前验证状态

说明：

当前 main 已完成：

1. WSL2 Gazebo 仿真验证
2. Ubuntu 20.04 x86_64 Linux 仿真验证

Linux 验证环境：

- Ubuntu 20.04.6
- ROS Noetic
- Gazebo Classic 11
- x86_64

已验证完整二维自主探索链：

```text
MID360 simulation
→ cloud_range_filter
→ sensor_scan_generation
→ OctoMap /projected_map
→ ARiADNE /way_point
→ ariadne_goal_bridge /initial_path
→ SCAN-Planner
→ /cmd_vel
→ go2_kinematic_model_plugin
→ Go2 autonomous motion
```

强调：

- 该验证代表：**二维仿真链已稳定**。
- 不代表：**真机 NX 已经完成**（真机仍需按 `HANDOFF_LINUX_REAL_GO2.md` 分阶段 bring-up）。

## 环境要求

```text
Ubuntu 20.04
ROS Noetic desktop-full
Gazebo 11
Python 3.8（当前项目复现一致性要求）
Conda / Miniconda
```

## 环境与依赖

> 前提：已正确安装 ROS Noetic Desktop-Full；ROS 官方安装请参考 [ROS Noetic 文档](http://wiki.ros.org/noetic/Installation/Ubuntu)（本 README 不展开软件源 / key 配置）。

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-octomap-server \
  ros-noetic-octomap-ros \
  ros-noetic-octomap-msgs \
  ros-noetic-grid-map-msgs \
  build-essential cmake pkg-config python3-dev \
  libeigen3-dev libpcl-dev libopencv-dev \
  libarmadillo-dev libboost-all-dev python3-numpy
```

> Gazebo 11 / `libgazebo11-dev` 由 `ros-noetic-desktop-full` 提供，无需手装 `libgazebo-dev`（Ubuntu 20.04 无该包名）。

> `rosdep` 可作为可选推荐方式处理大部分 ROS 对应包，但不是必需。若需要：

```bash
sudo apt install python3-rosdep && sudo rosdep init && rosdep update
```

## ARiADNE Python 环境

> PyTorch 只用于 ARiADNE，不是 Gazebo / OctoMap / SCAN 的依赖。

```bash
conda create -n ariadne python=3.8 -y
conda activate ariadne

pip install torch==2.3.1 \
  --index-url https://download.pytorch.org/whl/cpu

pip install numpy matplotlib scikit-image
```

当前 `algorithms/global_planning/ariadne/src/rl_planner/run_planner.sh` 使用**固定的 ariadne Python 路径**（`/home/pumpkin-db/miniconda3/envs/ariadne/bin/python`）。若新机器 Miniconda 不在该项地址，需检查并修改 `run_planner.sh` 中的 Python 路径。

**Python 3.8 说明**：这是当前项目复现一致性要求（当前 launcher 依赖 ROS Noetic 的 Python 3.8 环境），不是「ROS Noetic 理论上只能用 3.8」。

## 首次编译

统一用仓库根目录变量，避免脆弱的相对跳转：

```bash
cd /path/to/go2-scan_2D
export GO2_ROOT="$(pwd)"
source /opt/ros/noetic/setup.bash

cd "$GO2_ROOT/simulation/cmu_env"
catkin_make \
  -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"

cd "$GO2_ROOT/algorithms/local_planning/scan_planner"
catkin_make
```

其中：

```text
algorithms/global_planning/ariadne
integration/go2_bridge
```

当前标准仿真不需要 catkin 编译。

## 启动

```bash
cd ~/claude/raicom/go2-scan_2D
bash simulation/kill_all_sim.sh
bash simulation/launch_gazebo_sim.sh scene:=indoor_1
```

启动器会自动：

```text
source ROS Noetic
source SCAN workspace
注入 cmu_env 路径
加入 ARiADNE / go2_bridge 的 ROS_PACKAGE_PATH
加载 scenes/indoor_1/env.sh
```

当前：

```text
SPAWN_Z = 0.32
```

（不要使用旧值 0.25——会撞地。）

## 成功标准

启动成功后至少应看到：

```text
/projected_map
/way_point
/initial_path
/cmd_vel
/quad_0/body_pose
```

Go2 应**持续自主探索运动**，`/projected_map` 应为**非空二维地图**。快速检查：

```bash
rostopic echo -n1 /way_point
rostopic echo -n1 /cmd_vel
rostopic echo -n1 /quad_0/body_pose
```

### 完整仿真 PASS 需要（更新）

以下**全部满足**才算完整仿真 PASS：

```text
Gazebo 正常启动
Go2 plugin 加载
/quad_0/body_pose 持续变化
/mid360_points 发布
/projected_map 非空
/rl_planner 运行
/way_point 发布
/initial_path 生成
/cmd_vel 输出
Go2 自主探索移动
```

以下现象**已验证不是阻塞**，可忽略：

```text
Spawn service failed
TF_REPEATED_DATA
KDL root link inertia
```

## 已解决的环境迁移问题

以下是一名 `clone → build → run` 新 Linux x86_64 机器实际踩过并已解决的坑。均为**环境 / workspace 问题**（不是 URDF、Go2 模型、OctoMap 或代码问题），新机复现时无需再排查这些方向。

### 1. Gazebo plugin 搜索路径

**问题**：新 Linux 环境首次运行时：

```text
/quad_0/body_pose  无数据
/quad_0/lidar_pose 无数据
/mid360_points     无数据
OctoMap 为空
```

**根因**：多 workspace 环境下，Gazebo plugin 搜索路径缺少 `algorithms/local_planning/scan_planner/devel/lib`，导致 `libgo2_kinematic_model_plugin.so` 无法加载 → Go2 模型插件失效 → 整条传感器链无输出。

**当前解决**：`simulation/launch_gazebo_sim.sh` 已保证 `GAZEBO_PLUGIN_PATH` 包含：

```text
scan_planner/devel/lib
cmu_env/devel/lib
```

> 注意：Gazebo 加载 `<plugin>` 用的是 `GAZEBO_PLUGIN_PATH`，不是 `LD_LIBRARY_PATH`。**不要重新排查 URDF、Go2 模型、Octomap。**

### 2. ARiADNE conda + ROS Python 隔离

**问题**：`rl_planner` 启动：

```text
ModuleNotFoundError: No module named 'rospkg'
```

**根因**：ARiADNE 运行环境 `/home/pumpkin-db/miniconda3/envs/ariadne`；conda Python 无法自动访问系统 ROS Python 包（`rospkg` 在 `/usr/lib/python3/dist-packages`，不在 `/opt/ros/noetic/...`，也不在 conda env 的 sys.path）。

**解决**：ariadne 环境安装 `rospkg==1.6.0`。

当前 ARiADNE 环境：

```text
Python  3.8
依赖    torch 2.3.1+cpu / numpy / matplotlib / scikit-image / rospkg
```

> **不要**把 `/usr/lib/python3/dist-packages` 加入 PYTHONPATH，也**不要**改 `run_planner.sh`。

### 3. ROS Noetic apt 源问题

**问题**：安装 `ros-noetic-grid-map-msgs` 提示 `Unable to locate package`。

**原因**：ROS1 Noetic 源被禁用（`/etc/apt/sources.list.d/ros-latest.list` 为 `#DISABLED#deb ...`）。

**解决**：恢复 Noetic apt 源后 `sudo apt update` 再安装。

> **长期提醒**：机器同时存在 ROS1/ROS2 时，必须确认当前工作环境对应的是 ROS Noetic（本机额外配了 `snapshots.ros.org/foxy/final` 的 ROS2 源，勿误认）。

## 已知事项

`live_score_rviz` 是可选评估节点，不属于核心探索链。当前 `live_score` 默认 `true`，但它依赖 `tools/evaluate_exploration.py`，而当前启动链没有显式保证 `tools/` 已加入 PYTHONPATH。

> 全新环境中 `live_score_rviz` 可能因 `evaluate_exploration` 不可导入而单独退出；这不代表 ARiADNE + SCAN 核心链失败。该路径问题后续应由启动器统一处理。

- **Known Limitation**：`algorithms/global_planning/ariadne/src/rl_planner/run_planner.sh` 仍硬编码 ariadne Python 绝对路径（`/home/pumpkin-db/miniconda3/envs/ariadne/bin/python`），新机需手动检查/修改（见「ARiADNE Python 环境」）。严格意义上尚未做到无需改动的 clone → build → run；未来宜将该项参数化。

## 真机

```text
真机迁移请看 HANDOFF_LINUX_REAL_GO2.md
WSL2 仿真细节请看 HANDOFF_WSL2_SIM.md
```

> 真机 external LIO 的 map z=0 不能默认等于物理地面；2D map 启动前必须按 Linux handoff 做 floor-z check。

## 状态与文档

```text
HANDOFF_WSL2_SIM.md
HANDOFF_LINUX_REAL_GO2.md
PROGRESS.md
third_party.md
```
