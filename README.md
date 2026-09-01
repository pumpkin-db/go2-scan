# go2-scan

RAICOM 2026 四足自主扫描项目：以 Unitree Go2、Livox MID360 和外部 LIO 为目标平台；仿真使用 ROS1 Noetic + Gazebo Classic 11。

本仓库包含 ARiADNE 全局探索、SCAN-Planner 局部导航、场景、仿真适配和实机接入胶水。第三方来源见 [third_party.md](third_party.md)，进度与已验证结论见 [PROGRESS.md](PROGRESS.md)，实机接口见 [docs/实机迁移准备.md](docs/实机迁移准备.md)。

## 当前状态

- `indoor_1` 是当前二维探索回归场景；ARiADNE → SCAN-Planner 可运行。
- `scenes/hotel_stairs/` 是默认三层多楼层 benchmark；Depot 只保留作历史回归。
- 当前默认 Gazebo 狗是**运动学身体 + 视觉步态**：用于二维探索、传感器与界面验证，不是物理四足仿真，不能作为楼梯接触/攀爬验收依据。
- 物理 Go2 路线位于 `simulation/physical_go2_ws/`：它复用 ROS1
  HIMLoco/`rl_sar`，由 Gazebo contact + PD torque 决定身体运动。它与默认
  运动学模型严格隔离；物理楼梯验收仍未通过。
- 多楼层 P1--P5 的控制/生命周期代码已存在，但历史 synthetic-z 产生的“完成”不等于几何一致的导航通过。不要将其当作 physical stair PASS。

## 当前仿真数据流

```text
/cmd_vel
  → go2_kinematic_model_plugin (Gazebo WorldUpdate)
  → Go2 model pose + /quad_0/body_pose + /quad_0/lidar_pose

/quad_0/body_pose
  → go2_gait_publisher → /joint_states
  → robot_state_publisher (RViz TF)
  → ModelPlugin (Gazebo visual joints)

/mid360_points (world-frame)
  → cloud/map bridge → /projected_map
  → ARiADNE → /way_point → ariadne_goal_bridge → /initial_path
  → SCAN-Planner → /cmd_vel
```

身体位姿的唯一写入者是 `go2_kinematic_model_plugin`；腿关节的唯一目标源是 `go2_gait_publisher`。旧的 `go2_kinematic_sim → gazebo_bridge → /gazebo/set_model_state` 链不能与当前主 launch 混用。

## 快速开始

首次编译至少需要当前 SCAN workspace 和传感器插件：

```bash
cd ~/claude/raicom/go2-scan/algorithms/local_planning/scan_planner
catkin_make

cd ~/claude/raicom/go2-scan/simulation/cmu_env
catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins;livox_laser_simulation;sensor_scan_generation"
```

交互启动二维探索：

```bash
cd ~/claude/raicom/go2-scan
bash simulation/kill_all_sim.sh
pgrep -x gzserver                    # 必须没有输出
bash simulation/launch_gazebo_sim.sh global_planner:=ariadne
```

`launch_gazebo_sim.sh` 从脚本自身位置推导仓库根目录，并打印 runtime provenance；不要用其它 checkout 的启动脚本。

常用场景：

```bash
# 默认：indoor_1 单层二维探索
bash simulation/launch_gazebo_sim.sh global_planner:=ariadne

# 三层 hotel 多楼层场景（当前只用于场景/接口开发）
bash simulation/launch_gazebo_sim.sh scene:=hotel_stairs global_planner:=ariadne

# Depot：仅历史回归
bash simulation/launch_gazebo_sim.sh scene:=depot global_planner:=ariadne
```

`kill_all_sim.sh` 会激进地终止匹配的 ROS/Gazebo 进程；运行前确认没有需要保留的其它 ROS 会话。

### 物理 Go2 / 三维楼梯开发

`launch_gazebo_sim_3D.sh` 只启动 hotel 场景、physical Go2、`ros_control`
和 HIMLoco；它不启动 ARiADNE、SCAN 或旧运动学 ModelPlugin。默认出生点是
`stair_test`，可显式选择 `exploration`：

```bash
bash simulation/setup_physical_go2_deps.sh
(cd simulation/physical_go2_ws && catkin_make --pkg go2_scan_physical_sim robot_msgs robot_joint_controller rl_sar go2_description)
bash simulation/kill_all_sim.sh
bash simulation/launch_gazebo_sim_3D.sh spawn_mode:=stair_test
```

默认 `auto_getup:=true`：hotel 与完整 13-link Go2 生成后解除暂停；`rl_sim`
等待 12 关节反馈和趴下状态稳定，再自动执行一次 keyboard `0` 等效 GetUp。
需要 `/cmd_vel` locomotion 时仍须按 `1`。该入口会检查解析到的
`go2_description`/`rl_sar` 必须来自 `physical_go2_ws`，防止与旧视觉模型同名包串用。

## Go2 视觉模型

当前内部模型位于：

```text
algorithms/local_planning/scan_planner/src/simulator/Utils/go2_description/
```

关键文件：

- `urdf/go2_description.urdf`：link、collision、12 关节、Gazebo plugin 与 kinematic 视觉腿设置。
- `planner/plan_manage/src/go2_kinematic_model_plugin.cpp`：唯一 body pose writer，发布 body/lidar pose，并写入 Gazebo 腿关节角。
- `planner/plan_manage/src/go2_gait_publisher.cpp`：唯一 `/joint_states` publisher，提供固定站姿和视觉 trot。
- `planner/plan_manage/launch/gazebo_sim.launch`：模型 spawn、默认平地 `init_z=0.32m`、RSP 与步态节点。

当前站姿下前脚 collision 最低点约为 base 下方 `0.31793m`，所以平地出生高度使用 `init_z=0.32m`。该数值只适用于此视觉模型和平地；它不提供真实接触/支撑。

## 场景

```text
scenes/
├── indoor_1/       单层二维探索基线
├── hotel_stairs/   三层酒店，两个标准楼梯组；默认多层 benchmark
├── depot/          历史多层回归场景
└── gtools/         场景生成/检查工具
```

hotel 场景的语义与出生点记录在 [scenes/hotel_stairs/README.md](scenes/hotel_stairs/README.md)。场景 metadata 仅用于仿真评价，不能进入楼梯感知、Supervisor 或导航决策。

## Runtime provenance：修改 C++/Gazebo 后必须做

Gazebo 会把已加载的 `.so` 保持在内存中；重编译不会替换已经运行的插件。每次改动 ModelPlugin 或其他 C++ Gazebo 插件后必须：

1. 停止完整仿真，确认 `pgrep -x gzserver` 为空；
2. 在当前 checkout 重编译对应 workspace；
3. 从当前 checkout 的 `simulation/launch_gazebo_sim.sh` 重启；
4. 以 `/proc/<gzserver-pid>/maps` 记录实际加载 `.so` 的绝对路径和 inode；
5. 对该产物执行 `sha256sum`，确认环境变量中没有旧 workspace 抢优先级。

看到 `(deleted)` 的 plugin mapping 即表示 gzserver 仍在使用重编译前的旧二进制，必须重启，不要继续调算法。

## 实机边界

go2-scan 不绑定 FAST-LIO：它只需要 LIO 提供 robot/base pose、world-frame registered cloud 与 TF。默认兼容历史 `/Odometry` 与 `/cloud_registered`，但应通过参数/remap 对接 FAST-LIO、Point-LIO 或其它 LIO。

真实控制链保持：

```text
SCAN / safe velocity → existing cmd_vel_bridge → unitree_sdk2 / CycloneDDS → SportClient → Go2
```

不应在本仓库新建 ROS2 控制 bridge 或替换 SportClient。首次实机测试只允许平地低速；启动前须检查 NX 与 MID360 时钟同步、TF、Odometry/点云频率、watchdog STOP 和人工遥控接管。

## 保留与废弃的仿真脚本

- 标准交互入口：`simulation/launch_gazebo_sim.sh`、`simulation/kill_all_sim.sh`。
- `simulation/run_benchmark.sh`、`simulation/spawn_go2.py`：旧无头评估工具，physical backend 接入前保留，勿与交互实例并行运行。
- `simulation/verify_run.sh`：ARiADNE 运行时诊断工具。
- 已删除：旧核弹重启、旧 ARiADNE 重启、synthetic-z Depot 楼梯验收和 run7 专项脚本；它们包含失效的路径、场景或几何假设。

## 下一步

下一步只在 `stair_test` 出生点验证 HIMLoco 对真实 hotel stair collision 的
第一 flight 能力。当前只完成 physical Go2 的可重复生成、自然落地和自动起立，
不得宣称楼梯 locomotion 已通过；也不得让 physical 与 kinematic backend 同时启动。
