# go2-scan_2D：Linux/Orin NX 真机交接

> 面向下一位直接接入 Unitree Go2 + MID360 的 AI。标签：`[VERIFIED]` 已核实；`[CURRENT]` 当前仓库实现；`[TARGET]` 目标；`[TODO]` 待做；`[HISTORICAL]` 旧 Linux 交接资料事实，仅在当前真机复核后使用。**真机迁移前必须让 WSL2 侧 AI 根据最新 main 再检查/更新本文；本文不是永久有效配置。**

## Linux x86_64 仿真验证结果

`[VERIFIED]`

记录：

当前 x86 Linux 开发机已经验证：

```text
Ubuntu 20.04
ROS Noetic
Gazebo 11
```

完整二维链：

```text
sensor simulation
→ mapping
→ ARiADNE
→ SCAN
→ velocity command
```

说明：

这是进入真实 Go2 前的**中间验证平台**（二维仿真链已稳定，但**不等于真机已通过**）。

## 1. 最终目标与边界

`[TARGET]`

```text
MID360 + IMU
 → external LIO
 → Pose/Odometry + registered/world cloud + TF
 → local-ground-relative 2D map
 → ARiADNE
 → exploration goal
 → SCAN
 → /cmd_vel
 → existing cmd_vel_bridge
 → unitree_sdk2 SportClient
 → Real Go2
```

go2-scan 不内置或绑定 FAST-LIO；FAST-LIO、Point-LIO 或其它 LIO 只要提供统一的 odometry/pose、registered cloud、TF 即可。`/Odometry`、`/cloud_registered` 是历史默认/示例接口，不是永恒硬编码。

## 2. 硬件与网络（历史已核实，启动前复核）

`[HISTORICAL][VERIFIED in old handoff]` Unitree Go2；Livox MID360；Jetson Orin NX 16GB；Ubuntu 20.04 / ROS Noetic；aarch64；`unitree_sdk2`、`Livox-SDK2`、`livox_ros_driver2`。历史网段：NX `eth10≈192.168.123.18`，MID360 `≈192.168.123.201`。每次以实际 `ip addr`、MID360 配置和路由为准，不能假设地址不变。

启动前只读检查：

```bash
uname -m
lsb_release -a
echo "$ROS_DISTRO"
ip addr
ip route
echo "$ROS_IP"
echo "$ROS_MASTER_URI"
```

`eth10` 是 Go2 控制通道，不能随意 down；Wi-Fi 可作 ROS/互联网接口，但 `ROS_IP` 必须选对实际接口。

## 3. 已验证的 cmd_vel 控制链

`[HISTORICAL][VERIFIED in old handoff]`

```text
ROS1 /cmd_vel
 → existing cmd_vel_bridge
 → unitree_sdk2 ChannelFactory/CycloneDDS
 → SportClient
 → /api/sport/request
 → Go2
```

旧 bridge 位于真机资产 `~/cmd_vel_bridge/`，**可能不在当前 2D main 仓库中，不能凭空假定已随仓库存在**。它的关键约束：先 `ChannelFactory::Init`，再构造 `SportClient`；默认 `_interface:=eth10`、10 Hz、`_cmd_timeout_s≈0.5`、速度/角速度 deadband、超时调用 `StopMove()`、启动可 `RecoveryStand()`、通常关闭 Go2 自带避障以免与自主规划冲突。保持人工遥控接管。

不要新建 ROS2/FastDDS 控制栈，不替换 SportClient。未来可在自主层和 bridge 间加 `/cmd_vel_safe`，但本轮不要大规模重构。

## 4. 真机编译策略

`[TODO]` 先在 NX 原生 workspace 重编，不复制 WSL2 的 `build/`、`devel/`、`install/` 或 `.so`。WSL2 通常是 x86_64；NX 是 aarch64，二进制/SDK 库不可混用。

```bash
source /opt/ros/noetic/setup.bash
# 根据 NX 实际 workspace 再 source
echo "$ROS_DISTRO"
```

历史 NX SOP（需按最新机器复核）：

* `livox_ros_driver2` 选择 ROS1：`-DROS_EDITION=ROS1`，必要时让 `package_ROS1.xml` 成为 `package.xml`；
* Orin NX 内存有限，优先 `catkin_make ... -j2 -l2`，OOM 时降到 `-j1 -l1`；
* 先编 livox driver，再生成消息头，最后全量 catkin；
* `unitree_sdk2` 使用 aarch64 library，检查 `LD_LIBRARY_PATH` 和 CMake 实际 SDK 路径；
* SCAN 的 C++ package 也必须在 NX 原生重编。

不要把 WSL2 仿真 Go2 plugin 或任何 x86_64 产物复制到 NX。

## 5. 外部 LIO 接口

当前 main 的 `algorithms/local_planning/scan_planner/src/planner/plan_manage/launch/fastlio_integration.launch` 是 `[CURRENT]` 外部 LIO 接口模板，不是 FAST-LIO 实现，也不启动模拟器/gait/cloud relay。参数默认：

```text
body_pose_topic=/Odometry
sensor_pose_topic=/Odometry
cloud_topic=/cloud_registered
cloud_is_world=true
sensor_type=lidar
navi_mode=1
```

参数可通过 roslaunch 覆盖；SCAN 接口需要 body pose、sensor pose、registered cloud、frame/`cloud_is_world`。旧真机资料 `[HISTORICAL][VERIFIED]`：FAST-LIO 约 10 Hz `/Odometry`（大写 O）、约 10 Hz world-frame `/cloud_registered`，原始 `/livox/lidar` 约 10 Hz、`/livox/imu` 约 200 Hz。接入时必须重新用 `rostopic info/hz/echo`、`tf_echo/tf_monitor` 证明语义，不能只相信名字。

特别防止重复变换：若 cloud 已是 world/registered，不能再把 Go2→MID360 外参重复乘一次；若 cloud 是 sensor frame，则必须有正确的 sensor pose/TF。`LIO origin` 不等于局部地面高度。

## 6. MID360 外参与 frame

### Body center 定义

`[VERIFIED]` 当前二维仿真 URDF 的 root link 名为 `base`，没有 `base_link` 或 `trunk`。四个 hip joint 相对该原点分别位于前/后 `x=±0.1934 m`、左右 `y=±0.0465 m`、hip 轴高度 `z=0`。因此 `base` 位于前后 hip 轴中点、机身左右中线和 hip 轴高度平面附近。

当前仿真中，`base` 同时是 Gazebo model pose、`/quad_0/body_pose`、SCAN double-cylinder planning center 和 simulated MID360 extrinsic 的起点。真机接入时：

```text
simulation base ≈ real-robot base_link semantic origin
```

这里是几何语义对应，不要求 frame 名相同。SCAN body center 不应取狗背、腹部、足端或 LiDAR 中心。

### 当前人工测量的真机 MID360 外参

`[CURRENT][MEASURED APPROXIMATION]` 当前机械安装的人工近似：

```text
base_link/body-center → MID360

x = +0.155 m
y = 0
z = +0.255 m

roll  = 0
pitch = -0.139626 rad   # -8°
yaw   = 3.141593 rad    # 180°
```

方向约定：MID360 `+X` 指向 Go2 `-X`（狗尾方向）；MID360 `+Z` 向 Go2 `+X`（狗头方向）倾斜约 8°。

这不是最终标定结果。真机正式运行前必须核实机械安装是否变化、ROS RPY 正方向、MID360 轴定义及 TF convention；不允许以 SCAN 源码中的历史硬编码 extrinsic 替代此真机外参。

### 仿真与真机外参必须区分

`[CURRENT]` 已验收二维仿真的外参为：

```text
simulation base → MID360
translation ≈ (+0.200, 0, +0.2077) m
rotation ≈ identity / body orientation
```

它与当前真机人工测量的 `(+0.155, 0, +0.255)`、RPY=`(0, -8°, 180°)` 不同。不要为了匹配真机而修改当前已验收二维仿真；真机阶段应在 real-robot TF / adapter 层正确表达真实外参。

`[TODO]` 真机正式 bring-up 前复核 `base_link→MID360` 6DoF 外参。

## 7. 时钟同步硬检查

`[HISTORICAL][VERIFIED in old handoff]` 旧 NX 与雷达时间差约 27 s 会造成 TF timeout、costmap/navigation 起不来、表面看似无 `/cmd_vel`。当前机器必须重新确认。目标建议误差 <2 s（至少远小于 TF tolerance）。历史做法是禁用自动 NTP 后用可信设备取时：

```bash
sudo timedatectl set-ntp false
# 再按现场可信时钟源校准；不要把旧密码写入脚本/文档
date +%s
```

确认 NX、MID360、LIO timestamp 一致后才继续。不同机器/网络不要盲目复用旧 OK3588 地址。

## 8. 真机二维地图语义

`[TARGET]` 可靠障碍应定义为相对局部地面：`local_ground+0.20 m` 到 `local_ground+0.90 m` 的点进入对应 XY occupied。旧 Linux 管线的 `robot_foot_init z≈0` 与 `[0.2,1.0]` slice 是 `[HISTORICAL]` 参数，不等于当前 main 已实现；WSL2 侧已审计（见 §9.2）：当前 OctoMap 为 absolute map z ∈ (0.20,0.80)，非 ground-relative，ground filter 并非通用 ground_z estimator（见 §9.3）；真机需重新确认 LIO world z、点云过滤、OctoMap 投影和 SCAN obstacle interpretation。

当前 2D 仿真侧使用 `/projected_map`，OctoMap resolution=0.2、occupancy z=`0.2..0.8`、range=6、hit/miss/max/min=`0.70/0.40/0.97/0.12`；这些是仿真基线，不能直接当真机参数。`[TODO]` 在 NX 上证明 ARiADNE 与 SCAN 是否消费同一障碍语义。

## 9. 真机地面高度 / LIO z 原点：2D 建图前硬检查

> 本节与第 8 节「真机二维地图语义」互补，回答一个独立的硬问题：**启动任何 2D 建图/探索前，必须先知道物理地面在 LIO map frame 里的 z。** 标签沿用本文头部定义。

### 9.1 历史真机事实（只作参照，不作当前保证）

`[HISTORICAL][VERIFIED in old handoff]` 旧 Linux/NX 管线（FAST-LIO + `/Odometry` + `/cloud_registered` + `robot_foot_init`）曾把：
```text
robot_foot_init z = 0
```
解释为「机器人足端高度 ≈ 当前地面」，并据此用固定 absolute-z 切片运行 RRT/move_base：
```text
ground + 0.2 ~ 1.0 m
```
**但这只是旧 FAST-LIO + 旧 TF 配置下的历史事实，不能作为当前新 main 或任何未来 LIO 的通用保证。** 旧 RRT/move_base 管线不属于当前架构，不要据此推断当前行为。

### 9.2 当前 WSL2 仿真已审计的事实

`[CURRENT][VERIFIED]` 当前 WSL2 indoor_1 仿真中 OctoMap：
```text
filter_ground        = true
base_frame_id        = world
frame_id             = map
map ↔ world          = identity   ← 仅当前 WSL2 indoor_1 仿真
occupancy_min_z      = 0.20
occupancy_max_z      = 0.80
```
**注意：`map↔world` 恒等只是当前 WSL2 indoor_1 仿真配置（由 launch 的 `map_world_bridge` 静态桥 `0 0 0` 保证），此事实不适用于未来真机 LIO——真机不能默认 `map == world`。** 当前 `/projected_map` 的二维障碍高度实际是 **absolute map/world z ∈ (0.20, 0.80)**，**不是** local-ground-relative。

当前使用的是 ROS Noetic 系统 `octomap_server`（非项目自定义版本，项目内无其源码）。其 `filterGroundPlane()` 用：
```text
PCL SACMODEL_PERPENDICULAR_PLANE + SAC_RANSAC
```
寻找**法向接近 z 轴的水平地面平面**。`[PARAMETER VERIFIED DEFAULT]` 下列关键参数为 ROS Noetic 系统 `octomap_server` 的**默认值**（当前 launch 未 override；仿真本轮已关闭，**非本轮 runtime 实测**，distance/plane_distance 未运行时复核）：
```text
ground_filter_distance       = 0.04   (ROS 默认值)
ground_filter_angle          = 0.15 rad
ground_filter_plane_distance = 0.07   (ROS 默认值)
```
当前仿真中它能工作，核心原因是 **indoor_1 地面 ≈ world z=0**——RANSAC 找到近水平平面后，还要求该平面接近 base-frame 的 z=0（`|d| < 0.07`）。

所以当前仿真**不是**「自动寻找任意高度的 local ground」，而是**「在 world z≈0 附近寻找地面」**。随后二维投影**仍独立使用** `absolute map z ∈ (0.20, 0.80)`。

### 9.3 当前 ground filter 不是通用 ground_z estimator

必须明确：当前 ground filter **不是**一个通用的 `ground_z` estimator。它内部虽然解出平面 `a*x + b*y + c*z + d = 0`，但：
* coefficients 只在函数内部使用；
* 不持久保存；
* 不通过 ROS topic/service 输出；
* `occupancy_min_z/max_z` 与这个 ground plane **完全独立**。

因此当前二维投影只是 absolute map-z 门控。

### 9.4 真机风险（分离两个问题）

`[TODO][IMPORTANT]` 未来真实 LIO：
```text
MID360 + IMU → external LIO → pose/cloud/TF
```
目前**绝对不能假设 `LIO map z=0 == physical floor`**。不同 LIO（FAST-LIO / Point-LIO / 其它）、不同初始化姿态、不同 TF 配置，都会导致 map z 原点不同。可能出现**两个独立问题**：

**问题 A：ground filter 失效。** 若真实地面 `floor_z = -0.30 m` 而仍用 `ground_filter_plane_distance ≈ 0.07`，系统 OctoMap ground filter 可能不会把该平面认作 ground。

**问题 B：二维障碍高度切片错误。** 即使 ground filtering 正常，`occupancy_min_z=0.20 / occupancy_max_z=0.80` 仍是 absolute map z，**不会**自动变成 `floor_z+0.20 ~ floor_z+0.90`。

必须把这两个问题**分开理解、分开验证**——不能用「ground filter 正常」推导「高度带正确」。

### 9.5 真机 Stage 3 的 FLOOR-Z CHECK

在 bring-up 阶段（见第 12 节表格）给 **Stage 3** 增加一个硬检查：在原「TF/pose/cloud health」之上确认地面的 z。

机器人应**静止、站在平坦地面、不发送自主速度**。检查 `/Odometry`、`/cloud_registered`、TF tree、cloud frame_id、body pose z，并**必须回答**：
```text
当前 physical floor 在 LIO map frame 中：floor_z ≈ ?
```
**禁止仅根据 frame 名推测**，必须通过真实点云 / TF 确认。同时记录：
```text
floor_z 是否稳定
短时间 z 漂移大小
机器人静止时 body pose z
MID360 高度是否与人工外参基本一致
```

### 9.6 2D Map Gate

在启动当前 OctoMap 二维地图之前，加一个明确 Gate：**必须已知 floor_z**。

* 若 `floor_z ≈ 0` 且稳定：当前 `filter_ground + absolute z slice` 才可能直接复用。
* 若 `|floor_z|` 明显大于 `ground_filter_plane_distance`，或 ground 与 map z=0 明显不一致：**STOP**，不要直接启动自主探索。**不要靠增大 `ground_filter_plane_distance` 粗暴解决。**

### 9.7 目标语义（尚未实现）

`[TARGET][TODO]` 最终二维语义仍是：
```text
physical local floor + 0.20 m ~ 0.90 m
```
范围内存在可靠障碍 → 对应 XY occupied。当前 main **尚未**实现完全 LIO-independent 的 ground-relative 语义，**不要写成已完成**。

### 9.8 若真机 LIO 的 floor_z ≠ 0（仅候选方向，不实施）

目前只记录候选处理方向，**不要确定实施**：
```text
方案1：进入二维地图前建立 floor-aligned frame，使当前楼层地面 z=0。
方案2：进入 OctoMap 前对点云做固定 z-offset，把当前楼层地面平移到 z=0。
方案3：根据实际 LIO/TF 配置调整对应高度门限。
```
当前项目是 **Pure 2D / Single-floor / Indoor / 基本平地**，暂时不要引入每帧 terrain mapping、elevation_mapping、复杂 local ground surface、楼梯地形估计。若只存在一个稳定的整体 z offset，优先使用**最简单的固定 floor reference**。

### 9.9 SCAN 单独复核（不展开）

`[TODO]` 即使 **ARiADNE `/projected_map`** 已实现正确的地面相对高度语义，**SCAN 当前仍直接使用 3D cloud**，其障碍高度 / footprint 语义必须在真机阶段**单独复核**。不要假设 `ARiADNE 正确 = SCAN 自动正确`。本轮不展开 SCAN 修改。

## 10. ARiADNE / SCAN 真机接口

ARiADNE `[CURRENT]` 需要二维 occupancy map 与 body pose，发布 exploration `/way_point`；main 已删除 TARE。SCAN `[CURRENT]` 需要 registered cloud、body pose、sensor pose 和 `/initial_path`，发布 `/cmd_vel`。当前仿真 body topic 是 `/quad_0/body_pose`、sensor topic 是 `/quad_0/lidar_pose`；真机 launch 必须通过 remap/config 对齐到实际 `Odometry`/TF，不能假设 `/quad_0/*` 存在。

真机桥接链仍是已有 `/cmd_vel → cmd_vel_bridge → SportClient`。确认唯一最终控制发布者、watchdog 和人工停止路径。

## 11. 真机安全红线

* 首次接入必须有人持遥控器并能立即切断自主命令；
* 先平地、低速、短时，再静态障碍，再自主探索；首次不要测试楼梯；
* bridge watchdog 超时必须 `StopMove()`，命令失联即停；
* 检查 Go2 自带避障与自主规划是否冲突；
* 未知 downward stair/drop = NO-GO；不要让机器人后退进入未知区域；
* 未确认时钟、TF、odometry/cloud frame、限速前，不允许放开速度或无人值守。

## 12. 分阶段 bring-up

| 阶段 | 动作 | PASS 条件；失败即停 |
|---|---|---|
| 0 | 网络 + 时钟（ROS master） | eth10/ROS_IP/时间明确 |
| 1 | MID360 raw lidar/IMU | `/livox/lidar`、`/livox/imu` 频率和 frame 正常 |
| 2 | 外部 LIO | `/Odometry`、`/cloud_registered` 持续输出 |
| 3 | TF/pose/cloud health + **FLOOR-Z CHECK** | `map/odom/base` 链可解释，时间无 timeout；静止平地不发速度，确认 `floor_z`（§9.5） |
| 4 | 只启动 2D map（确认高度语义后） | 地图更新；不发送速度；启动前需已知 `floor_z`（§9.6 Gate） |
| 5 | ARiADNE 观察模式 | 新 goal 正常但仍不动狗 |
| 6 | SCAN 观察模式 | `/cmd_vel` 合理、无持续 A*/TF 错误 |
| 7 | bridge 低速人工目标 | watchdog、StopMove、遥控接管有效 |
| 8 | 平地 autonomous exploration | 短时稳定后再延长 |

## 13. Runtime provenance

真机也必须执行：Git HEAD → NX 原生 rebuild → source 正确 workspace → 检查 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`LD_LIBRARY_PATH` → `rosnode info`/`rostopic info` → 必要时 `readlink /proc/<pid>/exe`、`/proc/<pid>/maps`、`sha256sum`。源码更新但运行旧 build 是已发生过的真实故障模式。

## 14. 常见坑与未完成项

常见坑：`/Odometry` 大写 O；cloud frame 误判；world cloud 重复乘外参；NX/MID360 不同步；ROS_IP/eth10 错；aarch64 与 x86_64 库混用；旧 workspace/旧 `.so` 抢优先级；`SportClient` 在 `ChannelFactory::Init` 前构造；watchdog 失效；LIO origin 被误当 absolute ground z；WSL2 frame/TF 直接照搬真机。

`[TODO]` 当前未完成：main 在 NX 真正部署；external LIO adapter 原生编译；ARiADNE+SCAN+OctoMap 真机组合；local-ground-relative 障碍保证；最终 MID360 外参；NX 频率/性能与速度限制；平地长时稳定性；真机 autonomous exploration。它们均不能写成已完成。

## 已知环境迁移经验

### Workspace plugin 管理

`[VERIFIED]`

多 workspace ROS / Gazebo 项目中，必须确认 `GAZEBO_PLUGIN_PATH` 包含：

```text
scan_planner/devel/lib
cmu_env/devel/lib
```

否则 `Go2 model plugin` 无法加载 → 传感器链整体无输出。

### Conda + ROS Python

`[VERIFIED]`

ARiADNE 使用独立 conda 环境。**不要假设** conda python 拥有 ROS Python 包。当前解决：

```text
ariadne env: rospkg==1.6.0
```

未来迁移机器时，优先检查：

```text
python 路径
PYTHONPATH
rospy
rospkg
```

## 真机阶段前状态说明

当前：

```text
二维仿真： [VERIFIED]
真机：     [TARGET]
```

尚未验证：

```text
Orin NX 环境
aarch64 重新编译
MID360 真实驱动
外部 LIO
TF
floor-z
cmd_vel_bridge
SportClient
```

**不要因为 Linux 仿真通过，直接认为真机可运行。**

## 下一位 AI 第一任务

进入 NX 前：

1. 阅读 README
2. 阅读 HANDOFF_LINUX_REAL_GO2
3. 确认当前 main 状态
4. 不恢复历史 V2 / TARE / multifloor 路线
5. 按真机 bring-up 阶段执行
