# go2-scan_2D：Linux/Orin NX 真机交接

> 面向下一位直接接入 Unitree Go2 + MID360 的 AI。标签：`[VERIFIED]` 已核实；`[CURRENT]` 当前仓库实现；`[TARGET]` 目标；`[TODO]` 待做；`[HISTORICAL]` 旧 Linux 交接资料事实，仅在当前真机复核后使用。**真机迁移前必须让 WSL2 侧 AI 根据最新 main 再检查/更新本文；本文不是永久有效配置。**

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

`[TARGET]` 可靠障碍应定义为相对局部地面：`local_ground+0.20 m` 到 `local_ground+0.90 m` 的点进入对应 XY occupied。旧 Linux 管线的 `robot_foot_init z≈0` 与 `[0.2,1.0]` slice 是 `[HISTORICAL]` 参数，不等于当前 main 已实现；当前 main 仍需重新审计 LIO world z、点云过滤、OctoMap 投影和 SCAN obstacle interpretation。

当前 2D 仿真侧使用 `/projected_map`，OctoMap resolution=0.2、occupancy z=`0.2..0.8`、range=6、hit/miss/max/min=`0.70/0.40/0.97/0.12`；这些是仿真基线，不能直接当真机参数。`[TODO]` 在 NX 上证明 ARiADNE 与 SCAN 是否消费同一障碍语义。

## 9. ARiADNE / SCAN 真机接口

ARiADNE `[CURRENT]` 需要二维 occupancy map 与 body pose，发布 exploration `/way_point`；main 已删除 TARE。SCAN `[CURRENT]` 需要 registered cloud、body pose、sensor pose 和 `/initial_path`，发布 `/cmd_vel`。当前仿真 body topic 是 `/quad_0/body_pose`、sensor topic 是 `/quad_0/lidar_pose`；真机 launch 必须通过 remap/config 对齐到实际 `Odometry`/TF，不能假设 `/quad_0/*` 存在。

真机桥接链仍是已有 `/cmd_vel → cmd_vel_bridge → SportClient`。确认唯一最终控制发布者、watchdog 和人工停止路径。

## 10. 真机安全红线

* 首次接入必须有人持遥控器并能立即切断自主命令；
* 先平地、低速、短时，再静态障碍，再自主探索；首次不要测试楼梯；
* bridge watchdog 超时必须 `StopMove()`，命令失联即停；
* 检查 Go2 自带避障与自主规划是否冲突；
* 未知 downward stair/drop = NO-GO；不要让机器人后退进入未知区域；
* 未确认时钟、TF、odometry/cloud frame、限速前，不允许放开速度或无人值守。

## 11. 分阶段 bring-up

| 阶段 | 动作 | PASS 条件；失败即停 |
|---|---|---|
| 0 | 网络、ROS master、时钟 | eth10/ROS_IP/时间明确 |
| 1 | MID360 raw lidar/IMU | `/livox/lidar`、`/livox/imu` 频率和 frame 正常 |
| 2 | 外部 LIO | `/Odometry`、`/cloud_registered` 持续输出 |
| 3 | TF/pose/cloud health | `map/odom/base` 链可解释，时间无 timeout |
| 4 | 只启动 2D map | 地图更新；不发送速度 |
| 5 | ARiADNE 观察模式 | 新 goal 正常但仍不动狗 |
| 6 | SCAN 观察模式 | `/cmd_vel` 合理、无持续 A*/TF 错误 |
| 7 | bridge 低速人工目标 | watchdog、StopMove、遥控接管有效 |
| 8 | 平地 autonomous exploration | 短时稳定后再延长 |

## 12. Runtime provenance

真机也必须执行：Git HEAD → NX 原生 rebuild → source 正确 workspace → 检查 `ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`LD_LIBRARY_PATH` → `rosnode info`/`rostopic info` → 必要时 `readlink /proc/<pid>/exe`、`/proc/<pid>/maps`、`sha256sum`。源码更新但运行旧 build 是已发生过的真实故障模式。

## 13. 常见坑与未完成项

常见坑：`/Odometry` 大写 O；cloud frame 误判；world cloud 重复乘外参；NX/MID360 不同步；ROS_IP/eth10 错；aarch64 与 x86_64 库混用；旧 workspace/旧 `.so` 抢优先级；`SportClient` 在 `ChannelFactory::Init` 前构造；watchdog 失效；LIO origin 被误当 absolute ground z；WSL2 frame/TF 直接照搬真机。

`[TODO]` 当前未完成：main 在 NX 真正部署；external LIO adapter 原生编译；ARiADNE+SCAN+OctoMap 真机组合；local-ground-relative 障碍保证；最终 MID360 外参；NX 频率/性能与速度限制；平地长时稳定性；真机 autonomous exploration。它们均不能写成已完成。

## 给下一位 AI 的第一条任务

不要立刻让狗自主移动。先根据最新 main 更新本文，再完成网络、时钟、MID360、LIO、TF 的只读健康检查。
