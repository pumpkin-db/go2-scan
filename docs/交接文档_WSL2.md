# Linux/NX 侧开发现状交接文档（给 WSL2 的 AI）

> 整理日期：2026-08-17。作者：在纯 Linux（`/home/pumpkin-db`，有真机）一侧工作的 AI。
> 读者：在 Windows WSL2 一侧工作的 AI（你已调研/跑通 SCAN-Planner，并写过 `SCAN_Planner_调研与交接.md`）。
> **纪律**：本文只列「已核实 / 已确定」的信息；未核实的一律标「待验证」，不要当成事实。配置参数的**当前实际值以本包附件里从 NX 拉出的真实文件为准**（文中标注的值是拉取时的快照，若与附件不符以附件为准）。
> **自包含说明**：你在另一台机器，读不到 `/home/pumpkin-db`。本文已把关键命令/参数内联，源码与配置文件放在同目录 `附件/` 里，直接用即可。

---

## 0. 一分钟摘要

Linux/NX 侧用**真机**把 `Go2_frontier_based_exploration` 的 **2D 前沿探索管线**调试到了「大半可用」，并沉淀出几样**与具体规划算法无关、换任何 planner 都要用**的可复用资产——这几样恰好回答了你交接文档 §5 里的开放问题：

1. **`cmd_vel_bridge`**（✅ 真机验证过）：ROS1 `/cmd_vel` → unitree_sdk2 SportClient（CycloneDDS）直发 Go2。**这就是你 §5.1「cmd_vel→狗的桥怎么做」的答案**，源码在 `附件/cmd_vel_bridge/`。
2. **FAST-LIO 实机接线**（✅ 真机验证过）：里程计话题是 **`/Odometry`（大写 O）**、点云 **`/cloud_registered`**、TF 帧 `robot_foot_init`/`body_foot`/`base_footprint`。**回答你 §5.2「实机 SLAM 话题细节」**。
3. **时钟同步坑 + 修复**（✅ 已解决）：NX 与雷达时钟差 ~27s 会直接导致 move_base 起不来；你的交接文档**完全没提这个**，实机必踩，务必看 §2.3。
4. **NX 环境 SOP**：`ros1/ros2-only/ros12` 别名、三步 `catkin_make`、`-DROS_EDITION=ROS1`、`-j2 -l2` 防 OOM、`BASE_TYPE=GO2`、aarch64 `LD_LIBRARY_PATH`——迁实机必读，见 §3。

**导航/控制链路已在真机证明是通的**：时钟同步修好后，`whole.launch` + 桥起来，狗能朝目标移动（用户实测，只是嫌慢→调了速度参数）。所以 TF、move_base、`/cmd_vel`→桥→Go2 这条链**不用再怀疑**。真正没收尾的是**探索覆盖质量**——RRT 前沿决策不行（扫不全面），这正是要换 SCAN-Planner/覆盖规划的原因。`/odom` remap 只是个**不影响移动**的小配置项（见 §4.2/§6），且**这类实机小毛病不影响你跑仿真**，你可以直接开始仿真。

两个仿真（2D Gazebo 前沿、SCAN-Planner）本文都写了（§5），但照你的说法**它们大概率都不是最终结果**、你还会试新算法——所以本文把它们当「参考与验证手段」，不当结论；真正值得你带走的是上面 1~4 这些算法无关的资产。

---

## 1. 两条开发线的关系（先对齐背景）

| | Linux/NX 侧（我） | WSL2 侧（你） |
|---|---|---|
| 有无真机 | ✅ 有 Go2 + MID360 + Orin NX | ❌ 只有仿真 |
| 主线 | 2D 前沿探索管线真机调试 | 2D 仿真 → 转向 SCAN-Planner |
| 产出 | `cmd_vel_bridge`、FAST-LIO 实机接线、时钟同步修复、NX 环境、一套调好的参数 | SCAN-Planner 调研/编译/默认仿真、`fastlio_integration.launch` |

**交汇点（可迁移资产）**：`cmd_vel_bridge`、FAST-LIO 接线、时钟同步、NX 环境 SOP、参数集。这些**与规划器无关**——无论你最后用 SCAN-Planner、还是别的算法，只要还是「FAST-LIO 出里程计/点云 + 某 planner 出 `/cmd_vel` + 桥发给狗」这个骨架，这几样都直接复用。

**关于 2D 前沿管线**：你在交接文档里把它标为「大概率弃用」（纯 2D、不赋色）。这个判断我认同——**但请注意**：弃用的是「RRT 前沿决策 + 2D 栅格」这一层；而 Linux 侧沉淀的桥/SLAM 接线/时钟/环境**不属于这一层**，依然有效，别一起扔了。

---

## 2. 已验证的核心成果（算法无关，本文重点）★

### 2.1 `cmd_vel_bridge` —— 回答你 §5.1「cmd_vel→狗的桥怎么做」

- **是什么**：纯 ROS1 的 C++ 桥节点。roscpp 订阅 move_base 的 `/cmd_vel`（`geometry_msgs::Twist`），用 **unitree_sdk2 的 `SportClient`** 通过 **CycloneDDS 直发 Go2**（`/api/sport/request`，10Hz），**完全不经过 ROS2 / ros1_bridge / Fast-DDS**。
- **为什么自写桥**：旧路线 `ros1_bridge(Fast-DDS) → ROS2 control` 有四个根因，全是硬伤：
  1. Fast-DDS 2.x 的 SHM 传输在 aarch64 上泄漏 ~14GB 内存（之前用 `FASTDDS_SHM_TRANSPORT_DISABLED` 这类**错误的环境变量名**从未生效；正确做法是 XML profile `FASTRTPS_DEFAULT_PROFILES_FILE` 禁 SHM）；
  2. NX 上 CycloneDDS 与 Foxy rmw 版本不匹配，`ros2 node list` 直接段错误；
  3. `control_bridge.launch.py` 桥接规格类型串不一致；
  4. 架构上 3 跳干 1 跳的活。
  `cmd_vel_bridge` 把这四个根因**全部移出关键路径**。
- **位置（NX 上）**：`~/cmd_vel_bridge/`，源码 `cmd_vel_bridge.cpp`，二进制 `build/cmd_vel_bridge`。**本包附件 `附件/cmd_vel_bridge/` 有 `.cpp` + `CMakeLists.txt`。**
- **关键实现要点（复用/改写时务必注意）**：`SportClient` 必须在 `ChannelFactory::Init` **之后**构造，否则段错误——所以代码里用 `std::unique_ptr<SportClient>`，在 init 完成后再 `make_unique`。这是原始值类型成员默认构造导致 segfault 的修复点。
- **完整参数**（私有命名空间 `~`，命令行用 `_前缀`；以附件 `cmd_vel_bridge.cpp` 为准）：

  | 参数 | 默认 | 作用 |
  |---|---|---|
  | `_interface` | `eth10` | CycloneDDS 绑定的网卡。**eth10 是 NX↔Go2 有线控制通道，绝对不能 down** |
  | `_hz` | `10.0` | 转发频率（Hz） |
  | `_cmd_timeout_s` | `0.5` | **安全看门狗**：`/cmd_vel` 超过此时间没新消息 → 发 `StopMove()` |
  | `_threshold_lin` | `0.05` | 线速度死区（m/s），三分量都低于死区 → `StopMove()` |
  | `_threshold_ang` | `0.05` | 角速度死区（rad/s） |
  | `_auto_stand` | `true` | 启动时自动 `RecoveryStand()`（狗自动站起） |
  | `_stand_settle_s` | `2.0` | 站立后稳定等待秒数 |
  | `_disable_avoid` | `true` | 关闭 Go2 自带 AI 避障（探索时由 move_base/planner 负责避障） |

- **行为逻辑**（10Hz 定时器）：没收过 cmd_vel → 静默；cmd_vel 年龄 > `cmd_timeout_s` → `StopMove()`；三分量都 < 死区 → `StopMove()`；否则 `Move(vx, vy, vyaw)`。这个看门狗是实机安全兜底，接 SCAN-Planner 时保留。
- **验证状态**：✅ 已实测——狗走 3m 直线后停住，底层控制完全打通。启动后终端打印 `[bridge] up hz=… auto_stand=… disable_avoid=…` 即正常。
- **编译**（在 NX，`ros1` 之后）：
  ```bash
  cd ~/cmd_vel_bridge && mkdir -p build && cd build && cmake .. && make -j2
  ```
  ⚠️ **这个桥是 NX 专属，别在 WSL2 里编**：附件 `CMakeLists.txt` 里写死了 `UNITREE_SDK_ROOT=/home/unitree/unitree_sdk2` 且链接 **aarch64** 的 `unitree_sdk2/ddsc/ddscxx`——它依赖实机网卡 eth10 直连 Go2，只能在 Orin NX（aarch64）上编译运行。WSL2 侧仿真阶段用不到它；迁实机时直接拷 `cmd_vel_bridge/` 整个目录到 NX 再 `cmake .. && make -j2` 即可（若 unitree_sdk2 路径不同，改 CMakeLists 第 18 行）。
- **启动命令**（在 NX，等 `whole.launch` 起好约 25s 后）：
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  ~/cmd_vel_bridge/build/cmd_vel_bridge _interface:=eth10
  ```
- **对你的意义**：你 §5.1 列的候选 (a)「unitree_sdk2 自写桥节点」就是这个，且已在真机验证。你接 SCAN-Planner 时，SCAN-Planner 出的 `/cmd_vel` 直接喂给这个桥即可（SCAN-Planner 也是 `geometry_msgs::Twist`，接口一致）。

### 2.2 FAST-LIO 实机接线 —— 回答你 §5.2「实机 SLAM 选型/话题细节」

- **选型**：单层场地用现有 **FAST-LIO（FAST-LIO2 系）** 足够，已在真机跑通建图 + 里程计。你 §5.2 担心的「`/Odometry` 频率、`/cloud_registered` 的 frame_id 与坐标系语义」实机确认如下：
- **关键话题**：

  | 话题 | 类型 | 频率 | 说明 |
  |---|---|---|---|
  | `/Odometry` | nav_msgs/Odometry | ~10Hz | **注意是大写 O**，不是 `/odom`。FAST-LIO 的里程计 |
  | `/cloud_registered` | sensor_msgs/PointCloud2 | ~10Hz | 配准后的 3D 点云，**世界系** |
  | `/livox/lidar` | livox CustomMsg | 10Hz | livox_ros_driver2 原始雷达 |
  | `/livox/imu` | sensor_msgs/Imu | 200Hz | livox_ros_driver2 IMU |

- **TF 帧**：
  - FAST-LIO 侧：`robot_foot_init`（地图/里程计原点帧；**z=0 = 机器人足端高度 ≈ 地面**，octomap 切片以此为基准）、`body_foot`（机体）。
  - move_base 期望：`map` / `odom` / `base_footprint`。
  - 桥接（`go2_SLAM/go2_slam/launch/build_map.launch` 里的两个 static_transform_publisher）：
    - `tf_pub_1`：`base_footprint → body_foot`（平移 0.01/0.01/0.1）
    - `tf_pub_2`：`map → robot_foot_init`（平移 0.01/0.01/0.01，近似重合）
  - **TF 状态**：2D 管线的 TF 链（`map→robot_foot_init`、`base_footprint→body_foot`）**已被"狗实机朝目标移动"证明可用**——move_base 需要 `map→base_footprint` 才能规划，狗能动说明这条链通。早先"只列出 3 帧"是时钟同步还坏着时记的，已不作数。**唯一要留意**：你接 SCAN-Planner 时它期望的 frame 名（real 模式 `/LIO/odom_vehicle`、`/LIO/odom_imu`，或 fastlio_integration 里 remap 到 `/Odometry`）与这里的帧名不同，接线时按 §5.2 对齐即可，不是"TF 坏了"。
- **数据流**：
  ```
  MID360 (192.168.123.201)
    │ /livox/lidar + /livox/imu            ← livox_ros_driver2
    ▼
  FAST-LIO (go2_SLAM)
    │ /cloud_registered (世界系3D) + /Odometry + TF
    ▼
  octomap_server → /projected_map (2D栅格)   + pointcloud_to_laserscan → /scan
    ├─→ RRT 检测器 → filter.py → assigner.py → move_base goal
    ▼
  move_base → /cmd_vel → cmd_vel_bridge → Go2
  ```
- **对你的意义**：你在 `fastlio_integration.launch` 里把 SCAN-Planner 的 `/grid_map/body_pose` 与 `/grid_map/sensor_pose` 双 remap 到同一条 `/Odometry`、`/grid_map/cloud` remap 到 `/cloud_registered`、`cloud_is_world:=true`——这个接法与上面实机确认的话题语义**一致**（`/cloud_registered` 确实是世界系，pose 低频也 OK），仿真已验证，真机按此接即可，只需复验 TF 帧名与频率。

### 2.3 时钟同步 —— 实机必踩坑，你的文档没提

- **现象**：雷达/FAST-LIO 时间戳比 NX 系统时钟**慢约 27s**，move_base costmap 的 transform tolerance 仅 **0.5s** → costmap 持续 `transform timeout` → **move_base action server 起不来** → assigner 卡死在 `wait_for_server()`。表象是「探索节点都在、但狗不动、/cmd_vel 没输出」。
- **根因**：NX 的 `systemd-timesyncd`（NTP）会**自动把手动设置的时钟拉回**，导致手动同步失效/复发。
- **约束**：NX 与雷达时间差必须 **<10s**（建议 <2s）。
- **解决（✅ 已验证）**，每次实机启动前在 NX 执行：
  ```bash
  sudo timedatectl set-ntp false                                   # 禁用自动同步（每次都要跑）
  sudo date -s @$(ssh root@192.168.123.30 'date +%s')              # 用 OK3588 的时钟校准 NX
  date +%s; ssh root@192.168.123.30 'date +%s'                     # 验证：两值差 <2s 即同步成功
  ```
  - 「永久 disable timesyncd」试过但**问题复发过**，所以现在每次启动都跑上面第一行。
  - NX 上有脚本 `/usr/local/bin/sync_clock.sh`（已拉入附件）：它**只做 `date -s` 那一步**（ssh 取 OK3588 时间戳再 `sudo date -s @{}`），**不含 `timedatectl set-ntp false`**——所以仍需先手动禁 NTP，再跑 `sudo /usr/local/bin/sync_clock.sh`。
- **注意**：这里校准源是 OK3588（`192.168.123.30`，密码 `cnugis123`）。如果你那边雷达/OK3588 的网段或 ssh 有变，以能取到一个可信时钟为准。

---

## 3. NX 环境（迁实机必读）

- **硬件/系统**：Orin NX 16GB，JetPack 5.x / Ubuntu 20.04，**aarch64**，内核 `5.10.104-tegra`。用户 `unitree`，密码 `123`。
- **网络**：
  - `eth10` = `192.168.123.18`（静态），NX↔Go2 本体**有线控制通道，绝对不能 down**（`cmd_vel_bridge _interface:=eth10` 走它发 `/api/sport/request`）。
  - `wlan0`：USB WiFi 网卡（磊特 MT7612U，内核自带 `mt76x2u.ko` 免驱），可连路由器 `GL-MT3600BE-bf8-5G`（密码 `cnu12345678`，DHCP 保留 MAC `fc:22:1c:30:01:bc`）或手机热点。eth10(123 段) 与 wlan0 网段不重叠可共存，默认路由只走 wlan0。
  - **启动 ROS 前必须设 `ROS_IP`**：
    ```bash
    # WiFi 场景：取 wlan0 实际 IP
    export ROS_IP=$(ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
    # 有线直连场景：
    export ROS_IP=192.168.123.18
    ```
- **ROS 环境激活（NX 不自动加载 ROS，靠 bashrc 别名手动激活）**：

  | 别名 | 加载内容 | 用途 |
  |---|---|---|
  | `ros1` | Noetic + `~/catkin_ws/devel` | 编译/运行 ROS1 |
  | `ros2-only` | Foxy + `~/workspace_ros2/install` | 编译 ROS2（⚠️ 别取名 `ros2`，与 CLI 冲突） |
  | `ros12` | 双环境 | 旧 ros1_bridge 路线用；**cmd_vel_bridge 路线只需 `ros1` 级** |

  bashrc 关键行（`ros1` 别名示例 + 两个必需 export）：
  ```bash
  export LD_LIBRARY_PATH=~/unitree_sdk2/thirdparty/lib/aarch64:$LD_LIBRARY_PATH   # ⚠️ aarch64 不是 x86_64
  export BASE_TYPE=GO2                                                            # ⚠️ move_base 靠它加载机器人参数
  alias ros1='source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && echo "ROS1 noetic ready"'
  ```
  验证环境是否激活：`echo $ROS_DISTRO`（空 = 未激活）。
- **编译 SOP（ROS1 `catkin_ws`，先 `ros1`，三步顺序不能乱）**：
  ```bash
  # 步骤1：先单独编 livox_ros_driver2（FAST-LIO 依赖它）
  catkin_make -DROS_EDITION=ROS1 -j2 -l2 --pkg livox_ros_driver2
  # 步骤2：生成消息头文件
  source ~/catkin_ws/devel/setup.bash && cd ~/catkin_ws/build
  make fast_lio_generate_messages && make go2_slam_generate_messages
  # 步骤3：全量编译
  cd ~/catkin_ws && source ~/catkin_ws/devel/setup.bash
  catkin_make -DROS_EDITION=ROS1 -j2 -l2
  ```
  - ⚠️ `-DROS_EDITION=ROS1` **必须加**（否则 livox driver 误走 ROS2 分支）。
  - ⚠️ `-j2 -l2` 限 2 核防 OOM（16GB 内存不够大项目全速编）；OOM 就降 `-j1 -l1` 或临时加 swap。
  - livox_ros_driver2 需 `ln -sf package_ROS1.xml package.xml` 选 ROS1 版。
- **RRT Python 节点两处必修复**（否则报 `cannot locate node of type [filter.py]` / `No module named 'functions'`）：
  1. `go2_rrt_exploration/CMakeLists.txt` 的 `catkin_package()` 后加 `catkin_install_python(PROGRAMS scripts/filter.py scripts/assigner.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})`；
  2. `filter.py`/`assigner.py` 开头加 `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))`。
  3. ⚠️ **关键坑**：catkin 装的是**副本不是软链**，`~/catkin_ws/devel/lib/go2_rrt_exploration/` 下的 `filter.py`/`assigner.py` 副本也要同步 patch，否则改了源码不生效。
- **关键路径**：`~/catkin_ws`（ROS1 主工作区，已编译）、`~/cmd_vel_bridge`（桥）、`~/unitree_sdk2`（已编译）、`~/Livox-SDK2`（已编译）、`~/workspace_ros2`（旧 ros1_bridge 路线用，cmd_vel_bridge 路线已不用）。
- **NX 侧 MID360_config.json**：`host=192.168.123.18`、`lidar=192.168.123.201`，与当前 123 网段一致，正确（网段迁移是你做的，此处仅确认状态）。

---

## 4. 当前 2D 前沿探索管线现状

### 4.1 数据流
```
MID360 (192.168.123.201)
   │ /livox/lidar + /livox/imu
   ▼
FAST-LIO (go2_slam)
   │ /cloud_registered (3D) + /Odometry + TF
   ▼
octomap_server → /projected_map (2D栅格)
   │
   ├─→ RRT (global_rrt_detector + local_rrt_detector) → /detected_points
   │      → filter.py (MeanShift聚类, scikit-learn) → /filtered_points
   │      → assigner.py (收益计算) → move_base goal
   ▼
move_base (global_planner + TEB) → /cmd_vel
   ▼
cmd_vel_bridge (C++ + unitree_sdk2) → /api/sport/request → Go2
```

### 4.2 状态表

| 状态 | 事项 | 说明 |
|---|---|---|
| ✅ 已解决 | 控制链路 | `cmd_vel_bridge` 绕开坏掉的 ROS2 桥，实测 3m 直线 |
| ✅ 已解决 | 时钟同步 | 禁 NTP + `date -s`，move_base 正常起 |
| ✅ 已解决 | 3-8m 假黑点 | octomap z-slice 调为 [0.2, 1.0] + sensor_model |
| ✅ 已解决 | filter/assigner `No module named 'functions'` | sys.path 修复（src + devel 副本都改） |
| ✅ 已通 | TF / move_base / `/cmd_vel` 链 | **狗已实机朝目标移动**，证明 TF、costmap、move_base、桥整条链可用。早先"TF 只 3 帧"的疑虑是时钟同步还坏着那阵子记的，时钟修好+狗能动后已不成立 |
| 🔧 小优化(不阻塞) | `/odom` 话题名不一致 | FAST-LIO 发 `/Odometry`（大写 O），TEB 配置写死 `odom_topic: odom`（附件 `teb_local_planner_params.yaml` 第 3 行）。**狗照样能动**，说明没阻塞；想更严谨可传 `odom_topic:=/Odometry`/改 yaml/写 relay，但不做也不影响跑、更不影响仿真 |
| ⚠️ 真·短板 | 探索覆盖质量 | RRT 前沿决策扫不全面、"已探索区不回访"，不适合赋色点云扫描。**这是唯一实质性缺口**，也是迁移 SCAN-Planner + 覆盖规划的动机 |
| 🔧 非阻塞 | TEB 启动警告 | `max_vel_x_backwards`/`penalty_epsilon` 冲突、`inscribed radius`——不影响运行 |

> **如实说明"狗能动"的依据与边界**：我直接核实的是——时钟同步修好后 `whole.launch`+桥起来，move_base action server 正常、无 transform timeout、节点齐全；随后用户反馈狗朝目标移动（RViz 里设过 goal，狗走但偏慢，故调了速度参数）。**但我无法 100% 区分当时是"RRT 全自主探索驱动"还是"RViz 手动设 goal 驱动 move_base"**——从 RViz 日志的 `Setting goal` 看更像后者。因此能确定的是**move_base→桥→狗**这条导航/控制链通；**"RRT 自主探索的覆盖效果"并未被证明是好的**，这正是 §4.2 把它列为真·短板的原因。

### 4.3 关键参数当前值（快照；**以附件实际文件为准**）

**octomap 切片** — `go2_SLAM/FAST_LIO/launch/Pointcloud2Map.launch`：
| 参数 | 当前值 | 说明 |
|---|---|---|
| `pointcloud_min_z` | 0.2 | 切片下限（地面以上 0.2m），滤地面杂波 |
| `pointcloud_max_z` | 1.0 | 切片上限，抓椅子靠背/桌面 |
| `sensor_model/max_range` | 10.0 | 最大建图距离（m） |
| `sensor_model/hit` | 0.6 | 命中概率 |
| `sensor_model/miss` | 0.45 | 空穿概率（加快清除旧障碍） |
| `sensor_model/min` / `max` | 0.12 / 0.97 | 概率阈值 |
| `outrem_radius` / `outrem_neighbors` | 0.5 / 20 | 离群点滤波 |

> 说明：z=0 = 足端高度≈地面（`frame_id="robot_foot_init"`）。切片 [0.2,1.0] = 只保留地面以上 0.2~1.0m 的点做 2D 导航图。`max_range` 越小建图越近但更干净；RRT `Geta` 可以大于 `max_range`（走一步看一步逐步探索）。

**move_base** — `robot_navigation/param/GO2/move_base_params.yaml`：`controller_frequency=5.0` Hz、`planner_frequency=1.0` Hz。

**TEB** — `robot_navigation/param/GO2/teb_local_planner_params.yaml`：`max_vel_x=1.0`、`max_vel_x_backwards=0.0`（禁后退）、`max_vel_theta=2.0`、`acc_lim_x=1.5`、`acc_lim_theta=1.0`、`min_obstacle_dist=0.10`、`xy_goal_tolerance=0.1`、`yaw_goal_tolerance=0.2`。

**footprint / costmap** — `robot_navigation/param/GO2/costmap_common_params.yaml`：
```yaml
footprint: [[-0.325,-0.225],[0.325,-0.225],[0.325,0.225],[-0.325,0.225]]   # 0.65m × 0.45m，相对 base_footprint
transform_tolerance: 0.2
obstacle_layer: { obstacle_range: 2.5, raytrace_range: 3.0, inflation_radius: 0.05 }
inflation_layer: { inflation_radius: 0.5, cost_scaling_factor: 10.0 }
```
⚠️ costmap 的障碍物观测源是 **`/scan`**（`pointcloud_to_laserscan` 把 `/cloud_registered` 转成的 2D LaserScan），`marking+clearing` 都开；static_layer 订阅 `/map`（实际 remap 到 `/projected_map`）。也就是说导航避障用的是 2D 激光切片，不是 octomap 3D——这是 2D 管线的固有局限之一。

**RRT** — `go2_rrt_exploration/launch/simple.launch`：`Geta=60.0`（全局搜索半径）、`eta=2.0`（局部步长）、`info_radius=1.0`、`hysteresis_radius=3.0`。实际探索范围 = min(RViz 点的边界, Geta)。

### 4.4 启动 SOP（实机）
```bash
# ① NX：同步时钟（见 §2.3）
# ② NX Tab1：
export ROS_IP=[NX的IP]; export BASE_TYPE=GO2
source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash
roslaunch go2_rrt_exploration whole.launch        # 等 ~25s
# ③ NX Tab2：
~/cmd_vel_bridge/build/cmd_vel_bridge _interface:=eth10   # 狗自动站起+关避障，看到 [bridge] up
# ④ 笔记本：
export ROS_MASTER_URI=http://[NX的IP]:11311; export ROS_IP=[笔记本IP]
source /opt/ros/noetic/setup.bash; rviz -d ~/go2.rviz
# ⑤ RViz "Publish Point" 点 5 个点：4 角圈灰色未知区 + 1 个狗位置（不点 RRT 不启动）
```
**紧急停止**（先用遥控器保安全）：`killall roslaunch roscore cmd_vel_bridge` 或 `pkill -9 -f ros; pkill -9 -f cmd_vel_bridge`。

---

## 5. 仿真情况（两个都写；都不是终点）

### 5.1 2D Gazebo 前沿探索仿真
- **环境**：**原生 Ubuntu 20.04 笔记本**（`ros-noetic-desktop-full`，含 Gazebo 11）。⚠️ Gazebo 需要 GUI/OpenGL，**WSL2 里跑 Gazebo GUI 需 WSLg 且可能有兼容问题**——你如果在 WSL2 跑不顺，优先原生 Linux。
- **仿真 vs 实机**：共享同一套导航/探索代码（move_base + RRT + 割草机），唯一区别在建图：

  | 模块 | 仿真 | 实机 |
  |---|---|---|
  | 建图 | gmapping（2D 激光） | FAST-LIO（3D 点云） |
  | 传感器 | Gazebo 虚拟激光 | MID360 |
  | 地图发布 | `/map` → relay → `/projected_map` | `/projected_map`（octomap 投影） |

- **源码/依赖/编译**：
  ```bash
  mkdir -p ~/catkin_ws/src
  ln -s ~/Go2_frontier_based_exploration/workspace_ros1/one_ws/src/* ~/catkin_ws/src/
  sudo apt install -y ros-noetic-octomap-ros ros-noetic-octomap-server \
    ros-noetic-octomap-rviz-plugins ros-noetic-pointcloud-to-laserscan \
    ros-noetic-move-base ros-noetic-teb-local-planner ros-noetic-navigation \
    ros-noetic-gmapping ros-noetic-dwa-local-planner
  pip3 install scikit-learn
  sudo apt install -y python3-catkin-tools
  cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin build
  ```
- **6 终端顺序启动**（每个终端先 `source ~/catkin_ws/devel/setup.bash`、`export BASE_TYPE=GO2`）：
  1. `roslaunch robot_description simulation.launch`（Gazebo 场地；`world:=room` 换封闭房间）
  2. `roslaunch robot_navigation gmapping.launch simulation:=true`
  3. `roslaunch robot_navigation move_base.launch simulation:=true planner:=teb`
  4. `rosrun topic_tools relay /map /projected_map` ⚠️ **不能省**！move_base 和 RRT 都订阅 `/projected_map`，漏了狗不动
  5. `roslaunch go2_rrt_exploration rrt_rviz.launch`
  6. 模式二选一：
     - **RRT**：`roslaunch go2_rrt_exploration simple.launch`，然后 RViz **Publish Point 点 5 个点**（前 4 框矩形四角，第 5 在机器人附近）。
     - **割草机（比赛推荐）**：`rosrun go2_rrt_exploration lawnmower_coverage.py _width:=15.0 _height:=10.0 _strip_width:=1.5 _origin_x:=-7.0 _origin_y:=-5.0`（默认 `_width=12.0 _height=10.0 _strip_width=1.2 _origin_x=-6.0 _origin_y=-5.0 _timeout=40.0`）。
- **验证/排错**：`rostopic hz /map`、`rostopic echo /cmd_vel`、`rosnode list`；RViz 无地图 → Fixed Frame 设 `map`；狗不动 → 查终端 4 relay 和终端 6。

### 5.2 SCAN-Planner 仿真（你已跑通，此处仅对齐 + 补充）
- 细节以**你自己的交接文档** `SCAN_Planner_调研与交接.md` 为准（编译、`run.launch`、mode1/2/3、`/initial_path` 挂点、外参硬编码位置 `grid_map.cpp:57-67` 等）。
- Linux 侧补充：你交接包里已有 `src/planner/plan_manage/launch/fastlio_integration.launch` + `scripts/fastlio_test_monitor.py`；其接线方案与 §2.2 实机确认的话题语义一致，真机按此接、复验 TF 即可。
- **提醒**：SCAN-Planner 只到 `/cmd_vel`，**不含探索决策、不含 SLAM、不含赋色、不含 SDK 桥**——探索决策（去哪）仍需上层做，`/cmd_vel` 之后仍需接 §2.1 的 `cmd_vel_bridge`。

---

## 6. 开发进度与待办

**已完成（Linux/NX 侧）**：控制链路（桥，3m 验证）、时钟同步、假黑点、filter/assigner sys.path、速度/规划频率/footprint/RRT 范围调参、NX 环境重建（ros 别名、三步编译）、2D 仿真跑通、**以及导航/控制链真机跑通（狗能朝目标移动）**。

**待办（按优先级）**：
1. **探索覆盖质量**——唯一的实质缺口：RRT 前沿决策扫不全面，需换覆盖规划/SCAN-Planner（你正在做）；
2. （可选，不阻塞）`/odom` 话题名对齐：`odom_topic:=/Odometry`——不做狗也能动，做了更严谨；**不影响仿真**；
3. （非阻塞）TEB 参数警告调优。

> 说明：原先列的"TF 复核 / 端到端验证"已被"狗实机移动"覆盖，不再单列。上面这些**都不阻塞你跑仿真**——2D Gazebo 仿真走 gmapping+relay（§5.1），SCAN-Planner 仿真自包含（§5.2），都与实机的 `/odom` 话题名无关，你可以直接开始。

---

## 7. 对你交接文档 §5 开放问题的直接回答（对照表）

| 你的 §5 | Linux/NX 侧答案 |
|---|---|
| 5.1 cmd_vel→狗的桥 | ✅ **已有 `cmd_vel_bridge`**（unitree_sdk2 SportClient + CycloneDDS，真机验证 3m）。源码在 `附件/cmd_vel_bridge/`。对应你候选 (a) |
| 5.2 实机 SLAM 选型/话题 | ✅ **FAST-LIO 已接**：`/Odometry`（大写 O）+ `/cloud_registered`（世界系）。单层够用。见 §2.2 |
| 5.3 传感器外参 | ⚠️ 仍待真机。`grid_map.cpp:57-67` 硬编码是作者安装位，我们 MID360/相机挂载不同，需实测标定后参数化 |
| 5.4 body_height/双圆柱 | ⚠️ 仍待真机。默认 0.4 是作者值，需按我们实测机身包络重调 |
| 5.5 爬楼/步态策略 | ⚠️ 仍待真机。作者未公开，安全红线相关，慢慢调 |
| 5.6 规划频率/时延 | ⚠️ 仍待真机。Orin NX 上实测 |
| 5.7 mode3 成熟度 | ⚠️ 用前查 GitHub 最新 issue/commit，或先用 mode1/2 |
| 5.8 WSL2 特有坑 | （你侧的事；CPU 版渲染应没问题） |
| **补充：时钟同步** | ✅ **你的文档没提，但实机必踩**，已解决，见 §2.3 |

---

## 8. 文件/附件清单 + 权威参考

**本包附件（`附件/`，从 NX 拉取的真实文件，均已核对非空）**：
- `附件/cmd_vel_bridge/cmd_vel_bridge.cpp` + `CMakeLists.txt` + `README.md`（桥的完整说明，含编译/参数/排错）
- `附件/NX关键配置/Pointcloud2Map.launch`
- `附件/NX关键配置/teb_local_planner_params.yaml`
- `附件/NX关键配置/move_base_params.yaml`
- `附件/NX关键配置/costmap_common_params.yaml`
- `附件/NX关键配置/simple.launch`
- `附件/NX关键配置/whole.launch`
- `附件/NX关键配置/sync_clock.sh`（只做 `date -s`，见 §2.3）

**本机权威文档**（你在 WSL2 读不到，要点已内联到本文）：
- Obsidian `raicom/操作说明/Go2操作说明-1.1.md`（最新操作流程 + 参数当前值）
- `raicom/CLAUDE.md`（项目总纪律）、`raicom/7.14总结.md`（桥诞生 + 问题清单）
- `raicom/Orin NX 环境配置与运行指南.md`（NX SOP 出处）

---

## 9. 工作纪律（给接手的你）

1. **区分「已核实的事实」与「推断」**：引用仓库/论文/能力时给证据；未验证前不要断言。本项目历史上被「名字真、功能假」的二手材料坑过多次。
2. **不要把东西写死**：实机环境在变，参数/选型/桥法都是候选，实机说了算。
3. **算法不许从零写**：改 + 组合 + 胶水可以，发明新算法不行。
4. **摔倒 = 取消资格**：任何实机改动先想安全兜底（人持遥控器跟随、限速、姿态看门狗）。
5. **雷达/网段少折腾**：雷达返厂修好后，IP 是你修好并整体迁到 123 网段的，手机 APP 正常——这块现状你比我清楚，本文不再赘述。
6. 文档与沟通用中文（技术名词保留英文）。
