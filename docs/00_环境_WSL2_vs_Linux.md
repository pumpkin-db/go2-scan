# 00 · 环境篇：WSL2 vs 纯 Linux（2026-08-18 快照）

> 迁移时对照本文：🐧W 的条目在纯 Linux 上消失或变化，🐧L 的条目照搬。

## 机器快照（WSL2 侧，✅ 已核实）

| 项 | 值 |
|---|---|
| 发行版 | Ubuntu 20.04.6 LTS |
| 内核 | 6.6.114.1-microsoft-standard-WSL2（x86_64） |
| WSL 版本 | 2.7.3.0 |
| 资源 | 16 核 / 11GiB RAM / 磁盘充足（>900G 可用） |
| GUI | WSLg 可用：`DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`、`/mnt/wslg` 存在；RViz 窗口直接弹到 Windows 桌面 ✅ 实测 |
| GPU | NVIDIA 驱动透传可用（nvidia-smi 正常、`/dev/dxg` 存在）——本轮仿真用 CPU 渲染，GPU 渲染（`use_gpu:=true`）未测 ⚠️ |
| ROS | Noetic `desktop-full`（273 个 ros-noetic 包，含 rviz/robot_state_publisher/Gazebo 11）+ Foxy 并存（**只用 Noetic，别 source Foxy**） |

## ROS 环境激活（🐧L 通用）

```bash
source /opt/ros/noetic/setup.bash
source ~/claude/raicom/new_algorithm/SCAN-Planner/devel/setup.bash   # SCAN-Planner
# 旧 rrt 管线（如需）：source ~/catkin_ws/devel/setup.bash
echo $ROS_DISTRO   # 应为 noetic；空 = 没激活
```
- 每个新终端都要重新 source（shell 状态不跨命令保留）。
- `ROS_MASTER_URI` 保持默认 localhost:11311 即可（单机仿真）。

## conda 干扰（🐧W+本机特有，🐧L 按对方环境重查）

- `~/miniconda3` 在 PATH，`python3` 指向 conda py3.13（无 rospy）。
- **catkin_make**：必须 `-DPYTHON_EXECUTABLE=/usr/bin/python3`。SCAN-Planner 的 `build/CMakeCache.txt` 已缓存该值，增量编译无需再加；**纯 Linux 重新 clone 编译时记得带上**。
- **rosrun python 节点**：会用 conda python → `No module named rospy`。本轮没跑 python 节点未触发；要跑时用 `/usr/bin/python3 节点路径` 显式调用，或临时 `conda deactivate`。

## 编译（🐧L 通用，SCAN-Planner）

```bash
cd ~/claude/raicom/new_algorithm/SCAN-Planner
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3   # 首次；增量可省参数
```
- 依赖：ros-noetic-{cv-bridge,pcl-ros,tf,robot-state-publisher,rviz}、libarmadillo-dev、libeigen3-dev（本机 ✅ 全装）。
- GPU 渲染可选（GLEW/GLFW + `-DUSE_GPU=ON`），未启用 ⚠️。

## 旧工作区盘点（本机历史遗留，本轮未改动）

| 路径 | 内容 | 状态 |
|---|---|---|
| `~/catkin_ws` | 5 月编译的 rrt 全套：go2_SLAM / go2_rrt_exploration / livox_ros_driver2 / robot_description / robot_navigation / pointcloud_to_laserscan / map + **早期 x86 版 go2_cmd_vel_bridge**（链接 `~/unitree/unitree_sdk2`，无看门狗版） | 已编译产物在；Linux/NX 侧交接文档里的**带看门狗版 cmd_vel_bridge 才是现役版**，这个是早期迭代 |
| `~/workspace_ros2` | Foxy + unitree_api/control（已废弃的 ros1_bridge 路线） | 不再使用 |
| `~/catkin_ws/src/map/map1.pgm` | 旧项目的 2D 占据栅格地图（map_saver，0.15m/格，234×69） | 已被转成 PCD 墙场景（见 10 篇 run2） |
| `~/unitree/unitree_sdk2` | x86_64 + aarch64 库 | 编译桥用 |

## 迁移回纯 Linux 时的差异清单

| 项 | WSL2 现状 | 纯 Linux 预期 |
|---|---|---|
| GUI | WSLg（偶发 libGL 问题，软渲染兜底） | 原生 X11，一般更稳 |
| GPU 渲染 | 驱动透传在，未测 | 有独显可测 `use_gpu:=true` |
| conda | 在 PATH，需注意 | 按对方环境查 |
| 编译产物 | build/devel 不可移植（含绝对路径） | **重新 catkin_make** |
| 性能 | CPU 渲染 10Hz 无压力（16 核） | Orin NX 上另测（规划时延是开放问题） |
| 实机件 | 无（纯仿真） | cmd_vel_bridge / FAST-LIO / 时钟同步见《Linux 侧交接文档》 |
