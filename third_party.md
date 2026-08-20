# 第三方依赖清单

> 本仓库只放**自研胶水代码 + 文档 + 仿真配置**，不包含第三方源码。
> 第三方仓库按下面的地址 clone 到本地，源码不进本仓库（体积大 + 版权）。

所有依赖均在 **ROS1 Noetic + Ubuntu 20.04** 下编译验证（WSL2）。

| 依赖 | 地址 | commit | 作用 |
|------|------|--------|------|
| SCAN-Planner | https://github.com/wuyi2121/SCAN-Planner.git | `348e8a5` | 局部规划核心（B样条 + 栅格地图 + FSM） |
| autonomous_exploration_development_environment (CMU) | https://github.com/HongbiaoZ/autonomous_exploration_development_environment.git | `bf0cba7` | velodyne 插件 + go2 URDF 底座 + vehicle_simulator |
| HPHS | https://github.com/bit-lsj/HPHS.git | `62914f8` | Gazebo 场景（indoor_1.world 等） |
| Mid360_simulation_plugin | https://github.com/fratopa/Mid360_simulation_plugin.git | `aae8ee3` | Livox MID360 真实扫描模式仿真插件 |

## 编译步骤

### 1. SCAN-Planner（主规划包）

```bash
cd <SCAN-Planner>
catkin_make
```

本仓库 `simulation/` 下的胶水文件要复制回 SCAN-Planner 对应位置：

| 本仓库文件 | 复制到 SCAN-Planner 位置 |
|-----------|--------------------------|
| `simulation/launch/gazebo_sim.launch` | `src/planner/plan_manage/launch/` |
| `simulation/launch/default.rviz` | `src/planner/plan_manage/launch/` |
| `simulation/scripts/gazebo_bridge.py` | `src/planner/plan_manage/scripts/` |
| `simulation/scripts/scan_cloud_accumulator.py` | `src/planner/plan_manage/scripts/` |
| `simulation/urdf/go2_description.urdf` | `src/simulator/Utils/go2_description/urdf/` |

### 2. CMU 环境（velodyne 插件 + go2 模型）

```bash
cd <autonomous_exploration_development_environment>
catkin_make -DCATKIN_WHITELIST_PACKAGES="velodyne_description;velodyne_gazebo_plugins"
```

### 3. Livox MID360 仿真插件

```bash
# clone Mid360_simulation_plugin 后，把 livox_laser_simulation 包复制进 CMU 环境的 src/
cp -r <Mid360_simulation_plugin>/livox_laser_simulation <CMU>/src/

# 两个编译补丁（必须，否则编译失败）：
#   ① package.xml 删除 <build_depend>livox_ros_driver2</build_depend>
#   ② CMakeLists.txt 删除 target_link_libraries(... libprotobuf.so.9)

cd <CMU>
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_laser_simulation"
```

产出 `devel/lib/liblivox_laser_simulation.so`。

### 4. HPHS 场景

只用到 `world/indoor_1.world`（本仓库 `simulation/worlds/` 里的修改版）：
- 修了 turtlebot 关节撞名（`create::left_wheel` → `create::left_wheel_joint`）
- 物理频率降到 100Hz

world 里 `model://` 引用的模型（arm_part/turtlebot/kinect 等）来自 Gazebo 官方模型库，
首次启动 gzserver 会自动在线下载到 `~/.gazebo/models`。

## 启动

```bash
bash simulation/launch_gazebo_sim.sh
```

> ⚠️ `gazebo_sim.launch` 和 `launch_gazebo_sim.sh` 里有 `$HOME/claude/raicom/...` 硬编码路径，
> 换环境时需改成你的实际路径（详见 `docs/40_...标准流程.md` 第七节）。
