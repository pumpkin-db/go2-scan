# 40 · Gazebo 场景仿真 + 替换场景标准流程（2026-08-19）

> 目标：把 SCAN-Planner 的仿真场景从 mockamap 换成 Gazebo 的 indoor_1.world（HPHS 场景），
> 实现「Gazebo 场景 + Go2 狗 + LiDAR → 实时点云 → SCAN-Planner」。
> 本文件是**标准流程**：以后换任何 Gazebo world 场景都照这套走。

## 一、架构（数据流）

```
Gazebo 加载 indoor_1.world + Go2 狗（URDF 带 velodyne LiDAR 插件）
   ├─ velodyne 插件 → /velodyne_points（传感器坐标，frame=velodyne）
   └─ go2_kinematic_sim 积分 /cmd_vel → /quad_0/body_pose（Odometry）

gazebo_bridge（胶水，~50 行）：
   ├─ 订阅 /quad_0/body_pose → /gazebo/set_model_state 同步 Gazebo 狗模型（LiDAR 跟着动）
   └─ 发布 /quad_0/lidar_pose（LiDAR 世界系位姿 = body_pose + z 外参 0.1577）

SCAN-Planner：订阅 /velodyne_points + /quad_0/lidar_pose + /quad_0/body_pose
   （cloud_is_world=false，因为 velodyne 是传感器坐标，用 sensor_pose 变换到世界系）

scan_cloud_accumulator（胶水）：velodyne → 世界系体素累积 → /scan_map
map_pub：indoor_1.pcd → /map（ground truth 对照）
```

**关键理解**：狗的运动是 `go2_kinematic_sim` 纯运动学积分（不是 Gazebo 物理），
位姿通过桥节点 `set_model_state` teleport 回 Gazebo。这正是 CMU `vehicleSimulator` 的标准做法。

## 二、替换场景标准流程（七步）

1. **修 world 文件**：indoor_1.world 的 turtlebot 模型 link/joint 同名 `create::left_wheel`，gzserver 卡死。把 joint 改名为 `create::left_wheel_joint`。
2. **编译 velodyne 插件**：CMU 环境 `autonomous_exploration_development_environment` 里 whitelist 编译 `velodyne_description;velodyne_gazebo_plugins`（产出 `libgazebo_ros_velodyne_laser.so`）。
3. **狗 URDF 加 LiDAR**：`go2_description.urdf` 末尾加 `velodyne_base_link` + `velodyne` link + fixed joint（固定到 base，z 偏移 0.12）+ `<gazebo reference="velodyne">` 传感器插件。
4. **写 gazebo_bridge**：订阅 body_pose → set_model_state + 发 sensor_pose。
5. **写 gazebo_sim.launch**：Gazebo(empty_world + world 文件) + spawn 狗 + 运动学 + 桥 + SCAN-Planner(advanced_param.xml) + robot_state_publisher + map_pub + accumulator + RViz。
6. **world → PCD**（map 对照用）：`world_to_pcd.py` 采样 box/cylinder/sphere，**必须读 `<state>` 块坐标**（见坑 2）。
7. **性能调参**：物理频率 1000→100Hz + velodyne samples 1875→720（见下）。

## 三、关键文件清单

| 文件 | 位置 | 作用 |
|------|------|------|
| gazebo_sim.launch | SCAN-Planner/src/planner/plan_manage/launch/ | 统一启动入口 |
| gazebo_bridge.py | scan_planner/scripts/ | 位姿同步桥 |
| scan_cloud_accumulator.py | scan_planner/scripts/ | 累积点云 /scan_map |
| world_to_pcd.py | 仿真迁移/tools/ | world→PCD（map 对照） |
| launch_gazebo_sim.sh | 仿真迁移/ | 启动脚本（清理 PATH + source 双 workspace） |
| go2_description.urdf | SCAN-Planner/.../go2_description/urdf/ | 加了 velodyne LiDAR |

## 四、踩坑记录（全部已核实）

1. **turtlebot 关节撞名**：`indoor_1.world` 里 link 和 joint 同名 `create::left_wheel`/`right_wheel`，Gazebo link/joint 共享命名空间 → gzserver 卡死（headless 测试退出码 124=卡住）。改 joint 名解决。
2. **world 的 `<state>` 块坐标覆盖**：Gazebo 加载 world 时，用 `<state>` 块记录的模型**实际位姿**覆盖顶层 `<model>` 的 pose。`world_to_pcd.py` 若不读 state 块，PCD 整体偏移（实测偏移 30.5m）。必须解析 state 块覆盖。
3. **conda/anaconda 污染**：PATH 里有 miniconda + Windows Anaconda（/mnt/e/），导致 ①cmake 找到 Anaconda 的 protobuf 报错 ②rospy import yaml 失败。启动/编译前必须清理 PATH 里的 conda/anaconda/miniconda。
4. **velodyne_description 在 devel 缺 package.xml**：它用 `catkin_package()`（空）+ `install(DIRECTORY...)`，whitelist 编译后 devel/share 只有 cmake/ 没 package.xml，rospack 找不到。用 `ROS_PACKAGE_PATH=$CMU/src/velodyne_simulator` 直指 src 解决。
5. **spawn 超时误报**：gzserver 加载大 world（indoor_1 缺 mesh 模型要在线下载）很慢，spawn_model 2 秒超时报 "Spawn service failed"，但模型**后来成功加载**（二次 spawn 报 "entity already exists"）。无害，忽略即可。
6. **RViz dae 不显示**：WSLg 下 dae mesh 渲染不出来（只有 collision 盒），给 rviz 节点加 `LIBGL_ALWAYS_SOFTWARE=1`。
7. **双 workspace 库冲突**：velodyne 在 CMU 环境、scan_planner 在 SCAN-Planner 环境，需 source 两个 devel；但 SCAN-Planner 的 setup.bash 会重置 CMAKE_PREFIX_PATH 挤掉 CMU，要手动补 CMAKE_PREFIX_PATH/LD_LIBRARY_PATH/GAZEBO_PLUGIN_PATH/ROS_PACKAGE_PATH。

## 五、三个地图（RViz 对照）

| 名字 | 话题 | 来源 | 颜色 |
|------|------|------|------|
| map | /map | indoor_1.pcd（world_to_pcd.py 转的完整场景） | 白 |
| scan_map | /scan_map | scan_cloud_accumulator 累积 velodyne | 绿 |
| occ_map | /grid_map/occupancy | SCAN-Planner 占据地图 | 灰 |

## 六、性能调参结论

| 参数 | 初始 | 现在 | 效果 |
|------|------|------|------|
| 物理 real_time_update_rate | 1000Hz | 100Hz | gzserver CPU 120%→80%，real_time_factor 0.43→0.68 |
| velodyne samples | 1875 | 720 | ray 扫描量降 60%，0.5° 分辨率（MID360 量级） |
| 运动速度 max_vel | 0.75 m/s | 0.75（未动） | 真机够用，仿真因 real_time_factor 实际 ~0.51 m/s |

物理频率影响（已核实）：不影响狗运动（运动学 teleport）、不影响 velodyne 点云（ray 独立 update_rate）、
只影响接触检测/重力下沉精度和动态刚体。当前静态场景 100Hz 足够。

## 七、腿步态（Gazebo 狗腿动画）

- **实现**：`gazebo_bridge.py` 订阅 `/joint_states`（`go2_gait_publisher` 60Hz），30Hz 节流调 `/gazebo/set_model_configuration` 设置 12 个 revolute 腿关节（FL/FR/RL/RR 的 hip/thigh/calf）。
- **效果**：Gazebo 狗腿和 RViz 一样有步态摆动（不再纯平移），静止时呈站立姿态。
- **抖动根因**：`set_model_configuration`（运动学设角度）与 Gazebo 物理引擎（重力/惯性）本质冲突，每帧互相打架 → 轻微抖动。
- **已试方案（都失败，记录备查）**：
  1. 全 link `kinematic` → **散架**（kinematic link 不参与物理，`set_model_state` 只设 base 位姿、不传播到 kinematic 子 link，身体走了腿停在原地）。
  2. 全 link `gravity=0` → **仍抖**（惯性对抗仍在，重力不是唯一抖动源）。
- **最终结论**：保留物理方案 + 接受轻微抖动（运动正常、视觉可接受）。不要再折腾 kinematic/gravity=0。

## 八、livox MID360 替换（已完成，2026-08-19）

用社区 fork **`fratopa/Mid360_simulation_plugin`**（官方 `Livox-SDK/livox_laser_simulation` 只支持 Melodic+Gazebo9；本 fork 支持 Noetic+Gazebo11、无需 Livox SDK/driver、修正了点云畸变）。

**编译步骤**：
1. clone 后把 `livox_laser_simulation` 包复制到 CMU 环境 `src/`。
2. **两个编译坑（必须改）**：
   - `package.xml` 移除 `<build_depend>livox_ros_driver2</build_depend>`（假依赖，CMakeLists 实际没用到）；
   - `CMakeLists.txt` 删除 `target_link_libraries(... libprotobuf.so.9)`（硬编码 protobuf9，系统是 17，且源码根本不 include protobuf）。
3. `catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_laser_simulation"`，产出 `liblivox_laser_simulation.so`。

**URDF 替换**（狗身上 `<gazebo reference="velodyne">` 段；注意 livox 的 `<ray>` 在 `<plugin>` 内，velodyne 的在 `<sensor>` 内）：
- `filename="liblivox_laser_simulation.so"`；
- `csv_file_name=mid360-real-centr.csv`（MID360 真实扫描模式；插件用 `__FILE__` 相对路径找 `../scan_mode/`，编译后路径已硬编码）；
- `publish_pointcloud_type=1`（纯 xyz，SCAN-Planner 直接吃）；
- `ros_topic=/velodyne_points`、`frameName=velodyne`（保持 launch/桥不用改）；
- `samples=20000` + `downsample=2`（10000 点/帧，性能/密度平衡）。

**外观**：visual 用真实 mesh `package://livox_laser_simulation/meshes/livox_mid-360-90x.dae`（31MB，真实半球形状）；位置 `base` 前方 0.2m、上方 0.17m（`velodyne_base_mount_joint` origin `0.2 0 0.17`）；桥节点 `lidar_z=0.2077`（0.17 + scan joint 0.0377）。

**坑**：`livox_laser_simulation` 和 `velodyne_description` 一样，devel/share 里**缺 package.xml**（rospack 找不到），mesh 的 `package://` 解析失败。启动脚本 `ROS_PACKAGE_PATH` 需加 `$CMU/src`。
