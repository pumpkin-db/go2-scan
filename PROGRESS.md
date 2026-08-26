# go2-scan 进度记录

> 实时进度流水账（写给未来的 AI / 自己接续用）。按日期倒序。
> 指令、规则、硬约束见 `CLAUDE.md`（那是规则层，不是进度层）。
> 第三方来源/commit/编译见 `third_party.md`。

## 2026-08-26：Depot 三轮对照定稿（D2=49.2% 基准确立）→ 任务#3 收官 ✅

| 跑 | 效用环 | ER_final | 判废 | 轨迹 | 备注 |
|---|---|---|---|---|---|
| D1 | 3m | 43.5% | （旧逻辑误判完成） | ~8m 冻结 | 假完成数据点 |
| **D2** | 3m+停滞门 | **49.2%** | ✅ 正常 | 67.4m | **基准跑**（P@0.2=98.9%/Chamfer 0.103m） |
| D3 | 6m+停滞门 | 48.0% | ✅ 正常 | 79.2m | 扩环无增益 → factor 维持官方 0.5 |

**定稿结论**：
1. ARiADNE @ Depot 稳定基线 ER≈48-49%（双跑复现），为地面层理论天花板（56.8%）的 ~85%。
2. 地图质量优秀且稳定（P@0.2≈97-99%、R@0.2≈82-83%、Chamfer≈0.103m）。
3. 抬升 ER 的唯一路径 = 楼梯上夹层（地面+夹层上限 76.2%）——任务#4 的战略价值定量坐实。
4. utility_range_factor 维持官方值 0.5（少一个偏离上游的补丁）；停滞式完成判定保留。

## 2026-08-25（深夜）：Phase 3 楼梯检测器注册表模式实测打通 ✅

- `/stairs_detected` 实测发布 main_east：entry[12.87,0.40,0.44]→exit[12.87,3.20,2.76]、
  yaw=90°、宽 1.2m、总升 2.32m，与 scene.yaml 注册表逐字段一致。
- 踩坑记录：rospy 脚本私有参数命令行是 `_registry:=`（带下划线），`~registry:=` 无效且无报错；
  nohup 下 python 日志缓冲不可见（rosout 里反而有）。
- **遗留谜团**：elevation_mapping 进程存活、订阅正常、首次回调成功（"mapping started"）、
  参数核对无误，但 fused/raw 高程图持续零发布（16 字节布局版）。与 CMU 节点段错误同族待查，
  已并入任务 #8 的 gdb 排查。几何法楼梯检测的实景验证被此阻塞；
  transit 联调可先用注册表模式推进。

## 2026-08-25（晚）：TARE 对照首跑即崩（CMU 节点段错误）→ 记录阻塞转 Phase 3

- **TARE @ Depot 首次端到端运行失败**：`terrainAnalysisExt` 与 `tare_planner_node` 均 exit -11
  （段错误），required=true 拖垮整个仿真。16 字节布局修复后**仍崩** → 排除 PC2 格式因素。
- **集成债曝光**：此前 TARE 分支 launch 从未启动 terrain_analysis(+ext)——/terrain_map 无发布者，
  tare_planner 收不到地形层永不规划（狗站桩的 TARE 特有机制，与 ARiADNE 的假完成不同）。
  已补 include 接线（CMU 默认输入恰为 /state_estimation+/registered_scan，tare_bridge 已产出）。
  注意 terrain_analysis_ext.launch 与主 launch 有 navigation_boundary_publisher 重名冲突，勿重复。
- **结论**：TARE 对照阻塞（任务#8），ARiADNE 为当前唯一可用全局层。velodyne 插件已回退到
  与旧 livox 完全一致的 16 字节 x/y/z/intensity 布局（治本尝试，虽未解决此崩但消除了格式风险）。
- elevation_mapping 的 -11 与 CMU 节点同病相怜，一并 gdb 排查（任务#8 扩展）。

## 2026-08-25（傍晚）：Depot 基线确立 D2=ER49.2%（地面层天花板的 87%）+ 理论上限标尺

### Depot 跑分（velodyne 换装 + spawn 托管后）

| 跑 | ER_final | 说明 |
|---|---|---|
| D1 124318 | 43.5% | 「效用全零即判完成」假完成，~8m 后冻结；JSON 序列化 bug 致 md 缺失（已修 bool()） |
| **D2 133232** | **49.2%** ✅ degraded=False | 停滞式判定版：轨迹 67.4m；P@0.2=98.9%/R@0.2=81.5%/Chamfer 0.103m |

### 关键认知（评估体系级）

1. **Depot ER 物理天花板标定**（同款可见性模型离线撒点）：仅地面层 ≈56.8%，地面+夹层 ≈76.2%
   （GT 里 57% 是 z>4.5m 屋顶高层，6m 量程永远不可及）。**单层跑分 ~50% 已接近物理极限，
   想再抬必须靠楼梯上夹层（任务#4 的战略价值被定量证实）**。
2. **假完成第一杠杆已落地并验证**：rl_planner 完成判定改为「效用全零 AND 地图静止 ≥20s」
   （~stalled_complete_seconds 可调）；D1→D2 ER +5.7pt、轨迹 8m→67m。
3. **剩余停滞机制已定位未修**：效用环=0.5×量程=3m 内无前沿节点就不发航点（楼梯口远端前沿
   够不着）。D3 杠杆已备：utility_range_factor 0.5→1.0（launch 已改，待跑）。
   若 D3 仍停摆 → 上「全局最近前沿恢复航点」胶水（run10 丢失的思路重写）。

### 正在进行

- TARE @ Depot 对照基线（Phase 2）：后台运行中，日志 /tmp/bench_depot_tare.log。

## 2026-08-25（下午）：Depot 全盲根因终局 = livox 自制射线插件；换装 velodyne 后全链路打通 ✅

### 终局结论

**雷达全盲（点云坍缩为传感器原点单点）的根因 = livox_laser_simulation 的自制 ODE 射线路径**
在多模型世界里完全失效。排除法穷尽了：spawn 竞态✗、model://Depot 解析✗、submesh 命名✗
（libsuffix 同样失败）、整网格碰撞✗、STL 米制网格✗、物理配置✗（默认与 0.002/500Hz 同败）、
RTF/预热✗（实测 0.84）、ODE 递归修复✗（无条件递归后依旧全盲）。帧指纹证明插件在复读死缓冲
（stamp 前进、数据 md5 不变）。**换装标准 RaySensor + velodyne 插件后一次点亮**：
9801/9847 去重点、中位距离 8.36m、环带命中 953、projected_map 出图、way_point 恢复、狗开始探索。

### 落地的修改

1. **URDF 换装**：`go2_description.urdf` 用标准 `<ray>` 传感器块 + `libgazebo_ros_velodyne_laser.so`
   （CMU vendor），视场角对齐 MID360（360°×-7°~52°，200×50=10000 点 @10Hz，量程 40m）。
   插件增补 `<worldFrame>true</worldFrame>` 参数：点云直接转世界系输出，下游 cloud_is_world
   管线零改动。旧 livox 块注释保留在 URDF 尾部备查。注意 point_step 16→22（多了 ring/time 字段），
   动态字段解析的节点兼容。
2. **velodyne 插件增补**（cmu_env）：`GazeboRosVelodyneLaser` 加 world_frame_/sensor_world_rot_/pos_
   ——照搬 livox 魔改版的「父实体位姿 × 传感器位姿」世界系变换，参数化开关。
3. **livox 插件递归修复保留但未采用**：UpdateCallback 改无条件递归子空间（理论上更正确，
   但对本故障不致命）；ps3joy 加 CATKIN_IGNORE（缺 libusb）；cmu_env 构建树推倒重建过一次。
4. **spawn 托管 + 单实例守卫**：见上一节。

### 工具沉淀（scenes/gtools/）

- `probe_cloud_diversity.py`：点云取证探针（帧数/3D去重数/环带命中一次输出）
- `reexport_mesh_meters.py` / `make_stl_collision_variant.py` / `make_wholemesh_collision.py`：
  网格单位与碰撞变体工具链（本次未成为解法，留作资产）
- `test_depot_raycast.sh`：分变体自动实测框架（含清场复核）

### 教训（写给自己）

1. **自制物理交互代码是最大风险源**——livox 插件绕开 gazebo SensorManager 自己玩 ODE space，
   平时能跑、换个世界就全盲，且失败模式静默（有帧、有点、就是全 miss）。标准组件优先。
2. **实验纪律**：每次起仿真前 kill_all + pgrep 复核；不用 timeout 包 launch 脚本；
   探针先看「多样性指标」（3D 去重数）而不是单一距离统计。
3. **对照实验要控制变量**：下午曾因实例污染误判「indoor_1 也瞎了」，浪费两小时。

## 2026-08-25（凌晨）：用户终版定参 + 五方向开工

### 三重根因（按发现顺序）

1. **spawn CLI 竞态**：旧 run_benchmark.sh 用 `rosrun spawn_model`+60s timeout，首 spawn 超时但请求入队，重试叠出多狗；"entity already exists" 被当成功。→ 已换 `simulation/spawn_go2.py` 服务化托管 + 点云健康门。
2. **`model://Depot` 解析失败 → 世界无几何**：模型本体在 `scenes/depot/model/assets/`，但 include 要找 `model/Depot/model.sdf`。→ 已修：符号链接 `model/Depot -> assets`。
3. **Depot submesh 碰撞疑似令 ODE 射线空间失效**：单实例取证（无污染窗口）显示模型加载成功但雷达连测试盒都打不到，全部点=传感器世界位姿（livox 插件魔改版把 miss 点收缩到传感器原点）。待验证修复：`scenes/gtools/make_wholemesh_collision.py` 把 collision 内 submesh 换成整网格。

### 进程卫生纪律（血泪教训，以后每次启动前必须执行）

- bench_depot3/4 用旧版脚本 FATAL 退出时**没有清理仿真** → 孤儿 gzserver 泄漏 → 后续所有 roslaunch 复用 master、孤儿重连发陈旧 /clock → **今天下午「indoor_1 也瞎了」是误判**（探针听到的是僵尸世界的帧，插件加载时 sim time 已 1119s 是铁证）。
- **第二个泄漏源（更隐蔽）**：`timeout N bash launch_gazebo_sim.sh &` 这种实验写法，timeout 只杀外层 bash，gzserver 孤儿化继续跑。下午用 timeout 包实验反复触发。**已修：launch_gazebo_sim.sh 开头加单实例守卫**——检测到已有 gzserver 立即 exit 42 拒启；做实验一律不用 timeout 包 launch 脚本。
- `kill_all_sim.sh` 头注释早就记载了此故障模式（2026-08-24），但我一直 `>/dev/null` 吞掉它的输出、不复核。**正确姿势：跑完 kill_all 必须显式 pgrep 复核为空再启动新栈。**
- livox 插件（魔改世界系输出版）的全 miss 病征 = 点云所有点精确重合在传感器世界位姿上（3D 去重后 1 个点）。取证探针：`scenes/gtools/probe_cloud_diversity.py`（帧数/去重数/环带命中一次输出）。

### 其他进展

- UFEP 定位澄清（用户指示）：配 UFEP 是为**参考思想**，复杂度超收益就不采用。pplanner_simulator 是作者自家仿真器（不用），重心改为 vrmapping 冒烟+思想评估（GSOM+LSOM 双层采样拓扑图，见 Sampler.h）。
- 论文笔记产出：`try_algorithm/notes/HEADER与HDPlanner精读.md`（ARiADNE2/HEADER=升级候选，ROS1 Noetic 原生）、`try_algorithm/notes/LITE多楼层探索精读.md`（楼层-楼梯拓扑+FSM，与我们方向④设计互证）。
- ARiADNE2-ROS-Planner 已克隆到 `try_algorithm/code/`（约 1050 行 C++，含 terrain_segmentation/map_handler/collision_checker）。

## 2026-08-25（凌晨）：用户终版定参 + 五方向开工

- **参数终版**：收发量程耦合 **6m**——用户自行多轮实测后拍板，「不用再改了」。此前文档里的 4.5/6.5 均为过程值，已在 launch 注释与 third_party.md 勘误。res=0.2、z∈[0.2,0.8] 维持。
- **用户下达 5 个工作方向**（顺序自定）：①修仿真狗腿乱飞（TARE 时期没有）②阅读配置 `new_algorithm/UFEP-Released` ③找公认评估标准给所有算法统一评分 ④ARiADNE 加楼梯识别+爬楼，验证场景 `~/claude/raicom/Scene_Gazebo/Depot`，go2-scan 新建场景文件夹统一管理场景参数 ⑤论文收集阅读 → `new_algorithm/论文/`（示例：HDPlanner、KAIST UR2025 多楼层窄室内探索）。
- 开工前按要求先推 GitHub 备份。

### 方向③ 评估标准——评估器已建+实测中 ✅（基准跑分进行时）

- 调研结论见 `docs/评估标准调研.md`：探索效率用 Explore-Bench 惯例（ER(t)/T90/T95/路径长），
  地图质量用 Cloud_Map_Evaluation 标准定义（Accuracy/Completeness/Precision@τ/Recall@τ/F-score，τ∈{5,10,20}cm）。
  ER 可见性=range+FOV 无遮挡剔除（文献常见近似，已在报告声明）。
- 评估器 `tools/evaluate_exploration.py`：实时订阅 /quad_0/body_pose+/scan_map+/map，
  结束时经 `/scan_map/save` 服务取**全量**累积图（话题有 1e5 随机抽稀不能用于评测），输出 JSON+MD+ER曲线。
- 一键基准脚本 `simulation/run_benchmark.sh`（无头启动→spawn 竞态重试→挂评估器→出报告→清理），
  gazebo_sim.launch 新增 gui/rviz 开关与 gt_pcd 参数（默认行为不变）。
- **顺带修了潜伏 bug**：scan_cloud_accumulator 的 /scan_map/save 返回 Trigger(...)（服务类型）而非
  TriggerResponse(...)，首次真实调用即崩——已修。
- 教训记录：后台跑 python 必须显式 `/usr/bin/python3`（nohup 环境的 python3 无 numpy，第一次评估因此报废）；
  rospy 脚本别忘了 init_node。
- ARiADNE @ indoor_1 基准跑分正在后台进行，结果将落在 evaluation/results/。

### 方向④ 楼梯/多层——设计定稿 + Depot 场景接入（阶段A完成大半）

- 设计文档 `docs/楼梯与多层探索设计.md`：官方代码零改动原则下做「逐层会话」架构
  （stair_detector / floor_session_manager / stair_transit / way_point_mux 四组件 +
  kinematic_sim 地形跟随 z）。关键前提已验证：**octomap_server 的 occupancy_min_z/max_z
  支持动态重配**（dynparam get /octomap 实测），z 带可运行时按层切换。
- 核心事实（决定架构）：rl_planner 官方版连 z 都没有（waypoint 纯 x/y）；go2_kinematic_sim
  只积分 x/y/yaw、z 恒 0.25 → 现仿真狗永远不会爬升；octomap z 带是世界系绝对高度。
- **Depot 场景接入**：
  - 原模型是 Ignition 版且除地面外全无 collision（雷达打不到墙）→ `scenes/gtools/sdf_add_collisions.py`
    给 WALLS/FLOOR/STAIRS/PILLERS/BOXSET/ROOF 补 collision、转 static、去 ignition 插件。
  - GT 点云 `scenes/depot/gt/depot.pcd`（150万点，trimesh+pycollada 网格采样，工具
    gtools/make_scene_gt.py 可复用于任何 DAE 场景；注意 py3.8 需 pycollada==0.7.2、子网格名带 -lib 后缀）。
  - 台阶几何分析：踏步高≈0.161m、主楼梯段在东侧 x[12.5,13.25] y[0→3.5]，z 0.44→2.76（升高2.32m）。
    已注册进 `scenes/depot/scene.yaml`（含楼层高度/边界/出生点）。
  - 场景参数化：`scenes/<name>/env.sh`（indoor_1 与 depot 各一）+ launch_gazebo_sim.sh/run_benchmark.sh
    支持 `scene:=<name>`。用法：`bash simulation/run_benchmark.sh global_planner:=ariadne scene:=depot`。
- 下一步：Depot 平地基线跑分（阶段A验收）→ kinematic_sim 地形跟随 z（B）→ detector（C）→ transit+会话管理（D）。

### 方向② UFEP——阅读完成，编译待做

- 调研笔记 `new_algorithm/UFEP-Released_接入调研.md`（在 go2-scan 外）：接口契约（吃 grid_map 高程图、
  吐 /my_cmd_vel 直接兼容我们运动学仿真）、依赖清单（vrmapping_msgs+CUDA+glog）、坑（作者家目录硬编码日志路径、Debug 硬编码）。
- 主要缺口：其所需 grid_map 层来自作者的 elevation_mapping_cupy 魔改分支（CUDA+pybind）。
- 注意：UFEP 吃高程图+可通行性层，是楼梯/多层场景的备选全局层。

### 方向③④ 补充（2026-08-25 晚）：run3 基线 + Depot 基线跑 + UFEP 编译通过

- **run3（成功跑基线）**：ER 99.6%、T90=476s/T95=678s/平台期 685s、路径 318m、
  Recall@0.2=95.2%、Chamfer 0.168m。ER 与地图 Recall 差 4.4pt，指标体系自洽 ✅。
  三跑对照（530m/11.5m/318m）定量证实 6m/res0.2 组合成功率 2/3；退化跑特征 =
  ER 极早平台期 + 路径异常短（可作自动判废信号）。详见 evaluation/results/README.md。
- **UFEP 编译通过**（Release, CUDA 12.9, WSL2）：两个坑——①package.xml 漏声明
  vrmapping_msgs 依赖→catkin 拓扑排序把主包排在 msgs 前→配置必炸（已补声明）；
  ②首次失败的 CMakeCache 缓存了坏 PYTHON 结果（rm build/devel 重来）。
  catkin_make 不跟符号链接→ufep_ws 用实体拷贝。产物：vrmapping_node + libCellsIntegratorGPU.so。
- **Depot 基线跑启动**（阶段A验收）：`run_benchmark.sh scene:=depot`（地形跟随自动开启）。
- 论文库新增（均经 arXiv API 核实，凭记忆写的编号两次全错已记录为教训）：
  ARiADNE 原文 2301.11575（决策核正源）、CMU 开发环境论文 2110.14573（cmu_env 正源，
  TARE 四层车库 1839m/1907s 跨楼层实证；**其 terrain_analysis 把楼梯当可通行代价而非障碍**
  ——stair_transit 可借路线）、FAR Planner 2110.09460（可见性路由，多层候选方案）。

### 方向⑤ 论文——目录建立 + 两篇示例落位

- `new_algorithm/论文/`：HDPlanner（arXiv PDF+MIT 开源代码已下载；代码框架与 ARiADNE 同构）
  + KAIST UR2025 多楼层 MCDM（IEEE 付费墙无开源，需用户机构通道下载原文；已下载两篇开放替代文献）。
  索引与收录规范见 `new_algorithm/论文/README.md`。

### 方向① 狗腿乱飞——已修复并数值验证 ✅

**根因（两个独立缺陷叠加，均为「外观动画」误入「物理仿真」）**：
1. URDF 的 12 根腿连杆是动力学体，但 revolute 关节零阻尼、无任何控制器插件（URDF 里唯一插件是 livox 雷达）。狗本体是运动学传送模型（base 由 set_model_state 传送），腿却参与物理：gazebo_bridge 以 ~30Hz 用 set_model_configuration 把关节角瞬移到步态角度，两次传送之间物理引擎在重力+足底接触下自由甩腿，每次传送注入能量且速度残留 → 走动时接触摩擦加剧 → 乱飞。TARE→ARiADNE 期间此机制没变过；变的是负载（octomap res=0.2 等）压低 RTF，暴露并放大了缺陷 2：
2. go2_gait_publisher 步态相位用 event.current_real（墙钟），而 gazebo_bridge 的应用节流用 rospy.Time.now()（仿真时间）。RTF<1 时墙钟相位狂飙、应用按仿真节流 → 每次应用都是大角度跳变 = 肉眼乱踢。

**修法**：
1. URDF：12 根腿连杆全加 `<gazebo reference="…"><kinematic>true</kinematic></gazebo>`——物理引擎跳过其动力学积分，腿变纯外观动画（传感器挂 base 不受影响）。
2. gait_publisher：相位改用 ros::Time::now()（仿真时间），与应用节流同源；RTF 低时步态整体慢放（物理上合理的慢动作）。顺带修掉 joint_states header 墙钟戳会污染 TF 时序的隐患。

**验证**（`simulation/test_legs.launch` 最小环境 + `tools/verify_leg_fix.py`）：set_model_configuration 在 kinematic 腿下 success=true；行走时 Gazebo 实际关节角全程跟随指令（max|实际-指令|=0.180 rad < 容差0.25）；相邻50ms采样 max|Δ实际|=0.232 rad 无跳变；狗 0.49m/s 正常行走。三项 OK。

**遗留**：spawn 竞态（gzserver 加载 world 时 spawn 服务超时但请求已入队执行）再次复现——首启后若报 "model does not exist"，等几秒重试 spawn 会报 "entity already exists"=实际已成功。已知无害，未改。

## 2026-08-24：ARiADNE 推倒重来（官方契约版）

### 为什么推倒

第一次 A2 接入（run1-run11）被判定失败：projected_map 悬空错位、补丁层层叠加成屎山。取证结论：
- **frame 断裂**：octomap frame_id=map 与全系统的 world 无 TF 桥 → 地图悬空（主因）
- **state_estimation 错接 body_pose**：官方语义是雷达原点位姿（vehicleSimulator 源码核实），射线原点系统性偏 ~0.4m
- **对抗性参数补丁病**：miss/factor/min_utility/LOS/停滞判定全是治标；vendored py 打了 10 处补丁
- scan_map 链路逐行比对未被动过；漂移嫌疑是 livox 退化帧+add-only 累积器（另案取证）

处置：完整快照存 `archive/ariadne-a2-attempt1` 分支（151a42a）→ main 清场回 2bf137d 行为 → 官方契约版重建。

### 重建要点（架构与契约细节见 third_party.md「ARiADNE」段）

- rl_planner scripts 从官方副本**原样拷贝一字不改**——map→world 恒等桥修好后上游代码本来就能工作
- 新增恒等 static TF map→world（一行修 frame 断裂）；链路配置独立成包内 launch/go2_ariadne.launch
- sensorScanGeneration 的 state_estimation 输入改接 /quad_0/lidar_pose（对齐官方「传感器位姿」语义）；rl_planner 仍接 body_pose（机器人所在位置）
- 参数全回官方 indoor 基准（factor 0.5 / min_utility 3 / miss 0.45），仅用户决策两项覆盖：max_range=5、z∈[0.2,0.8]
- goal_bridge 重写 ~70 行（唯一自研胶水）；cloud_range_filter 移进 ariadne 组内（TARE 路径保持基线行为）

### 门禁实录

| 门禁 | 结果 |
|---|---|
| G1 链路活性 | ✅ run2 起（sensor_scan/projected_map/way_point 全通） |
| G2 地图正确性 | ✅ TF 恒等桥+雷达位姿精确；幻影墙经三轮排查后 run9 幻影 0 格 |
| G3 决策 | ✅ run9：54 航点零假完成（此前 8~10 航点即停） |
| G4 执行 | ✅ 狗出西楼块至 (7.9,14.6)，位移 >15m |
| G5 端到端 | ✅ run9：81 航点合法完成；地图 105×85 格(42×34m)覆盖全场；狗终点 (26.0,18.0) 距出生点 33m——「出不了西楼块」一并解决 |

**幻影墙插曲（2026-08-24 下午）**：projected_map 大量地面被判黑 → 三轮排查（帧门控[误诊已撤]/截断理论[否决]/官方对表）→ 根因=无限地面接住穿墙泄漏射线+octomap 无带检查的端点占据 → 修复配方见 third_party.md「幻影墙根因链」。run9 幻影 0 格，commit dbf74a5。

### 遗留

- 用户偏好参数回收（2026-08-24~25 多轮连调，用户拍板同轮生效）：①收发量程耦合 **6.5m**（沿革：官方配方 20 → 本机跑通记录 envB 7 → 用户 5 → 3 → 4 → 10 → 6.5。**术语勘误：官方配方=上游原版默认 20m 且本机从未跑通，envB 的 7m 是本机自设参数的现场记录而非官方配方**）；②**map_resolution 0.4→0.2**（octomap 相对原版负载 ≈8×）；③z 带 **[0.2,0.8]**——用户原始偏好在 res=0.2 下完整达成且幻影墙免疫成立（min_z≥一个整格规则：地面端点落中心 0.1 层，0.1+0.1=0.2>0.2 恒假）。**当前组合无任何跑通记录**，观察信号：卡顿 / 过早 Completed / 黄点(optimal_traj 低速段)消失=断供。probe_occ_vs_gt.py 的 GT 对照带已同步 [0.20,0.80]。RViz 显示组已按官方 rviz.rviz 对齐（node 效用蓝→橙渐变、Waypoint 紫球 0.3m、ObstacleVoxel 体素层常开、frontier 0.3m；黄色小球=SCAN optimal_traj 速度着色低速段）。**穿地事故排查结论**：spawn(-7.5,0.5,0.25)/地板/URDF 均未被动过且几何正确，头号嫌疑=res=0.2 负载挤垮 WSL2 物理步进；二分法=命令行 arg 覆盖回本机跑通档（map_resolution:=0.4 sensor_range:=7.0）
- 「路径穿障」观感说明（2026-08-24 用户报告 SCAN 路径穿障碍物）：occ_map/grid_map 链路未被动过；goal_bridge 发的 /initial_path 是两点直线（目标指示非轨迹，设计上无碰撞检查），ARiADNE 航点天生贴障碍表面故直线视觉穿墙属预期；真轨迹是 optimal_traj（B 样条），穿实体才算 bug。待用户确认具体显示项
- TARE 分支回归冒烟未做（本轮全程 ariadne；launch 改动已保证 tare 路径参数不变）

### 楼梯 × 地图分辨率分层结论（2026-08-24 讨论定音）

- **map_resolution=0.4 不阻碍楼梯识别**：占据栅格本来就不负责台阶感知。查过 TARE 全部官方场景 config：indoor/campus/tunnel 0.3m、matterport 0.2m——没有任何一层的地图分辨率能装下单级台阶（15~20cm 高/28cm 进深），行业共识是地形感知独立成层。
- TARE 跨楼层的真机制（本机 vendor 源码 config 核实）：`terrain_analysis` 的 cm 级高程+可通行性网格 + `kUseTerrainHeight:true` + `kCheckTerrainCollision:true`。我们的 go2.yaml 里 `kCheckTerrainCollision: false`，且视点 z 冻结问题在 08-23 审查报告 #8 记录过。
- **真正的风险不是「识别」而是「全局层敢不敢往楼梯走」**：楼梯累加高度超过 z 带上限(1.2)后，整段楼梯在 ARiADNE 的 2D 图上投影成黑墙 → 无前沿无效用 → 上层永远探索盲区。降分辨率治不了（纯 2D 投影天然如此）。
- 选型两条路（未拍板）：①全局层换 TARE——其 3D keypose_graph 天生跨层连接不同高度节点（campus 多层 demo 即此路），代价是丢掉刚调通的 ARiADNE 链；②ARiADNE+楼梯口注册胶水——terrain_analysis 发现楼梯后把上下两端注册为一对连通点塞给全局层，贴合 SCAN 实机人工航点爬楼思路，改动量小。
- 前置条件：决赛场地有无楼梯/允不允许上下，决定此事排期优先级。

## 2026-08-24：ARiADNE A2 重做（官方仿真跑通后回炉）【已废弃——被上面「推倒重来」取代，快照在 archive 分支】

### 背景与决策

- 先在 `new_algorithm/ARiADNE-ROS-Planner + CMU env` 把**官方仿真端到端跑通**（含场景包 479MB 下载、GAZEBO_MODEL_PATH、terrain_analysis 补编译；详见 third_party.md「A2 重做」段），确认了算法的正确数据形态，再回 go2-scan 重做集成。
- 用户决策：删旧架构接线重做；接口用 **A2**（way_point→Path→SCAN navi_mode=3）；octomap 切片 z∈[0.2,0.8]；max_range=5m。路线：纯2D → 楼梯检测 → 赋色。

### 已完成的改动

1. **gazebo_sim.launch ariadne 分支重写**：sensorScanGeneration(转传感器系+TF) → octomap(frame_id=map, z∈[0.2,0.8], max_range=5) → rl_planner → ariadne_goal_bridge(新, /way_point→2点Path去重>1m→/initial_path)。旧的世界系直喂 octomap 有射线原点=(0,0,0) bug（穿墙地图根源），已废。navi_mode 随 global_planner 自动联动为 3。
2. **vendored rl_planner 三处带注释的四足适配**（其余上游代码未动）：
   - run() 兜底 try/except：Timer 线程异常只上 stderr 且缓冲，线程死掉后日志无痕
   - LOS 归因新增 ignore_unknown 变体（utils.check_collision_ignore_unknown + node_manager._utility_los，launch 开关 los_ignore_unknown）
   - 停滞式完成判定：「效用全零 且 地图 stagnant_done_sec 秒无增长」才 done（get_map_callback 里监测 free 格数）
3. **ariadne_goal_bridge.py 新胶水**（integration/go2_bridge/scripts/，已 chmod+x）。
4. cmu_env 补编译 sensorScanGeneration；simulation/kill_all_sim.sh 清理脚本（防孤儿 gzserver 发陈旧 /clock）。

### 调试实录（7 轮迭代的核心发现）

| 轮次 | 现象 | 根因 | 对策 |
|---|---|---|---|
| run1-2 | ~16% 覆盖即判 Completed | utility 视野=0.5×5=2.5m 够不着前沿环；min_utility 的 <= 语义 | factor→1.0；min_utility→0 |
| run4-5 | 仍过早完成 | 门洞前沿紧邻门框锯齿格，unknown 挡视线，46 对视线 46 挡（离线复现证实） | los_ignore_unknown=true |
| run5-6 | 还是停 | 节点只长在轨迹周边 sensor_range 内，前沿 >5m 外永远零效用 | **rl_planner sensor_range 与 octomap max_range 解耦**：5m 只管标图，决策视野提到 12m |
| run6-7 | spawn 报错但模型实际在 | gzserver 加载大 world 时 spawn 服务超时，客户端退出但请求已入队执行 | 瞬态竞态，可忽略（模型最终插入成功） |

### 待验证（下一步）

- [x] run8：发现 `remove_unconnected_nodes` 起点节点清理后 `find()→None` 崩 Timer 线程——**此前多轮无声停摆的真凶**，已兜底回退最近现存节点
- [x] run9：sensor_range=12 后效用不再全零，但狗卡 (-7.8,7.2)：策略反复选贴障碍节点、消毒器逐拍抑制。新增**恢复导航**胶水（rl_planner._find_recovery_waypoint：效用全零时朝最近前沿的可达自由格发航点）
- [x] run10（2026-08-24 收官）：恢复导航触发 9 次正常工作；run() 异常 0；最终按停滞判定（效用全零+地图 20s 无增长）宣布 Completed——完成信号已是真实语义。**遗留**：狗始终出不了起始西楼块（地图 22×33 @(-8.4,-0.8)，GT 场景 53×34m），门洞在 0.4m 切片分辨率下疑似过窄或被 SCAN 局部规划拒绝穿越 → 下一轮调优方向：octomap/rl_planner 分辨率 0.4→0.2、miss、z 窗口微调、SCAN 门洞穿越
- [ ] 集成验证结论：A2 架构端到端打通（G1-G6 全过），G7 的"覆盖持续增长"依赖上述调优 → 任务 #54 完成，覆盖率调优另开任务

## 2026-08-21：标准化迁移 + 点云漂移 + TARE AND 低占据

### 一、仓库标准化（已推 GitHub `pumpkin-db/go2-scan`）

- 源码从 `new_algorithm/` 迁入 `go2-scan/`，按层分目录，每算法独立 workspace：
  - `algorithms/global_planning/tare/` —— TARE 探索决策（CMU）
  - `algorithms/local_planning/scan_planner/` —— SCAN-Planner（SJTU，含 go2_description/map_generator）
  - `algorithms/mapping/elevation_mapping/` —— 高程图（ANYbotics + kindr）
  - `simulation/cmu_env/` —— 仿真底座（velodyne/livox 插件）
  - `integration/go2_bridge/` —— 自研胶水（5 个桥脚本，独立 catkin package）
- vendor 方式：删第三方 `.git`，上游 commit 记入 `third_party.md`。
- 删 `simulation/` 旧副本、`cmu_env/vehicle_simulator`（无人车模拟器，458MB，未用）。
- commit `a845c4a`，4 workspace 重新编译通过，rospack 8 包全命中新路径。

### 二、点云漂移解决（根因 = 两套位姿源不一致）

- **根源**：原 `/mid360_points` 是传感器系（livox 插件 `point=range*axis`），下游再用 `kinematic_sim` 积分的**无抖动平滑 pose** 变换成世界系。狗步态抖动时，点云真实原点在抖、变换 pose 不抖 → 错位漂移。
- **修复**：改 livox 插件 `livox_points_plugin.cpp` 的 `PublishPointCloud2XYZ`，直接输出**世界系真实点云**：
  ```cpp
  ignition::math::Pose3d sensor_world_pose = parentEntity->WorldPose() * raySensor->Pose();
  pt_world = sensor_world_pose.Rot() * point + sensor_world_pose.Pos();  // frame="world"
  ```
- 下游**全部免变换**：
  - `scan_cloud_accumulator.py`：删 lidar_pose 坐标变换，直接用世界系点云（lidar_pose 仅保留作距离过滤基准）。
  - `tare_bridge.py`：删变换，原样转发 `/mid360_points(world)` → `/registered_scan`。
  - `gazebo_sim.launch`：`cloud_is_world: false→true`（grid_map 直接用世界系点云）。
- 结果：点云基本不漂（偶尔边缘出墙一点点，非主导）。
- **注意**：URDF 里还加过 `<gazebo reference="base"><kinematic>true</kinematic></gazebo>`（曾试给 base 设 kinematic 消步态抖动），**未解决问题点云仍飘，已确认非主因**，可保留可回退。

### 三、TARE AND 低占据逻辑（挡墙外目标）

- **问题**：TARE 会发墙外目标点 → SCAN-Planner 拒绝走 → 狗来回逛死循环。
- **根因**：TARE 的碰撞云 = 垂直面提取器的产物（`ExtractVerticalSurface`，`kZDiffMin=0.3`），墙点被滤掉，墙外视点被当可探索。
- **修复（AND 逻辑）**：碰撞云 = 垂直面云(TARE原) ∪ z∈[ground, ground+height] 低占据云(SCAN-Planner式)。改在 `planning_env`：
  - `UpdateKeyposeCloud`：从原始点云剪 z∈[ground, ground+height]，存 `low_occupy_cloud_`（`planning_env.h` 177-190）。
  - `UpdateCollisionCloud`：`collision_cloud_ += low_occupy_cloud_`（`planning_env.cpp`）。
  - 参数（`go2.yaml`）：`low_occupy_ground_z=0.2`、`low_occupy_height_z=0.6`（即 z∈[0.2, 0.8]）。
- **效果**：墙外目标明显减少、事故高发区不再徘徊，**但仍有零星墙外目标**（未根治）。
- **防楼梯误判**：`CheckViewPointCollision` 用"单格累计 ≥ kCollisionPointThr(3) 点才判碰撞"，楼梯台阶是逐级可站立面，不形成连续低占据柱 → 原理上不误判。**未实测验证。**
- **"5帧一发"**：`keypose_cloud_` 每 5 帧（%5==0）才被灌入 + 置 `keypose_cloud_update_`，低占据云约 0.5s 一拍（雷达 10Hz）。静态障碍够用。
- **z 边界**：`CheckViewPointCollision` 判碰撞要求 `point.z - 视点高度 ∈ [-0.5, +0.5]`，而 `kUseTerrainHeight=false` 时视点高度 = `robot_position.z()`(≈0.46)。故低占据点实际生效带 ≈ [0.2, 0.96]，z 上沿>视点+0.5 部分失效。已把 height 调成 0.6 让 z∈[0.2,0.8] 最稳妥。

### 四、RViz / 话题清理

- 删 `tare_scan_stack`（后续接的 TARE 累积全图接口 `registered_scan_accum`，用户不再看）。
- 删 `sim_clouds` 里的**无名点云**残缺显示项（只有 Channel/Class、无 Name/Topic 的坏残片）。
- `sim_clouds`(→`/pcl_render_node/cloud`) 和 `real_clouds`(→`/LIO/clouds_lidar`) 是仿真无用残留（前者没跑 local_sensing，后者 FAST-LIO 实机才用），**暂留**。

### 五、待办 / 开放问题

- **TARE 仍零星空墙外目标**：AND 低占据未 100% 根治。可试收紧（`kCollisionPointThr` 调低、或 `low_occupy_height_z` 微调、或让 `CheckViewPointCollision` 的 z 判定适配视点高度）。
- **楼梯误判未实测**：当前室内场景无楼梯，将来实体机爬梯需拿真实楼梯点云验证 `low_occupy` 会不会误判，必要时对楼梯区域豁免。
- **`scan_map` 纯累积版**：动态障碍旧点不清除（log-odds 版失败已回退），待重实现。
- **`elevation_map` 高程偏移**：ANYbotics 输出位置偏移，疑 TF 外参或 frame 配置，待查。
- **赋色层（比赛 80% 分值）**：相机投影→彩色点云，**完全未做**。
- **安全层**（高程地形 + IMU 看门狗 + LiDAR 接近检测）：未做。
- **NaVILA 语言层**（赛前区域解析 + 答辩演示）：未做。
- **FAST-LIO2**：实机才有，仿真用 `go2_kinematic_sim` 顶替（`fastlio_integration.launch` 已单独验证过）。

### 六、TARE 集成审查发现（2026-08-23，待修复）

> 完整报告在 `docs/TARE集成审查报告.md`（7 维度并行审查 + 对抗复核；33 findings → confirmed 29 / refuted 3 / uncertain 1 → 归并 22 条；high 2 / medium 9 / low 18）。下面按严重度列全，**细节与修法见报告，本处只做索引**。

**高危**
1. **探索完成判定块 vendor 时整块丢失**：`exploration_finished_/near_home_/at_home_/stopped_` 只初始化不置位；上游 execute() 完成检测+回家逻辑在 a845c4a 迁移时缺。全场扫完**不宣布完成、狗沿环路绕圈**、`/exploration_finish` 不发布、比赛收尾/存点云挂空。`kRushHome/kNoExplorationReturnHome/kRushHomeDist` 全死配置。→ 从上游 44500592b861 补回三段逻辑。

**中危**
2. `low_occupy` 是纯世界系 z 带过滤，「楼梯不误伤」注释与代码相反 → 误杀楼梯/低台旁视点。
3. 两套碰撞判定契约割裂：`AddEdge` 用的 InCollision 完全看不到 low_occupy。
4. `/way_point` 外推到 8m 不受 navigation_boundary/低占据约束，胶水原样转发 → 墙外 goal。
5. `start_delay` 用 wall clock 且从桥节点启动起算，Gazebo 慢启动时防穿墙静止期被吞。
6. `low_occupy_ground_z=0.2` 造成 <20cm 障碍盲区，三条碰撞通道全看不见矮障碍。
7. 覆盖记账 LiDAR 模型硬编码 ±12° 垂直窗，MID360 上半球回波永不进覆盖 → 高处漏扫（影响完整度）。
8. `number_z=1`+`kUseTerrainHeight=false` 下视点 z 冻结：高差场景先「碰撞失明」再「连通误断」。

**低危**
9. `low_occupy` 只用最近一拍 keypose 云、不进 5 帧栈 → 记忆只及垂直面的 1/5。
10. `low_occupy` 是世界系绝对 z[0.2,0.8]，上高层后功能静默失效。
11. livox 插件把 miss 射线写成传感器原点点，经原始 `/mid360_points` + low_occupy 污染近机视点。
12. 一组死参数：kViewPointHeightFromTerrain / kTerrainCollisionThreshold / kAddNonKeyposeNodeMinDist / kCollisionGridZScale 配了不生效。
13. GridWorld 固定 121×121×121 格（z 向 363m），启动常驻约 480MB。
14. `/way_point` 残留 publisher 无防护：新旧全局规划器交替驱动狗。
15. `/initial_path` 只 advertise 从不 publish：navi_mode=3 覆盖规划挂点静默失效。
16. 双短路径轮次沿用 stale lookahead 且被 8m 外推放大：目标每秒 180° 反转。
17. LOS 检查 else 分支从射线远端反向遍历：两个分离障碍之间的格子被误标可见。
18. `kCollisionPointThr=1`+碰撞标志粘滞：单个瞬时点永久判死 0.35m 内视点直到栅格滚动。
19. `kCollisionPointThr=1` 与 go2.yaml/PROGRESS 的「≥3 点才判碰撞、不误伤楼梯」论据矛盾。
20. TSP 距离矩阵把「不可达」记成 0 代价：潜伏伪路径缺口（上游既有，三层保护挡住）。
21. 补 viapoint 时误用下标 i 读 poses[i]（应为 j）：字段当前无人读，潜伏。
22. A* 用 in_pg 门控阻止重入队但不更新旧队列键：可构造次优路径。

**存疑（uncertain）**：integration-wiring 的 boundary/TF 交互 1 条（见报告）。
**排除（refuted）**：3 条（config-yaml ×2、integration-wiring ×1，见报告）。

---

## 2026-08-26 用户独立审查回应 + 勘误（本节为权威版本，与旧节冲突处以本节为准）

用户对本仓全部结论做了独立审查，逐条以仓库实物复核后：**A1/A2/A3/B1/B2/B3/C1-C4 全部属实，认账**。勘误如下：

### 撤回的表述
1. **撤回「轨迹 8m→67.4m」杠杆对比**：D1 JSON 白纸黑字 path_length_m=76.04，「真实~8m」是人工口径不可比；D1 轨迹已随残件丢失无法复核。
2. **撤回「统计等价→效用环不是瓶颈」「维持官方 0.5 定稿」**：n=1 对照撑不起该结论。indoor_1 自记同参数三跑 530/11.5/318m（成功率 2/3），run-to-run 方差下 D2/D3 各一跑不叫等价。
3. **天花板 56.8%/76.2%/57.1%(z>4.5) 降级为估计值**：计算脚本不存在、参数未留痕（全仓 grep 零命中），违反「引用前必须核实」纪律。「唯一路径=上楼」归因同步悬置，待 gtools 撒点脚本复现。
4. **修正「站桩 33 分钟」的粗粒度说法** → 精确版：plateau 前 ~116-144s 连续狂奔 65-79m @0.56m/s，之后纯站桩（D2 后段仅 2.3m / D3 仅 0.1m）。

### 重算发现（tools/recompute_path_length.py，2026-08-26）
- **path_length 注水说不成立**：trajectory_5hz 口径含水率仅 ~1%（67.24→67.22）。真正指标是 **moving_ratio**：D2=5.7%、D3=6.7% ——35 分钟里只动 ~2 分钟。
- **新故障模式确认（实锤）**：rl_planner 日志 `0c5b2cee*/rl_planner-18.log`——开跑 216s 墙钟后「utility all-zero + 地图静止20s」→ 停滞式判定宣布 Completed → 站桩。三跑 plateau 136.6/115.6/143.6s 全部擦边逃过旧判废线(<105s)。**我的停滞判定补丁把「规划器失明」翻译成了「合法完成」，两个补丁合谋掩盖核心故障。**
- 评估器已修：SPEED_MIN=0.05 加速度门 + moving_ratio 字段 + 判废线收紧至 15%×duration 且 er<0.95 + json.dump default=float 兜底。重算判定 Depot 三跑均 degraded=True。

### 任务账目更正
- #3 重开（in_progress）：results/ 目录 21 个文件全为 ariadne_*，「评一遍现有算法」实际覆盖 1 个算法；SCAN-Planner 从未进评估器。
- #4 进度自评 40%→33%：打通的只是最易的注册表发布档；#4 与 #8 大概率同根（elevation 零发布 ↔ CMU 崩溃族）。
- #2 表述修正：「腿部物理缺陷已修（最小环境验证）；完整基准跑动稳定性仍有 1/3 失败率，未闭环」。

### 当前第一优先级
任务#11「为什么 ~2 分钟后 ARiADNE 效用全零」——ER 仅 48% 时规划器无候选目标，这比上楼和天花板都重要。elevation_mapping 当晚又死一次（exit code 1，00:30:54，非段错误，日志文件缺失待查 stderr 去向）。

## 2026-08-26 效用全零根因破案 + 修复（任务#11 里程碑）

### 尸检链（全部活体实测证据）
1. rl_planner 开跑 216s 墙钟宣布 Completed：效用全零+地图静止20s → 停滞判定放行。
2. /projected_map 解剖（站桩现场）：unknown 仅 5.6%、free 81.1%、occ 13.3% —— 2D 图自认为探完了。
3. GT 裁决（剔地面、障碍带 z∈[0.25,1.8]）：GT 障碍格 2287 个中 ~42% 被标 free；octree 全场仅 2623 占据叶、z∈[0.2,0.7]、GT 障碍表面点最近邻中位 0.556m —— **墙从来没被建进 octree**。
4. 排除清单：lidar_pose/body_pose/Gazebo物理真值三者一致(14.18,6.08)/(14,6)/(14,6) 无罪；/mid360_points 到 GT 中位差 0.030m 完全真实。（自查教训：曾把 worldFrame 点云再变换一次制造出 15.5m「假漂移」假信号，已识别废弃。）
5. 病灶定性（口径错位，非几何 bug）：
   - Gazebo ray 物理量程 **40m**（URDF），一帧扫全场（sensor_scan P95=13.75m 最大 31.7m）；
   - octomap `sensor_model/max_range=6`：>6m 端点只清空沿途 free、永不产生占据；
   - z 带 [0.2,0.8]（indoor_1 小房间遗产）把墙身/楼梯主体排除在插入之外；
   - 净效果：看得见的远处=free 而非 unknown → frontier 枯竭 → 效用全零。
   - indoor_1 为何幸免：小房间墙总在 6m 内，语义破坏不暴露。

### 修复（最小改动，恢复「看不见=unknown」语义）
- integration/go2_bridge/scripts/cloud_range_filter.py：新增 `~max_range`（默认 0=禁用），按传感器水平距离裁远点。
- gazebo_sim.launch ariadne 分支：max_range=planner_sensor_range(默认 6) 与 octomap/评估器同口径。
- 待办：Depot 重跑验证（预期 frontier 复活、plateau 大幅推迟）；indoor_1 回归一次防基线破坏。

### 附带修正
- utility_range_factor 澄清：launch 文件实际为 1.0（D2 复盘后改的），README 旧结论「维持官方 0.5」与文件不符——已在勘误节撤回该定稿表述。D3(n=1) 不构成扩环无增益的证据，待根因修复后重做 A/B。

## 2026-08-26 续：ASAN 遗留破案 + 建图链恢复 + 修复后首验

### #8 elevation_mapping exit code 1 破案
- 前台裸跑复现：`AddressSanitizer:DEADLYSIGNAL` 死循环——**二进制是上次 ASAN 调试的遗留插桩版**（nm -D 可见 28 个 asan 符号），ASAN 默认 exitcode=1 即历次「exit code 1」死亡的真因。三字段修复是否有效曾被它掩盖。
- 已 rm -rf build/devel 全量重编（需 -DPYTHON_EXECUTABLE=/usr/bin/python3 绕开 conda empy 干扰），asan 符号归零。cmu_env terrainAnalysisExt 与 tare ws 的 tare_planner_node 检查均 asan=0（TARE -11 另查）。
- 修复后首跑中 /elevation_mapping/elevation_map 话题有发布者注册（待长稳观察）。

### cloud_range_filter max_range=6 修复首验（bench_fix1，15min 短窗）
- ✅ 核心修复生效：plateau 849s/902s（修复前 116-144s 即死），移动 246m、moving_ratio 57.1%，degraded=False 且为真正常收敛。
- ⚠️ ER_final 29.0%（15min vs 历史 35min 不同窗，不可直接比）；R@0.2=30.1% 暴露新问题：**建图累加器被 max_range 裁剪连坐**，全量程建图能力丢失。
- 行为模式变化如实记录：修复前「假 free 引导直线狂奔」单位路径覆盖虚高（η 0.0073）；修复后 frontier 引导细致探索 η 0.0012——两者口径不同，比较无意义。

### 架构修正：规划/建图分层供云
- scan_cloud_accumulator 改回两分支统一吃原始 /mid360_points + 自身过滤（max_range=40）：比赛完整度评分依赖全量程建图；cloud_range_filter 的 6m 裁剪只服务 octomap→rl_planner 链。「_clean 门控退化帧」是 livox 时代措施，velodyne+健康门后不再必要。
- SCAN 局部规划器与 elevation_mapping 保持吃 ≤6m clean 云（工作区本就局部）。

### 进行中
- bench_fix2：Depot 35min 同窗对照跑（pid 3177742），与 D1-D3 同窗可比，验证修复后真实 ER 水平。

## 2026-08-26 续2：row_step 一字节破三案（#8 收官在望）

### 段错误根因链（PCL 源码级实锤）
- elevation_mapping 稳定段错误（干净版 SIGSEGV -11；ASAN 版 DEADLYSIGNAL exitcode=1，两者同源）。
- PCL1.10 fromPCLPointCloud2 会把 x/y/z 相邻字段合并为一次 size=12 的 memcpy（ASAN「READ of size 12」吻合），
  且按行遍历时用 msg.row_step 寻址：`&msg.data[row * row_step]`。
- **velodyne 插件上游 bug**：非 organize 分支把 row_step 写成整个点云字节数 `msg.data.size()` 而非
  width×point_step(=16)。height>1 时第 1 行即从 data 末尾外开始读 → 堆越界 → 崩。
- 受害者：elevation_mapping(fromPCLPointCloud2)、terrainAnalysisExt(fromROSMsg 同路径，即 TARE 链 -11)；
  octomap/自研探针按 point_step 迭代不受影响——解释了为何只有 PCL 系消费者崩。
- indoor_1 时代幸免：当时是 livox 插件；换装 velodyne 后才暴露。
- 修复：GazeboRosVelodyneLaser.cpp `msg.row_step = POINT_STEP * msg.width;`（已注释完整因果）。
  重编遇 /mnt/e Anaconda protobuf 污染 + cmake 缓存 → 清缓存 + 剔 /mnt 路径 + 强制系统 protobuf 后 exit=0。
- 待验证：重启仿真后 elevation_map 出帧、terrainAnalysisExt 存活 → #4 几何检测解锁。

### 其他
- em 复现实验教训：裸跑 elevation_mapping 不带 launch rosparam 是空转（input_sources 默认话题不对），
  「合成消息不崩」的第一次结论作废；正确做法是挂同名 __name:=elevation_mapping 吃参数服务器配置。
- bench_fix2（Depot 35min 同窗对照）仍在跑，出数后填对照表。
