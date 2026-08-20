# 20 · TravExplorer 调研（2026-08-18，三路并行精读）

> 仓库：`new_algorithm/TravExplorer`（github.com/wuyi2121/TravExplorer）。
> **定性：思想参考，不可入代码栈——代码未开源。**

## 基本信息（✅ 已核实）

- 论文：arXiv:2605.19958《TravExplorer: Cross-Floor Embodied Exploration via Traversability-Aware 3-D Planning》，无录用信息。
- 作者：Han Zheng, Zhe Chen, Yudong Huang, Haoran Liu, Jinghao Wang, Ming Yang, Tong Qin*——上海交大，与 SCAN-Planner 同组。
- **代码状态：零代码**。README:23 "Code will be released upon acceptance."；main 分支仅 README/LICENSE/assets 5 图；377M = gh_pages 分支 42 个演示 mp4（363M）+ 网站静态文件。无权重/无 pcd/无 bag。
- 许可：Apache-2.0（仓库）；网站 CC BY-SA 4.0。

## 它是什么 / 不是什么

- **是**：跨楼层物体目标探索系统。模块（framework.jpg + 主页 index.html:145-157）：
  - Volumetric Mapping：Occupancy / Traversability / ESDF 三层体素地图；
  - Traversable Frontiers：2D 光线投射（效率）+ 3D 语义可通行性（多层感知）+ Active Perception（2-DoF 云台主动扫视补 FOV）；
  - Semantic Guidance：开放词汇分割→概率 instance map（物体候选）；image-text matching→空间 value map（给 frontier 打分）；VLM 选房间；
  - Hierarchical Planning：TSP global tour → foothold-guided 3D path search → vertically constrained local planning（= SCAN-Planner，README:56）；execute-review 机制。
- **不是 VLA** ✅：学习组件全部是冻结零样本感知模块，决策为经典分层规划。用户「VLA 找目标」猜测对一半：目标以开放词汇文本给出（"blue trash can in a classroom"），但解析靠模块化管线。
- **不是无人机项目**：Go2 + MID360 + d435i(云台) + Elevator-LIO，与本平台硬件高度重合。
- 楼梯：走楼梯（benchmark 有 UPSTAIR/DOWNSTAIR/MIDSTAIR），机制 = 3D traversability 多层感知 + foothold 3D 路径 + 垂直约束局部规划——**SCAN-Planner 缺的楼梯层它设计了，但没开源**。

## 与我们的接口关系

- SCAN-Planner 侧已核实：mode3=REFERENCE_PATH（scan_replan_fsm.h:43-48），订阅 `/initial_path`（nav_msgs/Path，scan_replan_fsm.cpp:113/357），取 pose.position、**z 加 body_height_**、0.5m 重采样。喂路径时高度基准要对齐。
- TravExplorer 侧：零代码，`initial_path` 字符串全仓不出现；推断其 3D Path Search 输出即 /initial_path（架构吻合但无代码证据）。

## 对我们架构的意义（思想清单，待自研/找开源实现）

1. **3D traversable frontier**（替代纯 2D 前沿）：2D 光线投射做效率 + 3D 可通行性做多层正确性——可作我们「3D 前沿补漏」的设计模板。
2. **value map 给 frontier 打分**：语义/信息价值引导选择，而非最近距离。
3. **TSP 全局巡游 + execute-review**：访问序列编排 + 走完复核漏扫——对应比赛「完整度」评分。
4. **Active Perception 云台**：我们没有云台，但「定点转头补扫」可用狗自身 yaw 实现（思想可借鉴）。
5. 基准对照：HM3D/MP3D vs ApexNav（ApexNav 是同类四足探索 baseline，可列入后续调研名单）。

## 坑

- 别信网上「TravExplorer 代码已开源」的搜索条目（核实过与官方 README 矛盾，仿冒/过期信息）。
- 若将来放码：警惕 ML 语义可通行性依赖（torch/onnxruntime/GPU）与 Habitat 系仿真栈，与 Noetic 割裂。
- 克隆建议 shallow / 单分支，371M pack 全是视频。
