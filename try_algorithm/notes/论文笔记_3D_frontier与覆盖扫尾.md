# 论文笔记：Asymptotically-Bounded 3D Frontier + Becoy Coverage CPP + MA-SLAM

> 2026-08-26 阅读记录（bench 空窗）。来源均为已下载 PDF，要点经原文核对后摘录；
> 「对本项目启示」为我们的推断，已标注。任务 #6。

---

## 1. Asymptotically-Bounded 3D Frontier Exploration（RA-L 2026，Lima 组）

**解决什么问题**：经典 3D frontier 方法每轮要在全局点云里重算 frontier 集合，
复杂度随地图增长爆炸；本文给出渐近有界的 frontier 维护方法。

**核心机制（已核实）**：
1. **frontier 并入 raycast 前向模型**：不单独维护 frontier 点列表，而是把
   frontier 信息附着在 raycast 过程中增量更新，整体 O(|F|)。
2. **Alg.3 近距删除**：frontier 离机器人过近（进入传感器已充分观测区）即删除，
   避免在已探索边界反复产生候选。
3. **GP 回归估增益**：用高斯过程回归估计未观测区域的信息增益，用于排序候选。
4. 架构：SLAM 子图 → Octomap，与我们「FAST-LIO2 → octomap」管线同构。

**对本项目启示（推断）**：
- 我们刚破案的「效用全零」本质是 frontier 语义被口径错位摧毁（40m 物理量程 vs
  6m 更新半径）；该文在 3D 直接维护 frontier 可避免 ARiADNE 官方「3D→2D 投影」
  带来的口径失真。但违背 ARiADNE 官方原版原则（其核心卖点就是 2D 投影+RL），
  **仅记录，不改主线**。
- Alg.3 近距删除思路可低成本借鉴到我们自己的 frontier 后处理（若有）。

---

## 2. Becoy et al. Go2 Coverage CPP（Frontiers 2025，TU Delft）

**场景前提**：**先验 2D 地图已知** → 形态学骨架化 → 叶节点贪心 + Dijkstra 排序
→ FSM 执行覆盖路径。平台 Unitree Go2 + 相机。

**关键判断（已核实）**：前提是有先验地图，与比赛「完全未知场地」不符，
**不能作为探索主方案**。

**真正价值（推断）**：探索完成后的**扫尾覆盖级**——当 frontier 耗尽但 ER 未达
100% 时（正是我们 Depot plateau@862s 的形态），用骨架覆盖法做第二阶段补扫，
比继续让 frontier 探索空转更保完整度评分。开源 ROS2 实现，算法本体是
scipy.ndimage.morphology + networkx 可自写 Python 胶水（符合「现成算法+胶水」纪律）。

---

## 3. MA-SLAM（arXiv 25.11，2D DRL active SLAM）

**内容**：2D DRL 主动 SLAM，结构化张量环境表示；环境 400-520m²、Gmapping 2D 栅格。

**关键判断（已核实）**：
- 2D Gmapping 管线，与比赛 3D 彩色点云需求不匹配；
- DRL 训练成本高、跨场景泛化差（比赛场地未知）。
- **结论：价值低，不跟进。**

---

## 关联

- 效用全零根因与口径修复：见 `evaluation/results/README.md` Depot 结论第 2 条、
  PROGRESS.md 2026-08-26 续4。
- TARE 对照进行中：bench_tare（站桩根因 = CMU 节点 /state_estimation 饿死，
  已修 tare_bridge 双发位姿）。
