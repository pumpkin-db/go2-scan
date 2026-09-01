## Agent skills

## 临时文件规则

- 禁止在 Linux `/tmp` 中创建、保存或依赖任何项目临时文件。
- 跨项目临时文件统一放在 `/home/pumpkin-db/claude/raicom/tmp/`。
- 仅属于本项目的临时文件可放在仓库内明确命名且被 Git 忽略的临时目录。
- 启动脚本、测试和诊断命令不得依赖 `/tmp` 中遗留的 launch、配置或日志。

## Physical Go2 / hotel 启动硬规则

- 物理狗唯一入口是 `simulation/launch_gazebo_sim_3D.sh`；二维运动学狗仍由 `simulation/launch_gazebo_sim.sh` 启动，禁止混用两个同名 `go2_description`。
- 3D launcher 必须确认 `rospack find go2_description` 和 `rospack find rl_sar` 均来自仓库内 `simulation/physical_go2_ws`；不得 source SCAN workspace 后直接启动物理狗。
- hotel 是重场景。禁止 world、gzclient、Go2 并发盲目生成：先 paused 加载 world，等待 `hotel_L1/hotel_stair1/hotel_stair2` 存在且模型集合稳定，再等待 bounded render grace，之后 spawn Go2、验证完整 13 links，最后 unpause。
- Gazebo Model 列表中出现 `go2_gazebo` 不等于模型可见/可用。若模型不可见且 MoveTo 无效，先查 spawn 时序与 `GetModelProperties.body_names`，禁止继续乱改出生点或相机。
- paused world 中立即启动 controller spawner/switch 可能阻塞 controller manager。正确顺序：spawn 完整模型 → unpause → 12 joint controllers ready → HIMLoco。
- 自动起立不能依赖固定 sleep。`rl_sim` 必须同时确认模型已收到、12 关节状态均到达、body 已在低位稳定，再发送一次 keyboard `0` 等效 GetUp。
- physical workspace 重配前须剔除 Conda/Windows Anaconda，并检查 `build/CMakeCache.txt` 的 `Protobuf_DIR`、`absl_DIR`、`utf8_range_DIR`；缓存污染不是算法问题。
- 已人工验收的两个 physical spawn：`stair_test=(27.35,-33.50,1.00,1.5708)`；`exploration=(20.2509,-38.00,1.00,1.5708)`。修改后必须分别做 GUI 验收。

### Issue tracker

Issues and PRDs live in GitHub Issues for `pumpkin-db/go2-scan`. See `docs/agents/issue-tracker.md`.

### Triage labels

Use the default five-role triage vocabulary. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context repository. See `docs/agents/domain.md`.

## Current simulation decision

- 二维探索使用 `launch_gazebo_sim.sh`：deterministic kinematic body + visual gait，只验证探索、地图和界面，不作为楼梯物理证据。
- 三维楼梯使用 `launch_gazebo_sim_3D.sh`：HIMLoco + Gazebo gravity/contact/PD torque。物理狗与运动学狗必须进程、workspace、模型包隔离。
- synthetic `/sim/body_z_target`、terrain-follow 和 GeometrySupportQuery 均不是正式楼梯 backend；禁止用感知结果反向抬高仿真 body 制造楼层通过。
- 检查 body pose、LiDAR pose、高度及 `world/map` frame 的完整契约；不能假设切换 physical body 后旧 ARiADNE/SCAN 接口自然保持不变。

## Codex 协作工作规范

本项目采用局部理解、共同假设、定点验证、最小修改的协作方式。复杂任务不要立即大范围搜索、修改或反复试错。

### 先理解，再行动

- 指定文件、函数或模块时，先阅读相关代码。
- 说明其作用、输入、输出、状态变化、调用关系和关键依赖。
- 除非必要，不主动扫描整个项目；理解后再行动，不要立即修改。

### 每次只解决一个明确问题

复杂问题拆分处理。每轮优先说明当前观察、直接相关代码、最可能原因和最值得验证的下一步。

### 优先验证用户发现

用户提供报错、现象、日志、怀疑位置或判断时，优先围绕线索验证。若判断可能有误，用代码证据说明。

### 控制搜索范围

按以下顺序搜索：当前文件 → 直接调用者/被调用者 → 相关配置 → 相关数据结构 → 整个项目。不要为完整理解而读取大量无关文件。

### 大量查询前先报告

若需要大量源码、第三方代码、文档或 Issue 调查，先暂停并说明：调查问题、搜索关键词、目标库/文档/Issue 及当前线索，等待用户决定是否交给其他工具处理。

### 修改代码前说明

修改前明确问题位置、原因依据、准备修改内容和潜在影响；优先最小修改，不顺手重构无关代码。

### 不确定时明确标注

区分“代码确认”“高度怀疑”“需要实验验证”和“可能性”，尽量以代码、日志和实验结果为依据，不假装确定。

### 推荐汇报格式

复杂排查尽量简洁地使用：**当前理解**、**发现**、**判断**、**下一步**。若需要用户提供信息，明确说明所需内容。

最终目标是多轮、低成本、高信息密度协作，由用户决定调查方向，Codex 负责代码理解、验证和执行，减少无效搜索、重复推理、大范围读取和盲目试错。
