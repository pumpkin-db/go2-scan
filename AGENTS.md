## Agent skills

### Issue tracker

Issues and PRDs live in GitHub Issues for `pumpkin-db/go2-scan`. See `docs/agents/issue-tracker.md`.

### Triage labels

Use the default five-role triage vocabulary. See `docs/agents/triage-labels.md`.

### Domain docs

This is a single-context repository. See `docs/agents/domain.md`.

## Current simulation decision

- 2026-08-27: CHAMP + Gazebo physics stopped leg flight, but introduced idle wobble, failed locomotion, and ARiADNE blue-tile pose drift.
- Check body pose, LiDAR pose, height, and `world/map` frame together; do not assume dynamic Gazebo pose preserves the existing ARiADNE contract.
- Fast fallback (preferred for basic demo): deterministic kinematic body motion + visual gait, with registered stair height changing body `z`. Physical contacts are optional.

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
