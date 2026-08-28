# ZCODE_RUNBOOK.md — ZCode 驱动 WSL2 ROS/Gazebo 强制守则

> 长期记忆文件。存放于 ZCode 工作区，**不放入 go2-scan 项目仓库**。
> 来源：GPT 2026-08-28 指导（针对 go2-scan 调试中实际踩坑的复盘，见附录 A）。
> 适用范围：一切通过 ZCode（Windows）驱动 WSL2 内 ROS1 Noetic / Gazebo / catkin 工作的动作。
> 执行原则：每轮运行严格按 §标准流程 逐步走，任何一步失败，停止，不进入下一步。

## 1. 会话管理

### 每次长仿真前

必须确认自己运行在持久 WSL 会话中。

优先：

* `tmux`
* `screen`
* 持久 WSL shell

禁止：

```bash
wsl.exe ... "nohup roslaunch ... &"
```

然后直接结束 Windows 工具调用。

如果必须从 Windows 启动后台任务，至少使用：

```bash
setsid nohup ... < /dev/null >LOG 2>&1 &
```

启动后必须验证：

```bash
pgrep -af roslaunch
pgrep -af gzserver
```

**判定：**
Windows 调用结束后进程仍存活，才算启动成功。

---

## 2. 禁止复杂命令直接穿过 wsl.exe

涉及以下任一内容：

* 管道 `|`
* 多层引号
* awk/sed/grep复杂表达式
* 循环
* 多条 ROS 命令

一律写成 WSL 内脚本：

```bash
/tmp/go2_task.sh
```

再执行：

```bash
wsl -- bash /tmp/go2_task.sh
```

任何“结果为0/空”的观测，在用于推理前必须先验证命令本身执行正确。

---

## 3. 每个脚本固定初始化

脚本开头统一：

```bash
set -e
source /opt/ros/noetic/setup.bash
source /home/pumpkin-db/claude/raicom/go2-scan/devel/setup.bash
```

如果 Conda 会污染环境，先清理 Conda PATH/变量。

执行前检查：

```bash
which roscore
which roslaunch
which catkin_make
echo $ROS_DISTRO
```

必须得到正确 ROS Noetic 环境。

---

## 4. 每轮仿真前必须清场

启动前检查：

```bash
pgrep -af 'roslaunch|gzserver|gzclient|scan_planner|rl_planner|evaluator|diag'
rosnode list
```

如果存在上一轮相关进程，先结束。

然后再次执行同样检查。

**只有确认为空后才能启动下一轮。**

禁止“觉得应该杀干净了”就直接跑。

---

## 5. 每轮使用唯一 run_id

例如：

```text
run_20260828_103700
```

所有：

* roslaunch日志
* diag JSON
* PCD
* evaluator报告
* 临时文件

都必须带同一 run_id。

禁止复用：

```text
/tmp/diag.log
/tmp/run.log
```

旧文件不覆盖，新旧运行绝不能混在一起。

---

## 6. Windows/UNC 编辑后禁止直接相信增量编译

只要源码从 Windows / `\\wsl.localhost` 修改过：

至少执行：

```bash
touch <修改过的源文件>
```

重要 C++ 修改优先：

```bash
catkin_make clean
catkin_make
```

或明确删除对应目标后重编。

**编译失败必须查看完整错误附近内容，禁止只看 tail -4。**

---

## 7. 编译后必须验证新代码真的进入产物

先判断代码属于：

* executable
* shared library

检查：

```bash
ldd <node_binary>
```

然后对正确产物执行：

```bash
strings <binary-or-so> | grep 'UNIQUE_DIAG_MARKER'
```

每次关键修改临时加入唯一字符串。

**只有 strings 找到新字符串，才能把后续运行结果归因于新代码。**

---

## 8. 诊断代码必须防御初始化状态

所有临时诊断代码在访问：

* trajectory
* spline
* map
* odom
* path
* pointer
* vector index

之前必须检查：

```text
是否初始化
是否为空
尺寸是否合法
pointer是否有效
```

原则：

**宁可这次快照少字段，也不能让诊断代码改变系统行为或造成崩溃。**

---

## 9. 节点生死不能靠 roslaunch stdout 判断

优先级：

1. `pgrep / PID`
2. `rosnode list`
3. `rosnode info`
4. `/rosout`
5. 最后才看 launch stdout

对关键节点记录 PID：

```bash
pgrep -f scan_planner_node
```

周期检查 PID 是否变化。

如果 `respawn=true`：
PID 变化 = 节点发生过崩溃/重启。

---

## 10. 每次运行先做 30 秒健康检查

正式长跑前先确认：

```text
/clock 正常
/body_pose 正常
/projected_map 正常
/way_point 正常
/initial_path 正常
/cmd_vel 正常
scan_planner_node 存活
rl_planner 存活
```

同时确认：

* topic publisher/subscriber 数正确
* 没有重复 publisher
* sim time 单调推进
* 狗实际发生位移

健康检查不过，禁止进入长跑。

---

## 11. 长跑不能无反馈 sleep

禁止：

```bash
sleep 500
```

改成后台运行，并每 20～30 秒检查：

* 关键 PID
* body pose
* ER
* fatal/error计数
* 是否仍在移动

如果出现目标故障条件，立即停止采集并保存现场。

---

## 12. 任何“0”都必须二次确认

例如：

```text
0次 crash
0条 waypoint
0个 subscriber
0个 error
```

在用于判断前必须至少通过另一种通道交叉确认。

示例：

```text
grep结果=0
+
rosnode/PID也正常
```

才能认为“真的为0”。

---

## 13. sim time 与 RTF

当前 indoor_1 world 自带非零 `<sim_time>`（1119.89s，world 文件 line 6692）。

因此：

* 不要用“sim time是否从0开始”判断旧进程残留
* 判断残留必须看 PID / rosnode / process
* duration 应使用 sim-time差值

RTF > 1 在 headless Gazebo 中允许。

只要：

* sim time单调
* 所有评估和timeout统一使用sim time

就可以比较实验。

---

## 14. respawn 特别检查

当前已知风险：

`scan_planner_node respawn=true`
+
`ariadne_goal_bridge repub_dist=1.0`

可能导致：

节点崩溃重启
→ FSM重新等待路径
→ bridge认为 waypoint 没变化
→ 不重新发布
→ 狗永久冻结

所以每次 scan_planner PID 变化后立即检查：

```text
/initial_path 是否重新收到消息
FSM 是否重新进入规划状态
```

这个问题单独挂账，不能把“重启后冻死”误判为新的规划算法问题。

---

# 每轮运行标准流程

必须严格按照：

**清场**
→ **环境检查**
→ **编译验证**
→ **唯一run_id**
→ **启动**
→ **30秒健康检查**
→ **正式实验**
→ **短周期监控**
→ **故障触发立即抓快照**
→ **停止全部进程**
→ **确认清场**
→ **分析**

任何一步失败，停止，不进入下一步。

# 最重要原则

在得出任何代码/算法结论前，先确认：

**运行环境是真的干净、执行的是真的新代码、观测到的数据是真的。**

---

# 附录 A：踩坑记录（本守则的由来，2026-08-28）

1. **wsl.exe 会话拆除杀后台进程**：`nohup &` + 结束工具调用 → 进程树被杀，白跑一次。→ §1
2. **复杂命令内联穿过 wsl.exe 被引号/管道破坏**（≥3 次），“计数为 0”被误当真数据。→ §2、§12
3. **非交互 shell 缺 ROS 环境**：catkin_make not found；`ROS_DISTRO: unbound variable` 秒崩。→ §3
4. **僵尸编排进程污染新栈**：陈旧 /clock、新旧进程同写一个日志，证据混杂。→ §4、§5
5. **UNC 编辑 + 增量编译 mtime 陷阱（核心事故）**：改过的文件被跳过编译 + 代码本身有 Eigen 二义性编译错误从未暴露 → 新旧 .o 混链 → ABI 不一致 → scan_planner_node exit -11 重启循环 → 狗冻死。→ §6、§7
6. **构建失败只看 tail -4**，漏掉真实报错。→ §6
7. **bspline_opt 是动态库**，`strings 节点二进制` 验证方法错误，误判“没编进去”。→ §7
8. **诊断代码对未初始化轨迹调 evaluateDeBoorT** → 段错误。→ §8
9. **依赖 roslaunch stdout 判生死**：stdout 块缓冲吞掉死亡行。→ §9、§11
10. **长 sleep 阻塞交互**，用户观感“卡死”。→ §11
11. **gdb 未安装**，想用时才发现。→ 环境检查前置。
