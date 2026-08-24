#!/bin/bash
# 核弹级清场重启：按进程类型全量杀 → 确认真零 → 单栈启动 → 确认只有一个 gzserver。
# 背景：2026-08-24 出现过双仿真栈共存（陈旧 roslaunch 未死透又拉起新栈），双 gzserver
# 抢发 /clock 导致时钟冻结、全链断粮。本脚本是唯一的可信重启路径。
# 用法：bash ~/claude/raicom/go2-scan/simulation/nuclear_restart.sh [run编号]
S=$HOME/claude/raicom/go2-scan

PATTERNS="roslaunch|rosmaster|rosout|gzserver|gzclient|rviz|scan_planner_node|go2_kinematic_sim|closed_loop_controller|gazebo_bridge.py|cloud_range_filter.py|scan_cloud_accumulator.py|octomap_server|sensorScanGeneration|rl_planner.py|ariadne_goal_bridge.py|elevation_mapping|map_pub|odom_visualization|go2_gait_publisher|robot_state_publisher|tare_bridge.py|tare_planner_node|navigation_boundary|tare_goal_bridge"

kill_all() {
  local pids
  pids=$(pgrep -f "$PATTERNS" 2>/dev/null | grep -vw $$)
  [ -n "$pids" ] && kill -9 $pids 2>/dev/null
}

# 杀三轮，确保父子关系全部斩断
for round in 1 2 3; do
  kill_all
  sleep 2
done

# 真零校验：任何相关名字都不允许存在
left=$(pgrep -f "$PATTERNS" 2>/dev/null | grep -vw $$)
if [ -n "$left" ]; then
  echo "[nuclear] 仍有残留: $(ps -o pid,comm -p $left --no-headers 2>/dev/null | tr '\n' ' ')"
  echo "[nuclear] 放弃启动，请手动处理"; exit 1
fi
echo "[nuclear] 环境已归零"

# 启动
N=${1:-$(ls "$S/logs"/ariadne_redo_run*.log 2>/dev/null | grep -oE "[0-9]+$" | sort -n | tail -1)}
[ -z "$N" ] && N=1
LOG="$S/logs/ariadne_redo_run$((N)).log"
cd "$S/simulation"
PYTHONUNBUFFERED=1 nohup bash launch_gazebo_sim.sh global_planner:=ariadne > "$LOG" 2>&1 &
echo "[nuclear] 已启动 run$N，日志: $LOG"

# 等 60 秒后核查：必须恰好 1 个 gzserver 且无 Spawn failed
sleep 60
ngz=$(pgrep -xc gzserver)
if [ "$ngz" != "1" ]; then
  echo "[nuclear] ❌ gzserver 数量=$ngz（应为1），栈异常"
elif grep -q "Spawn service failed" "$LOG"; then
  echo "[nuclear] ⚠️ spawn 曾报错（若点云正常则可忽略，属已记录瞬态竞态）"
else
  echo "[nuclear] ✅ 单栈运行正常"
fi
