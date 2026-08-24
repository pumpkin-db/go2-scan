#!/bin/bash
# 重启 ARiADNE 仿真：杀干净 → 后台重启 → 日志到 logs/ariadne_redo_run<N>.log
# 用法：bash ~/claude/raicom/go2-scan/simulation/restart_ariadne.sh [run编号，默认自动+1]
S=$HOME/claude/raicom/go2-scan

# 1) 杀干净
bash "$S/simulation/kill_all_sim.sh"

# 2) 确认无残留
left=$(pgrep -x gzserver; pgrep -x rosmaster)
if [ -n "$left" ]; then
  echo "[restart] 仍有残留: $left ，退出"; exit 1
fi

# 3) 编号：默认取已有 redo_run 日志最大编号 +1
N=${1:-$(ls "$S/logs"/ariadne_redo_run*.log 2>/dev/null | grep -oE "[0-9]+" | sort -n | tail -1)}
[ -z "$N" ] && N=1
LOG="$S/logs/ariadne_redo_run$((N)).log"

# 4) 后台启动
cd "$S/simulation"
PYTHONUNBUFFERED=1 nohup bash launch_gazebo_sim.sh global_planner:=ariadne > "$LOG" 2>&1 &
echo "[restart] 已后台启动 run$N，日志: $LOG"
