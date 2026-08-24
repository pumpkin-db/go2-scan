#!/bin/bash
# 重启 ARiADNE 仿真：杀干净（带等待复核）→ 后台重启 → 日志到 logs/ariadne_redo_run<N>.log
# 用法：bash ~/claude/raicom/go2-scan/simulation/restart_ariadne.sh [run编号]
# 教训（2026-08-24 run4）：kill 后立即检查有竞态——垂死的 roslaunch 可能再拉起 gzserver，
# 双 gzserver 抢 master 发陈旧 /clock → spawn 失败、狗模型缺失、全链断粮。
S=$HOME/claude/raicom/go2-scan

# 1) 杀两轮，中间隔 2 秒
bash "$S/simulation/kill_all_sim.sh"
sleep 2
bash "$S/simulation/kill_all_sim.sh" >/dev/null

# 2) 等待真正干净：连续 5 秒无任何仿真进程才算干净
for i in $(seq 1 20); do
  left=$(pgrep -x gzserver; pgrep -x rosmaster; pgrep -x gzclient)
  if [ -z "$left" ]; then
    sleep 1
    left2=$(pgrep -x gzserver; pgrep -x rosmaster; pgrep -x gzclient)
    [ -z "$left2" ] && break
  fi
  sleep 1
done
if [ -n "$(pgrep -x gzserver; pgrep -x rosmaster)" ]; then
  echo "[restart] 20 秒后仍有残留，放弃启动（手动排查）"; exit 1
fi

# 3) 编号
N=${1:-$(ls "$S/logs"/ariadne_redo_run*.log 2>/dev/null | grep -oE "[0-9]+$" | sort -n | tail -1)}
[ -z "$N" ] && N=1
LOG="$S/logs/ariadne_redo_run$((N)).log"

# 4) 启动 + 等 master 起来后再返回
cd "$S/simulation"
PYTHONUNBUFFERED=1 nohup bash launch_gazebo_sim.sh global_planner:=ariadne > "$LOG" 2>&1 &
echo "[restart] 已后台启动 run$N，日志: $LOG"
