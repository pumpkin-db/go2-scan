#!/bin/bash
# go2-scan ARiADNE A2 链路体检：一条命令输出全部门禁状态。
# 用法：bash simulation/verify_run.sh   （需仿真已在跑）
export PATH="/opt/ros/noetic/bin:/usr/bin:/bin"
source /opt/ros/noetic/setup.bash 2>/dev/null

printf "%-22s %-12s %s\n" "话题" "频率" "判定"
for t in /mid360_points_clean /quad_0/body_pose /sensor_scan /projected_map /way_point /initial_path /cmd_vel; do
  r=$(timeout 4 rostopic hz "$t" 2>&1 | grep -m1 -oE "average rate: [0-9.]+")
  st="❌ DEAD"; [ -n "$r" ] && st="✅"
  printf "%-22s %-12s %s\n" "$t" "${r%-*}" "$st"
done

echo "--- 关键参数 ---"
for p in sensor_range utility_range_factor min_utility los_ignore_unknown stagnant_done_sec; do
  printf "%s = %s\n" "$p" "$(timeout 3 rosparam get /rl_planner/$p 2>/dev/null || echo 未设置)"
done

echo "--- 完成判定/异常计数（run 日志）---"
LOG=${1:-$HOME/claude/raicom/go2-scan/logs/ariadne_a2_run9.log}
for pat in "Exploration Completed" "run() 异常" "未达停滞阈值" "本拍不发"; do
  printf "%-24s %s 次\n" "$pat" "$(grep -c "$pat" "$LOG" 2>/dev/null)"
done

echo "--- 车辆位置 ---"
timeout 4 rostopic echo -n1 /quad_0/body_pose/pose/pose/position 2>/dev/null | head -2 | tr '\n' ' '; echo
