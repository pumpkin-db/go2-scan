#!/bin/bash
# go2-scan ARiADNE 链路体检（官方契约重做版）：一条命令输出全部门禁状态。
# 用法：bash simulation/verify_run.sh [日志路径]   （需仿真已在跑）
export PATH="/opt/ros/noetic/bin:/usr/bin:/bin"
source /opt/ros/noetic/setup.bash 2>/dev/null

echo "=== G1 话题活性 ==="
printf "%-24s %-12s %s\n" "话题" "频率" "判定"
for t in /mid360_points_clean /quad_0/body_pose /quad_0/lidar_pose /sensor_scan /projected_map /way_point /initial_path /cmd_vel; do
  r=$(timeout 4 rostopic hz "$t" 2>&1 | grep -m1 -oE "average rate: [0-9.]+")
  st="❌ DEAD"; [ -n "$r" ] && st="✅"
  printf "%-24s %-12s %s\n" "$t" "${r%-*}" "$st"
done

echo "=== G2a TF 解析 ==="
echo -n "world→map: "; timeout 4 rosrun tf tf_echo world map 2>/dev/null | grep -A2 Translation | head -3 | tr '\n' ' '; echo
echo -n "map→sensor_at_scan: "; timeout 4 rosrun tf tf_echo map sensor_at_scan 2>/dev/null | grep -A2 Translation | head -3 | tr '\n' ' '; echo

echo "=== G2c 地图范围（origin+尺寸应与场景边界吻合）==="
timeout 5 rostopic echo -n1 /projected_map/info 2>/dev/null | grep -E "^(width|height)|^    x:|^    y:"

echo "=== 关键参数 ==="
for p in /rl_planner/sensor_range /rl_planner/utility_range_factor /rl_planner/min_utility /rl_planner/map_resolution; do
  printf "%s = %s\n" "$(basename $p)" "$(timeout 3 rosparam get $p 2>/dev/null || echo 未设置)"
done

echo "=== 完成判定/异常计数 ==="
LOG=${1:-$HOME/claude/raicom/go2-scan/logs/ariadne_redo_run1.log}
for pat in "Exploration Completed" "Traceback" "Transform error"; do
  printf "%-26s %s 次\n" "$pat" "$(grep -c "$pat" "$LOG" 2>/dev/null)"
done

echo "=== 车辆位置 ==="
timeout 4 rostopic echo -n1 /quad_0/body_pose/pose/pose/position 2>/dev/null | head -3 | tr '\n' ' '; echo
