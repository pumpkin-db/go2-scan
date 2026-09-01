#!/bin/bash
# 彻底清理 go2-scan 仿真的全部残留进程。
# 背景：TaskStop/pkill 不彻底时，孤儿 gzserver 会在新 master 出现后重连并发布陈旧
# /clock（表现为新栈一启动 sim time 就 >1000、spawn_go2 报 "Spawn service failed"、
# 全链路冻结）。每次重启仿真前先跑本脚本，并以 pgrep 复核为空为准（2026-08-24）。
pkill -9 -f roslaunch 2>/dev/null
pkill -9 -x rosmaster 2>/dev/null
pkill -9 -x rosout 2>/dev/null
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -x rl_sim 2>/dev/null
for pat in scan_planner_node go2_kinematic_sim closed_loop_controller \
           gazebo_bridge.py cloud_range_filter.py scan_cloud_accumulator.py \
           octomap_server sensorScanGeneration rl_planner.py ariadne_goal_bridge.py \
           elevation_mapping map_pub odom_visualization go2_gait_publisher \
           stair_detector stair_tracker stair_gt_backend \
           stair_traverser motion_arbiter terrain_follow_sim_adapter \
           controller_manager/spawner \
           robot_state_publisher realTimePlot visualizationTools tare_bridge.py \
           tare_planner_node navigation_boundary_publisher.py tare_goal_bridge.py; do
  pkill -9 -f "$pat" 2>/dev/null
done
# rviz 单独处理（避免误杀其他含 rviz 字样的进程名）
pkill -9 -f "rviz.*default.rviz" 2>/dev/null
sleep 2

# 复核：还有任何残留则再杀一轮
left=$(pgrep -f "[g]zserver|[g]zclient|[r]osmaster|[r]osout|[r]oslaunch|[r]l_sim|controller_manager/[s]pawner|[s]can_planner_node|[g]o2_kinematic_sim|[o]ctomap_server|[s]ensorScanGeneration|[r]l_planner.py|[a]riadne_goal_bridge.py" 2>/dev/null)
if [ -n "$left" ]; then
  echo "[kill_all_sim] 残留 PID: $left ，再补一刀"
  kill -9 $left 2>/dev/null
  sleep 1
fi
echo "[kill_all_sim] done"
exit 0
