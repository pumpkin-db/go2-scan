#!/bin/bash
# run7（filter_ground 修复版）验证流水：
#   核弹重启 -> 等 /projected_map 流动 -> 探索 150 秒 -> 快照 + GT 对照，全部落盘 logs/
S=$HOME/claude/raicom/go2-scan
export PATH=/opt/ros/noetic/bin:/usr/bin:/bin
source /opt/ros/noetic/setup.bash 2>/dev/null

bash $S/simulation/nuclear_restart.sh

# 等 projected_map 活起来（最长90秒）
for i in $(seq 1 30); do
  r=$(timeout 4 rostopic hz /projected_map 2>&1 | grep -m1 -oE "average rate: [0-9.]+")
  [ -n "$r" ] && break
  sleep 3
done
echo "projected_map: ${r:-DEAD}"

# 探索期：让狗走 150 秒
sleep 150

P=/home/pumpkin-db/miniconda3/envs/ariadne/bin/python
timeout 60 $P $S/tools/probe_map_once.py "RUN7-filter_ground" > $S/logs/run7_map.txt 2>/dev/null
timeout 120 $P $S/tools/probe_occ_vs_gt.py > $S/logs/run7_gt.txt 2>/dev/null
echo "=== 快照头部 ==="; head -3 $S/logs/run7_map.txt
echo "=== GT 对照头部 ==="; grep -aE "占据格共|幻影" $S/logs/run7_gt.txt | head -4
