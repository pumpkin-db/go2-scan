#!/bin/bash
# STL 碰撞修复固化 + Depot 基线重跑（2026-08-25）
# 前提：/tmp/depot_variants/stlcol 实测探针已确认射线恢复（unique>100 且有环带命中）
set -eu
GO2=$HOME/claude/raicom/go2-scan

echo "[1/4] 清场..."
bash $GO2/simulation/kill_all_sim.sh >/dev/null 2>&1
for i in $(seq 1 15); do pgrep -x gzserver >/dev/null || break; sleep 2; done

echo "[2/4] 固化：STL 碰撞写入 assets/model.sdf，恢复 Depot 链接"
cp /tmp/depot_variants/stlcol/Depot/model.sdf $GO2/scenes/depot/model/assets/model.sdf
[ -e $GO2/scenes/depot/model/Depot ] || mv $GO2/scenes/depot/model/Depot.bak_test $GO2/scenes/depot/model/Depot
ls -la $GO2/scenes/depot/model/ | grep Depot

echo "[3/4] 重启 Depot 基线（后台）"
cd $GO2
nohup bash simulation/run_benchmark.sh global_planner:=ariadne scene:=depot max_min:=35 \
    > /tmp/bench_depot_final.log 2>&1 &
echo "bench 已启动，日志 /tmp/bench_depot_final.log"

echo "[4/4] 提交固化修复"
cd $GO2
git add scenes/depot/model/assets/model.sdf scenes/depot/model/assets/meshes/Depot_collision.stl \
        scenes/gtools/*.py scenes/gtools/*.sh simulation/spawn_go2.py simulation/run_benchmark.sh \
        simulation/launch_gazebo_sim.sh PROGRESS.md third_party.md 2>/dev/null || true
git commit -m "fix(depot): 碰撞网格换用米制 STL 绕开 ColladaLoader 单位歧义；spawn 全托管+健康门；单实例守卫" || echo "(无变更可提交)"
echo "完成。"
