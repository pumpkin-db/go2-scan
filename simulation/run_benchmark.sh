#!/bin/bash
# 一键基准评测：无头启动仿真 → 处理 spawn 竞态 → 挂评估器 → 出报告
# 用法：bash simulation/run_benchmark.sh [global_planner:=ariadne|tare] [scene:=indoor_1] [max_min:=30]
set -u

GO2=$HOME/claude/raicom/go2-scan
PLANNER="global_planner:=ariadne"
SCENE_NAME="indoor_1"
MAX_MIN=30
for a in "$@"; do
  case "$a" in
    global_planner:=*) PLANNER="$a" ;;
    scene:=*) SCENE_NAME="${a#scene:=}" ;;
    max_min:=*) MAX_MIN="${a#max_min:=}" ;;
  esac
done

# 场景参数（出生点/世界/GT）
export GO2_ROOT=$GO2
# shellcheck disable=SC1091
source $GO2/scenes/$SCENE_NAME/env.sh || { echo "未知场景 $SCENE_NAME"; exit 1; }

export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "conda|anaconda|miniconda" | tr '\n' ':')
echo "[bench] 清理残留进程..."
bash $GO2/simulation/kill_all_sim.sh >/dev/null 2>&1 || true
sleep 3

echo "[bench] 启动仿真 ($PLANNER scene: $SCENE_NAME, 无头)..."
nohup bash $GO2/simulation/launch_gazebo_sim.sh $PLANNER scene:=$SCENE_NAME gui:=false rviz:=false \
      > /tmp/bench_sim.log 2>&1 &
SIM_PID=$!

# 等 gazebo 服务起来，然后处理 spawn 竞态（已知问题：首 spawn 可能超时但请求已入队）
sleep 40
SCAN=$GO2/algorithms/local_planning/scan_planner
source /opt/ros/noetic/setup.bash
source $SCAN/devel/setup.bash
CMU=$GO2/simulation/cmu_env
export ROS_PACKAGE_PATH=$CMU/src/velodyne_simulator:$CMU/src:$ROS_PACKAGE_PATH
export GAZEBO_MODEL_PATH
for i in 1 2 3; do
  OUT=$(timeout 60 rosrun gazebo_ros spawn_model -param robot_description -urdf \
        -model go2_description -x $SPAWN_X -y $SPAWN_Y -z $SPAWN_Z 2>&1 | grep -oE "(successfully spawned|entity already exists)")
  if [ -n "$OUT" ]; then echo "[bench] spawn OK ($OUT, 第${i}次尝试)"; break; fi
  echo "[bench] spawn 第${i}次失败，20s 后重试..."; sleep 20
done

echo "[bench] 挂载评估器（最长 ${MAX_MIN}min）..."
cd $GO2
ALGO="${PLANNER#global_planner:=}"
/usr/bin/python3 tools/evaluate_exploration.py \
    --algo "$ALGO" --scene "$SCENE_NAME" --live-min $((MAX_MIN * 60)) \
    2>&1 | tee /tmp/bench_eval.log

echo "[bench] 完成。报告在 $GO2/evaluation/results/"
echo "[bench] 停止仿真..."
bash $GO2/simulation/kill_all_sim.sh >/dev/null 2>&1 || true
