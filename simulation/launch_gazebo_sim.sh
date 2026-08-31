#!/bin/bash
# Gazebo 场景仿真启动：indoor_1.world + Go2 狗 + velodyne LiDAR + SCAN-Planner
# 用法：bash launch_gazebo_sim.sh
set -e

# 0) 单实例守卫：已有 gzserver 在跑就拒绝启动（孤儿 gzserver 会连上新 master 发布
#    陈旧 /clock 和 /mid360_points，把新栈污染成"世界加载了但雷达全瞎"的假象。
#    2026-08-25 下午的连环误诊皆源于此。先跑 kill_all_sim.sh 再来。）
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "[launch_gazebo_sim] FATAL: 检测到已存在的 gzserver (pid $(pgrep -x gzserver | tr '\n' ' '))，拒绝启动。"
  echo "[launch_gazebo_sim] 请先: bash $GO2_ROOT/simulation/kill_all_sim.sh 并复核 pgrep -x gzserver 为空"
  exit 42
fi

# 1) 清理 conda/anaconda 污染（否则 cmake/protobuf/rospy 全乱）
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "conda|anaconda|miniconda" | tr '\n' ':')

# repo root 从脚本自身位置推导（worktree-safe）：任何 checkout/worktree 都默认
# 加载"自己"的代码；可用环境变量 GO2_ROOT 显式覆盖。以下组件均可单独 override。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2_ROOT="${GO2_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
CMU="${CMU:-$GO2_ROOT/simulation/cmu_env}"
SCAN="${SCAN:-$GO2_ROOT/algorithms/local_planning/scan_planner}"
ELEV="${ELEV:-$GO2_ROOT/algorithms/mapping/elevation_mapping}"
TARE="${TARE:-$GO2_ROOT/algorithms/global_planning/tare}"
BRIDGE="${BRIDGE:-$GO2_ROOT/integration}"
STAIR_PERCEPTION="${STAIR_PERCEPTION:-$GO2_ROOT/algorithms/perception/stair_perception}"
echo "[go2-scan launcher]"
echo "[go2-scan launcher] repo_root=$GO2_ROOT"
echo "[go2-scan launcher] scan=$SCAN"
echo "[go2-scan launcher] cmu=$CMU"
echo "[go2-scan launcher] bridge=$BRIDGE"
echo "[go2-scan launcher] stair_perception=$STAIR_PERCEPTION"

# 2) source ROS + SCAN-Planner（注意：不能 source ELEV/TARE，catkin setup 会挤掉 SCAN 的 CMAKE_PREFIX_PATH）
source /opt/ros/noetic/setup.bash
source $SCAN/devel/setup.bash

# 3) 补 CMU 环境（velodyne 插件 + velodyne_description）
export CMAKE_PREFIX_PATH=$CMU/devel:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$CMU/devel/lib:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$CMU/devel/lib:$GAZEBO_PLUGIN_PATH
# velodyne_description / livox_laser_simulation 只在 src 里（devel 没生成 package.xml），rospack 直指 src
export ROS_PACKAGE_PATH=$CMU/src/velodyne_simulator:$CMU/src:$ROS_PACKAGE_PATH

# 4) 补 elevation_mapping 环境（同样手动补，不 source，避免挤掉 SCAN）
export CMAKE_PREFIX_PATH=$ELEV/devel:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$ELEV/devel/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$ELEV/src:$ROS_PACKAGE_PATH

# 5) 补 TARE 环境（全局探索决策层，同样手动补）
export CMAKE_PREFIX_PATH=$TARE/devel:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$TARE/devel/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$TARE/src:$ROS_PACKAGE_PATH

# 5b) 补 ARiADNE 环境（RL 探索决策层：纯 Python，不用 catkin_make，rospack 直指 src/ 即可找到 rl_planner 包）
ARIADNE="${ARIADNE:-$GO2_ROOT/algorithms/global_planning/ariadne}"
export ROS_PACKAGE_PATH=$ARIADNE/src:$ROS_PACKAGE_PATH

# 6) go2_bridge（自研胶水，纯 Python，rospack 直指 integration/）
export ROS_PACKAGE_PATH=$BRIDGE:$ROS_PACKAGE_PATH

# 6a) 可移植楼梯感知（独立 catkin workspace；仿真/实机共用同一核心）
export CMAKE_PREFIX_PATH=$STAIR_PERCEPTION/devel:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$STAIR_PERCEPTION/devel/lib:$LD_LIBRARY_PATH
export ROS_PACKAGE_PATH=$STAIR_PERCEPTION/src:$ROS_PACKAGE_PATH
export PYTHONPATH=$STAIR_PERCEPTION/devel/lib/python3/dist-packages:${PYTHONPATH:-}

# 6b) 场景支持：scene:=<name> 时 source scenes/<name>/env.sh（世界/GT/出生点/model路径）
SCENE=""
SPAWN_MODE=""
REST_ARGS=()
MULTI_FLOOR_REQUEST=0
for a in "$@"; do
  case "$a" in
    scene:=*) SCENE="${a#scene:=}" ;;
    spawn_mode:=*) SPAWN_MODE="${a#spawn_mode:=}" ;;
    multi_floor:=true|floor_handoff:=true) MULTI_FLOOR_REQUEST=1; REST_ARGS+=("$a") ;;
    *) REST_ARGS+=("$a") ;;
  esac
done
# The canonical multi-floor benchmark is self-contained hotel_stairs. Keep the
# legacy indoor_1 default for single-floor invocations; Depot is explicit only.
if [ -z "$SCENE" ] && [ "$MULTI_FLOOR_REQUEST" = 1 ]; then
  SCENE="hotel_stairs"
fi
EXTRA_ARGS=()
if [ -n "$SCENE" ]; then
  export SCENE_SPAWN_MODE="${SPAWN_MODE:-}"
  # shellcheck disable=SC1091
  source $GO2_ROOT/scenes/$SCENE/env.sh 2>/dev/null || { echo "未知场景: $SCENE"; exit 1; }
  echo "[go2-scan launcher] world=$SCENE"
  echo "[go2-scan launcher] world_path=$SCENE_WORLD"
  echo "[go2-scan launcher] spawn_mode=${SPAWN_MODE:-exploration}"
  echo "[go2-scan launcher] spawn_pose=$SPAWN_X $SPAWN_Y $SPAWN_Z $SPAWN_YAW"
  EXTRA_ARGS+=(world_file:="$SCENE_WORLD" gt_pcd:="$SCENE_GT"
               init_x:="$SPAWN_X" init_y:="$SPAWN_Y" init_z:="$SPAWN_Z"
               init_yaw:="$SPAWN_YAW" ${SCENE_EXTRA_ARGS:-})
  # 场景自带高程图窗口配置（Depot 等非 indoor_1 中心场景必须）
  if [ -f "$GO2_ROOT/scenes/$SCENE/elevation/elevation_map.yaml" ]; then
    ELEV_CFG_DIR=$GO2_ROOT/scenes/$SCENE/elevation
  fi
fi

# 7) 启动（场景自带高程配置时覆盖 cfg_dir）
ELEV_ARGS=()
[ -n "${ELEV_CFG_DIR:-}" ] && ELEV_ARGS+=(cfg_dir:="$ELEV_CFG_DIR")
# 显式命令行参数最后传入，允许测试时覆盖场景默认值。
roslaunch scan_planner gazebo_sim.launch "${EXTRA_ARGS[@]}" "${ELEV_ARGS[@]}" "${REST_ARGS[@]}"
