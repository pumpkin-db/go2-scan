#!/usr/bin/env bash
# Physical HIMLoco Go2 + Gazebo Classic hotel launcher.  This intentionally
# does not source scan_planner: its kinematic go2_description has the same
# ROS package name as the physical model.
set -euo pipefail

# ROS Noetic's xacro/rospkg must use the system Python 3.8, not an inherited
# Conda Python. This mirrors the canonical 2D launcher.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda|anaconda|miniconda' | tr '\n' ':')"

if pgrep -x gzserver >/dev/null 2>&1; then
  echo "[launch_gazebo_sim_3D] FATAL: gzserver already running: $(pgrep -x gzserver | tr '\n' ' ')" >&2
  echo "[launch_gazebo_sim_3D] Stop it first with: bash simulation/kill_all_sim.sh" >&2
  exit 42
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2_ROOT="${GO2_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PHYSICAL_WS="${PHYSICAL_WS:-$GO2_ROOT/simulation/physical_go2_ws}"
SPAWN_MODE="stair_test"
REST_ARGS=()
for arg in "$@"; do
  case "$arg" in
    spawn_mode:=*) SPAWN_MODE="${arg#spawn_mode:=}" ;;
    scene:=hotel_stairs|scene:=hotel) ;; # hotel is intentionally the only 3D world
    scene:=*) echo "[launch_gazebo_sim_3D] unsupported scene: ${arg#scene:=} (only hotel_stairs)" >&2; exit 2 ;;
    *) REST_ARGS+=("$arg") ;;
  esac
done

SETUP="$PHYSICAL_WS/devel/setup.bash"
if [[ ! -f "$SETUP" ]]; then
  echo "[launch_gazebo_sim_3D] physical workspace is not built: $SETUP" >&2
  echo "[launch_gazebo_sim_3D] run: bash simulation/setup_physical_go2_deps.sh && (cd simulation/physical_go2_ws && catkin_make --pkg go2_scan_physical_sim robot_msgs robot_joint_controller rl_sar go2_description)" >&2
  exit 2
fi

# Do not let an already-sourced legacy workspace win the duplicate
# go2_description package lookup. The physical overlay must resolve first.
source /opt/ros/noetic/setup.bash
source "$SETUP"
PHYSICAL_GO2_DESC="$PHYSICAL_WS/src/rl_sar_zoo/go2_description"
RESOLVED_GO2_DESC="$(rospack find go2_description 2>/dev/null || true)"
RESOLVED_RLSAR="$(rospack find rl_sar 2>/dev/null || true)"
if [[ "$RESOLVED_GO2_DESC" != "$PHYSICAL_GO2_DESC" || "$RESOLVED_RLSAR" != "$PHYSICAL_WS/src/rl_sar" ]]; then
  echo "[launch_gazebo_sim_3D] FATAL: physical ROS overlay is not authoritative" >&2
  echo "  go2_description=$RESOLVED_GO2_DESC" >&2
  echo "  rl_sar=$RESOLVED_RLSAR" >&2
  exit 3
fi

export SCENE_SPAWN_MODE="$SPAWN_MODE"
# shellcheck disable=SC1091
source "$GO2_ROOT/scenes/hotel_stairs/env.sh"
export GAZEBO_MODEL_PATH="$GO2_ROOT/scenes/hotel_stairs/models:${GAZEBO_MODEL_PATH:-}"

echo "[go2-scan physical launcher]"
echo "  repo_root=$GO2_ROOT"
echo "  physical_ws=$PHYSICAL_WS"
echo "  world=hotel_stairs"
echo "  world_path=$SCENE_WORLD"
echo "  spawn_mode=$SPAWN_MODE"
echo "  spawn_pose=$SPAWN_X $SPAWN_Y $SPAWN_Z $SPAWN_YAW"
echo "  go2_description=$RESOLVED_GO2_DESC"
echo "  rl_sar=$RESOLVED_RLSAR"

exec roslaunch go2_scan_physical_sim hotel_physical_go2.launch \
  world_file:="$SCENE_WORLD" init_x:="$SPAWN_X" init_y:="$SPAWN_Y" \
  init_z:="$SPAWN_Z" init_yaw:="$SPAWN_YAW" "${REST_ARGS[@]}"
