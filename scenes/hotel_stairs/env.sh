#!/usr/bin/env bash
# Self-contained three-floor hotel benchmark.
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE_WORLD="$SCENE_DIR/worlds/hotel_stairs.world"
SCENE_GT=""
SCENE_CONFIG="$SCENE_DIR/scene.yaml"
case "${SCENE_SPAWN_MODE:-exploration}" in
  exploration) SPAWN_KEY="exploration_start_spawn" ;;
  stair_test) SPAWN_KEY="stair_test_spawn" ;;
  *) echo "hotel_stairs: unknown spawn_mode '${SCENE_SPAWN_MODE}' (expected exploration|stair_test)" >&2; return 2 ;;
esac
read -r SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW < <(
  /usr/bin/python3 -c \
    'import sys, yaml; p=yaml.safe_load(open(sys.argv[1]))["spawns"][sys.argv[2]]; print(p["x"], p["y"], p["z"], p["yaw"])' \
    "$SCENE_CONFIG" "$SPAWN_KEY"
)
GAZEBO_MODEL_PATH="$SCENE_DIR/models:${GAZEBO_MODEL_PATH:-}"
# Geometry backend integration is intentionally not enabled here.
SCENE_EXTRA_ARGS="terrain_follow:=false stair_detect:=true"
export SCENE_WORLD SCENE_GT SCENE_CONFIG SPAWN_KEY SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW GAZEBO_MODEL_PATH SCENE_EXTRA_ARGS
