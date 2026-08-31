#!/usr/bin/env bash
# Self-contained three-floor hotel benchmark.
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE_WORLD="$SCENE_DIR/worlds/hotel_stairs.world"
SCENE_GT=""
SPAWN_X=20.0
SPAWN_Y=-35.0
SPAWN_Z=0.55
SPAWN_YAW=0.0
GAZEBO_MODEL_PATH="$SCENE_DIR/models:${GAZEBO_MODEL_PATH:-}"
# Geometry backend integration is intentionally not enabled here.
SCENE_EXTRA_ARGS="terrain_follow:=false stair_detect:=true"
export SCENE_WORLD SCENE_GT SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW GAZEBO_MODEL_PATH SCENE_EXTRA_ARGS
