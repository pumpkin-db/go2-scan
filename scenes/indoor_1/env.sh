#!/bin/bash
# indoor_1 场景环境变量（默认场景）
SCENE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GO2="$(cd "$SCENE_DIR/../.." && pwd)"
SCENE_WORLD=$GO2/simulation/worlds/indoor_1.world
SCENE_GT=$GO2/maps/indoor_1.pcd
SPAWN_X=-7.5
SPAWN_Y=0.5
SPAWN_Z=0.32
SPAWN_YAW=0.0
export SCENE_WORLD SCENE_GT SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW
