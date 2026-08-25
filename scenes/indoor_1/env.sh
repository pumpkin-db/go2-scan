#!/bin/bash
# indoor_1 场景环境变量（默认场景）
GO2=$HOME/claude/raicom/go2-scan
SCENE_WORLD=$GO2/simulation/worlds/indoor_1.world
SCENE_GT=$GO2/maps/indoor_1.pcd
SPAWN_X=-7.5
SPAWN_Y=0.5
SPAWN_Z=0.25
SPAWN_YAW=0.0
export SCENE_WORLD SCENE_GT SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW
