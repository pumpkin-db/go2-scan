#!/bin/bash
# Depot 场景环境变量（run_benchmark.sh / launch_gazebo_sim.sh source 用）
GO2=$HOME/claude/raicom/go2-scan
SCENE_WORLD=$GO2/scenes/depot/depot.world
SCENE_GT=$GO2/scenes/depot/gt/depot.pcd
SPAWN_X=-12.0
SPAWN_Y=0.0
SPAWN_Z=0.35
SPAWN_YAW=0.0
# model://Depot 解析路径
GAZEBO_MODEL_PATH=$GO2/scenes/depot/model:$GAZEBO_MODEL_PATH
export SCENE_WORLD SCENE_GT SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW GAZEBO_MODEL_PATH
