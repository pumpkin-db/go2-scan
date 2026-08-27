#!/bin/bash
# Depot 场景环境变量（run_benchmark.sh / launch_gazebo_sim.sh source 用）
GO2=$HOME/claude/raicom/go2-scan
SCENE_WORLD=$GO2/scenes/depot/depot.world
SCENE_GT=$GO2/scenes/depot/gt/depot.pcd
SPAWN_X=-12.0
SPAWN_Y=0.0
SPAWN_Z=0.35
SPAWN_YAW=0.0
# model://Depot 解析路径（:- 兜底空值，调用方可能开 set -u）
GAZEBO_MODEL_PATH=$GO2/scenes/depot/model:${GAZEBO_MODEL_PATH:-}
# 材质机制（2026-08-27 查清，对齐官方模型用法）：不设 GAZEBO_RESOURCE_PATH。
# 材质由 model.sdf 里 <script><uri>model://Depot/materials/...</uri> 声明，
# Gazebo 按 model:// 解析（同 ~/.gazebo/models 官方模型，如 brick_box_3x1x3）。
# 早年「全白」根因：转换时 script uri 写成相对路径，经典 Gazebo 解析不了。
# 多层场景开地形跟随（kinematic_sim z 查高程图）。
# 数据源用场景 GT 高程二进制（干净无天花板，2026-08-27；感知高程图楼梯区不可用）
# stair_detect:=true：Depot 自动起楼梯检测（注册表兜底橙箭头，见 gazebo_sim.launch 8c）
SCENE_EXTRA_ARGS="terrain_follow:=true terrain_source:=gt_file gt_elev_file:=$GO2/scenes/depot/gt/elev_gt.bin stair_detect:=true"
export SCENE_WORLD SCENE_GT SPAWN_X SPAWN_Y SPAWN_Z SPAWN_YAW GAZEBO_MODEL_PATH SCENE_EXTRA_ARGS
