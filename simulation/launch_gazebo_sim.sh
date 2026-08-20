#!/bin/bash
# Gazebo 场景仿真启动：indoor_1.world + Go2 狗 + velodyne LiDAR + SCAN-Planner
# 用法：bash launch_gazebo_sim.sh
set -e

# 1) 清理 conda/anaconda 污染（否则 cmake/protobuf/rospy 全乱）
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "conda|anaconda|miniconda" | tr '\n' ':')

CMU=$HOME/claude/raicom/new_algorithm/autonomous_exploration_development_environment
SCAN=$HOME/claude/raicom/new_algorithm/SCAN-Planner

# 2) source ROS + SCAN-Planner
source /opt/ros/noetic/setup.bash
source $SCAN/devel/setup.bash

# 3) 补 CMU 环境（velodyne 插件 + velodyne_description）
export CMAKE_PREFIX_PATH=$CMU/devel:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$CMU/devel/lib:$LD_LIBRARY_PATH
export GAZEBO_PLUGIN_PATH=$CMU/devel/lib:$GAZEBO_PLUGIN_PATH
# velodyne_description / livox_laser_simulation 只在 src 里（devel 没生成 package.xml），rospack 直指 src
export ROS_PACKAGE_PATH=$CMU/src/velodyne_simulator:$CMU/src:$ROS_PACKAGE_PATH

# 4) 启动
roslaunch scan_planner gazebo_sim.launch "$@"
