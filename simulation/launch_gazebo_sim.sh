#!/bin/bash
# Gazebo 场景仿真启动：indoor_1.world + Go2 狗 + velodyne LiDAR + SCAN-Planner
# 用法：bash launch_gazebo_sim.sh
set -e

# 1) 清理 conda/anaconda 污染（否则 cmake/protobuf/rospy 全乱）
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE "conda|anaconda|miniconda" | tr '\n' ':')

CMU=$HOME/claude/raicom/go2-scan/simulation/cmu_env
SCAN=$HOME/claude/raicom/go2-scan/algorithms/local_planning/scan_planner
ELEV=$HOME/claude/raicom/go2-scan/algorithms/mapping/elevation_mapping
TARE=$HOME/claude/raicom/go2-scan/algorithms/global_planning/tare
BRIDGE=$HOME/claude/raicom/go2-scan/integration

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
ARIADNE=$HOME/claude/raicom/go2-scan/algorithms/global_planning/ariadne
export ROS_PACKAGE_PATH=$ARIADNE/src:$ROS_PACKAGE_PATH

# 6) go2_bridge（自研胶水，纯 Python，rospack 直指 integration/）
export ROS_PACKAGE_PATH=$BRIDGE:$ROS_PACKAGE_PATH

# 7) 启动
roslaunch scan_planner gazebo_sim.launch "$@"
