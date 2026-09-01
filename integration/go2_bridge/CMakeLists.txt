cmake_minimum_required(VERSION 2.8.3)
project(go2_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  std_srvs
  gazebo_msgs
)

catkin_package()

# 5 个胶水节点（纯 Python，catkin_install_python 安装到 devel/lib/go2_bridge）
catkin_install_python(PROGRAMS
  scripts/gazebo_bridge.py
  scripts/scan_cloud_accumulator.py
  scripts/ariadne_goal_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
