cmake_minimum_required(VERSION 3.0.2)
project(stair_navigation)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure geometry_msgs nav_msgs rospy stair_perception std_msgs tf)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs rospy stair_perception std_msgs tf)

catkin_install_python(PROGRAMS
  scripts/motion_arbiter_node.py
  scripts/stair_approach_node.py
  scripts/stair_traverser_node.py
  scripts/terrain_follow_sim_adapter.py
  scripts/floor_manager_node.py
  scripts/multifloor_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_cores.py)
endif()
