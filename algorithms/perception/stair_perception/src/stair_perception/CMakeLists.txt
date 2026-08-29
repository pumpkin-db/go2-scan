cmake_minimum_required(VERSION 3.0.2)
project(stair_perception)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation nav_msgs pcl_conversions pcl_ros roscpp
  sensor_msgs std_msgs visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters sample_consensus segmentation)

add_message_files(FILES
  StairObservation.msg
  StairObservationArray.msg
  StairTrack.msg
  StairTrackArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stair_detector_core
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs pcl_conversions pcl_ros
                 roscpp sensor_msgs std_msgs visualization_msgs
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(stair_detector_core src/detector_core.cpp)
add_dependencies(stair_detector_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stair_detector_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(stair_detector_node src/stair_detector_node.cpp)
add_dependencies(stair_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stair_detector_node stair_detector_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(stair_tracker_node src/stair_tracker_node.cpp)
add_dependencies(stair_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stair_tracker_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_detector test/test_detector.cpp)
  if(TARGET test_detector)
    add_dependencies(test_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_detector stair_detector_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
endif()

install(TARGETS stair_detector_core stair_detector_node stair_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
