cmake_minimum_required(VERSION 2.8.3)
project(laser_plane_merger)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  sensor_msgs
  message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp tf tf2_ros sensor_msgs message_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_plane_merger_node
  src/laser_plane_merger_node.cpp
  src/${PROJECT_NAME}/LaserPlaneMerger.cpp
)

target_link_libraries(laser_plane_merger_node
  ${catkin_LIBRARIES}
)

install(TARGETS laser_plane_merger_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
