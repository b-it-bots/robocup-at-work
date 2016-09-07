cmake_minimum_required(VERSION 2.8.3)
project(mcr_interpolation_planner_moveit)

find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
  serialization
)

find_package(orocos_kdl)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    roscpp
    pluginlib
    tf
    eigen_conversions
    trajectory_msgs
  )

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    trajectory_msgs
  LIBRARIES
    moveit_interpolation_planner_plugin
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(moveit_interpolation_planner_plugin
  ros/src/interpolation_planner_manager.cpp
  ros/src/interpolation_planner_interface.cpp
  ros/src/interpolation_planner_context.cpp
)

target_link_libraries(moveit_interpolation_planner_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS moveit_interpolation_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
