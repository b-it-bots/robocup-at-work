cmake_minimum_required(VERSION 2.8.3)
project(mcr_trajectory_time_parameterizer)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    trajectory_msgs
    std_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    roslint
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    trajectory_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(trajectory_time_parameterizer
  ros/src/trajectory_time_parameterizer_node.cpp
  ros/src/trajectory_time_parameterizer.cpp
  ros/src/time_parameterizer.cpp
)

target_link_libraries(trajectory_time_parameterizer
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)

### TESTS
roslint_cpp()

install(TARGETS trajectory_time_parameterizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
