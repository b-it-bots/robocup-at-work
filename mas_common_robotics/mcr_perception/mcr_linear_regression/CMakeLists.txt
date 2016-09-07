cmake_minimum_required(VERSION 2.8.3)
project(mcr_linear_regression)

find_package(catkin REQUIRED
  COMPONENTS
    mcr_perception_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs 
)

include_directories(
  common/include/
  ros/include/
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_linear_regression_node 
  ros/src/laser_scan_linear_regression_node.cpp
  ros/src/laser_scan_linear_regression_util.cpp
  common/src/laser_scan_linear_regression.cpp
)
add_dependencies(laser_scan_linear_regression_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(laser_scan_linear_regression_node
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS laser_scan_linear_regression_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch/
)
