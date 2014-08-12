cmake_minimum_required(VERSION 2.8.3)
project(mcr_multi_joint_forward_controller)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    controller_interface
    hardware_interface
    brics_actuator
    rostest
)

catkin_package(
  INCLUDE_DIRS 
    ros/include
  LIBRARIES
    mcr_multi_joint_forward_controller
  CATKIN_DEPENDS
    actionlib
    controller_interface
    hardware_interface
    brics_actuator
    rostest
)

# build
include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_multi_joint_forward_controller
  ros/src/multi_joint_forward_controller.cpp
)

target_link_libraries(mcr_multi_joint_forward_controller
  ${catkin_LIBRARIES}
)

# install
install(TARGETS mcr_multi_joint_forward_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# test
add_rostest_gtest(multi_joint_forward_controller_test
  ros/test/multi_joint_forward_controller.test
  ros/test/multi_joint_forward_controller_test.cpp
)

target_link_libraries(multi_joint_forward_controller_test
  ${catkin_LIBRARIES}
)
