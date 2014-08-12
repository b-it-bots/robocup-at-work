cmake_minimum_required(VERSION 2.8.3)
project(mcr_trajectory_controller)

find_package(catkin REQUIRED
  COMPONENTS
    controller_interface
    hardware_interface
    joint_trajectory_controller
    urdf
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    joint_trajectory_controller
    urdf
  LIBRARIES
    mcr_trajectory_controller
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_trajectory_controller
  ros/src/trajectory_controller.cpp
)

target_link_libraries(mcr_trajectory_controller
  ${catkin_LIBRARIES}
)


install(TARGETS mcr_trajectory_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

