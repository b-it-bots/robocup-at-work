cmake_minimum_required(VERSION 2.8.3)
project(mcr_twist_limiter)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
)

install(PROGRAMS
  ros/scripts/twist_limiter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)