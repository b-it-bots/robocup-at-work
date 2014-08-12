cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_converters)

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
    tf
)

install(PROGRAMS
  ros/scripts/transform_to_pose_converter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)