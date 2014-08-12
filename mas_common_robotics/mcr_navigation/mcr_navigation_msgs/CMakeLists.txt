cmake_minimum_required(VERSION 2.8.3)
project(mcr_navigation_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    roscpp
    rospy
    std_msgs
)

add_service_files(
  FILES
    MoveRelative.srv
    GetBasePose.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
