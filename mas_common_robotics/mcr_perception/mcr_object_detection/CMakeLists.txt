cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_detection)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
    mcr_scene_segmentation
    rospy
    smach
    smach_ros
    tf
)

install(PROGRAMS
  ros/scripts/object_detector
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
