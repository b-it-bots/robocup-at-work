cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_utils)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    urdf
    kdl_parser
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ros/include
  LIBRARIES mcr_manipulation_utils
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    urdf
    kdl_parser
    geometry_msgs
    sensor_msgs
    moveit_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_manipulation_utils
   ros/src/ros_urdf_loader.cpp
)

target_link_libraries(mcr_manipulation_utils
   ${catkin_LIBRARIES}
)

install(TARGETS mcr_manipulation_utils
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(ros/test/extractors_unittest.py)
endif()
