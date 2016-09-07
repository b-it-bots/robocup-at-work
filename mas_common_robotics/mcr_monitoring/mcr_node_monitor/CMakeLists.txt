cmake_minimum_required(VERSION 2.8.3)
project(mcr_node_monitor)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    rospy
)

catkin_python_setup()


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(PROGRAMS
    ros/scripts/node_status
    ros/src/mcr_node_monitor/node_status.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
