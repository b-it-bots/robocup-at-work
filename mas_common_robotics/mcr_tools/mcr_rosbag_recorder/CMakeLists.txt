cmake_minimum_required(VERSION 2.8.3)
project(mcr_rosbag_recorder)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
    std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS
  ros/scripts/rosbag_recorder_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
