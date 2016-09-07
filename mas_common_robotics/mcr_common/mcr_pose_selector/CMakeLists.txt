cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_selector)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    geometry_msgs
    std_msgs
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### INSTALLS
install(PROGRAMS
  ros/scripts/pose_selector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()