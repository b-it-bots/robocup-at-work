cmake_minimum_required(VERSION 2.8.3)
project(mcr_direct_base_controller)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roslint
    rospy
    std_msgs
    mcr_monitoring_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    rospy
    std_msgs
    mcr_monitoring_msgs
    mcr_geometric_relation_monitors
    mcr_common_converters
    mcr_twist_controller
    mcr_twist_limiter
    mcr_twist_synchronizer
    mcr_manipulation_measurers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### INSTALLS
install(PROGRAMS
  ros/scripts/direct_base_controller_coordinator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()
