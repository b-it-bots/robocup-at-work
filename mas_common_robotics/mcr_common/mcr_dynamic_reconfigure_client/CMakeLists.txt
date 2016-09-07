cmake_minimum_required(VERSION 2.8.3)

project(mcr_dynamic_reconfigure_client)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    std_msgs
    roslint
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/TestNode.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    rospy
    std_msgs
)


### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/dynamic_reconfigure_client.test)
endif()


install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS
  ros/scripts/dynamic_reconfigure_client_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
