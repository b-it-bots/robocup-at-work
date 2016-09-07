cmake_minimum_required(VERSION 2.8.3)
project(mcr_twist_synchronizer)

find_package(catkin REQUIRED
  COMPONENTS
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
    topic_tools
)

install(PROGRAMS
  ros/scripts/twist_synchronizer_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/twist_synchronizer.test)
  catkin_add_nosetests(ros/test/twist_synchronizer_unittest.py)
endif()
