cmake_minimum_required(VERSION 2.8.3)
project(mcr_event_gui)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  rqt_gui_py
)
catkin_python_setup()
catkin_package()

roslint_python()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

install(PROGRAMS
  ros/scripts/event_gui_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
