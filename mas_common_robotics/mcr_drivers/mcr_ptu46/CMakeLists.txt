cmake_minimum_required(VERSION 2.8.3)
project(mcr_ptu46)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp 
    rospy 
    sensor_msgs
    message_generation
    actionlib
    actionlib_msgs
    tf
)

add_action_files(
  DIRECTORY
    action
  FILES
    PtuGotoPosition.action  
    PtuGotoPositionByName.action  
    PtuReset.action  
    PtuSetVel.action
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    actionlib_msgs
    message_runtime
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
)

add_library(ptu46 
	common/src/ptu46_driver.cc
)

add_executable(ptu46_node 
  ros/src/ptu46_node.cc
)
target_link_libraries(ptu46_node
  ptu46
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS ptu46_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ptu46
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
