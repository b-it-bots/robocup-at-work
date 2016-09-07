cmake_minimum_required(VERSION 2.8.3)

project(mcr_topic_tools)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    nodelet
    pluginlib
    topic_tools
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    brics_actuator
    std_msgs
    nodelet
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_library(topic_mux
  ros/src/topic_mux.cpp
)

target_link_libraries(topic_mux
  ${catkin_LIBRARIES}
)

### TESTS
roslint_cpp()
roslint_python(
  ros/scripts/brics_joints_to_joint_states_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/brics_joints_to_joint_states.test)
  catkin_add_nosetests(ros/test/test_transform_topic_utils.py)
endif()

### INSTALLS
install(TARGETS topic_mux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  ros/scripts/brics_joints_to_joint_states_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
