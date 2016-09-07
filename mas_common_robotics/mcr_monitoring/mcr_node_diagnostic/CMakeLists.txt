cmake_minimum_required(VERSION 2.4.6)
project(mcr_node_diagnostic)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    diagnostic_msgs
    diagnostic_aggregator
    diagnostic_analysis
    pluginlib
)


catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES
    mcr_node_analyzer
  CATKIN_DEPENDS
    diagnostic_aggregator
    diagnostic_msgs
    pluginlib
    rospy
)

catkin_python_setup()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_node_analyzer
  ros/src/mcr_node_analyzer.cpp
)
  
add_executable(node_analyzer_example
  ros/examples/cpp_example.cpp
)
target_link_libraries(node_analyzer_example
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS mcr_node_analyzer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS node_analyzer_example
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES mcr_node_analyzer_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

