cmake_minimum_required(VERSION 2.8.3)
project(mcr_twist_demultiplexer)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    mcr_monitoring_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
    mcr_monitoring_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_demultiplexer_node
  ros/src/twist_demultiplexer_node.cpp
)

add_dependencies(twist_demultiplexer_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(twist_demultiplexer_node
  ${catkin_LIBRARIES}
)

install(TARGETS twist_demultiplexer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
