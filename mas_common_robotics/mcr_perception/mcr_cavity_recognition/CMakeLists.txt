cmake_minimum_required(VERSION 2.8.3)
project(mcr_cavity_recognition)

find_package(catkin REQUIRED
  COMPONENTS
    message_filters
    roscpp
    roslib
    sensor_msgs
    geometry_msgs
    mcr_perception_msgs
)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(cavity_message_builder
  ros/src/cavity_message_builder_node.cpp
  ros/src/cavity_message_builder_ros.cpp
)

add_dependencies(cavity_message_builder
  ${catkin_EXPORTED_TARGETS}
)

add_executable(cavity_template_publisher
  ros/src/cavity_template_publisher_node.cpp
  ros/src/cavity_template_publisher_ros.cpp
)

target_link_libraries(cavity_message_builder
  ${catkin_LIBRARIES}
)

target_link_libraries(cavity_template_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

### INSTALLS
install(TARGETS cavity_message_builder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS cavity_template_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
