cmake_minimum_required(VERSION 2.8.3)
project(mcr_collision_velocity_filter)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    laser_geometry
    message_filters
    roscpp
    sensor_msgs
    std_msgs
    tf
    roslint
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### LIBS ###
add_library(mcr_collision_velocity_filter
    ros/src/collision_velocity_filter.cpp
)
target_link_libraries(mcr_collision_velocity_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

### EXECUTABLES ###
add_executable(collision_velocity_filter_node
  ros/src/collision_velocity_filter_node.cpp
)
add_dependencies(collision_velocity_filter_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(collision_velocity_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  mcr_collision_velocity_filter
)


### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch footprint_file:=${PROJECT_SOURCE_DIR}/ros/test/footprint.yaml)
endif()


### INSTALLS ###
install(
  TARGETS 
    collision_velocity_filter_node
    mcr_collision_velocity_filter
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

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)
