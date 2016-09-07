cmake_minimum_required(VERSION 2.8.3)
project(mcr_nearest_object_detection)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    laser_filters
    mcr_algorithms
    roscpp
    sensor_msgs
    std_srvs
    visualization_msgs
)
find_package(PCL 1.5 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    mcr_algorithms
    sensor_msgs
    std_srvs
    visualization_msgs
)

add_executable(nearest_object_detection_node 
  ros/src/nearest_object_detection_node.cpp
)
add_dependencies(nearest_object_detection_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(nearest_object_detection_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS nearest_object_detection_node nearest_object_detection_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
