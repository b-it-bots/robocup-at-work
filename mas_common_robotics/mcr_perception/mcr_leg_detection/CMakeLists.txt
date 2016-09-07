cmake_minimum_required(VERSION 2.8.3)
project(mcr_leg_detection)

find_package(catkin REQUIRED
  COMPONENTS
    laser_geometry
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
    message_generation
    dynamic_reconfigure
    mcr_algorithms
    message_filters
    tf
    visualization_msgs
)

find_package(OpenCV REQUIRED)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

generate_dynamic_reconfigure_options(
  ros/config/LegDetection.cfg
)

add_message_files(
  FILES
  PositionMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    dynamic_reconfigure
    mcr_algorithms
    std_msgs
    geometry_msgs
)

include_directories(
  ros/include/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${BFL_INCLUDE_DIRS}
)

link_directories(
  ${BFL_LIBRARY_DIRS}
)


add_executable(leg_detection_node 
  ros/src/leg_detection_node.cpp
  ros/src/laser_processor.cpp
  ros/src/calc_leg_features.cpp
  ros/src/tracker_kalman.cpp
)
add_dependencies(leg_detection_node ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(leg_detection_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BFL_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(
  TARGETS
    leg_detection_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
