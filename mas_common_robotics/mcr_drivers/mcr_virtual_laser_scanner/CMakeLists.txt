cmake_minimum_required(VERSION 2.8.3)
project(mcr_virtual_laser_scanner)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    dynamic_reconfigure
    tf
    pcl_ros
    sensor_msgs
    std_msgs
)
find_package(PCL 1.5 REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/VirtualLaserScanner.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    message_runtime
    sensor_msgs
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(virtual_laser_scanner_node
  ros/src/virtual_laser_scanner_node.cpp
)
target_link_libraries(virtual_laser_scanner_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(virtual_laser_scanner_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(TARGETS virtual_laser_scanner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
