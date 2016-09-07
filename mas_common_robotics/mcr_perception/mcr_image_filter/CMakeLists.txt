cmake_minimum_required(VERSION 2.8.3)
project(mcr_image_filter)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/ImageFilter.cfg
)

catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    cv_bridge
    dynamic_reconfigure
    image_transport
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_filter_node
  ros/src/image_filter_node.cpp
)

add_dependencies(image_filter_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_filter_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

install(TARGETS image_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
