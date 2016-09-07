cmake_minimum_required(VERSION 2.8.3)
project(mcr_background_change_detection)

find_package(catkin REQUIRED 
  COMPONENTS
    dynamic_reconfigure
    image_transport
    roscpp
    sensor_msgs
    cv_bridge
)

find_package(OpenCV 2.3 REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/BackgroundChange.cfg
)

catkin_package(
  INCLUDE_DIRS 
    ros/include
    common/include
  LIBRARIES
    background_change_detection
  CATKIN_DEPENDS 
    dynamic_reconfigure 
    image_transport 
    roscpp 
    sensor_msgs
    cv_bridge
  DEPENDS
    OpenCV
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(background_change_detection
  common/src/background_change_detection.cpp
)

add_dependencies(background_change_detection
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(background_change_detection
  ${OpenCV_LIBRARIES}
)

add_executable(background_change_detection_node 
  ros/src/background_change_detection_node.cpp
)

add_dependencies(background_change_detection_node 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(background_change_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  background_change_detection
)

install(
  TARGETS
    background_change_detection 
    background_change_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY common/include/${PROJECT_NAME}/
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