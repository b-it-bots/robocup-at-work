cmake_minimum_required(VERSION 2.8.3)
project(mcr_body_detection_3d)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    mcr_algorithms
    mcr_perception_msgs 
    pcl_ros
    roscpp
    roslint
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)

find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  ros/config/BodyDetection3D.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    mcr_algorithms
    mcr_perception_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
)

include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}  
)

add_library(mcr_body_detection_3d
  common/src/body_detection_3d.cpp
)
add_dependencies(mcr_body_detection_3d
  ${catkin_EXPORTED_TARGETS}
)

add_executable(body_detection_3d_node
  ros/src/body_detection_3d_node.cpp
)
add_dependencies(body_detection_3d_node
  ${PROJECT_NAME}_gencfg 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(body_detection_3d_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  mcr_body_detection_3d
)


### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)

  add_rostest_gtest(body_detection_3d_integration_test
    ros/test/integration.test
    ros/test/integration_test.cpp
  )
  target_link_libraries(body_detection_3d_integration_test
    ${catkin_LIBRARIES}
  )

endif()


### INSTALLS
install(
  TARGETS
    mcr_body_detection_3d
    body_detection_3d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY common/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)
