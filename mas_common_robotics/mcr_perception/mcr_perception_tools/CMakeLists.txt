cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_tools)

find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    mcr_object_detection
    mcr_object_recognition_bounding_box
    smach
    smach_ros
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(save_pointcloud
  ros/src/save_pointcloud.cpp
)

target_link_libraries(save_pointcloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch input_topic:=/dummy_topic)
endif()


### INSTALLS
install(TARGETS save_pointcloud
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/scripts
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
