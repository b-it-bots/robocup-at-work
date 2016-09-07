cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_selectors)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    mcr_perception_msgs
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
    rospy
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(object_selector
  ros/src/object_selector.cpp
  ros/scripts/cavity_pose_selector_node
)

add_dependencies(object_selector
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_selector
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()

roslint_python()

roslint_add_test()

### INSTALLS
install(TARGETS object_selector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
