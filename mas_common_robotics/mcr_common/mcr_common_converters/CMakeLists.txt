cmake_minimum_required(VERSION 2.8.3)
project(mcr_common_converters)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    tf
    std_msgs
    mcr_manipulation_msgs
    dynamic_reconfigure
    roslint
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/LinearOffset.cfg
)

catkin_package(
  INCLUDE_DIRS
    common/include
    ros/include
  LIBRARIES
    motion_direction_calculation
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    tf
    mcr_manipulation_msgs
    dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  common/include
  ros/include
)

### LIBRARIES
add_library(motion_direction_calculation
  common/src/motion_direction_calculation.cpp
)

add_executable(twist_to_motion_direction_conversion_node
  ros/src/twist_to_motion_direction_conversion_node.cpp
)
target_link_libraries(twist_to_motion_direction_conversion_node
  ${catkin_LIBRARIES}
  motion_direction_calculation
)
add_dependencies(twist_to_motion_direction_conversion_node ${catkin_EXPORTED_TARGETS})


### TESTS
roslint_cpp()
roslint_python(
  ros/scripts/transform_to_pose_converter_node
  ros/scripts/pose_to_transform_converter_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/transform_to_pose_converter.test)
  add_rostest(ros/launch/relative_displacement_calculator.test)
endif()


### INSTALL
install(PROGRAMS
  ros/scripts/transform_to_pose_converter_node
  ros/scripts/pose_to_transform_converter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS motion_direction_calculation twist_to_motion_direction_conversion_node
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
