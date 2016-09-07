cmake_minimum_required(VERSION 2.8.3)
project(mcr_geometry_transformer)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    tf
    tf_conversions
    geometry_msgs
    rosunit
)
find_package(orocos_kdl REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS ros/include
  LIBRARIES geometry_transformer
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf_conversions
    geometry_msgs
    rosunit
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  common/include
  ros/include
)

add_library(geometry_transformer
  ros/src/geometry_transformer.cpp
  common/src/geometry_transformer_util.cpp
  ${catkin_LIBRARIES}
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)

  catkin_add_gtest(geometry_transformer_util_test
    common/test/geometry_transformer_util_test.cpp
  )
  target_link_libraries(geometry_transformer_util_test
    geometry_transformer
    ${catkin_LIBRARIES}
  )

  catkin_add_nosetests(common/test)
endif()

### INSTALL
install(
  TARGETS geometry_transformer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
