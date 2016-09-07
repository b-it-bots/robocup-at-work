cmake_minimum_required(VERSION 2.8.3)
project(mcr_image_cartesian_mapper)

find_package(catkin REQUIRED 
  COMPONENTS
    cmake_modules
    cv_bridge
    image_transport
    geometry_msgs
    sensor_msgs
    roscpp
    std_msgs
    tf
)

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    cv_bridge
    image_transport
    geometry_msgs
    sensor_msgs
    roscpp
    std_msgs
    tf
  DEPENDS
    Eigen
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(image_cartesian_mapper_node
  ros/src/image_cartesian_mapper_node.cpp
)

add_dependencies(image_cartesian_mapper_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_cartesian_mapper_node
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS image_cartesian_mapper_node
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

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(ros/launch)
endif()
