cmake_minimum_required(VERSION 2.8.3)
project(mcr_recovery_behaviors)

find_package(catkin REQUIRED
 COMPONENTS
   cmake_modules
   costmap_2d
   nav_core
   pluginlib
   roscpp
   geometry_msgs
   pcl_ros
   tf
   roslint
)

find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
    ros/include
  LIBRARIES 
    force_field_recovery
  CATKIN_DEPENDS
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf
)

### LIBRARIES
add_library(force_field_recovery ros/src/force_field_recovery.cpp)
target_link_libraries(force_field_recovery ${catkin_LIBRARIES})


### TESTS
roslint_cpp()


### INSTALLS
install(TARGETS force_field_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
