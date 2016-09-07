cmake_minimum_required(VERSION 2.8.3)
project(mcr_people_tracking)

find_package(catkin REQUIRED 
  COMPONENTS 
    dynamic_reconfigure
    mcr_algorithms 
    mcr_perception_msgs
    pcl_ros    
    roscpp     
    tf     
    visualization_msgs
)

find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)


generate_dynamic_reconfigure_options(
  ros/config/WaistTracking.cfg
)

catkin_package(
  CATKIN_DEPENDS 
    dynamic_reconfigure
    mcr_algorithms
    mcr_perception_msgs
    visualization_msgs
)

include_directories(
  common/include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(waist_tracking_node  
  ros/src/waist_tracking_node.cpp
  common/src/particle_filter.cpp
)
add_dependencies(waist_tracking_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(waist_tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}   
  ${PCL_LIBRARIES}   
)


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS waist_tracking_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
