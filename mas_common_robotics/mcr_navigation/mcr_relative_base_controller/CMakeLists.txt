cmake_minimum_required(VERSION 2.8.3)
project(mcr_relative_base_controller)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf  
)

generate_dynamic_reconfigure_options(
  ros/config/RelativeBaseController.cfg
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    std_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(relative_base_controller_node
  ros/src/relative_base_controller_node.cpp 
)
add_dependencies(relative_base_controller_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(relative_base_controller_node
  ${catkin_LIBRARIES}
)


add_executable(relative_base_controller_mockup
  ros/mockup/relative_base_controller_mockup.cpp
)
add_dependencies(relative_base_controller_mockup
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(relative_base_controller_mockup
  ${catkin_LIBRARIES}
)


install(TARGETS relative_base_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS relative_base_controller_mockup
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

add_executable(relative_base_controller_test
  ros/test/relative_base_controller_test.cpp
)

target_link_libraries(relative_base_controller_test
  ${GTEST_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_rostest(ros/test/relative_base_controller.test)
