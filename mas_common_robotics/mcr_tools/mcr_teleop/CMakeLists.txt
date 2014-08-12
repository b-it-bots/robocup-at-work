cmake_minimum_required(VERSION 2.8.3)
project(mcr_teleop)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
)

add_executable(teleop_keyboard_node 
  ros/src/teleop_keyboard_node.cpp
)
add_dependencies(teleop_keyboard_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(teleop_keyboard_node
  ${catkin_LIBRARIES}
)

install(TARGETS teleop_keyboard_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
