cmake_minimum_required(VERSION 2.8.3)
project(mcr_fake_camera)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    image_transport
    roscpp
)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(fake_camera_node 
  ros/src/fake_camera_node.cpp
)

target_link_libraries(fake_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS fake_camera_node fake_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
