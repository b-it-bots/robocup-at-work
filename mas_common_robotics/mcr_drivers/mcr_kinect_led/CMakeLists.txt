cmake_minimum_required(VERSION 2.8.3)
project(mcr_kinect_led)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules (LIBUSB REQUIRED libusb-1.0)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(kinect_led_node
  ros/src/kinect_led_node.cpp
)

add_dependencies(kinect_led_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinect_led_node
  ${catkin_LIBRARIES}
  ${LIBUSB_LIBRARIES}
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

install(
  TARGETS
    kinect_led_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
