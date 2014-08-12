cmake_minimum_required(VERSION 2.8.3)
project(mcr_laptop_battery_monitor)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    pr2_msgs    
    rospy
)


install(PROGRAMS
  ros/scripts/laptop_battery_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
