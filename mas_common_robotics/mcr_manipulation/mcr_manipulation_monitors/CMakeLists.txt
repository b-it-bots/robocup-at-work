cmake_minimum_required(VERSION 2.8.3)
project(mcr_manipulation_monitors)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf
)

install(PROGRAMS
  ros/scripts/joint_position_monitors_node
  ros/scripts/cartesian_distance_monitor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_nosetests(ros/test/joint_position_monitors_unittest.py)