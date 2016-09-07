cmake_minimum_required(VERSION 2.8.3)
project(mcr_guarded_approach_pose)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    mcr_manipulation_msgs
    mcr_monitoring_msgs
)

### TESTS
roslint_cpp()
roslint_python(
  ros/scripts/approach_pose_distance_constrained_node
  ros/scripts/distance_constrained_node
  ros/scripts/guarded_approach_pose_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  #add_rostest(ros/launch/distance_constrained.test)
  #add_rostest(ros/launch/guarded_approach_pose.test)
  catkin_add_nosetests(ros/test/distance_constrained_unittest.py)
endif()

### INSTALL
install(PROGRAMS
  ros/scripts/approach_pose_distance_constrained_node
  ros/scripts/distance_constrained_node
  ros/scripts/guarded_approach_pose_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
