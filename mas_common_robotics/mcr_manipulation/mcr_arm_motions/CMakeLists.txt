cmake_minimum_required(VERSION 2.8.3)
project(mcr_arm_motions)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    actionlib
    std_msgs
    geometry_msgs
    moveit_msgs
    brics_actuator
    moveit_commander
)

### TESTS
roslint_python(
  ros/scripts/cartesian_motion_node
  ros/scripts/planned_motion_node
  ros/scripts/planned_motion_with_feedback_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/cartesian_motion.test)
  catkin_add_nosetests(ros/test/planned_motion_unittest.py)
endif()

install(PROGRAMS
  ros/scripts/cartesian_motion_node
  ros/scripts/planned_motion_node
  ros/scripts/planned_motion_with_feedback_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)