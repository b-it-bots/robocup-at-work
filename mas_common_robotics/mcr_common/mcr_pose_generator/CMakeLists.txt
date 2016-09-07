cmake_minimum_required(VERSION 2.8.3)
project(mcr_pose_generator)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    tf
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
)

### TESTS
roslint_python(
  ros/scripts/pose_generator_node
  ros/scripts/pose_publisher_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/pose_generator.test)
  catkin_add_nosetests(ros/test/pose_generator_unittest.py)
  catkin_add_nosetests(common/test/transformation_unit_test.py)
endif()

install(PROGRAMS
  ros/scripts/pose_generator_node
  ros/scripts/pose_publisher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
