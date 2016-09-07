cmake_minimum_required(VERSION 2.8.3)
project(mcr_trajectory_generation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/TaskSpaceParams.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    dmp
    actionlib
    std_msgs
    geometry_msgs
    trajectory_msgs
    control_msgs
)

### TESTS
roslint_python(
  ros/scripts/dmp_based_task_space_node
  ros/scripts/dmp_trajectory_demo_node
  ros/scripts/ik_trajectory_solver_node
  ros/scripts/linear_interpolator_demo_node
  ros/scripts/linear_interpolator_node
  ros/scripts/trajectory_generator_node
  ros/scripts/trajectory_executor_node
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch)
  add_rostest(ros/launch/dmp_based_task_space.test)
  add_rostest(ros/launch/trajectory_generator.test)
  catkin_add_nosetests(ros/test/dmp_based_task_space_unittest.py)
  catkin_add_nosetests(ros/test/ik_trajectory_solver_unittest.py)
  catkin_add_nosetests(ros/test/trajectory_generator_unittest.py)
endif()

install(PROGRAMS
  ros/scripts/dmp_based_task_space_node
  ros/scripts/dmp_trajectory_demo_node
  ros/scripts/ik_trajectory_solver_node
  ros/scripts/linear_interpolator_demo_node
  ros/scripts/linear_interpolator_node
  ros/scripts/trajectory_generator_node
  ros/scripts/trajectory_executor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
