cmake_minimum_required(VERSION 2.8.3)
project(mcr_moveit_commander_gui)

find_package(catkin REQUIRED COMPONENTS
  roslint
  rospy
  rqt_gui_py
)

catkin_python_setup()
catkin_package()

roslint_python()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
