cmake_minimum_required(VERSION 2.8.3)
project(mcr_gazebo_objects)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  ros/scripts/spawn_objects.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(INSTALL_DIRS common/models/ ros/launch/ ros/templates/ ros/urdf/) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir) 


