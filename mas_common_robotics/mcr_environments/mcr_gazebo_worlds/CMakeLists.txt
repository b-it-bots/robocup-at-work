cmake_minimum_required(VERSION 2.8.3)
project(mcr_gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()

set(INSTALL_DIRS common/objects/ common/worlds/ ros/launch/) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir) 


