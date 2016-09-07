cmake_minimum_required(VERSION 2.8.3)
project(mcr_gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()


### INSTALLS
set(INSTALL_DIRS common/objects/ common/worlds/ ros/launch/) 
foreach(dir ${INSTALL_DIRS}) 
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir) 


### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch robot_env:=brsu-c025-sim)
endif()
