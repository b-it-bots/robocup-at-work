cmake_minimum_required(VERSION 2.8.3)
project(mcr_default_env_config)
find_package(catkin REQUIRED)

catkin_package()

### TESTS
set(env_list
  brsu-c025
  brsu-c069
  empty-world
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  #roslaunch_add_file_check(upload_param.launch robot_env:=brsu-c025)

  foreach(env ${env_list})
    roslaunch_add_file_check(${env}/upload_param_${env}.launch)
  endforeach()

endif()


### INSTALLS
install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/upload_param.launch
)

install(DIRECTORY brsu-c069/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/brsu-c069/
)

install(DIRECTORY brsu-c025/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/brsu-c025/
)

install(DIRECTORY empty-world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/empty-world/
)
