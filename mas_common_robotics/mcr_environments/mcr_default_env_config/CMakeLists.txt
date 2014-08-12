cmake_minimum_required(VERSION 2.8.3)
project(mcr_default_env_config)
find_package(catkin REQUIRED)

catkin_package()

install(FILES upload_param.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/upload_param.launch
)

install(DIRECTORY brsu-c069/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/brsu-c069/
)

install(DIRECTORY brsu-c025/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/brsu-c069/
)
