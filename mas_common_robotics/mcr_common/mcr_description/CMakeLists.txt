cmake_minimum_required(VERSION 2.8.3)
project(mcr_description)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

