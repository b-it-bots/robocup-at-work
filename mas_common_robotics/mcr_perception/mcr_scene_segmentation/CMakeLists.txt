cmake_minimum_required(VERSION 2.8.3)
project(mcr_scene_segmentation)

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    mcr_perception_msgs
    polyclipping
    roscpp
    rospy
    visualization_msgs
)
find_package(PCL 1.6 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  ros/config/PlaneExtraction.cfg
)


catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  LIBRARIES
    scene_segmentation
    bounding_box
  CATKIN_DEPENDS
    dynamic_reconfigure
    mcr_perception_msgs
    polyclipping
    visualization_msgs
)

include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else()
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif()
endif()

add_definitions(-fpermissive)


### LIBRARIES ####################################################
add_library(scene_segmentation
  common/src/plane_extraction.cpp
  common/src/cloud_accumulation.cpp
  common/src/polyclipping.cpp
)
add_dependencies(scene_segmentation 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scene_segmentation
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(bounding_box
  common/src/bounding_box.cpp
)
add_dependencies(bounding_box 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bounding_box
  ${OpenCV_LIBRARIES}
)

### EXECUTABLES ###############################################
add_executable(workspace_finder
  ros/src/workspace_finder_node.cpp
)
add_dependencies(workspace_finder 
  ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg
)
target_link_libraries(workspace_finder
  ${catkin_LIBRARIES}
  scene_segmentation
)

add_executable(tabletop_cloud_accumulator
  ros/src/tabletop_cloud_accumulator_node.cpp
)
target_link_libraries(tabletop_cloud_accumulator
  scene_segmentation
)

add_executable(tabletop_cloud_clusterer
  ros/src/tabletop_cloud_clusterer_node.cpp
)
target_link_libraries(tabletop_cloud_clusterer
  ${catkin_LIBRARIES}
  scene_segmentation
)

add_executable(bounding_box_maker
  ros/src/bounding_box_maker_node.cpp
)
target_link_libraries(bounding_box_maker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  bounding_box 
)

add_executable(offline_plane_extraction_test
  ros/src/offline_plane_extraction_test.cpp
)
add_dependencies(offline_plane_extraction_test 
  ${catkin_EXPORT_TARGETS} ${PROJECT_NAME}_gencfg
)
target_link_libraries(offline_plane_extraction_test
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  scene_segmentation
)

add_executable(online_plane_extraction_test
  ros/src/online_plane_extraction_test.cpp
)
target_link_libraries(online_plane_extraction_test
  ${catkin_LIBRARIES}
  scene_segmentation
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS scene_segmentation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS bounding_box
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS workspace_finder
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bounding_box_maker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tabletop_cloud_clusterer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS tabletop_cloud_accumulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
