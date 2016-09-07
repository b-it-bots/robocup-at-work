cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_recognition_mean_circle)

find_package(PCL 1.6 REQUIRED)

find_package(catkin REQUIRED 
  COMPONENTS
    mcr_scene_segmentation
    mcr_perception_msgs
    pcl_ros
    roscpp
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    mcr_perception_msgs
    rospy
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

#...: enable all but certain warnings
add_definitions(-Wall)
add_definitions(-Wno-unknown-pragmas)
add_definitions(-Wno-deprecated)
add_definitions(-fpermissive)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


roslint_python()
roslint_cpp()

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(ros/launch workspace_finder_config_file:=${mcr_scene_segmentation_workspace_constraints_file})

  add_rostest_gtest(object_recognition_mean_circle_test
    ros/test/object_recognition_mean_circle.test
    ros/test/object_recognition_mean_circle_test.cpp
  )

  add_dependencies(object_recognition_mean_circle_test
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(object_recognition_mean_circle_test
    ${catkin_LIBRARIES}
  )
endif()


### INSTALLS
install(PROGRAMS
  ros/scripts/object_recognizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/config
)
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
install(DIRECTORY ros/scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/scripts
)
