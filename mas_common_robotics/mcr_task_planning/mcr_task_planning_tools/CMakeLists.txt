cmake_minimum_required(VERSION 2.8.3)
project(mcr_task_planning_tools)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   roslib
   roslint
   rostest
   rosunit
   std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    std_msgs
)

include_directories(
    common/include
    ros/include
    ${catkin_INCLUDE_DIRS}
)

### LIBRARY
add_library(run_script common/src/run_script.cpp)

### EXECUTABLES
add_executable(run_script_node ros/src/run_script_node.cpp)
target_link_libraries(run_script_node ${catkin_LIBRARIES} run_script)

### TESTS
if(CATKIN_ENABLE_TESTING)
    find_package(roslaunch REQUIRED)
    roslaunch_add_file_check(ros/launch)

    add_rostest_gtest(run_script_node_test 
            ros/test/run_script_node.test
            ros/test/run_script_node_test.cpp
        )
    target_link_libraries(run_script_node_test ${catkin_LIBRARIES})
endif()

roslint_cpp()

### INSTALLS

install(TARGETS run_script_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/run_script/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(TARGETS run_script
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)