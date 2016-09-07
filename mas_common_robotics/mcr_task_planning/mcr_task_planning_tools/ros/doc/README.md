mcr_task_planning_tools
=======================

This package contains tools for task planning.

tested under : ROS indigo on 22.02.2016

1. run script tool
==================

run script exposes a ros wrapper interface for system() command in c++.

reference:

        http://www.cplusplus.com/reference/cstdlib/system/

This command allows to call an external script (bash script) located

somewhere in your PC (you need to send the location as argument).

Example : This tools is used by mercury planner for calling a 3 step process

translate -> preprocess -> search

that needs to be called via bash script (for now) until a proper ROS wrapper

is available.

A. Use the component
====================

There is an example file available that you can copy over to your package

        roscd mcr_task_planning_tools/ros/launch
        cp run_script_example.launch /home/user/.../location_of_my_package

Modify launch file according to your needs

B. Test manually this component
===============================

        roscore
        roslaunch mcr_task_planning_tools run_script_example.launch
        rostopic pub /run_script_node/event_in std_msgs/String "data: 'e_trigger'"
        rostopic echo /run_script_node/event_out

In the terminal with rostopic echo ... you should see : "data: e_success"

C. Run unit tests
=================

I. compile your package

        roscd
        catkin_make --pkg mcr_task_planning_tools

II. compile the tests:

launch needed components (because it compiles and tests at the same time)

        roscore
        roslaunch mcr_task_planning_tools run_script_example.launch

in a separate terminal go to catkin_workspace root and compile/run tests for this package

        roscd
        catkin_make run_tests_mcr_task_planning_tools

NOTE: If you dont have the components launched the test will fail

III. run the tests:

Once compiled the tests can be launched by using this command:

        rostest mcr_task_planning_tools script_unit_test.test

for this command to succeed you dont need to have the components launched. Instead they will be
launched from the file :

        script_unit_test.test


Run automatic code error check
==============================

1. Check for errors in CMakeLists.txt and package.xml (catkin_lint)

        roscd mcr_task_planning_tools
        catkin_lint

2. Check for coding convention errors (roslint)

        roscd
        catkin_make roslint_mcr_task_planning_tools
