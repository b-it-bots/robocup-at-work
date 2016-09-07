mcr_task_planners
=================

1. mercury planner

Mercury planner is a task planner from International Planning Competition 2014 (IPC) created by 

Michael Katz and Jörg Hoffmann from Saarland University, Germany. The source of the planner is

provided under the repository mas_third_party_software/mercury_planner/build/seq-sat-mercury

(you need to compile for this folder to appear as downloads from the web the planner).


About this node
===============

Uses mcr_task_planning_tools to call an external script located under : 

        mas_third_party_software/mercury_planner/build/seq-sat-mercury/plan

With the following arguments : (see launch file for more details)

        /home/user/path../domain.pddl /home/user/path../problem.pddl mercury_plan
        
When you publish event_in (see A.Test the component) the script gets called and mercury planner

pipeline is triggered (translate -> preprocess -> search).

As a result of this and if the planner is able to find solution, mercury_plan.1 file will be created

under the following location:

        ~/.ros/mercury_plan.1

A. To test manually the component:

        roslaunch mcr_task_planners mercury_planner.launch
        
        rostopic pub /mercury_planner/event_in std_msgs/String "data: 'e_trigger'"
        rostopic echo /mercury_planner/event_out
        
expected outcome on rostopic echo .. terminal :

        data: e_success
