/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Uses common class RunScript which calls system() function 
 * for calling external bash scripts from code.
 * 
 */

#ifndef MCR_TASK_PLANNING_TOOLS_RUN_SCRIPT_NODE_H
#define MCR_TASK_PLANNING_TOOLS_RUN_SCRIPT_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <mcr_task_planning_tools/run_script.h>
#include <string>
#include <vector>

class RunScriptNode
{
    public:
        RunScriptNode();
        ~RunScriptNode();

        // get parameters from param server
        void getParams();

        // callback for event_in received msg
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);

        // ros node main loop
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Subscriber sub_event_in_;

        // flag used to know when we have received a callback
        bool is_event_in_received_;

        // stores the received msg in event_in callback (runScriptCallBack)
        std_msgs::String event_in_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;

        // for storing the arguments that will be read from param server
        std::vector<std::string> script_arguments_;

        // to store the path of the script
        std::string full_path_to_script_;

        // generic class to call external scripts
        RunScript script_handler_;
};
#endif  // MCR_TASK_PLANNING_TOOLS_RUN_SCRIPT_NODE_H
