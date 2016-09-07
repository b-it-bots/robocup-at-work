/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Uses common class RunScript which calls system() function 
 * for calling external bash scripts from code.
 * 
 */

#include <mcr_task_planning_tools/run_script_node.h>
#include <string>
#include <vector>

RunScriptNode::RunScriptNode() : nh_("~"), is_event_in_received_(false)
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &RunScriptNode::eventInCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);

    // querying parameters from parameter server
    getParams();

    // set script path
    script_handler_.setScriptPath(full_path_to_script_);

    // set script arguments (if any)
    if (script_arguments_.at(0) == std::string("no_args"))
        ROS_INFO("Script will run with no arguments");
    else
    {
        script_handler_.setScriptArgs(script_arguments_);

        std::string args;
        for (int i =0 ; i < script_arguments_.size() ; i++)
        {
            args += script_arguments_.at(i);
            args += std::string(" ");
        }
    }
}

RunScriptNode::~RunScriptNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    pub_event_out_.shutdown();
}

void RunScriptNode::getParams()
{
    // setup script default arguments
    std::vector<std::string> default_args;
    default_args.push_back("no_args");

    nh_.param<std::string>("script_path", full_path_to_script_, "/home/user/my_script.sh");
    nh_.param<std::vector<std::string> >("script_arguments", script_arguments_, default_args);

    // informing the user about the parameters which will be used
    ROS_INFO("Script path : %s", full_path_to_script_.c_str());

    ROS_INFO("Script will run with the following arguments :");
    for (int i = 0; i < script_arguments_.size() ; i++)
    {
        ROS_INFO("arg %d : %s", i + 1, script_arguments_.at(i).c_str());
    }
}

void RunScriptNode::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

void RunScriptNode::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (!is_event_in_received_) return;

    // reset flag
    is_event_in_received_ = false;

    // checking for event in msg content
    if (event_in_msg_.data != "e_trigger")
    {
        ROS_ERROR("Received unsupported event: %s", event_in_msg_.data.c_str());
        return;
    }

    // run script
    if (script_handler_.run())
    {
        even_out_msg_.data = std::string("e_success");
        pub_event_out_.publish(even_out_msg_);

        ROS_INFO("Script successfully called !");
    }
    else
    {
        even_out_msg_.data = std::string("e_failure");
        pub_event_out_.publish(even_out_msg_);

        ROS_ERROR("A failure occurred while running the script, "
                "either script does not exist, or executed and returned with error");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_script_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (RunScriptNode)
    RunScriptNode run_script_node;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        run_script_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
