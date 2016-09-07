/* 
 * Copyright [2015] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array  
 * of poses and calculates the path lenght based on the distance between two points of each pose.  
 * 
 */

#include <mcr_navigation_tools/path_length_calculator_node.h>
#include <string>

PathLengthCalcNode::PathLengthCalcNode() : nh_("~"), is_global_plan_available_(false), is_event_in_received_(false)
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &PathLengthCalcNode::eventInCallback, this);
    sub_path_plan_ = nh_.subscribe("plan", 1, &PathLengthCalcNode::pathPlanCallback, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);
    pub_path_length_ = nh_.advertise<std_msgs::Float64>("path_length", 1);

    even_out_msg_.data = "unset";
}

PathLengthCalcNode::~PathLengthCalcNode()
{
    // shut down publishers and subscribers
    sub_event_in_.shutdown();
    pub_event_out_.shutdown();

    sub_path_plan_.shutdown();
    pub_path_length_.shutdown();
}

void PathLengthCalcNode::eventInCallback(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

void PathLengthCalcNode::pathPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_plan_ = *msg;
    is_global_plan_available_ = true;
}

void PathLengthCalcNode::update()
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

    // checking if nav_msgs::Path was received in the callback
    if (!is_global_plan_available_)
    {
        ROS_ERROR("event_in trigger was received, but there is not path");
        ROS_WARN("Did you already query about its length before?");

        // publish even_out : "e_failure"
        even_out_msg_.data = std::string("e_failure");
        pub_event_out_.publish(even_out_msg_);
        return;
    }

    // reset flag
    is_global_plan_available_ = false;

    // stores the result of the generic length class to be published later on
    std_msgs::Float64 path_length;

    // set global plan
    path_length_calculator_.setPath(path_plan_);

    // calculate path length of global plan (array of poses)
    path_length.data = path_length_calculator_.computeLength();

    if (path_length.data == -1.0)
    {
        ROS_ERROR("Error while calculating path length");

        // publish even_out : "e_failure"
        even_out_msg_.data = std::string("e_failure");
        pub_event_out_.publish(even_out_msg_);
    }
    else
    {
        // publish result
        pub_path_length_.publish(path_length);

        // publish even_out : "e_success"
        even_out_msg_.data = std::string("e_success");
        pub_event_out_.publish(even_out_msg_);

        ROS_INFO("Path length = %lf meters", path_length.data);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_length_calculator");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (PathLengthCalcNode)
    PathLengthCalcNode path_length_calculator;

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
        path_length_calculator.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
