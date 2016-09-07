/**
 * @file component_wise_pose_error_monitor_node.h
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @date June, 2015
 */

#include <mcr_geometric_relation_monitors/component_wise_pose_error_monitor_node.h>

ComponentWisePoseErrorMonitorNode::ComponentWisePoseErrorMonitorNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    dynamic_reconfig_server_.setCallback(boost::bind(&ComponentWisePoseErrorMonitorNode::dynamicReconfigCallback, this, _1, _2));
    event_sub_ = node_handler_.subscribe("event_in", 1, &ComponentWisePoseErrorMonitorNode::eventCallback, this);
    error_sub_ = node_handler_.subscribe("pose_error", 1, &ComponentWisePoseErrorMonitorNode::errorCallback, this);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    feedback_pub_ = node_handler_.advertise<mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback>("feedback", 1);
    current_state_ = INIT;
    has_pose_error_data_ = false;
}

ComponentWisePoseErrorMonitorNode::~ComponentWisePoseErrorMonitorNode()
{
    event_sub_.shutdown();
    error_sub_.shutdown();
    event_pub_.shutdown();
    feedback_pub_.shutdown();
}

void ComponentWisePoseErrorMonitorNode::dynamicReconfigCallback(mcr_geometric_relation_monitors::ComponentWisePoseErrorMonitorConfig &config, uint32_t level)
{
    threshold_linear_x_ = config.threshold_linear_x;
    threshold_linear_y_ = config.threshold_linear_y;
    threshold_linear_z_ = config.threshold_linear_z;
    threshold_angular_x_ = config.threshold_angular_x;
    threshold_angular_y_ = config.threshold_angular_y;
    threshold_angular_z_ = config.threshold_angular_z;
}

void ComponentWisePoseErrorMonitorNode::eventCallback(const std_msgs::String &event_command)
{
    event_in_msg_ = event_command;
}

void ComponentWisePoseErrorMonitorNode::errorCallback(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr &pose_eror)
{
    error_ = *pose_eror;
    has_pose_error_data_ = true;
}

void ComponentWisePoseErrorMonitorNode::states()
{
    switch (current_state_)
    {
    case INIT:
        initState();
        break;
    case IDLE:
        idleState();
        break;
    case RUNNING:
        runState();
        break;
    default:
        initState();
    }
}

void ComponentWisePoseErrorMonitorNode::initState()
{
    if (event_in_msg_.data == "e_start")
    {
        current_state_ = IDLE;
        has_pose_error_data_ = false;
        event_in_msg_.data = "";
    }
    else
    {
        current_state_ = INIT;
    }
}

void ComponentWisePoseErrorMonitorNode::idleState()
{
    if (event_in_msg_.data == "e_stop")
    {
        current_state_ = INIT;
        event_in_msg_.data = "";
    }
    else if (has_pose_error_data_)
    {
        current_state_ = RUNNING;
        has_pose_error_data_ = false;
    }
    else
    {
        current_state_ = IDLE;
    }
}

void ComponentWisePoseErrorMonitorNode::runState()
{
    if (isComponentWisePoseErrorWithinThreshold())
    {
        status_msg_.data = "e_stop";
        event_pub_.publish(status_msg_);
    }
    feedback_pub_.publish(feedback_);
    current_state_ = IDLE;
}

bool ComponentWisePoseErrorMonitorNode::isComponentWisePoseErrorWithinThreshold()
{
    // Linear X component
    feedback_.is_linear_x_within_tolerance = fabs(error_.linear.x) < threshold_linear_x_;

    // Linear Y component
    feedback_.is_linear_y_within_tolerance = fabs(error_.linear.y) < threshold_linear_y_;

    // Linear Z component
    feedback_.is_linear_z_within_tolerance = fabs(error_.linear.z) < threshold_linear_z_;

    // Angular X component
    feedback_.is_angular_x_within_tolerance = fabs(error_.angular.x) < threshold_angular_x_;

    // Angular Y component
    feedback_.is_angular_y_within_tolerance = fabs(error_.angular.y) < threshold_angular_y_;

    // Angular Z component
    feedback_.is_angular_z_within_tolerance = fabs(error_.angular.z) < threshold_angular_z_;

    return (feedback_.is_linear_x_within_tolerance && feedback_.is_linear_y_within_tolerance && feedback_.is_linear_z_within_tolerance
            && feedback_.is_angular_x_within_tolerance && feedback_.is_angular_y_within_tolerance && feedback_.is_angular_z_within_tolerance);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "component_wise_pose_error_monitor");
    ros::NodeHandle nh("~");
    ROS_INFO("Initialised");
    ComponentWisePoseErrorMonitorNode cwpem(nh);

    double loop_rate = 30;
    nh.param<double>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        cwpem.states();
        rate.sleep();
    }

    return 0;
}
