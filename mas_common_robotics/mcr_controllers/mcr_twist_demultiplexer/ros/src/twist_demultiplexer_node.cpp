/**
 * @file twist_demultiplexer_node.cpp
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @date June, 2015
 */

#include <mcr_twist_demultiplexer/twist_demultiplexer_node.h>

TwistDemultiplexerNode::TwistDemultiplexerNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    node_handler_.param<std::string>("arm_tf", arm_tf_, "/arm_cam3d_rgb_optical_frame");

    node_handler_.param<bool>("is_base_linear_x_enabled", is_twist_part_enabled_in_base_[0], false);
    node_handler_.param<bool>("is_base_linear_y_enabled", is_twist_part_enabled_in_base_[1], false);
    node_handler_.param<bool>("is_base_linear_z_enabled", is_twist_part_enabled_in_base_[2], false);
    node_handler_.param<bool>("is_base_angular_x_enabled", is_twist_part_enabled_in_base_[3], false);
    node_handler_.param<bool>("is_base_angular_y_enabled", is_twist_part_enabled_in_base_[4], false);
    node_handler_.param<bool>("is_base_angular_z_enabled", is_twist_part_enabled_in_base_[5], false);

    node_handler_.param<bool>("is_arm_linear_x_enabled", is_twist_part_enabled_in_arm_[0], false);
    node_handler_.param<bool>("is_arm_linear_y_enabled", is_twist_part_enabled_in_arm_[1], false);
    node_handler_.param<bool>("is_arm_linear_z_enabled", is_twist_part_enabled_in_arm_[2], false);
    node_handler_.param<bool>("is_arm_angular_x_enabled", is_twist_part_enabled_in_arm_[3], false);
    node_handler_.param<bool>("is_arm_angular_y_enabled", is_twist_part_enabled_in_arm_[4], false);
    node_handler_.param<bool>("is_arm_angular_z_enabled", is_twist_part_enabled_in_arm_[5], false);

    node_handler_.param<bool>("is_error_monitor_enabled", is_error_monitor_enabled_, "false");

    arm_twist_stamped_pub_ = node_handler_.advertise<geometry_msgs::TwistStamped>("arm_twist", 1);
    base_twist_pub_ = node_handler_.advertise<geometry_msgs::Twist>("base_twist", 1);

    event_sub_ = node_handler_.subscribe("event_in", 1, &TwistDemultiplexerNode::eventCallback, this);
    twist_sub_ = node_handler_.subscribe("input_twist", 1, &TwistDemultiplexerNode::twistStampedCallback, this);
    error_feedback_sub_ = node_handler_.subscribe("error_feedback", 1, &TwistDemultiplexerNode::errorFeedbackCallback, this);

    current_state_ = INIT;
    has_twist_data_ = false;
    has_error_feedback_data_ = false;
}

TwistDemultiplexerNode::~TwistDemultiplexerNode()
{
    event_sub_.shutdown();
    twist_sub_.shutdown();
    error_feedback_sub_.shutdown();
    arm_twist_stamped_pub_.shutdown();
    base_twist_pub_.shutdown();
}

void TwistDemultiplexerNode::eventCallback(const std_msgs::String &event_command)
{
    event_in_msg_ = event_command;
}

void TwistDemultiplexerNode::twistStampedCallback(const geometry_msgs::TwistStamped &msg)
{
    input_twist_ = msg;
    has_twist_data_ = true;

    input_twist_array_[0] = input_twist_.twist.linear.x;
    input_twist_array_[1] = input_twist_.twist.linear.y;
    input_twist_array_[2] = input_twist_.twist.linear.z;
    input_twist_array_[3] = input_twist_.twist.angular.x;
    input_twist_array_[4] = input_twist_.twist.angular.y;
    input_twist_array_[5] = input_twist_.twist.angular.z;
}

void TwistDemultiplexerNode::errorFeedbackCallback(const mcr_monitoring_msgs::ComponentWisePoseErrorMonitorFeedback &error_feedback)
{
    error_feedback_msg_ = error_feedback;
    has_error_feedback_data_ = true;

    is_error_part_within_tolerance_[0] = error_feedback_msg_.is_linear_x_within_tolerance;
    is_error_part_within_tolerance_[1] = error_feedback_msg_.is_linear_y_within_tolerance;
    is_error_part_within_tolerance_[2] = error_feedback_msg_.is_linear_z_within_tolerance;
    is_error_part_within_tolerance_[3] = error_feedback_msg_.is_angular_x_within_tolerance;
    is_error_part_within_tolerance_[4] = error_feedback_msg_.is_angular_y_within_tolerance;
    is_error_part_within_tolerance_[5] = error_feedback_msg_.is_angular_z_within_tolerance;
}

void TwistDemultiplexerNode::states()
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

void TwistDemultiplexerNode::initState()
{
    if (event_in_msg_.data == "e_start")
    {
        current_state_ = IDLE;
        event_in_msg_.data = "";
        has_twist_data_ = false;
        has_error_feedback_data_ = false;
    }
    else
    {
        current_state_ = INIT;
    }
}

void TwistDemultiplexerNode::idleState()
{
    if (event_in_msg_.data == "e_stop")
    {
        current_state_ = INIT;
        event_in_msg_.data = "";
        geometry_msgs::TwistStamped zero_arm_twist;
        zero_arm_twist.header.frame_id = arm_tf_;
        arm_twist_stamped_pub_.publish(zero_arm_twist);
        geometry_msgs::Twist zero_base_twist;
        base_twist_pub_.publish(zero_base_twist);
    }
    else if (is_error_monitor_enabled_)
    {
        if (has_twist_data_ && has_error_feedback_data_)
        {
            current_state_ = RUNNING;
            has_twist_data_ = false;
            has_error_feedback_data_ = false;
        }
        else
        {
            current_state_ = IDLE;
        }
    }
    else
    {
        if (has_twist_data_)
        {
            current_state_ = RUNNING;
            has_twist_data_ = false;
        }
        else
        {
            current_state_ = IDLE;
        }
    }
}

void TwistDemultiplexerNode::runState()
{
    demultiplexTwist(base_twist_array_, is_twist_part_enabled_in_base_);
    demultiplexTwist(arm_twist_array_, is_twist_part_enabled_in_arm_);

    base_twist_.linear.x = base_twist_array_[0];
    base_twist_.linear.y = base_twist_array_[1];
    base_twist_.linear.z = base_twist_array_[2];
    base_twist_.angular.x = base_twist_array_[3];
    base_twist_.angular.y = base_twist_array_[4];
    base_twist_.angular.z = base_twist_array_[5];
    base_twist_pub_.publish(base_twist_);

    arm_twist_.twist.linear.x = arm_twist_array_[0];
    arm_twist_.twist.linear.y = arm_twist_array_[1];
    arm_twist_.twist.linear.z = arm_twist_array_[2];
    arm_twist_.twist.angular.x = arm_twist_array_[3];
    arm_twist_.twist.angular.y = arm_twist_array_[4];
    arm_twist_.twist.angular.z = arm_twist_array_[5];
    arm_twist_.header.frame_id = arm_tf_;
    arm_twist_stamped_pub_.publish(arm_twist_);

    current_state_ = IDLE;
}

void TwistDemultiplexerNode::demultiplexTwist(double *twist_as_array, bool *is_twist_part_enabled)
{
    for (size_t i = 0; i < NO_OF_PARTS_IN_TWIST ; i++)
    {
        twist_as_array[i] = input_twist_array_[i];
        if (!is_twist_part_enabled[i] || (is_error_monitor_enabled_ && is_error_part_within_tolerance_[i]))
        {
            twist_as_array[i] = 0.0;
        }
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_demultiplexer");
    ros::NodeHandle nh("~");
    ROS_INFO("Initialized");
    TwistDemultiplexerNode twist_demultiplexer_node(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        twist_demultiplexer_node.states();
        rate.sleep();
    }
    return 0;
}
