/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * twist_to_motion_direction_conversion_node.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#include <mcr_common_converters/twist_to_motion_direction_conversion_node.h>

#include <string>

TwistToMotionDirectionConversionNode::TwistToMotionDirectionConversionNode() :
    twist_msg_received_(false), event_msg_received_(false), current_state_(INIT)
{
    ros::NodeHandle nh("~");

    sub_event_ = nh.subscribe("event_in", 1, &TwistToMotionDirectionConversionNode::eventCallback, this);
    sub_twist_ = nh.subscribe("input/twist", 1, &TwistToMotionDirectionConversionNode::twistCallback, this);
    pub_pose_ = nh.advertise < geometry_msgs::PoseStamped > ("output/pose", 1);
    pub_point_ = nh.advertise < geometry_msgs::PointStamped > ("output/point", 1);

    nh.param < std::string > ("frame_id", frame_id_, "/base_link");
    nh.param<double>("distance_to_frame", distance_to_frame_, 1.5);
}

TwistToMotionDirectionConversionNode::~TwistToMotionDirectionConversionNode()
{
    sub_event_.shutdown();
    sub_twist_.shutdown();
    pub_pose_.shutdown();
    pub_point_.shutdown();
}

void TwistToMotionDirectionConversionNode::twistCallback(const geometry_msgs::TwistPtr &msg)
{
    twist_msg_ = *msg;
    twist_msg_received_ = true;
}

void TwistToMotionDirectionConversionNode::eventCallback(const std_msgs::StringPtr &msg)
{
    event_msg_ = *msg;
    event_msg_received_ = true;
}

void TwistToMotionDirectionConversionNode::computeMotionDirectionAndPublish()
{
    // if there is not motion, return
    if (fabs(twist_msg_.linear.x) + fabs(twist_msg_.linear.y) + fabs(twist_msg_.angular.z) == 0)
        return;

    double motion_direction = 0.0;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PointStamped point;

    motion_direction = getMotionDirectionFromTwist2D(twist_msg_.linear.x, twist_msg_.linear.y, twist_msg_.angular.z);

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;

    point.header = pose.header;

    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, motion_direction);

    point.point.x = distance_to_frame_ * cos(motion_direction);
    point.point.y = distance_to_frame_ * sin(motion_direction);
    point.point.z = 0.0;

    pub_pose_.publish(pose);
    pub_point_.publish(point);
}

void TwistToMotionDirectionConversionNode::update()
{
    // check if a new event has been received
    if (event_msg_received_)
    {
        ROS_INFO_STREAM("Received event: " << event_msg_.data);

        if (event_msg_.data == "e_start")
            current_state_ = RUN;
        else if (event_msg_.data == "e_stop")
            current_state_ = INIT;
        else
            ROS_ERROR_STREAM("Event not supported: " << event_msg_.data);


        event_msg_received_ = false;
    }

    // if state is INIT, do nothing
    if (current_state_ == INIT)
        return;

    // if not msg received, do nothing
    if (!twist_msg_received_)
        return;

    computeMotionDirectionAndPublish();

    twist_msg_received_ = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_to_motion_direction_conversion");
    ros::NodeHandle nh("~");

    TwistToMotionDirectionConversionNode conversion_node = TwistToMotionDirectionConversionNode();

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        conversion_node.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
