/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * trajectory_time_parameterizer.cpp
 *
 * Created on: Novemeber 03, 2015
 * Author: Shehzad
 */
#include <mcr_trajectory_time_parameterizer/trajectory_time_parameterizer.h>
#include <string>

TrajectoryTimeParameterizer::TrajectoryTimeParameterizer(const ros::NodeHandle &nh) :
        nh_(nh), is_trajectory_received_(false), state_(INIT)
{
    ros::param::param<std::string>("~group_name", group_, "arm");

    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("trajectory_out", 10);
    event_out_pub_ = nh_.advertise<std_msgs::String>("event_out", 10);

    trajectory_in_sub_ = nh_.subscribe("trajectory_in", 1, &TrajectoryTimeParameterizer::trajectoryInCallback, this);
    event_in_sub_ = nh_.subscribe("event_in", 1, &TrajectoryTimeParameterizer::eventInCallback, this);
    time_parameterizer_.reset(new TimeParameterizer(group_));

    ROS_DEBUG("State: INIT");
}

TrajectoryTimeParameterizer::~TrajectoryTimeParameterizer()
{
}


void TrajectoryTimeParameterizer::executeCycle()
{
    switch (state_)
    {
        case INIT:
            initState();
            break;

        case IDLE:
            idleState();
            break;

        case RUNNING:
            runningState();
            break;
    }
}


void TrajectoryTimeParameterizer::initState()
{
    if (event_in_ == "e_start") {
        state_ = IDLE;
        ROS_DEBUG("State: IDLE");
        event_in_ = "";
    }
}


void TrajectoryTimeParameterizer::idleState()
{
    if (is_trajectory_received_)
    {
        state_ = RUNNING;
        is_trajectory_received_ = false;
        ROS_DEBUG("State: RUNNING");
    }

    if (event_in_ == "e_stop") {
        std_msgs::String event_out_msg;
        event_out_msg.data = "e_stopped";
        event_out_pub_.publish(event_out_msg);
        state_ = INIT;
        ROS_DEBUG("State: INIT");
        event_in_ = "";
    }
}


void TrajectoryTimeParameterizer::runningState()
{
    parameterizeTrajectory(trajectory_in_);
    state_ = INIT;
    ROS_DEBUG("State: INIT");
}


void TrajectoryTimeParameterizer::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
    event_in_ = msg->data;
}


void TrajectoryTimeParameterizer::trajectoryInCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    trajectory_in_ = *msg;
    is_trajectory_received_ = true;
}


void TrajectoryTimeParameterizer::parameterizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in)
{
    trajectory_msgs::JointTrajectory trajectory_out;
    std_msgs::String event_out_msg;

    bool success = time_parameterizer_->performTrajectoryTimeParameterized(trajectory_in, trajectory_out);

    if (success)
    {
        event_out_msg.data = "e_success";
        trajectory_pub_.publish(trajectory_out);
    }
    else
    {
        event_out_msg.data = "e_failure";
        ROS_DEBUG("Trajectory parameterization failed.");
    }

    event_out_pub_.publish(event_out_msg);
}
