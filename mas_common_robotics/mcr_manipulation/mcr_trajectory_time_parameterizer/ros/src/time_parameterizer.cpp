/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * time_parameterizer.cpp
 *
 * Created on: November 03, 2015
 * Author: Shehzad
 */
#include <mcr_trajectory_time_parameterizer/time_parameterizer.h>
#include <string>

TimeParameterizer::TimeParameterizer(const std::string &arm_group_name) :
        move_group_(arm_group_name), group_name_(arm_group_name)
{
}


TimeParameterizer::~TimeParameterizer()
{
}

bool TimeParameterizer::performTrajectoryTimeParameterized(const trajectory_msgs::JointTrajectory &trajectory_in,
                trajectory_msgs::JointTrajectory &trajectory_out)
{
    moveit_msgs::RobotTrajectory moveit_trajectory;

    moveit_trajectory.joint_trajectory = trajectory_in;

    robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), group_name_);

    rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), moveit_trajectory);

    if (!iptp_.computeTimeStamps(rt))
    {
        return false;
    }

    rt.getRobotTrajectoryMsg(moveit_trajectory);

    trajectory_out = moveit_trajectory.joint_trajectory;

    return true;
}
