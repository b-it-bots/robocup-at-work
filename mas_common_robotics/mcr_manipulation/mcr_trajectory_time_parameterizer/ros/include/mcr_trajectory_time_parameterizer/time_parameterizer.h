/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * time_parameterizer.h
 *
 * Created on: Novemeber 03, 2015
 * Author: Shehzad
 */
#ifndef MCR_TRAJECTORY_TIME_PARAMETERIZER_TIME_PARAMETERIZER_H_
#define MCR_TRAJECTORY_TIME_PARAMETERIZER_TIME_PARAMETERIZER_H_

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


/**
 * A trajectory parameterizer component for robotic arms based on MoveIt!.
 * This component takes in a joint-space trajectory which only contains 
 * joint positions.
 * The resulting joint-space trajectories contrains velocities and accelerations.
 * The trajectory parameterizer assumes that Moveit! is running.
 */
class TimeParameterizer
{
    public:
        /**
         * Ctor.
         * The constructor takes in MoveIt!-specific configuration parameters.
         *
         * @param arm_group_name Group name for the robot arm.
         */
        explicit TimeParameterizer(const std::string &arm_group_name);

        /**
         * Dtor.
         */
        virtual ~TimeParameterizer();

        /** 
         * This method performs time parameterization of a given trajectory.
         *
         * @param trajectory_in Stores trajectory which needs to be parameterized.
         * @param trajectory_out Holds the returned parameterized trajectory in it.
         * @return True if time parameterization of the trajectory is succeeds.
         */
        bool performTrajectoryTimeParameterized(const trajectory_msgs::JointTrajectory &trajectory_in,
                trajectory_msgs::JointTrajectory &trajectory_out);


    private:
        /**
         * Copy ctor.
         */
        TimeParameterizer(const TimeParameterizer &other);

        /**
         * Assignment operator
         */
        TimeParameterizer &operator=(const TimeParameterizer &other);


    private:
        /**
         * Group name for the robot arm.
         */
        std::string group_name_;

        /**
         * This provides the interface to Moveit! to perform certain actions,
         * such as inverse kinematics calculations or motion planning, for a
         * specific part of the robot (e.g. the manipualtor).
         */
        move_group_interface::MoveGroup move_group_;

        /**
         * This provides the interface to the MoveIt! trajectory processing
         * to modify the time stamps of a trajectory..
         */
        trajectory_processing::IterativeParabolicTimeParameterization iptp_;
};

#endif  // MCR_TRAJECTORY_TIME_PARAMETERIZER_TIME_PARAMETERIZER_H_
