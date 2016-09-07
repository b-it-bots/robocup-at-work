/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * trajectory_time_parameterizer.h
 *
 * Created on: November 03, 2015
 * Author: Shehzad
 */
#ifndef MCR_TRAJECTORY_TIME_PARAMETERIZER_TRAJECTORY_TIME_PARAMETERIZER_H_
#define MCR_TRAJECTORY_TIME_PARAMETERIZER_TRAJECTORY_TIME_PARAMETERIZER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/shared_ptr.hpp>
#include <mcr_trajectory_time_parameterizer/time_parameterizer.h>
#include <string>

/**
 * This class provides ROS interface of the
 * trajectory time parameterizer.
 */
class TrajectoryTimeParameterizer
{
    public:
        /**
         * Ctor.
         *
         * @param nh An instance of the ROS node handle.
         */
        explicit TrajectoryTimeParameterizer(const ros::NodeHandle &nh);

        /**
         * Dtor.
         */
        virtual ~TrajectoryTimeParameterizer();

        /**
         * Performs state transitions and executes the
         * functionality related to current state.
         */
        void executeCycle();


    private:
        /**
         * Copy Ctor.
         */
        TrajectoryTimeParameterizer(const TrajectoryTimeParameterizer &other);

        /**
         * Assignment operator
         */
        TrajectoryTimeParameterizer &operator=(const TrajectoryTimeParameterizer &other);


        /**
         * Waits for the start event to receive and
         * sets the node state to IDLE.
         */
        void initState();

        /**
         * Checks if input trajectory is received and
         * sets the node state to RUNNING.
         */
        void idleState();

        /**
         * Time parameterizes the trajectory and publishes it to ROS.
         * Sets the state to INIT afterwards.
         */
        void runningState();

        /**
         * Callback to store start event.
         */
        void eventInCallback(const std_msgs::String::ConstPtr &msg);


        /**
        * Callback to set input trajectory
        */
        void trajectoryInCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

        /**
         * Passes a received trajectory to the trajectory parameterizer and
         * publishes the resulting trajectory with a success/failure event
         * to ROS network.
         */
        void parameterizeTrajectory(const trajectory_msgs::JointTrajectory &trajectory_in);


    private:
        enum State
        {
            INIT,
            IDLE,
            RUNNING
        };


    private:
        /**
         * An instance of the ROS Node handle.
         */
        ros::NodeHandle nh_;

        /**
         * Robot arm group name for MoveIt! move_group
         */
        std::string group_;

        /**
         * Flag to check if trajectory is set.
         */
        bool is_trajectory_received_;

        /**
         * Stores the state of the node.
         * The node has three states: INIT, IDLE and RUNNING.
         * - The node starts in INIT state.
         * - Transition from INIT to IDLE occurs upon receiving the start event.
         * - Transition from IDLE to RUNNING occurs upon receiving the input trajectory.
         * - Transition from IDLE to INIT occurs upon receiving the stop event.
         * - Transition from RUNNING to INIT occurs when the trajectory parameterization succeeds/fails.
         */
        State state_;

        /**
         * Subscribers and Publishers
         */
        ros::Subscriber event_in_sub_;
        ros::Subscriber trajectory_in_sub_;
        ros::Publisher event_out_pub_;
        ros::Publisher trajectory_pub_;

        /**
         * Stores event messages.
         * There are two supported events: e_start and e_stop.
         */
        std::string event_in_;

        /**
         * Store start and goal cartesian poses.
         */
        trajectory_msgs::JointTrajectory trajectory_in_;

        /**
         * The interface to the trajectory planner
         * to generate linear trajectories.
         */
        boost::shared_ptr<TimeParameterizer> time_parameterizer_;
};

#endif  // MCR_TRAJECTORY_TIME_PARAMETERIZER_TRAJECTORY_TIME_PARAMETERIZER_H_
