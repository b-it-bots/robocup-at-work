/**
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * trajectory_time_parameterizer_node.cpp
 *
 * Created on: November 03, 2015
 * Author: Shehzad
 */
#include <ros/ros.h>
#include <mcr_trajectory_time_parameterizer/trajectory_time_parameterizer.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trajectory_time_parameterizer");

    ros::NodeHandle nh("~");

    double loop_rate;

    /**
     * Cycle rate in Hz.
     */
    ros::param::param<double>("~loop_rate", loop_rate, 10.0);

    ros::Rate r(loop_rate);

    TrajectoryTimeParameterizer trajectory_time_parameterizer(nh);

    ROS_INFO_STREAM("Ready to start...");

    while (nh.ok())
    {
        ros::spinOnce();

        trajectory_time_parameterizer.executeCycle();

        r.sleep();
    }

    return 0;
}
