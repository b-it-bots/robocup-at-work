/*
 * Copyright [2016] <Bonn-Rhein-Sieg University>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Subscribes to pose array topic, republishes as nav_msgs/Path topic
 * 
 */

#ifndef MCR_NAVIGATION_TOOLS_POSE_ARRAY_TO_PATH_NODE_H
#define MCR_NAVIGATION_TOOLS_POSE_ARRAY_TO_PATH_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

class PoseArrayToPathNode
{
    public:
        PoseArrayToPathNode();
        ~PoseArrayToPathNode();

        // callback to receive the pose array msg from ros network
        void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg);

        // ros node main loop
        void update();

    private:
        // goes up when poseArrayCallBack gets data
        bool is_pose_array_msg_received_;

        // ros related variables
        ros::NodeHandle nh_;
        ros::Subscriber sub_pose_array_msg_;
        ros::Publisher pub_converted_path_;

        // stores pose array msg received from ros network
        geometry_msgs::PoseArray pose_array_msg_;
};
#endif  // MCR_NAVIGATION_TOOLS_POSE_ARRAY_TO_PATH_NODE_H
