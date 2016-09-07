/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Subscribes to pose array topic, republishes as nav_msgs/Path topic
 * 
 */

#include <mcr_navigation_tools/pose_array_to_path_node.h>

PoseArrayToPathNode::PoseArrayToPathNode() : nh_("~"), is_pose_array_msg_received_(false)
{
    // subscriptions
    sub_pose_array_msg_ = nh_.subscribe("pose_array", 1, &PoseArrayToPathNode::poseArrayCallback, this);

    // publications
    pub_converted_path_ = nh_.advertise<nav_msgs::Path>("path", 1);
}

PoseArrayToPathNode::~PoseArrayToPathNode()
{
    // shut down subscribers and publishers
    sub_pose_array_msg_.shutdown();
    pub_converted_path_.shutdown();
}

void PoseArrayToPathNode::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    pose_array_msg_ = *msg;
    is_pose_array_msg_received_ = true;
}

void PoseArrayToPathNode::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (!is_pose_array_msg_received_)
        return;

    // to publish the pose array as path
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_stamped_msg;

    // reset flag
    is_pose_array_msg_received_ = false;

    // republish pose array as path msg
    path_msg.header = pose_array_msg_.header;
    pose_stamped_msg.header = pose_array_msg_.header;

    path_msg.poses.resize(pose_array_msg_.poses.size());

    // loop over all array poses
    for (int i = 0; i < pose_array_msg_.poses.size(); i++)
    {
        // create intermediate pose stamped
        pose_stamped_msg.pose = pose_array_msg_.poses[i];

        // append the last pose stamped to path
        path_msg.poses[i] = pose_stamped_msg;
    }

    // publish path
    pub_converted_path_.publish(path_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_array_to_path");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (PoseArrayToPathNode)
    PoseArrayToPathNode pose_array_to_path_node;

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
        pose_array_to_path_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
