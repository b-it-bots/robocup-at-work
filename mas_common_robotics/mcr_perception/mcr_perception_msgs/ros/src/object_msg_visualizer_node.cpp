/*
 * object_msg_visualizer_node.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: Frederik Hegger
 */

#include <mcr_perception_msgs/Object.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class ObjectMsgVisualizerNode
{
public:
    ObjectMsgVisualizerNode()
    {
        ros::NodeHandle nh("~");

        sub_object_ = nh.subscribe("input/object", 1, &ObjectMsgVisualizerNode::objectCallback, this);
        pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    }

    virtual ~ObjectMsgVisualizerNode()
    {
        sub_object_.shutdown();
        pub_marker_.shutdown();
    }

    void objectCallback(const mcr_perception_msgs::Object::Ptr msg)
    {
        visualization_msgs::MarkerArray  marker_array;
        visualization_msgs::Marker marker_text;
        visualization_msgs::Marker marker_pose;

        marker_pose.header.frame_id = marker_text.header.frame_id = msg->pose.header.frame_id;
        marker_pose.header.stamp = marker_text.header.stamp = ros::Time::now();
        marker_pose.ns = "object - pose";
        marker_text.ns = " object - name";
        marker_pose.action = marker_text.action = visualization_msgs::Marker::ADD;

        marker_pose.id = marker_text.id = 0;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_pose.type = visualization_msgs::Marker::SPHERE;

        marker_pose.pose = marker_text.pose = msg->pose.pose;

        marker_text.text = msg->name;

        marker_pose.scale.x = 0.05;
        marker_pose.scale.y = 0.05;
        marker_pose.scale.z = 0.05;
        marker_pose.color.r = 1.0;
        marker_pose.color.g = 0.0;
        marker_pose.color.b = 0.0;
        marker_pose.color.a = 0.5;

        marker_text.scale.x = 0.05;
        marker_text.scale.y = 0.05;
        marker_text.scale.z = 0.05;
        marker_text.color.r = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.b = 1.0;
        marker_text.color.a = 1.0;

        marker_array.markers.push_back(marker_pose);
        marker_array.markers.push_back(marker_text);

        pub_marker_.publish(marker_array);
    }

private:
    ros::Subscriber sub_object_;
    ros::Publisher pub_marker_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_msg_visualizer");

    ObjectMsgVisualizerNode obj_visualizer;

    ros::spin();

    return 0;
}
