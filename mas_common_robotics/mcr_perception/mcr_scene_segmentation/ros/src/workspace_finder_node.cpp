/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#include <mcr_scene_segmentation/workspace_finder_node.h>
#include <mcr_scene_segmentation/impl/helpers.hpp>

WorkspaceFinderNode::WorkspaceFinderNode()
    : nh("~"), trigger_workspace_finder_(false), run_workspace_finder_(false), sync_message_received_(false),
      polygon_visualizer_("workspace_polygon", Color(Color::SALMON))
{
    double message_age;
    nh.param<double>("message_age_threshold", message_age, 1.0);
    message_age_threshold_ = ros::Duration(message_age);

    pub_event_out_ = nh.advertise<std_msgs::String>("event_out", 1);
    pub_polygon_ = nh.advertise<mcr_perception_msgs::PlanarPolygon>("polygon", 1);

    sub_event_in_ = nh.subscribe("event_in", 1, &WorkspaceFinderNode::eventInCallback, this);

    sub_polygon_.subscribe(nh, "input/polygon", 3);
    sub_coefficients_.subscribe(nh, "input/coefficients", 3);

    sync_input_ = boost::make_shared<message_filters::Synchronizer<PlanarPolygonSyncPolicy> >(3);
    sync_input_->connectInput(sub_polygon_, sub_coefficients_);
    sync_input_->registerCallback(boost::bind(&WorkspaceFinderNode::synchronizedCallback, this, _1, _2));
}

WorkspaceFinderNode::~WorkspaceFinderNode()
{
}

void WorkspaceFinderNode::eventInCallback(const std_msgs::String &msg)
{
    sync_message_received_ = false;
    if (msg.data == "e_trigger")
    {
        trigger_workspace_finder_ = true;
    }
    else if (msg.data == "e_start")
    {
        run_workspace_finder_ = true;
    }
    else if (msg.data == "e_stop")
    {
        run_workspace_finder_ = false;
        trigger_workspace_finder_ = false;
    }
}

void WorkspaceFinderNode::synchronizedCallback(const geometry_msgs::PolygonStamped::ConstPtr &polygon_msg,
        const pcl_msgs::ModelCoefficients::ConstPtr &coefficients_msg)
{
    if ((ros::Time::now() - polygon_msg->header.stamp) > message_age_threshold_)
    {
        return;
    }
    polygon_msg_ = polygon_msg;
    coefficients_msg_ = coefficients_msg;
    sync_message_received_ = true;
}

void WorkspaceFinderNode::update()
{
    if (sync_message_received_ && (trigger_workspace_finder_ || run_workspace_finder_))
    {
        mcr_perception_msgs::PlanarPolygon mcr_polygon_msg;
        mcr_polygon_msg.header = polygon_msg_->header;
        mcr_polygon_msg.contour = polygon_msg_->polygon.points;
        mcr_polygon_msg.contour.push_back(mcr_polygon_msg.contour.front());

        for (int i = 0; i < 4; i++)
        {
            mcr_polygon_msg.coefficients[i] = coefficients_msg_->values[i];
        }

        pub_polygon_.publish(mcr_polygon_msg);

        PlanarPolygon mcr_polygon;
        convertPlanarPolygon(mcr_polygon_msg, mcr_polygon);
        polygon_visualizer_.publish(mcr_polygon, polygon_msg_->header.frame_id);

        std_msgs::String event_out_str;
        event_out_str.data = "e_done";
        pub_event_out_.publish(event_out_str);

        sync_message_received_ = false;

        if (trigger_workspace_finder_)
        {
            trigger_workspace_finder_ = false;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "workspace_finder");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    WorkspaceFinderNode workspace_finder;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        workspace_finder.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
