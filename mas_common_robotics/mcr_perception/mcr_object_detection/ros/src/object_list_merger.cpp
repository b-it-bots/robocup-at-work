/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */

#include <mcr_object_detection/object_list_merger.h>
#include <cmath>

ObjectListMerger::ObjectListMerger() : nh_("~"), event_in_received_(false), accepting_object_lists_(false)
{
    nh_.param<double>("distance_threshold", distance_threshold_, 0.02);
    pub_object_list_ = nh_.advertise<mcr_perception_msgs::ObjectList>("output_object_list", 1);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);

    sub_object_list_ = nh_.subscribe("input_object_list", 1, &ObjectListMerger::objectListCallback, this);
    sub_event_in_ = nh_.subscribe("event_in", 1, &ObjectListMerger::eventInCallback, this);
}

ObjectListMerger::~ObjectListMerger()
{
}

void ObjectListMerger::update()
{
    if (event_in_received_)
    {
        std_msgs::String event_out;
        if (event_in_ == "e_trigger")
        {
            if (!merged_object_list_.objects.empty())
            {
                pub_object_list_.publish(merged_object_list_);
                event_out.data = "e_done";
            }
            else
            {
                event_out.data = "e_empty_list";
            }
        }
        else if (event_in_ == "e_stop")
        {
            accepting_object_lists_ = false;
            event_out.data = "e_stopped";
        }
        else if (event_in_ == "e_start")
        {
            accepting_object_lists_ = true;
            merged_object_list_.objects.clear();
            event_out.data = "e_started";
        }
        event_in_received_ = false;
        pub_event_out_.publish(event_out);
    }
}

void ObjectListMerger::mergeList(const mcr_perception_msgs::ObjectList::Ptr &new_object_list)
{
    for (int i = 0; i < new_object_list->objects.size(); i++)
    {
        bool merged = false;
        const mcr_perception_msgs::Object new_object = new_object_list->objects[i];
        for (int j = 0; j < merged_object_list_.objects.size(); j++)
        {
            mcr_perception_msgs::Object old_object = merged_object_list_.objects[j];
            double distance = getDistance(new_object.pose.pose.position, old_object.pose.pose.position);
            // if both objects are close together
            if (distance < distance_threshold_)
            {
                ROS_INFO_STREAM("New object " << new_object.name << " is close to " << old_object.name);
                // if classification of new object has higher probabliity
                // use the new classification
                if (new_object.probability > old_object.probability)
                {
                    ROS_INFO_STREAM("Replacing "
                                     << old_object.name << " with " << new_object.name);
                    old_object.name = new_object.name;
                }
                merged = true;
                break;
            }
        }
        if (!merged)
        {
            merged_object_list_.objects.push_back(new_object);
        }
    }
    ROS_INFO_STREAM("Total objects : " << merged_object_list_.objects.size());
}

void ObjectListMerger::objectListCallback(const mcr_perception_msgs::ObjectList::Ptr &msg)
{
    if (accepting_object_lists_)
    {
        mergeList(msg);
    }
}

void ObjectListMerger::eventInCallback(const std_msgs::String::Ptr &msg)
{
    event_in_ = msg->data;
    event_in_received_ = true;
}

double ObjectListMerger::getDistance(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
    return std::sqrt(std::pow(point1.x - point2.x, 2.0) +
                     std::pow(point1.y - point2.y, 2.0) +
                     std::pow(point1.z - point2.z, 2.0));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_list_merger");
    ros::NodeHandle nh("~");

    int frame_rate = 5;    // in Hz
    ObjectListMerger object_list_merger;

    nh.param<int>("frame_rate", frame_rate, 5);
    ROS_INFO("node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        object_list_merger.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
