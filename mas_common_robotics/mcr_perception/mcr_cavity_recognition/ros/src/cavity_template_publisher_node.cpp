#include <ros/ros.h>
#include <mcr_cavity_recognition/cavity_template_publisher_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cavity_template_publisher");

    CavityTemplatePublisherROS cavity_template_publisher_ros_;

    ROS_INFO("[cavity_template_publisher] node started");

    ros::spin();

    return 0;
}
