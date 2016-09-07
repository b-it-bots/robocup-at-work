#include <ros/ros.h>
#include <mcr_cavity_recognition/cavity_message_builder_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cavity_message_builder");

    CavityMessageBuilderROS cavity_message_builder_ros_;

    ROS_INFO("[cavity_message_builder] node started");

    ros::spin();

    return 0;
}
