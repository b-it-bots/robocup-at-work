#include <ros/ros.h>
#include <mcr_contour_matching/contour_finder_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contour_finder");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    ContourFinderROS contour_finder_ros_;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("[contour_finder] node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        contour_finder_ros_.update();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
