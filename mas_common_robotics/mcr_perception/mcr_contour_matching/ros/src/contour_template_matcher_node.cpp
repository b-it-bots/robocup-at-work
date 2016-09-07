#include <ros/ros.h>
#include <mcr_contour_matching/contour_template_matcher_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contour_template_matcher");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    ContourTemplateMatcherROS contour_template_matcher_ros_;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("[contour_template_matcher] node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        contour_template_matcher_ros_.update();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
