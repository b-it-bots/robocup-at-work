#include <ros/ros.h>
#include <mcr_pose_estimation/pca_pose_estimator_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pca_pose_estimator");
    ros::NodeHandle nh("~");

    int frame_rate = 30;    // in Hz
    PCAPoseEstimatorRos pca_pose_estimator_ros_;

    nh.param<int>("frame_rate", frame_rate, 30);
    ROS_INFO("[pca_pose_estimator] node started");

    ros::Rate loop_rate(frame_rate);

    while (ros::ok())
    {
        pca_pose_estimator_ros_.update();

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
