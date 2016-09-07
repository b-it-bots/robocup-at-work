#include <mcr_cavity_recognition/cavity_message_builder_ros.h>
#include <mcr_perception_msgs/Cavity.h>

CavityMessageBuilderROS::CavityMessageBuilderROS()
{
    ros::NodeHandle nh("~");
    sub_pointcloud_.subscribe(nh, "input/pointcloud", 3);
    sub_matching_error_.subscribe(nh, "input/matching_error", 3);
    sub_pose_.subscribe(nh, "input/pose", 3);
    sync_input_ = boost::make_shared<message_filters::Synchronizer<CavitySyncPolicy> >(3);
    sync_input_->connectInput(sub_pointcloud_, sub_matching_error_, sub_pose_);
    sync_input_->registerCallback(boost::bind(&CavityMessageBuilderROS::synchronizedCallback, this, _1, _2, _3));
    pub_cavity_ = nh.advertise<mcr_perception_msgs::Cavity>("output/cavity", 1);

}

CavityMessageBuilderROS::~CavityMessageBuilderROS()
{
}

void CavityMessageBuilderROS::synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg, const mcr_perception_msgs::MatchingErrorStamped::ConstPtr &matching_error_msg, const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    mcr_perception_msgs::Cavity cavity;
    cavity.pointcloud = *pointcloud_msg;
    cavity.template_matching_error = *matching_error_msg;
    cavity.pose = *pose_msg;

    pub_cavity_.publish(cavity);
}
