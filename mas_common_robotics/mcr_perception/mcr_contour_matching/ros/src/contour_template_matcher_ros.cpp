#include <mcr_contour_matching/contour_template_matcher_ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mcr_perception_msgs/MatchingErrorStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

ContourTemplateMatcherROS::ContourTemplateMatcherROS() : contours_msg_received_(false), template_pointcloud_msg_received_(false)
{
    ros::NodeHandle nh("~");
    sub_contour_pointclouds_ = nh.subscribe("input/contours", 1, &ContourTemplateMatcherROS::contourPointcloudsCallback, this);
    sub_template_pointcloud_ = nh.subscribe("input/template_pointcloud", 1, &ContourTemplateMatcherROS::templatePointcloudCallback, this);

    pub_matching_error_ = nh.advertise<mcr_perception_msgs::MatchingErrorStamped>("output/matching_error", 1);
    pub_contour_pointcloud_ = nh.advertise<sensor_msgs::PointCloud2>("output/matched_contour_pointcloud", 1);
}

ContourTemplateMatcherROS::~ContourTemplateMatcherROS()
{
}

void ContourTemplateMatcherROS::contourPointcloudsCallback(const mcr_perception_msgs::PointCloud2List::Ptr &msg)
{
    ROS_INFO("[contour_template_matcher] Received contour pointclouds");
    contours_msg_ = msg;
    contours_msg_received_ = true;
}

void ContourTemplateMatcherROS::templatePointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    ROS_INFO("[contour_template_matcher] Received template pointcloud");
    template_pointcloud_msg_ = msg;
    template_pointcloud_msg_received_ = true;
}

void ContourTemplateMatcherROS::update()
{
    if (contours_msg_received_ && template_pointcloud_msg_received_)
    {
        matchContours();
        contours_msg_received_ = false;
    }
}

void ContourTemplateMatcherROS::matchContours()
{
    pcl::PCLPointCloud2::Ptr template_cloud(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*template_pointcloud_msg_, *template_cloud);

    std::vector<pcl::PCLPointCloud2::Ptr> contours;

    for (size_t i = 0; i < contours_msg_->pointclouds.size(); i++)
    {
        pcl::PCLPointCloud2::Ptr contour(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(contours_msg_->pointclouds[i], *contour);
        contours.push_back(contour);
    }

    ROS_INFO("Going to match contours with template");
    pcl::PCLPointCloud2::Ptr matched_contour(new pcl::PCLPointCloud2);
    double match_error = contour_template_matcher_.match(contours, template_cloud, matched_contour);

    sensor_msgs::PointCloud2 ros_output_cloud;
    pcl_conversions::fromPCL(*matched_contour, ros_output_cloud);
    ros_output_cloud.header = contours_msg_->pointclouds[0].header;

    mcr_perception_msgs::MatchingErrorStamped matching_error;
    matching_error.matching_error = match_error;
    matching_error.header.stamp = contours_msg_->pointclouds[0].header.stamp;

    pub_matching_error_.publish(matching_error);
    pub_contour_pointcloud_.publish(ros_output_cloud);
}


