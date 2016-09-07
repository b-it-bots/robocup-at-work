#include <mcr_cavity_recognition/cavity_template_publisher_ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>


CavityTemplatePublisherROS::CavityTemplatePublisherROS() : nh_("~")
{
    template_path_ = ros::package::getPath("mcr_cavity_recognition");
    template_path_ /= "common";
    template_path_ /= "config";
    template_path_ /= "templates";

    sub_object_name_ = nh_.subscribe("input/object_name", 1, &CavityTemplatePublisherROS::objectNameCallback, this);
    pub_template_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("output/template_pointcloud", 1);
}

CavityTemplatePublisherROS::~CavityTemplatePublisherROS()
{
}

void CavityTemplatePublisherROS::objectNameCallback(const std_msgs::StringPtr &msg)
{
    ROS_INFO("Received object name %s", msg->data.c_str());

    std::string template_filename = getTemplateFilename(msg->data) + ".pcd";
    bfs::path full_path = template_path_ / template_filename;

    pcl::PCLPointCloud2::Ptr template_cloud(new pcl::PCLPointCloud2);

    if (pcl::io::loadPCDFile(full_path.string(), *template_cloud) == -1)
    {
        ROS_ERROR("Couldn't read template file %s\n", full_path.string().c_str());
        return;
    }

    sensor_msgs::PointCloud2 ros_output_cloud;
    pcl_conversions::fromPCL(*template_cloud, ros_output_cloud);
    pub_template_pointcloud_.publish(ros_output_cloud);

    ROS_INFO("Published template pointcloud: %s", template_filename.c_str());
}

std::string CavityTemplatePublisherROS::getTemplateFilename(const std::string &object_name)
{
    std::map<std::string, std::string> mapping;
    std::string orientation;

    nh_.getParam(object_name, mapping);
    nh_.getParam("preferred_orientation", orientation);

    std::string filename = mapping[orientation];

    return filename;
}
