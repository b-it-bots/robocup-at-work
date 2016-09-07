#include <mcr_contour_matching/contour_finder_ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <mcr_perception_msgs/PointCloud2List.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


ContourFinderROS::ContourFinderROS() : nh_("~"), pointcloud_msg_received_(false), publish_debug_image_(true)
{
    dynamic_reconfigure_server_.setCallback(boost::bind(&ContourFinderROS::dynamicReconfigCallback, this, _1, _2));
    image_transport::ImageTransport it(nh_);
    pub_contour_pointclouds_ = nh_.advertise<mcr_perception_msgs::PointCloud2List>("output/pointclouds", 1);
    pub_contour_pointclouds_combined_ = nh_.advertise<sensor_msgs::PointCloud2>("output/pointclouds_combined", 1);
    pub_debug_image_ = it.advertise("output/debug_image", 1);
    sub_event_in_ = nh_.subscribe("input/event_in", 1, &ContourFinderROS::eventInCallback, this);
}

ContourFinderROS::~ContourFinderROS()
{
}

void ContourFinderROS::update()
{
    if (pointcloud_msg_received_)
    {
        findContours();
        pointcloud_msg_received_ = false;
    }
}

void ContourFinderROS::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    ROS_INFO("[contour_finder] Received pointcloud message");
    pointcloud_msg_ = msg;
    pointcloud_msg_received_ = true;
    sub_pointcloud_.shutdown();
}

void ContourFinderROS::eventInCallback(const std_msgs::String &msg)
{
    if (msg.data == "e_trigger")
    {
        sub_pointcloud_ = nh_.subscribe("input/pointcloud", 1, &ContourFinderROS::pointcloudCallback, this);
        ROS_INFO("Subscribed to pointcloud");
    }
}

void ContourFinderROS::dynamicReconfigCallback(mcr_contour_matching::ContourFinderConfig &config, uint32_t level)
{
    contour_finder_.setCannyThreshold(config.canny_threshold);
    contour_finder_.setCannyMultiplier(config.canny_multiplier);
}

void ContourFinderROS::findContours()
{
    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

    // Convert to PCL data type
    pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(*pcl_input_cloud, pcl_image);

    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);


    cv::Mat debug_image;
    std::vector<std::vector<cv::Point> > contours = contour_finder_.find2DContours(cv_image->image, debug_image);
    ROS_INFO("[contour_finder] Found %i contours", (int)contours.size());
    if (contours.size() == 0)
    {
        return;
    }

    std::vector<pcl::PCLPointCloud2::Ptr> pcl_contours = contour_finder_.get3DContours(contours, pcl_input_cloud);

    mcr_perception_msgs::PointCloud2List ros_contours;

    for (size_t i = 0; i < pcl_contours.size(); i++)
    {
        sensor_msgs::PointCloud2 ros_pointcloud;
        ros_pointcloud.header = pointcloud_msg_->header;
        // Convert to ROS data type
        pcl_conversions::fromPCL(*(pcl_contours[i]), ros_pointcloud);
        ros_contours.pointclouds.push_back(ros_pointcloud);
    }

    // Publish the contours
    pub_contour_pointclouds_.publish(ros_contours);

    if (publish_debug_image_)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_image_msg.image = debug_image;
        pub_debug_image_.publish(debug_image_msg.toImageMsg());

        pcl::PointCloud<pcl::PointXYZ>::Ptr contours_combined(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < pcl_contours.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(*(pcl_contours[i]), *xyz_contour);
            if (i == 0)
            {
                *contours_combined = *xyz_contour;
            }
            else
            {
                *(contours_combined) += *xyz_contour;
            }
        }

        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*contours_combined, *pcl_contour);
        pcl_conversions::fromPCL(*pcl_contour, ros_pointcloud);
        ros_pointcloud.header = pointcloud_msg_->header;
        pub_contour_pointclouds_combined_.publish(pcl_contour);
    }
}
