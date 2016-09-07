#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <mcr_perception_msgs/AnalyzeCloudColor.h>

bool analyzeCallback(mcr_perception_msgs::AnalyzeCloudColor::Request& request, mcr_perception_msgs::AnalyzeCloudColor::Response& response)
{
    ROS_INFO("Received [analyze_cloud_color] request.");

    typedef boost::accumulators::tag::mean mean;
    typedef boost::accumulators::tag::median median;

    boost::accumulators::accumulator_set<float, boost::accumulators::stats<mean, median>> color;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::fromROSMsg(request.cloud, cloud);

for (const auto & point : cloud.points)
        color(0.2989f * point.r + 0.5870f * point.g + 0.1140f * point.b);

    response.mean = boost::accumulators::mean(color);
    response.median = boost::accumulators::median(color);
    response.points = cloud.points.size();

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_color_analyzer");
    ros::NodeHandle nh("~");

    ros::ServiceServer analyzer_service = nh.advertiseService("analyze_cloud_color", analyzeCallback);

    ROS_INFO("Started [analyze_cloud_color] service.");

    ros::spin();

    return 0;
}

