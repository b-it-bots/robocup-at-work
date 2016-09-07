/*
 * virtual_laser_scanner_node.cpp
 *
 *  Created on: Mar 24, 2011
 *      Author: Frederik Hegger
 *
 */

/*
 * Converts point cloud into laser scan in the following way:
 * laser scan has predefined angle range;
 * laser scan has predefined range_min and range_max;
 * points which are outside the reach of the scan are skipped;
 * points with the z value (height) outside predefined region are omitted,
 *     (rationale: virtual_laser_scanner is used in mcr_people_tracking;
 *      only standing objects' points should contribute to the scan,
 *      but only those which are not too high to belong to a person)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <dynamic_reconfigure/server.h>

#include <mcr_virtual_laser_scanner/VirtualLaserScannerConfig.h>

using namespace std;

ros::Publisher pub_virtual_scan;
tf::TransformListener *transform_listener;
ros::Subscriber sub_pointcloud;

double min_height = 0.0;
double max_height = 0.0;
string target_frame = "";
double scan_frequency = 0.0;

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    sensor_msgs::PointCloud2 cloud2_transformed;
    pcl::PointCloud < pcl::PointXYZ > cloud_pcl;

    try
    {
        ROS_DEBUG("waitForTransform: target %s source: %s", target_frame.c_str(), cloud_msg->header.frame_id.c_str());
        transform_listener->waitForTransform(target_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame, *cloud_msg, cloud2_transformed, *transform_listener);

        if (cloud2_transformed.width <= 0 || cloud2_transformed.height <= 0)
        {
            ROS_WARN("Skip point cloud, because it is empty");
            return;
        }

        pcl::fromROSMsg(cloud2_transformed, cloud_pcl);
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN_STREAM("could not transform pointcloud: " << ex.what());
        return;
    }

    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());

    //Copy Header
    pcl_conversions::fromPCL(cloud_pcl.header, output->header);

    output->angle_min = -M_PI / 2;
    output->angle_max = M_PI / 2;
    output->angle_increment = M_PI / 180.0 * 0.5;       // angular resolution of 0.5 degree
    output->time_increment = 0.0;
    output->scan_time = 1.0 / scan_frequency;
    output->range_min = 0.45;
    output->range_max = 10.0;

    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, output->range_max + 1.0);

    for (unsigned int i = 0; i < cloud_pcl.points.size(); ++i)
    {
        const float x = cloud_pcl.points[i].x;
        const float y = cloud_pcl.points[i].y;
        const float z = cloud_pcl.points[i].z;

        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
            continue;

        if (z > max_height || z < min_height)
            continue;

        double angle = atan(y / x);
        if (angle < output->angle_min || angle > output->angle_max)
            continue;

        int index = (angle - output->angle_min) / output->angle_increment;

        double range = sqrt((x * x) + (y * y));

        if (range < output->ranges[index])
            output->ranges[index] = range;
    }

    pub_virtual_scan.publish(output);
}


void eventCallback(const std_msgs::String::ConstPtr& msg)
{
    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("Received event: " << msg->data);

    if (msg->data == "e_start")
        sub_pointcloud = nh.subscribe < sensor_msgs::PointCloud2 > ("pointcloud_xyz", 1, pointcloudCallback);
    else if (msg->data == "e_stop")
        sub_pointcloud.shutdown();
    else
        ROS_ERROR_STREAM("Event not supported: " << msg->data);
}


void dynamic_reconfig_callback(mcr_virtual_laser_scanner::VirtualLaserScannerConfig &config, uint32_t level)
{
    target_frame = config.target_frame;
    min_height = config.min_height;
    max_height = config.max_height;
    scan_frequency = config.scan_frequency;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "virtual_laser_scanner");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server < mcr_virtual_laser_scanner::VirtualLaserScannerConfig > dynamic_reconfig_server;
    dynamic_reconfig_server.setCallback(boost::bind(&dynamic_reconfig_callback, _1, _2));

    ros::Subscriber sub_event = nh.subscribe < std_msgs::String > ("event_in", 1, eventCallback);
    pub_virtual_scan = nh.advertise < sensor_msgs::LaserScan > ("scan", 1);

    transform_listener = new tf::TransformListener();

    ros::Rate loop_rate(scan_frequency);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
