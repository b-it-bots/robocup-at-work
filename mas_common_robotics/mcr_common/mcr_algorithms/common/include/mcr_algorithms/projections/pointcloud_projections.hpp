/*
 * pointcloud_projections.hpp
 *
 *  Created on: 14.04.2011
 *      Author: Frederik Hegger
 */

#ifndef POINTCLOUD_PROJECTIONS_HPP_
#define POINTCLOUD_PROJECTIONS_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/LaserScan.h>

class PointCloudProjections
{
public:
    /*
     * 2.5D to projection of pointcloud to a laser scan
     */
    template <typename PointT>
    static void projection2_5D(pcl::PointCloud<PointT> &pcl_cloud_input, pcl::PointCloud<PointT> &pcl_cloud_output, const std::string axis)
    {
        pcl_cloud_output.points.resize(pcl_cloud_input.width * pcl_cloud_input.height);

        for (unsigned int i = 0; i < pcl_cloud_input.points.size(); ++i)
        {
            if (axis == "x")
            {
                pcl_cloud_output.points[i].x = 0.0;
                pcl_cloud_output.points[i].y = pcl_cloud_input.points[i].y;
                pcl_cloud_output.points[i].z = pcl_cloud_input.points[i].z;
            }

            else if (axis == "y")
            {
                pcl_cloud_output.points[i].x = pcl_cloud_input.points[i].x;
                pcl_cloud_output.points[i].y = 0.0;
                pcl_cloud_output.points[i].z = pcl_cloud_input.points[i].z;

            }

            else if (axis == "z")
            {
                pcl_cloud_output.points[i].x = pcl_cloud_input.points[i].x;
                pcl_cloud_output.points[i].y = pcl_cloud_input.points[i].y;
                pcl_cloud_output.points[i].z = 0.0;
            }
        }

        pcl_cloud_output.header = pcl_cloud_input.header;
        pcl_cloud_output.height = 1;
        pcl_cloud_output.width  = pcl_cloud_input.width;
    }

    template <typename PointT>
    static void pointCloudToLaserScan(pcl::PointCloud<PointT> &pcl_cloud_input, sensor_msgs::LaserScan &scan_output, const double min_height_meter, const double max_height_meter, const std::string frame_id)
    {
        //Copy Header
        scan_output.header = pcl_cloud_input.header;
        scan_output.header.frame_id = frame_id;
        scan_output.angle_min = -M_PI / 2;
        scan_output.angle_max = M_PI / 2;
        scan_output.angle_increment = M_PI / 180.0 * 0.25;
        scan_output.time_increment = 0.0;
        scan_output.scan_time = 1.0 / 30.0;
        scan_output.range_min = 0.45;
        scan_output.range_max = 10.0;

        uint32_t ranges_size = std::ceil((scan_output.angle_max - scan_output.angle_min) / scan_output.angle_increment);
        scan_output.ranges.assign(ranges_size, scan_output.range_max + 1.0);

        for (unsigned int i = 0; i < pcl_cloud_input.size(); ++i)
        {
            const float x = pcl_cloud_input.points[i].x;
            const float y = pcl_cloud_input.points[i].y;
            const float z = pcl_cloud_input.points[i].z;
            double angle = atan(y / x);

            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
                continue;
            if (z < min_height_meter || z > max_height_meter)
                continue;
            if (angle < scan_output.angle_min || angle > scan_output.angle_max)
                continue;

            int index = (angle - scan_output.angle_min) / scan_output.angle_increment;

            double range = sqrt((x * x) + (y * y));
            if (range < scan_output.ranges[index])
                scan_output.ranges[index] = range;
        }
    }
};

#endif /* POINTCLOUD_PROJECTIONS_HPP_ */
