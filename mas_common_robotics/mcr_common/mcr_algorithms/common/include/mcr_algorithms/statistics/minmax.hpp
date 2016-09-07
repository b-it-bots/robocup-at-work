/*
 * minmax.hpp
 *
 *  Created on: 14.04.2011
 *      Author: Frederik Hegger
 */

#ifndef MINMAX_HPP_
#define MINMAX_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class MinMax
{
public:
    template <typename PointT>
    static void determineMinMax3D(const pcl::PointCloud<PointT> &pcl_cloud_input, double &min_x, double &max_x, double &min_y, double &max_y, double &min_z, double &max_z)
    {
        min_x = min_y = min_z = DBL_MAX;
        max_x = max_y = max_z = -DBL_MAX;

        for (unsigned int i = 0; i < pcl_cloud_input.size(); ++i)
        {
            if (pcl_cloud_input.points[i].x < min_x)
                min_x = pcl_cloud_input.points[i].x;
            if (pcl_cloud_input.points[i].x > max_x)
                max_x = pcl_cloud_input.points[i].x;

            if (pcl_cloud_input.points[i].y < min_y)
                min_y = pcl_cloud_input.points[i].y;
            if (pcl_cloud_input.points[i].y > max_y)
                max_y = pcl_cloud_input.points[i].y;

            if (pcl_cloud_input.points[i].z < min_z)
                min_z = pcl_cloud_input.points[i].z;
            if (pcl_cloud_input.points[i].z > max_z)
                max_z = pcl_cloud_input.points[i].z;
        }
    }

    template <typename PointT>
    static void determineMinMax3D(const pcl::PointCloud<PointT> &pcl_cloud_input, PointT &min_x, PointT &max_x,
                                  PointT &min_y, PointT &max_y, PointT &min_z, PointT &max_z)
    {
        min_x.x = min_y.y = min_z.z = DBL_MAX;
        max_x.x = max_y.y = max_z.z = -DBL_MAX;     //TODO: Fix number because of trouble with FLT_MIN

        for (unsigned int i = 0; i < pcl_cloud_input.size(); ++i)
        {
            if (pcl_cloud_input.points[i].x < min_x.x)
            {
                min_x.x = pcl_cloud_input.points[i].x;
                min_x.y = pcl_cloud_input.points[i].y;
                min_x.z = pcl_cloud_input.points[i].z;
            }
            if (pcl_cloud_input.points[i].x > max_x.x)
            {
                max_x.x = pcl_cloud_input.points[i].x;
                max_x.y = pcl_cloud_input.points[i].y;
                max_x.z = pcl_cloud_input.points[i].z;
            }

            if (pcl_cloud_input.points[i].y < min_y.y)
            {
                min_y.x = pcl_cloud_input.points[i].x;
                min_y.y = pcl_cloud_input.points[i].y;
                min_y.z = pcl_cloud_input.points[i].z;
            }
            if (pcl_cloud_input.points[i].y > max_y.y)
            {
                max_y.x = pcl_cloud_input.points[i].x;
                max_y.y = pcl_cloud_input.points[i].y;
                max_y.z = pcl_cloud_input.points[i].z;
            }

            if (pcl_cloud_input.points[i].z < min_z.z)
            {
                min_z.x = pcl_cloud_input.points[i].x;
                min_z.y = pcl_cloud_input.points[i].y;
                min_z.z = pcl_cloud_input.points[i].z;
            }
            if (pcl_cloud_input.points[i].z > max_z.z)
            {
                max_z.x = pcl_cloud_input.points[i].x;
                max_z.y = pcl_cloud_input.points[i].y;
                max_z.z = pcl_cloud_input.points[i].z;
            }
        }
    }
};

#endif /* MINMAX_HPP_ */
