/*
 * means.hpp
 *
 *  Created on: 14.04.2011
 *      Author: Frederik Hegger
 */

#ifndef MEANS_HPP_
#define MEANS_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class Means
{
public:
    template <typename PointT>
    static void determineArithmeticMean3D(const pcl::PointCloud<PointT> &pcl_cloud_input, double &mean_x, double &mean_y, double &mean_z)
    {
        mean_x = mean_y = mean_z = 0.0;

        unsigned int i;
        for (i = 0; i < pcl_cloud_input.size(); ++i)
        {
            mean_x += pcl_cloud_input.points[i].x;
            mean_y += pcl_cloud_input.points[i].y;
            mean_z += pcl_cloud_input.points[i].z;
        }

        mean_x /= i;
        mean_y /= i;
        mean_z /= i;
    }
};

#endif /* MEANS_HPP_ */
