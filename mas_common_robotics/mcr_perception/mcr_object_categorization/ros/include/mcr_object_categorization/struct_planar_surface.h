/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef StructPlanarSurface_H
#define StructPlanarSurface_H

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

struct StructPlanarSurface
{
    int id;
    pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud;
    pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree;
    pcl::PointCloud<pcl::PointXYZRGBNormal> convexHull;
    pcl::PointXYZRGBNormal centroid;
    float plane_height;  // computed by avg z of hull(planar)
    float area;  // computed by hull(planar) so x,y used
    std::vector<StructPlanarSurface*> upperPlanarSurfaces;
    float ROI_height;  //height in which an object can have, due to e.g. shelf(max_object_height == upper planar surface)
    bool limited_ROI_height;  // the ROI height is possibly limited due to upper surface which is possible another level of a shelf
    // otherwise ROI_height is not limited since no potential surface is above or this plane can not consists of objects below the upper plane.
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGBNormal> > > clusteredObjects;
    std::vector<pcl::PointXYZ> clusteredObjectsCentroids;
};

#endif
