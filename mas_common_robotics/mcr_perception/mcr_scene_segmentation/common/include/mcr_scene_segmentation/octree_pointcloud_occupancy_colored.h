/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H
#define MCR_SCENE_SEGMENTATION_OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_base.h>

using pcl::octree::OctreePointCloud;
using pcl::octree::OctreeContainerPointIndex;
using pcl::octree::OctreeContainerEmpty;
using pcl::octree::OctreeBase;
using pcl::octree::OctreeKey;


template<typename PointT = pcl::PointXYZRGB, typename LeafContainerT = OctreeContainerPointIndex,
         typename BranchContainerT = OctreeContainerEmpty>
class OctreePointCloudOccupancyColored : public OctreePointCloud<PointT, LeafContainerT, BranchContainerT,
                                                                 OctreeBase<LeafContainerT, BranchContainerT> >
{
public:
    explicit OctreePointCloudOccupancyColored(const double resolution)
        : OctreePointCloud<PointT, LeafContainerT, BranchContainerT,
                           OctreeBase<LeafContainerT, BranchContainerT> >(resolution)
    {
    }

    virtual ~OctreePointCloudOccupancyColored() { }

    void setOccupiedVoxelAtPoint(const PointT& point)
    {
        OctreeKey key;
        adoptBoundingBoxToPoint(point);
        genOctreeKeyforPoint(point, key);
        OctreeContainerPointIndex* leaf = this->createLeaf(key);
        leaf->addPointIndex(point.rgba);
    }

    void setOccupiedVoxelsAtPointsFromCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
    {
        for (size_t i = 0; i < cloud->points.size(); i++)
            if (isFinite(cloud->points[i]))
                this->setOccupiedVoxelAtPoint(cloud->points[i]);
    }

    uint32_t getVoxelColorAtPoint(const PointT& point) const
    {
        uint32_t color = 0;
        OctreeContainerPointIndex* leaf = this->findLeafAtPoint(point);
        if (leaf)
            color = leaf->getPointIndex();
        return color;
    }

    void getOccupiedVoxelCentersWithColor(typename pcl::PointCloud<PointT>::VectorType& points)
    {
        this->getOccupiedVoxelCenters(points);
        for (size_t i = 0; i < points.size(); i++)
        {
            uint32_t color = this->getVoxelColorAtPoint(points[i]);
            points[i].rgba = color;
        }
    }
};
#endif  // MCR_SCENE_SEGMENTATION_OCTREE_POINTCLOUD_OCCUPANCY_COLORED_H

