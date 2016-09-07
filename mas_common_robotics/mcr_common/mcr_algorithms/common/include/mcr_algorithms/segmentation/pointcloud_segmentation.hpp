/*
 * pointcloud_segmentation.hpp
 *
 *  Created on: 01.03.2011
 *      Author: Frederik Hegger
 */

#ifndef POINTCLOUD_SEGMENTATION_H_
#define POINTCLOUD_SEGMENTATION_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/StdVector>

#include "mcr_algorithms/wrapper/pcl_wrapper.hpp"
#include "mcr_algorithms/statistics/means.hpp"

using namespace pcl;

template <typename PointT>
class PointCloudSegmentation
{
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudSegmentation(const double roi_min_height, const double roi_max_height, const double slice_height, const double cluster_tolerance,
                           const double min_cluster_size, const double max_cluster_size)
    {
        this->roi_min_height_ = roi_min_height;
        this->roi_max_height_ = roi_max_height;
        this->slices_height_ = slice_height;
        this->cluster_tolerance_ = cluster_tolerance;
        this->cluster_min_size_ = min_cluster_size;
        this->cluster_max_size_ = max_cluster_size;
    }

    void getSegments(PointCloudPtr pcl_input_cloud, std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > &pcl_cloud_segments)
    {
        vector<PointIndices> cluster_indices;
        PointCloudPtr pcl_cloud_slice(new pcl::PointCloud<pcl::PointNormal>);
        PointCloud pcl_cloud_segment;

        pcl_cloud_segments.clear();

        unsigned int num_of_slices = (unsigned int)((this->roi_max_height_ - this->roi_min_height_) / this->slices_height_);

        for (unsigned int i = 0; i < num_of_slices; ++i)
        {
            // slice point cloud according to predefined slice height
            double min_height =  this->roi_min_height_ + (i * this->slices_height_);    // MAYBE + small additional height to avoid overlapping !!!!!
            double max_height =  min_height + this->slices_height_;

            PCLWrapper<PointT>::passThroughFilter(pcl_input_cloud, pcl_cloud_slice, "z", min_height, max_height);

            if (pcl_cloud_slice->points.empty())
                continue;

            //cluster slice to segments
            PCLWrapper<PointT>::clustering(pcl_cloud_slice, cluster_indices, this->cluster_tolerance_, this->cluster_min_size_, this->cluster_max_size_);

            for (unsigned int l = 0; l < cluster_indices.size(); ++l)
            {
                pcl::copyPointCloud(*pcl_cloud_slice, cluster_indices[l], pcl_cloud_segment);

                if (pcl_cloud_segment.points.size() <= this->cluster_min_size_)
                    continue;

                pcl_cloud_segments.push_back(pcl_cloud_segment);
            }
        }
    }

private:
    //Parameters which are read from the parameter server and set by the launch file
    double slices_height_;
    double roi_min_height_;
    double roi_max_height_;
    double cluster_tolerance_;
    double cluster_min_size_;
    double cluster_max_size_;
};

#endif /* POINTCLOUD_SEGMENTATION_H_ */
