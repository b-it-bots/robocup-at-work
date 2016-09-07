/*
 * pcl_wrapper.hpp
 *
 *  Created on: 09.12.2010
 *      Author: Frederik Hegger
 */

#ifndef PCLWRAPPER_HPP_
#define PCLWRAPPER_HPP_

#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

using namespace pcl;
using namespace std;

template <typename PointT>
class PCLWrapper
{
    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
public:
    static void passThroughFilter(PointCloudPtr cloudPCLInput, PointCloudPtr cloudPCLOutput, string strAxis, double dMinRange, double dMaxRange)
    {
        PassThrough<PointT> passThroughFilter;
        passThroughFilter.setInputCloud(cloudPCLInput);
        passThroughFilter.setFilterFieldName(strAxis);
        passThroughFilter.setFilterLimits(dMinRange, dMaxRange);
        passThroughFilter.filter(*cloudPCLOutput);
    }

    static void downsampling(PointCloudPtr cloudPCLInput, PointCloudPtr cloudPCLOutput, double dLeafSize)
    {
        VoxelGrid<PointT> downsampler;
        downsampler.setInputCloud(cloudPCLInput);
        downsampler.setLeafSize(dLeafSize, dLeafSize, dLeafSize);
        downsampler.filter(*cloudPCLOutput);
    }

    static void clustering(PointCloudPtr cloudPCLInput, vector<PointIndices> &vecSegmentIndices, double dClusterTolerance, unsigned int unMinClusterSize, unsigned int unMaxClusterSize)
    {
        vecSegmentIndices.clear();

        typedef pcl::search::KdTree<PointT> KdTree;
        typedef typename KdTree::Ptr KdTreePtr;

        EuclideanClusterExtraction<PointT> cluster;
        KdTreePtr cluster_tree(new pcl::search::KdTree<PointT>);
        cluster_tree->setInputCloud(cloudPCLInput);

        cluster.setInputCloud(cloudPCLInput);
        cluster.setClusterTolerance(dClusterTolerance);
        cluster.setMinClusterSize(unMinClusterSize);
        cluster.setMaxClusterSize(unMaxClusterSize);
        cluster.setSearchMethod(cluster_tree);
        cluster.extract(vecSegmentIndices);
    }

    static void computeNormals(PointCloudPtr pcl_cloud_input, pcl::PointCloud<PointNormal>::Ptr pcl_cloud_normals, const double search_radius)
    {
        typedef pcl::search::KdTree<PointT> KdTree;
        typedef typename KdTree::Ptr KdTreePtr;

        // Create a KD-Tree
        KdTreePtr tree = boost::make_shared<pcl::search::KdTree<PointT> >();
        MovingLeastSquares<PointT, PointNormal> mls;

        mls.setComputeNormals(true);
        mls.setInputCloud(pcl_cloud_input);
        //mls.setPolynomialFit(true);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(search_radius);
        mls.process(*pcl_cloud_normals);
    }

    static void extractConvexHull(PointCloudPtr pcl_cloud_input, PointCloudPtr pcl_output_convex_hull)
    {
        pcl::ConvexHull<PointT> convex_hull;

        convex_hull.setInputCloud(pcl_cloud_input);
        convex_hull.reconstruct(*pcl_output_convex_hull);
    }

    static void get3DPointsWithinHull(PointCloudPtr pcl_full_cloud, PointCloudPtr pcl_2d_hull, const double min_height, const double max_height, PointCloudPtr pcl_output_3d_segment)
    {
        pcl::PointIndices object_indices;
        ExtractPolygonalPrismData<PointT> hull_limiter;

        hull_limiter.setInputCloud(pcl_full_cloud);
        hull_limiter.setInputPlanarHull(pcl_2d_hull);
        hull_limiter.setHeightLimits(min_height, max_height);
        hull_limiter.segment(object_indices);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(pcl_full_cloud);
        extract.setIndices(boost::make_shared<pcl::PointIndices> (object_indices));
        extract.setNegative(false);
        extract.filter(*pcl_output_3d_segment);
    }
};

#endif /* PCLWRAPPER_HPP_ */
