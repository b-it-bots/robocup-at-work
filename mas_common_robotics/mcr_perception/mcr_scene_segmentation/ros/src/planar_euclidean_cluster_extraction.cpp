/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: extract_clusters.h 35361 2011-01-20 04:34:49Z rusu $
 *
 * *****************************************************************
 *
 * Modified 14.06.2016 (Sergey, Santosh)
 *    - added subscription to plane convex hull and plane model
 *    - added constraints based on distance and height from plane
 *    - publishes PointCloud2List instead of single pointclouds
 *    - added cluster visualizer
 */
#include <limits>
#include <vector>
#include <mcr_scene_segmentation/planar_euclidean_cluster_extraction.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <mcr_perception_msgs/PointCloud2List.h>


void mcr_scene_segmentation::PlanarEuclideanClusterExtraction::onInit()
{
    // Call the super onInit ()
    PCLNodelet::onInit();

    cluster_visualizer_ = boost::make_shared
                          <mcr::visualization::ClusteredPointCloudVisualizer>(pnh_, "tabletop_clusters");

    // ---[ Mandatory parameters
    double cluster_tolerance;
    if (!pnh_->getParam("cluster_tolerance", cluster_tolerance))
    {
        NODELET_ERROR("[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!",
                      getName().c_str());
        return;
    }

    pub_cluster_list_ = pnh_->advertise<mcr_perception_msgs::PointCloud2List>("output", max_queue_size_);

    // Enable the dynamic reconfigure service
    srv_ = boost::make_shared<dynamic_reconfigure::Server<PlanarEuclideanClusterExtractionConfig> > (*pnh_);
    dynamic_reconfigure::Server<PlanarEuclideanClusterExtractionConfig>::CallbackType f
                 = boost::bind(&PlanarEuclideanClusterExtraction::config_callback, this, _1, _2);
    srv_->setCallback(f);

    // Subscribe to the input using a filter
    sub_cloud2_filter_.subscribe(*pnh_, "input", max_queue_size_);
    sub_indices_filter_.subscribe(*pnh_, "indices", max_queue_size_);
    sub_hull_filter_.subscribe(*pnh_, "polygon", max_queue_size_);
    sub_plane_model_.subscribe(*pnh_, "model", max_queue_size_);

    if (approximate_sync_)
    {
        sync_input_indices_a_ = boost::make_shared<message_filters::Synchronizer
                                <message_filters::sync_policies::ApproximateTime
                                <sensor_msgs::PointCloud2, pcl_ros::PCLNodelet::PointIndices,
                                 geometry_msgs::PolygonStamped, pcl_msgs::ModelCoefficients> > > (max_queue_size_);
        sync_input_indices_a_->connectInput(sub_cloud2_filter_, sub_indices_filter_,
                                            sub_hull_filter_, sub_plane_model_);
        sync_input_indices_a_->registerCallback(boost::bind(
                               &mcr_scene_segmentation::PlanarEuclideanClusterExtraction::input_indices_callback,
                               this, _1, _2, _3, _4));
    }
    else
    {
        sync_input_indices_e_ = boost::make_shared<message_filters::Synchronizer
                                <message_filters::sync_policies::ExactTime
                                <sensor_msgs::PointCloud2, pcl_ros::PCLNodelet::PointIndices,
                                 geometry_msgs::PolygonStamped, pcl_msgs::ModelCoefficients> > > (max_queue_size_);
        sync_input_indices_e_->connectInput(sub_cloud2_filter_, sub_indices_filter_,
                                            sub_hull_filter_, sub_plane_model_);
        sync_input_indices_e_->registerCallback(boost::bind(
                               &mcr_scene_segmentation::PlanarEuclideanClusterExtraction::input_indices_callback,
                               this, _1, _2, _3, _4));
    }

    NODELET_DEBUG("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                   " - max_queue_size    : %d\n"
                   " - cluster_tolerance : %f\n",
                   getName().c_str(),
                   max_queue_size_,
                   cluster_tolerance);

    // Set given parameters here
    impl_.setClusterTolerance(cluster_tolerance);
}

void mcr_scene_segmentation::PlanarEuclideanClusterExtraction::config_callback(
        PlanarEuclideanClusterExtractionConfig &config, uint32_t level)
{
    if (impl_.getClusterTolerance() != config.cluster_tolerance)
    {
        impl_.setClusterTolerance(config.cluster_tolerance);
        NODELET_DEBUG("[%s::config_callback] Setting new clustering tolerance to: %f.",
                      getName().c_str(), config.cluster_tolerance);
    }
    if (impl_.getMinClusterSize() != config.cluster_min_size)
    {
        impl_.setMinClusterSize(config.cluster_min_size);
        NODELET_DEBUG("[%s::config_callback] Setting the minimum cluster size to: %d.",
                      getName().c_str(), config.cluster_min_size);
    }
    if (impl_.getMaxClusterSize() != config.cluster_max_size)
    {
        impl_.setMaxClusterSize(config.cluster_max_size);
        NODELET_DEBUG("[%s::config_callback] Setting the maximum cluster size to: %d.",
                      getName().c_str(), config.cluster_max_size);
    }
    cluster_min_height_ = config.cluster_min_height;
    NODELET_DEBUG("[%s::config_callback] Setting the minimum cluster height above polygon to: %f.",
                   getName().c_str(), config.cluster_min_height);
    cluster_min_distance_to_polygon_ = config.cluster_min_distance_to_polygon;
    NODELET_DEBUG("[%s::config_callback] Setting the minimum cluster distance to polygon to: %f.",
                   getName().c_str(), config.cluster_min_distance_to_polygon);
}

void mcr_scene_segmentation::PlanarEuclideanClusterExtraction::input_indices_callback(
        const sensor_msgs::PointCloud2::ConstPtr &cloud,
        const pcl_ros::PCLNodelet::PointIndicesConstPtr &indices,
        const geometry_msgs::PolygonStamped::ConstPtr &hull,
        const pcl_msgs::ModelCoefficients::ConstPtr &plane_model)
{
    // No subscribers, no work
    if (pub_cluster_list_.getNumSubscribers() <= 0 && cluster_visualizer_->getNumSubscribers() <= 0)
        return;

    // If cloud is given, check if it's valid
    if (!pcl_ros::PCLNodelet::isValid (cloud))
    {
        NODELET_ERROR("[%s::input_indices_callback] Invalid input!", getName().c_str());
        return;
    }
    // If indices are given, check if they are valid
    if (indices && !pcl_ros::PCLNodelet::isValid(indices))
    {
        NODELET_ERROR("[%s::input_indices_callback] Invalid indices!", getName().c_str());
        return;
    }

    // debug
    std_msgs::Header cloud_header = cloud->header;
    std_msgs::Header indices_header = indices->header;
    NODELET_DEBUG("[%s::input_indices_callback]\n"
                  "          - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                  "          - PointIndices with %zu values, stamp %f, and frame %s on topic %s received."
                  "          - PlanarPolygon with stamp %f, and frame %s on topic %s received.",
                  getName().c_str(),
                  cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud_header.stamp.toSec(),
                  cloud_header.frame_id.c_str(), pnh_->resolveName("input").c_str(),
                  indices->indices.size(), indices_header.stamp.toSec(), indices_header.frame_id.c_str(),
                  pnh_->resolveName("indices").c_str(), hull->header.stamp.toSec(),
                  hull->header.frame_id.c_str(), pnh_->resolveName("polygon").c_str());

    pcl_ros::PCLNodelet::IndicesPtr indices_ptr;
    if (indices)
        indices_ptr.reset (new std::vector<int> (indices->indices));

    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*cloud, *pcl_input_cloud);

    pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*pcl_input_cloud, *input_cloud);

    impl_.setInputCloud(input_cloud);
    impl_.setIndices(indices_ptr);

    std::vector<pcl::PointIndices> clusters;
    impl_.extract(clusters);

    // ** create mcr_perception_msgs::PlanarPolygon
    mcr_perception_msgs::PlanarPolygon mcr_polygon_msg;
    mcr_polygon_msg.header = hull->header;
    mcr_polygon_msg.contour = hull->polygon.points;

    for (int i = 0; i < 4; i++)
    {
        mcr_polygon_msg.coefficients[i] = plane_model->values[i];
    }

    PlanarPolygon pcl_polygon;
    convertPlanarPolygon(mcr_polygon_msg, pcl_polygon);
    // **

    mcr_perception_msgs::PointCloud2List cluster_clouds;
    std::vector<pcl::PointCloud<PointT>::Ptr> viz_clusters;

    size_t rejected_count = 0;
    size_t distance_rejected_count = 0;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters[i];
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*input_cloud, cluster_indices, *cluster);
        if (getClusterCentroidHeight(*cluster, pcl_polygon) < cluster_min_height_)
        {
            rejected_count++;
            continue;
        }
        if (getClusterCentroidDistanceToPolygon(*cluster, pcl_polygon) < cluster_min_distance_to_polygon_)
        {
            distance_rejected_count++;
            continue;
        }
        viz_clusters.push_back(cluster);
        sensor_msgs::PointCloud2 ros_cluster;
        pcl::PCLPointCloud2::Ptr pc2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*cluster, *pc2);
        pcl_conversions::fromPCL(*pc2, ros_cluster);
        ros_cluster.header.stamp = cloud->header.stamp;
        ros_cluster.header.frame_id = cloud->header.frame_id;
        cluster_clouds.pointclouds.push_back(ros_cluster);
    }
    cluster_clouds.header = cloud->header;
    if (pub_cluster_list_.getNumSubscribers() > 0)
    {
        pub_cluster_list_.publish(cluster_clouds);
    }
    if (cluster_visualizer_->getNumSubscribers() > 0)
    {
        cluster_visualizer_->publish<PointT>(viz_clusters, cloud->header.frame_id);
    }
}

double mcr_scene_segmentation::PlanarEuclideanClusterExtraction::getClusterCentroidHeight(
    const pcl::PointCloud<PointT>& cluster, const PlanarPolygon& polygon)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    centroid[3] = 1;
    return centroid.dot(polygon.getCoefficients());
}

/**
 * Calculates shortest distance from centroid of cluster to polygon
 * - assumes last point of polygon is same as first
 * - assumes polygon is on x-y plane
 */
double mcr_scene_segmentation::PlanarEuclideanClusterExtraction::getClusterCentroidDistanceToPolygon(
    const pcl::PointCloud<PointT>& cluster, const PlanarPolygon& polygon)
{
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    pcl::PointCloud<PointT>::VectorType contour = polygon.getContour();
    for (int i = 0; i < contour.size() - 1; i++)
    {
        PointT p1 = contour.at(i);
        PointT p2 = contour.at(i + 1);
        // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        double line_length = std::sqrt(std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0));
        double distance = std::abs(
                              ((p2.y - p1.y) * centroid[0]) -
                              ((p2.x - p1.x) * centroid[1]) +
                              (p2.x * p1.y) -
                              (p2.y * p1.x)) / line_length;

        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    return min_distance;
}

