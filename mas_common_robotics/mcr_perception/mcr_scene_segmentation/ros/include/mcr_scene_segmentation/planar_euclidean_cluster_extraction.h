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
 * $Id: extract_clusters.hpp 32052 2010-08-27 02:19:30Z rusu $
 *
 * *****************************************************************
 *
 * Modified 14.06.2016 (Sergey, Santosh)
 *    - added constraints based on distance and height from plane
 *    - publishes PointCloud2List instead of single pointclouds
 *    - added cluster visualizer
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_PLANAR_EUCLIDEAN_CLUSTER_EXTRACTION_H
#define MCR_SCENE_SEGMENTATION_PLANAR_EUCLIDEAN_CLUSTER_EXTRACTION_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/segmentation/extract_clusters.h>
#include <dynamic_reconfigure/server.h>
#include <mcr_scene_segmentation/PlanarEuclideanClusterExtractionConfig.h>
#include "mcr_scene_segmentation/clustered_point_cloud_visualizer.h"
#include "mcr_scene_segmentation/aliases.h"
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include <mcr_perception_msgs/PlanarPolygon.h>
#include <geometry_msgs/PolygonStamped.h>

namespace mcr_scene_segmentation
{

namespace sync_policies = message_filters::sync_policies;

class PlanarEuclideanClusterExtraction : public pcl_ros::PCLNodelet
{
    public:
        PlanarEuclideanClusterExtraction() {}

    protected:
        void onInit();
        void config_callback(PlanarEuclideanClusterExtractionConfig &config, uint32_t level);
        void input_indices_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud,
                                    const pcl_ros::PCLNodelet::PointIndicesConstPtr &indices,
                                    const geometry_msgs::PolygonStamped::ConstPtr &hull,
                                    const pcl_msgs::ModelCoefficients::ConstPtr &plane_model);

        double getClusterCentroidHeight(const pcl::PointCloud<PointT>& cluster, const PlanarPolygon& polygon);
        double getClusterCentroidDistanceToPolygon(const pcl::PointCloud<PointT>& cluster,
                                                   const PlanarPolygon& polygon);

    protected:
        /** \brief Pointer to a dynamic reconfigure service. */
        boost::shared_ptr<dynamic_reconfigure::Server<PlanarEuclideanClusterExtractionConfig> > srv_;

        ros::Publisher pub_cluster_list_;

        /** \brief The PCL implementation used. */
        pcl::EuclideanClusterExtraction<PointT> impl_;

        message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_hull_filter_;
        message_filters::Subscriber<pcl_msgs::ModelCoefficients> sub_plane_model_;
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud2_filter_;

        /** \brief Synchronized input, and indices.*/
        boost::shared_ptr<message_filters::Synchronizer
                         <sync_policies::ExactTime
                         <sensor_msgs::PointCloud2, pcl_ros::PCLNodelet::PointIndices,
                          geometry_msgs::PolygonStamped, pcl_msgs::ModelCoefficients> > > sync_input_indices_e_;
        boost::shared_ptr<message_filters::Synchronizer
                         <sync_policies::ApproximateTime
                         <sensor_msgs::PointCloud2, pcl_ros::PCLNodelet::PointIndices,
                          geometry_msgs::PolygonStamped, pcl_msgs::ModelCoefficients> > > sync_input_indices_a_;

        double cluster_min_height_;
        double cluster_min_distance_to_polygon_;

        boost::shared_ptr<mcr::visualization::ClusteredPointCloudVisualizer> cluster_visualizer_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

PLUGINLIB_DECLARE_CLASS(mcr_scene_segmentation, PlanarEuclideanClusterExtraction,
                        mcr_scene_segmentation::PlanarEuclideanClusterExtraction, nodelet::Nodelet);
}  // namespace mcr_scene_segmentation
#endif  // MCR_SCENE_SEGMENTATION_PLANAR_EUCLIDEAN_CLUSTER_EXTRACTION_H */
