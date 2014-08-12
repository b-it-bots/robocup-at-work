#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <mcr_perception_msgs/ClusterTabletopCloud.h>

#include "mcr_scene_segmentation/clustered_point_cloud_visualizer.h"
#include "mcr_scene_segmentation/aliases.h"
#include "mcr_scene_segmentation/impl/helpers.hpp"

using namespace mcr::visualization;

/** This node provides a service to break a pointcloud into clusters.
  *
  * Internally the node uses PCL euclidean clustering algorithm, which is
  * parametrized by the min and max size (number of points) of a cluster,
  * and minimum euclidean distance between clusters.
  *
  * The "Tabletop" part of the name stands for the fact that additional
  * knowledge about the plane that supports the clusters is used to improve
  * the results. For now the detected clusters are filtered to remove those
  * that are too short (and are likely to be some garbage remaining from the
  * table plane). Additionaly a radius outlier removal algorithm is applied
  * before clustering to remove single garbage points.
  *
  * Provides services:
  *   1) "cluster_tabletop_cloud"
  *
  * Publishes:
  *   1) "tabletop_clusters"
  *      The detected clusters are published for the visualization purposes via
  *      this topic. All the points are colored according to the cluster where
  *      they belong and are merged into a single cloud.
  */
class TabletopCloudClustererNode
{

public:

  TabletopCloudClustererNode()
  : cluster_visualizer_("tabletop_clusters")
  {
    ros::NodeHandle nh("~");
    cluster_server_ = nh.advertiseService("cluster_tabletop_cloud", &TabletopCloudClustererNode::clusterCallback, this);
    ROS_INFO("Service [cluster_tabletop_cloud] started.");
  }

private:

  bool clusterCallback(mcr_perception_msgs::ClusterTabletopCloud::Request& request, mcr_perception_msgs::ClusterTabletopCloud::Response& response)
  {
    ROS_INFO("Received [cluster_tabletop_cloud] request.");
    updateConfiguration();

    // Convert request fields into ROS datatypes
    PointCloud::Ptr cloud(new PointCloud);
    PlanarPolygon polygon;
    pcl::fromROSMsg(request.cloud, *cloud);
    convertPlanarPolygon(request.polygon, polygon);

    // Step 1: radius outlier removal
    ror_.setInputCloud(cloud);
    ror_.filter(*cloud);

    // Step 2: euclidean clustering
    std::vector<pcl::PointIndices> clusters_indices;
    ece_.setInputCloud(cloud);
    ece_.extract(clusters_indices);

    std::vector<PointCloud::Ptr> clusters;
    size_t rejected_count = 0;
    for (const pcl::PointIndices& cluster_indices : clusters_indices)
    {
      PointCloud::Ptr cluster(new PointCloud);
      pcl::copyPointCloud(*cloud, cluster_indices, *cluster);
      if (getClusterCentroidHeight(*cluster, polygon) < object_min_height_)
      {
        rejected_count++;
        continue;
      }
      sensor_msgs::PointCloud2 ros_cluster;
      pcl::toROSMsg(*cluster, ros_cluster);
      response.clusters.push_back(ros_cluster);
      clusters.push_back(cluster);
    }
    ROS_INFO("Found %zu clusters, but rejected %zu due to low height.", clusters_indices.size(), rejected_count);

    cluster_visualizer_.publish<PointT>(clusters, request.cloud.header.frame_id);

    return true;
  }

private:

  static double getClusterCentroidHeight(const PointCloud& cluster, const PlanarPolygon& polygon)
  {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cluster, centroid);
    centroid[3] = 1;
    return centroid.dot(polygon.getCoefficients());
  }

  void updateConfiguration()
  {
    ros::NodeHandle pn("~");

    // Euclidean cluster extraction settings
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    pn.param("cluster_tolerance", cluster_tolerance, 0.01);
    pn.param("min_cluster_size", min_cluster_size, 25);
    pn.param("max_cluster_size", max_cluster_size, 5000);
    ece_.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
    ece_.setClusterTolerance(cluster_tolerance);
    ece_.setMinClusterSize(min_cluster_size);
    ece_.setMaxClusterSize(max_cluster_size);

    // Radius outlier removal settings
    double radius;
    int neighbors;
    pn.param("outlier_radius", radius, 0.01);
    pn.param("outlier_neighbors", neighbors, 20);
    ror_.setRadiusSearch(radius);
    ror_.setMinNeighborsInRadius(neighbors);

    // Other settings
    pn.param("object_min_height", object_min_height_, 0.011);
  }

  ros::ServiceServer cluster_server_;

  pcl::RadiusOutlierRemoval<PointT> ror_;
  pcl::EuclideanClusterExtraction<PointT> ece_;

  double object_min_height_;

  ClusteredPointCloudVisualizer cluster_visualizer_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletop_cloud_clusterer");
  TabletopCloudClustererNode tocn;
  ros::spin();
  return 0;
}

