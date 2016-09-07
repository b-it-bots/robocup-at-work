/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/common.h>

#include <mcr_perception_msgs/AccumulateTabletopCloud.h>

#include "mcr_scene_segmentation/aliases.h"
#include "mcr_scene_segmentation/cloud_accumulation.h"
#include "mcr_scene_segmentation/impl/helpers.hpp"

/** This node provides a service to accumulate parts of pointclouds that are
  * above some given planar polygon.
  *
  * In each pointcloud in the stream the algorithm extracts those points that
  * are above the given planar polygon and merges them into an occupancy octree.
  * The resulting cloud is then output to the user.
  *
  * Provides services:
  *   1) "accumulate_tabletop_cloud"
  *
  * Publishes:
  *   1) "accumulated_cloud"
  *      The cloud output as a response to the user of the service is also
  *      forwarded to this topic for the visualization purposes.
  *
  * Subscribes:
  *   1) "/camera/depth_registered/points"
  *      The subscription is activated on demand, i.e. when the service is idle
  *      the node unsubscribes to avoid bandwidth consumption.
  */
class TabletopCloudAccumulatorNode
{
public:
    TabletopCloudAccumulatorNode()
    {
        ros::NodeHandle nh("~");
        accumulate_service_ = nh.advertiseService("accumulate_tabletop_cloud",
                                                  &TabletopCloudAccumulatorNode::accumulateCallback, this);
        accumulated_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 1);
        tf_listener_ = new tf::TransformListener();
        ROS_INFO("Service [accumulate_tabletop_cloud] started.");
    }

    ~TabletopCloudAccumulatorNode()
    {
        delete tf_listener_;
    }

private:
    bool accumulateCallback(mcr_perception_msgs::AccumulateTabletopCloud::Request& request,
                            mcr_perception_msgs::AccumulateTabletopCloud::Response& response)
    {
        ROS_INFO("Received [accumulate_tabletop_cloud] request.");
        updateConfiguration();

        ros::NodeHandle nh("~");
        // set viewpoint of camera so that points are accumulated on the side facing the camera
        try
        {
            std::string camera_frame;
            nh.param<std::string>("camera_frame", camera_frame, "/arm_cam3d_rgb_optical_frame");
            tf::StampedTransform transform;

            tf_listener_->waitForTransform(request.polygon.header.frame_id, camera_frame,
                                           request.polygon.header.stamp, ros::Duration(1.0));
            tf_listener_->lookupTransform(request.polygon.header.frame_id, camera_frame,
                                          request.polygon.header.stamp, transform);
            eppd_.setViewPoint(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        }
        catch (std::exception &e)
        {
            ROS_ERROR_STREAM("Could not lookup transform to camera frame: " << e.what());
            return false;
        }

        PointCloud::Ptr polygon_cloud(new PointCloud);
        PlanarPolygon polygon;
        convertPlanarPolygon(request.polygon, polygon);
        polygon_cloud->points = polygon.getContour();
        eppd_.setInputPlanarHull(polygon_cloud);
        ca_ = CloudAccumulation::UPtr(new CloudAccumulation(octree_resolution_));

        ros::Subscriber subscriber = nh.subscribe("input_pointcloud", 1,
                                                  &TabletopCloudAccumulatorNode::cloudCallback, this);

        // Wait some time while data is being accumulated.
        ros::Time start = ros::Time::now();
        while (ca_->getCloudCount() < accumulate_clouds_ &&
               ros::Time::now() < start + ros::Duration(accumulation_timeout_) && ros::ok())
        {
            ros::spinOnce();
        }
        subscriber.shutdown();

        ROS_INFO("Accumulated %i clouds in %.2f seconds.", ca_->getCloudCount(), (ros::Time::now() - start).toSec());
        // Pack the response
        PointCloud cloud;
        cloud.header.frame_id = frame_id_;
        ca_->getAccumulatedCloud(cloud);

        pcl::PCLPointCloud2 pc2;
        pcl::toPCLPointCloud2(cloud, pc2);
        pcl_conversions::fromPCL(pc2, response.cloud);
        response.cloud.header.stamp = ros::Time::now();

        // Forward to the "accumulated_cloud" topic (if there are subscribers)
        if (accumulated_cloud_publisher_.getNumSubscribers())
            accumulated_cloud_publisher_.publish(response.cloud);

        return ca_->getCloudCount() != 0;
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
    {
        PointCloud::Ptr cloud(new PointCloud);
        pcl::PCLPointCloud2 pc2;
        pcl_conversions::toPCL(*ros_cloud, pc2);
        pcl::fromPCLPointCloud2(pc2, *cloud);
        frame_id_ = ros_cloud->header.frame_id;

        pcl::PointIndices::Ptr tabletop_indices(new pcl::PointIndices);
        eppd_.setInputCloud(cloud);
        eppd_.segment(*tabletop_indices);

        if (tabletop_indices->indices.size() == 0)
        {
            ROS_WARN("There are no points above the provided polygon.");
            return;
        }

        PointCloud::Ptr tabletop_cloud(new PointCloud);
        pcl::copyPointCloud(*cloud, *tabletop_indices, *tabletop_cloud);
        ca_->addCloud(tabletop_cloud);
    }

    void updateConfiguration()
    {
        ros::NodeHandle pn("~");

        // Extract polygonal prism settings
        double min_height, max_height;
        pn.param("min_height", min_height, 0.01);
        pn.param("max_height", max_height, 0.20);
        eppd_.setHeightLimits(min_height, max_height);

        // Other settings
        pn.param("accumulation_timeout", accumulation_timeout_, 10);
        pn.param("accumulate_clouds", accumulate_clouds_, 1);
        pn.param("octree_resolution", octree_resolution_, 0.0025);
    }

    pcl::ExtractPolygonalPrismData<PointT> eppd_;
    CloudAccumulation::UPtr ca_;

    ros::ServiceServer accumulate_service_;
    ros::Publisher accumulated_cloud_publisher_;

    std::string frame_id_;
    int accumulation_timeout_;
    int accumulate_clouds_;
    double octree_resolution_;

    tf::TransformListener *tf_listener_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tabletop_cloud_accumulator");
    TabletopCloudAccumulatorNode tcan;
    ros::spin();
    return 0;
}
