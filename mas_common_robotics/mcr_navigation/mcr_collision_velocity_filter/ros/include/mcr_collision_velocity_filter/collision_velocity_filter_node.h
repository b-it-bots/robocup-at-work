/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Frederik Hegger
 */

#ifndef MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_NODE_H
#define MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_NODE_H

#include <exception>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <pcl/io/pcd_io.h>

#include <mcr_collision_velocity_filter/collision_velocity_filter.h>

using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;

class CollisionVelocityFilterNode
{
public:
    CollisionVelocityFilterNode();
    ~CollisionVelocityFilterNode();

    void update();

private:
    void oneLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void twoSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
        const sensor_msgs::LaserScan::ConstPtr &scan_2);
    void threeSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
        const sensor_msgs::LaserScan::ConstPtr &scan_2, const sensor_msgs::LaserScan::ConstPtr &scan_3);

    void accumulateLaserScansToPointCloud(const std::vector<sensor_msgs::LaserScan> &scans);

    std::vector<std::string> readScanTopicsFromParameterServer();

    void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg);
    void twistCommadCallback(const geometry_msgs::Twist::ConstPtr &msg);

    bool getFootprintFromParameterServer(const std::string &parameter_name);
    bool getFootprintFromTopic(const std::string &topic_name);

    sensor_msgs::PointCloud2 getCloudFromLaserScan(const sensor_msgs::LaserScan &scan);

    ros::Subscriber sub_single_scan_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_1;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_2;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan_3;
    boost::shared_ptr<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
        sensor_msgs::LaserScan> > > two_synced_laser_scans_;
    boost::shared_ptr<Synchronizer<ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan,
        sensor_msgs::LaserScan> > > three_synced_laser_scans_;

    ros::Subscriber sub_footprint_;
    ros::Subscriber sub_twist_;

    // Publisher
    ros::Publisher pub_safe_twist_;
    ros::Publisher pub_event_;

    // Debug publisher
    ros::Publisher pub_real_footprint_;
    ros::Publisher pub_soft_footprint_;
    ros::Publisher pub_hard_footprint_;

    ros::Publisher pub_front_right_footprint_;
    ros::Publisher pub_front_left_footprint_;
    ros::Publisher pub_rear_right_footprint_;
    ros::Publisher pub_rear_left_footprint_;

    CollisionVelocityFilter *collision_velocity_filter_;

    laser_geometry::LaserProjection laser_projector_;
    tf::TransformListener transform_listener_;

    geometry_msgs::PolygonStamped footprint_msg_;
    bool footprint_msg_received;

    pcl::PointCloud<pcl::PointXYZ> laser_scans_as_pcl_cloud;
    bool laser_scans_as_pcl_cloud_received;

    geometry_msgs::Twist desired_twist_msg_;
    bool desired_twist_msg_received_;

    geometry_msgs::Twist zero_velocities_;

    sensor_msgs::PointCloud2 laser_scans_as_cloud_;
    geometry_msgs::Twist safe_twist_;

    std::string target_frame_;

    bool debug_mode_;
    std_msgs::String event_out_;
};

#endif  // MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_NODE_H
