/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2014, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Frederik Hegger
 */

#ifndef MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_H
#define MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_H

#include <string>
#include <vector>
#include <limits>
#include <math.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

const float ZERO_DEGREES = 0.0f;
const float NINETY_DEGREES = 90.0f;
const float ONE_EIGHTY_DEGREES = 180.0f;

class CollisionVelocityFilter
{
public:
    CollisionVelocityFilter();
    virtual ~CollisionVelocityFilter();

    enum VelocityDirections {NO_MOTION, FRONT, RIGHT, LEFT, REAR, FRONT_LEFT, FRONT_RIGHT, REAR_RIGHT, REAR_LEFT};

    geometry_msgs::Twist calculateSafeBaseVelocities(const geometry_msgs::Twist &desired_twist,
    const pcl::PointCloud<pcl::PointXYZ> &scans_as_pointcloud);
    void updateRealFootprint(const geometry_msgs::PolygonStamped &footprint);

    void setSoftPaddingParameter(const double &front_rear, const double &left_right);
    void setHardPaddingParameter(const double &front_rear, const double &left_right);
    void setVelocitiesInSoftPadding(const double &linear_velocity, const double &angular_velocity);
    void setAngleTolerance(const double &angle_tolerance);
    void setMinimumLinearVelocity(const double &min_linear_velocity);

    geometry_msgs::PolygonStamped getRealFootprint();
    geometry_msgs::PolygonStamped getSoftPaddingFootprint();
    geometry_msgs::PolygonStamped getHardPaddingFootprint();

    geometry_msgs::PolygonStamped getFrontRightFootprint();
    geometry_msgs::PolygonStamped getFrontLeftFootprint();
    geometry_msgs::PolygonStamped getRearRightFootprint();
    geometry_msgs::PolygonStamped getRearLeftFootprint();

private:
    void calculateSoftAndHardFootprints(const geometry_msgs::Twist &desired_twist);
    void calculateFootprintQuadrants(const geometry_msgs::PolygonStamped &footprint);

    double calculateVelocityOrientation(const geometry_msgs::Twist &twist);
    CollisionVelocityFilter::VelocityDirections calculateVelocityDirection(double velocity_orientation);

    geometry_msgs::PolygonStamped calculateFootprintQuadrant(
        const geometry_msgs::PolygonStamped &footprint, double corner_x, double corner_y,
        double center_x, double center_y);
    std::vector<bool> quadrantsWithPoint(const pcl::PointCloud<pcl::PointXYZ> &pointcloud);

    geometry_msgs::PolygonStamped real_footprint_;
    geometry_msgs::PolygonStamped soft_padding_footprint_;
    geometry_msgs::PolygonStamped hard_padding_footprint_;

    // Quadrants of the hard_padding_footprint
    geometry_msgs::PolygonStamped front_right_footprint_;
    geometry_msgs::PolygonStamped front_left_footprint_;
    geometry_msgs::PolygonStamped rear_right_footprint_;
    geometry_msgs::PolygonStamped rear_left_footprint_;

    geometry_msgs::Twist zero_twist_velocity_;

    // Parameter
    double soft_padding_front_rear_;
    double hard_padding_front_rear_;
    double soft_padding_left_right_;
    double hard_padding_left_right_;

    double linear_velocity_in_soft_padding_;
    double angular_velocity_in_soft_padding_;

    double angle_tolerance_;

    double min_linear_velocity_;
};

#endif  // MCR_COLLISION_VELOCITY_FILTER_COLLISION_VELOCITY_FILTER_H
