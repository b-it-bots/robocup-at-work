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

#include <mcr_collision_velocity_filter/collision_velocity_filter.h>
#include <vector>

CollisionVelocityFilter::CollisionVelocityFilter() :
    soft_padding_front_rear_(0.15), hard_padding_front_rear_(0.02), soft_padding_left_right_(0.15),
        hard_padding_left_right_(0.06), linear_velocity_in_soft_padding_(0.02), angular_velocity_in_soft_padding_(
        0.1)
{
    zero_twist_velocity_.linear.x = zero_twist_velocity_.linear.y = zero_twist_velocity_.linear.z = 0.0;
    zero_twist_velocity_.angular.x = zero_twist_velocity_.angular.y = zero_twist_velocity_.angular.z = 0.0;
}

CollisionVelocityFilter::~CollisionVelocityFilter()
{
}

void CollisionVelocityFilter::setSoftPaddingParameter(const double &front_rear, const double &left_right)
{
    soft_padding_front_rear_ = front_rear;
    soft_padding_left_right_ = left_right;
}

void CollisionVelocityFilter::setHardPaddingParameter(const double &front_rear, const double &left_right)
{
    hard_padding_front_rear_ = front_rear;
    hard_padding_left_right_ = left_right;
}

void CollisionVelocityFilter::setVelocitiesInSoftPadding(const double &linear_velocity, const double &angular_velocity)
{
    linear_velocity_in_soft_padding_ = linear_velocity;
    angular_velocity_in_soft_padding_ = angular_velocity;
}

void CollisionVelocityFilter::setAngleTolerance(const double &angle_tolerance)
{
    angle_tolerance_ = angle_tolerance;
}

void CollisionVelocityFilter::setMinimumLinearVelocity(const double &min_linear_velocity)
{
    min_linear_velocity_ = min_linear_velocity;
}

geometry_msgs::Twist CollisionVelocityFilter::calculateSafeBaseVelocities(const geometry_msgs::Twist &desired_twist,
    const pcl::PointCloud<pcl::PointXYZ> &scans_as_pointcloud)
{
    geometry_msgs::Twist safe_twist;
    int roi_start_index = 0, roi_end_index = 0;

    // result is stored in member variables hard_padding_footprint and soft_padding_footprint
    calculateSoftAndHardFootprints(desired_twist);

    calculateFootprintQuadrants(hard_padding_footprint_);

    pcl::PointCloud < pcl::PointXYZ > soft_footprint_cloud;
    pcl::PointCloud < pcl::PointXYZ > hard_footprint_cloud;

    // check if the footprints have the same point size
    if (soft_padding_footprint_.polygon.points.size() != hard_padding_footprint_.polygon.points.size())
    {
        ROS_ERROR_STREAM("soft- and hard padding footprint do not have the same length: " <<
            soft_padding_footprint_.polygon.points.size() << " != " << hard_padding_footprint_.polygon.points.size());
        return zero_twist_velocity_;
    }

    // copy the polygon data into a pointcloud structure
    for (unsigned i = 0; i < soft_padding_footprint_.polygon.points.size(); ++i)
    {
        soft_footprint_cloud.points.push_back(pcl::PointXYZ(soft_padding_footprint_.polygon.points[i].x,
            soft_padding_footprint_.polygon.points[i].y, soft_padding_footprint_.polygon.points[i].z));
        hard_footprint_cloud.points.push_back(pcl::PointXYZ(hard_padding_footprint_.polygon.points[i].x,
            hard_padding_footprint_.polygon.points[i].y, hard_padding_footprint_.polygon.points[i].z));
    }

    if ((soft_footprint_cloud.points.size() <= 0) || (hard_footprint_cloud.points.size() <= 0))
    {
        ROS_ERROR_STREAM("either the soft- or hard footprint has no data points");
        return zero_twist_velocity_;
    }

    // the function isXYPointIn2DXYPolygon later on needs a closed polygon,
    // thus the first point of the polygon is appended as last point to "close" the polygon
    soft_footprint_cloud.points.push_back(soft_footprint_cloud.points[0]);
    hard_footprint_cloud.points.push_back(hard_footprint_cloud.points[0]);

    bool in_soft_padding = false;

    double velocity_orientation;
    VelocityDirections velocity_direction;
    velocity_direction = NO_MOTION;

    std::vector<bool> quadrants_with_point;

    quadrants_with_point = quadrantsWithPoint(scans_as_pointcloud);

    if (fabs(desired_twist.linear.x) >= min_linear_velocity_ || fabs(desired_twist.linear.y) >= min_linear_velocity_)
    {
        velocity_orientation = calculateVelocityOrientation(desired_twist) * 180 / M_PI;  // in degrees
        velocity_direction = calculateVelocityDirection(velocity_orientation);
    }

    // Cases when there are two quadrants with points
    // Case: Rear side
    if (quadrants_with_point[0] && quadrants_with_point[1])
        if (velocity_direction != FRONT)
            return zero_twist_velocity_;
    // Case: Left side
    if (quadrants_with_point[1] && quadrants_with_point[2])
        if (velocity_direction != RIGHT)
            return zero_twist_velocity_;
    // Case: Right side
    if (quadrants_with_point[0] && quadrants_with_point[3])
        if (velocity_direction != LEFT)
            return zero_twist_velocity_;
    // Case: Front side
    if (quadrants_with_point[2] && quadrants_with_point[3])
        if (velocity_direction != REAR)
            return zero_twist_velocity_;

    // Cases when there is only one quadrant with points
    // Case: Rear right side
    if (quadrants_with_point[0])
        if (velocity_direction != FRONT && velocity_direction != FRONT_LEFT && velocity_direction != LEFT)
            return zero_twist_velocity_;
    // Case: Rear left side
    if (quadrants_with_point[1])
        if (velocity_direction != FRONT && velocity_direction != FRONT_RIGHT && velocity_direction != RIGHT)
            return zero_twist_velocity_;
    // Case: Front left side
    if (quadrants_with_point[2])
        if (velocity_direction != REAR && velocity_direction != REAR_RIGHT && velocity_direction != RIGHT)
            return zero_twist_velocity_;
    // Case: Front right side
    if (quadrants_with_point[3])
        if (velocity_direction != REAR && velocity_direction != REAR_LEFT && velocity_direction != LEFT)
            return zero_twist_velocity_;

    for (size_t i = 0; i < scans_as_pointcloud.points.size(); ++i)
    {
        if (pcl::isXYPointIn2DXYPolygon(scans_as_pointcloud.points[i], soft_footprint_cloud) && !in_soft_padding)
            in_soft_padding = true;
    }

    // reduce the velocity if the robot is close to an obstacle
    if (in_soft_padding)
    {
        if (desired_twist.linear.x != 0)
            safe_twist.linear.x = linear_velocity_in_soft_padding_ *
                (desired_twist.linear.x / fabs(desired_twist.linear.x));
        if (desired_twist.linear.y != 0)
            safe_twist.linear.y = linear_velocity_in_soft_padding_ *
                (desired_twist.linear.y / fabs(desired_twist.linear.y));
        if (desired_twist.angular.z != 0)
            safe_twist.angular.z = angular_velocity_in_soft_padding_ *
                (desired_twist.angular.z / fabs(desired_twist.angular.z));
    }
    // use the actually commanded velocity if not close to any obstacle
    else
    {
        safe_twist.linear = desired_twist.linear;
        safe_twist.angular = desired_twist.angular;
    }

    return safe_twist;
}

void CollisionVelocityFilter::updateRealFootprint(const geometry_msgs::PolygonStamped &footprint)
{
    real_footprint_ = footprint;
}

void CollisionVelocityFilter::calculateSoftAndHardFootprints(const geometry_msgs::Twist &desired_twist)
{
    // clear previously calculated soft-/hard footprints
    soft_padding_footprint_ = hard_padding_footprint_ = real_footprint_;

    // calculate soft and hard padding footprints
    for (size_t i = 0; i < soft_padding_footprint_.polygon.points.size(); ++i)
    {
        // if driving to the backwards, extend the polygon in this direction
        if ((desired_twist.linear.x < 0) && (soft_padding_footprint_.polygon.points[i].x < 0))
        {
            soft_padding_footprint_.polygon.points[i].x -= soft_padding_front_rear_;
            hard_padding_footprint_.polygon.points[i].x -= hard_padding_front_rear_;
        }
        // if driving to the forwards, extend the polygon in this direction
        else if ((desired_twist.linear.x > 0) && (soft_padding_footprint_.polygon.points[i].x > 0))
        {
            soft_padding_footprint_.polygon.points[i].x += soft_padding_front_rear_;
            hard_padding_footprint_.polygon.points[i].x += hard_padding_front_rear_;
        }

        // if turning extend both sides or if shifting to the left/right extend to the particular side
        if (((fabs(desired_twist.angular.z) > 0) || (desired_twist.linear.y < 0)) &&
            (soft_padding_footprint_.polygon.points[i].y < 0))
        {
            soft_padding_footprint_.polygon.points[i].y -= soft_padding_left_right_;
            hard_padding_footprint_.polygon.points[i].y -= hard_padding_left_right_;
        }
        if (((fabs(desired_twist.angular.z) > 0) || (desired_twist.linear.y > 0)) &&
            (soft_padding_footprint_.polygon.points[i].y > 0))
        {
            soft_padding_footprint_.polygon.points[i].y += soft_padding_left_right_;
            hard_padding_footprint_.polygon.points[i].y += hard_padding_left_right_;
        }
    }
}

/** @brief calculates the arc tangent for the linear components of the velocity.
 *      It returns an angle (in radians) of the velocity's orientation.
 *
 *  @param twist the twist with the linear velocity
 *
 */
double CollisionVelocityFilter::calculateVelocityOrientation(const geometry_msgs::Twist &twist)
{
    double velocity_orientation;
    velocity_orientation = atan2(twist.linear.y, twist.linear.x);
    return velocity_orientation;
}

/** @brief calculates the quadrants of a footprint.
 *      It returns an enum with the direction of the velocity (e.g front, left, front-left).
 *
 *  @param velocity_orientation the angle representing the orientation of the velocity]
 *
 */
CollisionVelocityFilter::VelocityDirections
    CollisionVelocityFilter::calculateVelocityDirection(double velocity_orientation)
{
    if (velocity_orientation <= ZERO_DEGREES + angle_tolerance_ &&
        velocity_orientation >= ZERO_DEGREES - angle_tolerance_)
        return FRONT;
    if (velocity_orientation >= ZERO_DEGREES + angle_tolerance_
        && velocity_orientation <= NINETY_DEGREES - angle_tolerance_)
        return FRONT_LEFT;
    if (velocity_orientation <= NINETY_DEGREES + angle_tolerance_
        && velocity_orientation >= NINETY_DEGREES - angle_tolerance_)
        return LEFT;
    if (velocity_orientation >= NINETY_DEGREES + angle_tolerance_
        && velocity_orientation <= ONE_EIGHTY_DEGREES - angle_tolerance_)
        return REAR_LEFT;
    if (velocity_orientation <= ONE_EIGHTY_DEGREES + angle_tolerance_
        && velocity_orientation >= ONE_EIGHTY_DEGREES - angle_tolerance_)
        return REAR;
    if (velocity_orientation <= ZERO_DEGREES - angle_tolerance_
        && velocity_orientation >= -NINETY_DEGREES + angle_tolerance_)
        return FRONT_RIGHT;
    if (velocity_orientation <= -NINETY_DEGREES + angle_tolerance_
        && velocity_orientation >= -NINETY_DEGREES - angle_tolerance_)
        return RIGHT;
    if (velocity_orientation <= -NINETY_DEGREES - angle_tolerance_
        && velocity_orientation >= -ONE_EIGHTY_DEGREES + angle_tolerance_)
        return REAR_RIGHT;
    return NO_MOTION;
}

/** @brief calculates the quadrants of a footprint.
 *
 *  @param footprint the footprint from where the quadrants will be calculated
 *
 */
void CollisionVelocityFilter::calculateFootprintQuadrants(const geometry_msgs::PolygonStamped &footprint)
{
    double rear_right_corner_x = footprint.polygon.points[0].x;
    double rear_right_corner_y = footprint.polygon.points[0].y;

    double rear_left_corner_x = footprint.polygon.points[1].x;
    double rear_left_corner_y = footprint.polygon.points[1].y;

    double front_left_corner_x = footprint.polygon.points[2].x;
    double front_left_corner_y = footprint.polygon.points[2].y;

    double front_right_corner_x = footprint.polygon.points[3].x;
    double front_right_corner_y = footprint.polygon.points[3].y;

    double center_x = (front_left_corner_x + rear_right_corner_x) / 2;
    double center_y = (front_left_corner_y + rear_right_corner_y) / 2;

    rear_right_footprint_ = calculateFootprintQuadrant(
        footprint, rear_right_corner_x, rear_right_corner_y, center_x, center_y);
    rear_left_footprint_ = calculateFootprintQuadrant(
        footprint, rear_left_corner_x, rear_left_corner_y, center_x, center_y);
    front_left_footprint_ = calculateFootprintQuadrant(
        footprint, front_left_corner_x, front_left_corner_y, center_x, center_y);
    front_right_footprint_ = calculateFootprintQuadrant(
        footprint, front_right_corner_x, front_right_corner_y, center_x, center_y);
}

/** @brief calculates a quadrant of a footprint based on a corner and the center of the footprint.
 *
 *  @param footprint the footprint from where the quadrant will be calculated
 *  @param corner_x the corner of the footprint in the Y axis
 *  @param corner_y the corner of the footprint in the Y axis
 *  @param center_x the center of the footprint in the X axis
 *  @param center_y the center of the footprint in the Y axis
 *
 */
geometry_msgs::PolygonStamped CollisionVelocityFilter::calculateFootprintQuadrant(
    const geometry_msgs::PolygonStamped &footprint, double corner_x, double corner_y, double center_x, double center_y
)
{
    geometry_msgs::PolygonStamped quadrant;
    quadrant = footprint;

    quadrant.polygon.points[0].x = corner_x;
    quadrant.polygon.points[0].y = corner_y;

    quadrant.polygon.points[1].x = corner_x;
    quadrant.polygon.points[1].y = center_y;

    quadrant.polygon.points[2].x = center_x;
    quadrant.polygon.points[2].y = center_y;

    quadrant.polygon.points[3].x = center_x;
    quadrant.polygon.points[3].y = corner_y;

    return quadrant;
}

/** @brief calculates which quadrants have, at least, a point inside.
 *      It returns a boolean array for each quadrant with the following order:
 *      [rear_right, rear_left, front_left, front_right]
 *
 *  @param pointcloud A point-cloud having the points obtained from laser scanner(s).
 *
 */
std::vector<bool> CollisionVelocityFilter::quadrantsWithPoint(const pcl::PointCloud<pcl::PointXYZ> &pointcloud)
{
    std::vector<bool> occupied_quadrants;

    bool rear_right_occupied = false;
    bool rear_left_occupied = false;
    bool front_left_occupied = false;
    bool front_right_occupied = false;

    pcl::PointCloud < pcl::PointXYZ > rear_right_cloud;
    pcl::PointCloud < pcl::PointXYZ > rear_left_cloud;
    pcl::PointCloud < pcl::PointXYZ > front_left_cloud;
    pcl::PointCloud < pcl::PointXYZ > front_right_cloud;

    // copy the polygon data into a point-cloud structure
    for (unsigned i = 0; i < rear_right_footprint_.polygon.points.size(); ++i)
    {
        rear_right_cloud.points.push_back(pcl::PointXYZ(rear_right_footprint_.polygon.points[i].x,
            rear_right_footprint_.polygon.points[i].y, rear_right_footprint_.polygon.points[i].z));
        rear_left_cloud.points.push_back(pcl::PointXYZ(rear_left_footprint_.polygon.points[i].x,
            rear_left_footprint_.polygon.points[i].y, rear_left_footprint_.polygon.points[i].z));
        front_left_cloud.points.push_back(pcl::PointXYZ(front_left_footprint_.polygon.points[i].x,
            front_left_footprint_.polygon.points[i].y, front_left_footprint_.polygon.points[i].z));
        front_right_cloud.points.push_back(pcl::PointXYZ(front_right_footprint_.polygon.points[i].x,
            front_right_footprint_.polygon.points[i].y, front_right_footprint_.polygon.points[i].z));
    }

    // the function isXYPointIn2DXYPolygon later on needs a closed polygon,
    // thus the first point of the polygon is appended as last point to "close" the polygon
    rear_right_cloud.points.push_back(rear_right_cloud.points[0]);
    rear_left_cloud.points.push_back(rear_left_cloud.points[0]);
    front_left_cloud.points.push_back(front_left_cloud.points[0]);
    front_right_cloud.points.push_back(front_right_cloud.points[0]);

    for (size_t i = 0; i < pointcloud.points.size(); ++i)
    {
        if (pcl::isXYPointIn2DXYPolygon(pointcloud.points[i], rear_right_cloud))
            rear_right_occupied = true;
        if (pcl::isXYPointIn2DXYPolygon(pointcloud.points[i], rear_left_cloud))
            rear_left_occupied = true;
        if (pcl::isXYPointIn2DXYPolygon(pointcloud.points[i], front_left_cloud))
            front_left_occupied = true;
        if (pcl::isXYPointIn2DXYPolygon(pointcloud.points[i], front_right_cloud))
            front_right_occupied = true;
    }

    occupied_quadrants.push_back(rear_right_occupied);
    occupied_quadrants.push_back(rear_left_occupied);
    occupied_quadrants.push_back(front_left_occupied);
    occupied_quadrants.push_back(front_right_occupied);

    return occupied_quadrants;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getRealFootprint()
{
    return real_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getSoftPaddingFootprint()
{
    return soft_padding_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getHardPaddingFootprint()
{
    return hard_padding_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getFrontRightFootprint()
{
    return front_right_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getFrontLeftFootprint()
{
    return front_left_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getRearRightFootprint()
{
    return rear_right_footprint_;
}

geometry_msgs::PolygonStamped CollisionVelocityFilter::getRearLeftFootprint()
{
    return rear_left_footprint_;
}
