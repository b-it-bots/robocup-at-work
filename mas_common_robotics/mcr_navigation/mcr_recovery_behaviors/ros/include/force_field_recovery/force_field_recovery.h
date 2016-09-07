/* Force field recovery behavior
 *
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 *
 * Original idea from : https://github.com/daenny
 *
 * Main code idea mostly taken from :
 *
 * https://github.com/smARTLab-liv/smartlabatwork-release/blob/master/slaw_registration/src/forcefield_recovery.cpp
 *
 * Refactored, enhaced and mantained by:    Oscar Lima (olima_84@yahoo.com)
 *
 * About this code: Plugin for the mobile base (move_base ros) : Recovery behavior
 * that moves away from obstacles by using the costmap. It calculates the vector resultant
 * in a pointcloud of obstacles and moves away from the cluster by publishing on cmd_vel topic
 *
 */

#ifndef FORCE_FIELD_RECOVERY_FORCE_FIELD_RECOVERY_H
#define FORCE_FIELD_RECOVERY_FORCE_FIELD_RECOVERY_H

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <string>
#include <std_msgs/String.h>

// For transforming costmap occupied cells into pointcloud
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// For moving the mobile base (publish in cmd_vel)
#include <geometry_msgs/Twist.h>

// For publishing neighbourhood and force field vector as marker
#include <visualization_msgs/Marker.h>

// Ros PCL includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

// For publishing local costmap tf
#include <tf/transform_broadcaster.h>

// For pointcloud reference frame transformer
#include <pcl_ros/transforms.h>

#define LETHAL_COST 254

namespace force_field_recovery
{
/**
* @class ForceFieldRecovery
* @brief A recovery behavior that moves the base away from obstacles when stucked
*/
class ForceFieldRecovery : public nav_core::RecoveryBehavior
{
public:
    /**
    * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
    * @param  Empty constructor, no arguments
    */
    ForceFieldRecovery();

    /**
    * @brief  Initialization function for the ForceField recovery behavior
    * Receives tf, global costmap, local costmap and copies their values into member variables
    * Also setups the publishers and reads values from the parameter server
    *
    * @param tf A pointer to a transform listener
    * @param global_costmap A pointer to the global_costmap used by the navigation stack
    * @param local_costmap A pointer to the local_costmap used by the navigation stack
    */
    void initialize(std::string name, tf::TransformListener* tf,
                    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

    /**
    * @brief  Run the ForceFieldRecovery recovery behavior.
    * This function checks if the costmaps are empty (if so then exits) and calls move_base_away \
    * function
    */
    void runBehavior();

private:
    /**
    * @brief  This function receives x and y velocity and publishes to cmd_vel topic to move the mobile base
    *
    * step 1. Get the current costmap
    * step 2. Convert all obstacles (cost = 254) inside local costmap into pointcloud
    * step 3. Publish the previous obtained obstacle pointcloud for debugging purposes
    * step 4. Change cloud to the reference frame of the robot since
    *         it was in map frame and needs to be in the reference frame of the robot
    * step 5. Publish base link obstacle cloud for debugging purposes, the cloud must match
    *         with the previous obstacle pointcloud
    * step 6. Compute force field vector as the negative of the resultant of all points
    *         assuming that they are vectors
    * step 7. Publish force field vector as marker for visualization in rviz (not implemented yet)
    * step 8. move base in the direction of the force field vector by multiplying the normalized
    *         vector times a scale factor which is a parameter that can be changed
    *
    * @param costmap A pointer to the local_costmap used by the navigation stack
    */
    void moveBaseAwayFromObstacles(costmap_2d::Costmap2DROS* costmap);

    /**
    * @brief  This function transforms occupied regions of a costmap, letal cost = 254 to
    * pointcloud xyz coordinate
    *
    * @param costmap A pointer to the local_costmap used by the navigation stack
    */
    pcl::PointCloud<pcl::PointXYZ> costmapToPointcloud(const costmap_2d::Costmap2D* costmap);

    /**
    * @brief This function receives a pcl pointcloud, transforms into ros pointcloud and then publishes the cloud
    * @param cloud A pcl pointcloud to be published
    * @param cloud_pub The ros publisher object to execute the method .publish
    * @param frame_id The frame id of the pointcloud to be published
    */
    sensor_msgs::PointCloud2 publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud, ros::Publisher &cloud_pub,
                                          std::string frame_id);

    /**
    * @brief This function receives a ros cloud (with an associated tf) and tranforms
    * all the points to another reference frame (target_reference_frame)
    * @param ros_cloud A ros pointcloud to be transformed
    * @param target_reference_frame The reference frame in which you want the pointcloud to be converted
    */
    pcl::PointCloud<pcl::PointXYZ> changeCloudReferenceFrame(sensor_msgs::PointCloud2 &ros_cloud,
                                                             std::string target_reference_frame);

    /**
    * @brief  This function receives a cloud and returns the negative of the resultant
    * assuming that all points in the cloud are vectors
    * @param cloud pointcloud of obstacles expresed in the reference frame of the robot
    */
    Eigen::Vector3f computeForceField(pcl::PointCloud<pcl::PointXYZ> &cloud);

    /**
    * @brief  Checks for conditions to stop the recovery behavior: A, B,C
    * A: When there are no more obstacles in the neighbourhood of the robot
    * B: When oscillations in the force field are detected
    * C: When the recovery is taking too much time to execute (timeout)
    * @param force_field the force field vector, computed based on obstacles comming formt the costmap
    * @param no_obstacles_in_radius this is information that this function returns to the user as pointer
    * @param number_of_oscillations
    * @param start_time
    * @param timeout
    */
    bool checkStoppingConditions(Eigen::Vector3f &force_field, ros::Time start_time);

    /**
    * @brief  Detects oscillations in the force field
    * Sometimes the robot gets stucked and has obstacles all around, this will result in an oscillating
    * behavior, this means the robot will go back and forward until the timeout has passed, this function
    * looks for big changes in the force field angle, therefore stopping the recovery if needed
    * @param force_field the latest force field vector computed from the obstacle cloud taken from the costmap
    */
    bool detectOscillations(Eigen::Vector3f &force_field);

    /**
    * @brief This function receives x and y velocity and publishes to cmd_vel topic to move the mobile base
    * @param x The x velocity to be send to the mobile base
    * @param y The y velocity to be send to the mobile base
    */
    void publishVelocities(double x, double y);

    /**
    * @brief This function is for visualization of the neighbourhood area in rviz
    */
    void publishObstacleNeighborhood();

    /**
    * @brief This function is for visualization of the force_field vector in rviz
    * @param force_field the force field vector, computed based on obstacles comming from the costmap
    */
    void publishForceField(Eigen::Vector3f force_field);

    /*
     * private member variables
     *
     */

    // Flag used for preventing the class to initialize more than one time
    bool initialized_;

    // For avoiding to check oscillations in the first iteration, since there is no previous force
    // to compare with
    bool is_oscillation_detection_initialized_;

    // A pointer to the transform listener sent by move_base
    tf::TransformListener* tf_;

    // Pointers to receive the costmaps from move_base
    costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;

    // The distance in meters away from the robot to include obstacles in the force field calculation
    double obstacle_neighborhood_;

    // The factor to which the force field will be multyplied before it is sent to the mobile base as velocity
    double velocity_scale_;

    // The maximum speed of the base to be commanded by the force field behavior
    double max_velocity_;

    // The timeout value until recovery behavior gives up from moving away from obstacles
    double timeout_;

    // The frequency update for the local costmap
    double recovery_behavior_update_frequency_;

    // A backup of the previous force field angle, used to detect oscillations in the force
    double previous_angle_;

    // Used for detecting oscillations in the force field by comparing the previous and current angle of
    // the force field +/- some angular tolerance
    double oscillation_angular_tolerance_;

    // The maximum number of allowed oscillations in the recovery behavior
    int allowed_oscillations_;

    // Variable used to count how many oscillations the robot has made so far
    int number_of_oscillations_;

    // For storing the recovery behavior robot reference frame
    std::string robot_base_frame_;

    // For storing the recovery behavior global reference frame (costmap reference frame)
    std::string robot_global_frame_;

    // A twist publisher for cmd_vel used to publish a velocity to the mobile base
    ros::Publisher pub_twist_;

    // A pointcloud publisher that will publish the obstacle cloud for visualization purposes
    ros::Publisher pub_obstacle_cloud_;

    // Publisher for visualizing the neighbourhood, this means the points from the costmap that will be
    // included to compute the force field vector
    ros::Publisher pub_neighbourhood_;

    // Publisher for the force field vector as marker for visualization purposes
    ros::Publisher pub_ff_marker_;
};
};  // namespace force_field_recovery

#endif  // FORCE_FIELD_RECOVERY_FORCE_FIELD_RECOVERY_H
