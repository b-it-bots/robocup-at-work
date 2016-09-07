#ifndef CAVITY_MESSAGE_BUILDER_ROS_H_
#define CAVITY_MESSAGE_BUILDER_ROS_H_

#include <ros/ros.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <mcr_perception_msgs/MatchingErrorStamped.h>

// Sync policy using exact time to sync pointcloud, matching error and pose
typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, mcr_perception_msgs::MatchingErrorStamped, geometry_msgs::PoseStamped> CavitySyncPolicy;

/**
 * This class combines a contour pointcloud message, a MatchingErrorStamped message and a PoseStamped message
 * into a Cavity message. The aim is to ensure all three have the same timestamp and are hence
 * generated from the same pointcloud.
 *
 * Subscribes to
 * - input/pointcloud - pointcloud for cavity contour
 * - input/matching error - matching error of cavity contour with template
 * - input/pose - pose of the cavity contour
 *
 * Publishes:
 * - output/cavity - Cavity message containing the three messages above
 *
 */
class CavityMessageBuilderROS
{
public:
    /**
     * Constructor
     * Initializes synced subscriber and Cavity publisher
     *
     */
    CavityMessageBuilderROS();

    /**
     * Destructor
     */
    virtual ~CavityMessageBuilderROS();


private:
    /**
     * Copy constructor.
     */
    CavityMessageBuilderROS(const CavityMessageBuilderROS &other);

    /**
     * Copy assignment operator.
     */
    CavityMessageBuilderROS &operator=(CavityMessageBuilderROS other);

    /**
     * Callback for pointcloud, matching error and pose
     * Constructs Cavity message and publishes it
     */
    void synchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg, const mcr_perception_msgs::MatchingErrorStamped::ConstPtr &matching_error_msg, const geometry_msgs::PoseStamped::ConstPtr &pose_msg);


private:
    /**
     * Cavity message publisher
     */
    ros::Publisher pub_cavity_;

    /**
     * PointCloud Subscriber
     */
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pointcloud_;

    /**
     * MatchingError Subscriber
     */
    message_filters::Subscriber<mcr_perception_msgs::MatchingErrorStamped> sub_matching_error_;

    /**
     * Pose Subscriber
     */
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;

    /**
     * Synchronized subscriber
     */
    boost::shared_ptr<message_filters::Synchronizer<CavitySyncPolicy> > sync_input_;
};

#endif
