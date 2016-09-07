#ifndef PCA_POSE_ESTIMATOR_ROS_H_
#define PCA_POSE_ESTIMATOR_ROS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/**
 * This class estimates the pose of a pointcloud by aligning the x,y,z axes along the
 * largest to smallest eigenvectors of the pointcloud.
 *
 * Subscribes to
 * - input/pointcloud
 *
 * Publishes
 * - output/pose PoseStamped pose of pointcloud
 */
class PCAPoseEstimatorRos
{
public:
    /**
     * Constructor
     */
    PCAPoseEstimatorRos();
    /**
     * Destructor
     */
    virtual ~PCAPoseEstimatorRos();

    /**
     * if pointcloud has been received, estimatePose is called
     */
    void update();


private:
    /**
     * Pointcloud callback
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);

    /**
     * Estimates pose of a pointcloud by calculating its eigenvectors and aligning the x,y,z
     * axes along the largest to smallest vector.
     * Publishes a PoseStamped message
     */
    void estimatePose();


private:
    /**
     * Subscriber to pointcloud
     */
    ros::Subscriber sub_pointcloud_;

    /**
     * Pose publisher
     */
    ros::Publisher pub_pose_;

    /**
     * Used to store pointcloud message
     */
    sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

    /**
     * Flag to indicate if pointcloud has been received
     */
    bool pointcloud_msg_received_;
};
#endif
