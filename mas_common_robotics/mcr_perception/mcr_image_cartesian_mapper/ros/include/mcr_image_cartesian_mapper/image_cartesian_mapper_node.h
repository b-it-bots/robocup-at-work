/**
 * @file image_cartesian_mapper_node.h
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @data June, 2015
 */

#ifndef IMAGECARTESIANMAPPERNODE_H_
#define IMAGECARTESIANMAPPERNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <string>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <Eigen/LU>
#include <math.h>

/**
 * ROS interface to transform image pixel space to cartesian space with respect to the required frame
 *
 * Transforms image pixel to the metrical space in the camera frame using camera internal matrix
 * Transforms metrical space with respect to the required frame using tf
 */
class ImageCartesianMapperNode
{

public:
    /**
     * Constructor.
     *
     * @param nh Private node handle.
     */
    ImageCartesianMapperNode(ros::NodeHandle &nh);

    /**
     * Destructor.
     */
    virtual ~ImageCartesianMapperNode();

    /**
     * Call back for event_in topic to decide the current state of the node.
     *
     * @param event_command The event string.
     */
    void eventCallback(const std_msgs::String &event_command);

    /**
     * Call back for the image pixel 2D pose which needs to be transformed.
     *
     * @param pose Image pixel 2D position.
     */
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose);

    /**
     * Handles the current state of the node based on the events and data availability.
     */
    void states();

    /**
     * Wait until event e_start is received and then switch to IDLE state.
     */
    void initState();

    /*
     * Wait until data (pixel pose and camera info) is available and then switch to RUNNING state.
     * If event e_stop is sent then switch to INIT state.
     */
    void idleState();

    /*
     * Process the data as per the functionality and publish the corresponding event in the event_out topic.
     * If event e_stop is sent then switch to INIT state if not switch to IDLE state.
     */
    void runState();

    /*
     * Converts the image pixel to metrical units in cartesian space based on the camera internal matrix.
     */
    bool cameraOpticalToCameraMetrical();

    /*
     * Transform the cartesian pose from camera frame into the required frame specified in the ros param.
     */
    bool cameraMetricalToCartesian();

private:

    /**
     * An enum to handle the current state of the node
     */
    enum States
    {
        INIT, /**< INIT state while waiting for start event */
        IDLE, /**< IDLE state while waiting for the data. */
        RUNNING /**< RUNNING state while processing the data. */
    };

private:
    /**
     * ROS Node handler for the node.
     */
    ros::NodeHandle node_handler_;

    /**
     * ROS subscriber for the event_in topic
     */
    ros::Subscriber event_sub_;

    /**
     * ROS subscriber for the pose topic to receive image pixel pose in 2D
     */
    ros::Subscriber pose_sub_;

    /**
     * ROS publisher to publish the event out message in event_out topic
     */
    ros::Publisher event_pub_;

    /**
     * ROS publisher to publish the calculated cartesian pose in cartesian_pose topic
     */
    ros::Publisher cartesian_pub_;

    /**
     * Represents the pose data availability in pose topic to receive image pixel pose in 2D
     */
    bool has_pose_data_;

    /**
     * Stores the event message sent in the event_in topic
     */
    std_msgs::String event_in_msg_;

    /**
     * Represents if image filter node is enabled
     */
    bool is_image_filter_enabled_;

    /**
     * Represents if image cropping is enabled through image filter node
     */
    bool is_image_crop_enabled_;

    /**
     * Stores the Pose2D message sent in pose topic
     */
    geometry_msgs::Pose2D pose_2d_;

    /**
     * Stores the cartesian pose with respect to camera frame
     */
    geometry_msgs::PoseStamped camera_optical_pose_;

    /**
     * Stores the cartesian pose with respect to the required frame as specified by the param
     */
    geometry_msgs::PoseStamped cartesian_pose_;

    /**
     * Stores the current state of the node
     */
    States current_state_;

    /**
     * Object to TransformLister class handle frame transformations
     */
    tf::TransformListener listener_;

    /**
     * Stores the target frame to which transformation has to be made
     */
    std::string target_frame_;

    /**
     * Stores the source frame from which the transformation has to be made
     */
    std::string source_frame_;

    /**
     * Camera Intrincsic Matrix stored in a vector list
     */
    std::vector<double> camera_intrinsic_list_;

    /**
     * Stores the pixel pose in camera coordinates
     */
    Eigen::Vector3f camera_coordinates_;

    /**
     * Stores the camera intrinsic matrix
     */
    Eigen::Matrix3f camera_intrinsic_matrix_;

    /**
     * Stores the cartesian pose in camera frame
     */
    Eigen::Vector3f camera_metrical_coordinates_;

    /**
     * Stores the event out messaged to be published in event_out topic
     */
    std_msgs::String event_out_msg_;

    /**
     * Image width as provided in the realsense camera driver
     */
    double image_width_;

    /**
     * Image height as provided in the realsense camera driver
     */
    double image_height_;

    /**
     * Crop factor top as provided in the image filter param
     */
    double crop_factor_top_;

    /**
     * Crop factor left as provided in the image filter param
     */
    double crop_factor_left_;

};

#endif /* IMAGECARTESIANMAPPERNODE_H_ */
