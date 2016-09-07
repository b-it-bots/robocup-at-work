#ifndef CONTOUR_FINDER_ROS_H_
#define CONTOUR_FINDER_ROS_H_

#include <ros/ros.h>
#include <mcr_contour_matching/contour_finder.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <mcr_contour_matching/ContourFinderConfig.h>

/**
 * ROS interface for contour finder
 * Subscribes to:
 *  -pointcloud: pointcloud in which to find contours
 *
 * Publishes:
 *  -contour pointclouds: array of contours as pointclouds
 *  -debug image: debug image showing detected edges in the 2D image
 */
class ContourFinderROS
{
public:
    /**
     * Constructor
     */
    ContourFinderROS();
    /**
     * Destructor
     */
    virtual ~ContourFinderROS();
    /**
     * If pointcloud message has been received, the findContours function is called.
     * This function can be called once or periodically.
     */
    void update();


private:
    /**
     * Copy constructor.
     */
    ContourFinderROS(const ContourFinderROS &other);

    /**
     * Copy assignment operator.
     */
    ContourFinderROS &operator=(ContourFinderROS other);

    /**
     * Callback for pointcloud. Saves the pointcloud message.
     *
     * @param msg
     *          sensor_msgs::PointCloud2 message
     */
    void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);

    /**
     * Callback for event_in topic. Starts subscription to pointcloud if event_in is "e_trigger"
     */
    void eventInCallback(const std_msgs::String &msg);

    /**
     * Callback for dynamic reconfigure server to set canny threshold and multiplier
     */
    void dynamicReconfigCallback(mcr_contour_matching::ContourFinderConfig &config, uint32_t level);

    /**
     * Finds 2D contours and the corresponding 3D contours and publishes the array of 3D contours as pointclouds
     */
    void findContours();


private:
    /**
     * Object of ContourFinder
     */
    ContourFinder contour_finder_;

    /**
     * Node handle
     */
    ros::NodeHandle nh_;

    /**
     * Subscriber for input pointcloud
     */
    ros::Subscriber sub_pointcloud_;

    /**
     * Subscriber for event_in topic
     */
    ros::Subscriber sub_event_in_;

    /**
     * Publisher for 3D contours list
     */
    ros::Publisher pub_contour_pointclouds_;

    /**
     * Publisher for 3D contours as a single pointcloud
     */
    ros::Publisher pub_contour_pointclouds_combined_;

    /**
     * Publisher for debug image showing edges
     */
    image_transport::Publisher pub_debug_image_;

    /**
     * Used to store pointcloud message received in callback
     */
    sensor_msgs::PointCloud2::Ptr pointcloud_msg_;

    /**
     * Dynamic reconfigure server
     */
    dynamic_reconfigure::Server<mcr_contour_matching::ContourFinderConfig> dynamic_reconfigure_server_;

    /**
     * Flag indicating whether pointcloud has been received
     */
    bool pointcloud_msg_received_;
    /**
     * Flag indicating whether debug image should be published
     */
    bool publish_debug_image_;
};

#endif
