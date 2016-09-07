#ifndef BACKGROUNDCHANGEDETECTIONNODE_H_
#define BACKGROUNDCHANGEDETECTIONNODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <string>
#include <mcr_background_change_detection/BackgroundChangeConfig.h>
#include <mcr_background_change_detection/background_change_detection.h>

class BackgroundChangeDetectionNode
{

public:
    BackgroundChangeDetectionNode(ros::NodeHandle &nh);
    virtual ~BackgroundChangeDetectionNode();
    void dynamicReconfigCallback(mcr_background_change_detection::BackgroundChangeConfig &config, uint32_t level);
    void eventCallback(const std_msgs::String &event_command);
    void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
    void states();
    void initState();
    void idleState();
    void runState();
    bool detectBackgroundChange();

private:
    enum States
    {
        INIT,
        IDLE,
        RUNNING
    };

private:
    ros::NodeHandle node_handler_;
    dynamic_reconfigure::Server<mcr_background_change_detection::BackgroundChangeConfig> dynamic_reconfigre_server_;
    ros::Publisher event_pub_;
    ros::Subscriber event_sub_;
    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::ImageConstPtr image_msg_;
    std_msgs::String event_out_msg_;
    bool is_debug_mode_;
    double background_change_threshold_;
    int timeout_time_;
    double background_learning_rate_;
    bool is_timeout_mode_;
    ros::Time start_time_;
    States current_state_;
    std_msgs::String event_in_msg_;
    bool has_image_data_;
    cv::Mat debug_image_;
    BackgroundChangeDetection bcd_;
    bool is_first_pass_;

};

#endif /* BACKGROUNDCHANGEDETECTIONNODE_H_ */