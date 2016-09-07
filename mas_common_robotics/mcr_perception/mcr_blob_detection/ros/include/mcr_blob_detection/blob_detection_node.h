#ifndef BLOBDETECTIONNODE_H_
#define BLOBDETECTIONNODE_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <mcr_blob_detection/BlobDetectionConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <mcr_blob_detection/blob_detection.h>
#include <mcr_perception_msgs/BlobList.h>
#include <mcr_perception_msgs/Blob.h>

using namespace cv;

class BlobDetectionNode
{

public:
    BlobDetectionNode(ros::NodeHandle &nh);
    virtual ~BlobDetectionNode();
    void dynamicReconfigCallback(mcr_blob_detection::BlobDetectionConfig &config, uint32_t level);
    void eventCallback(const std_msgs::String &event_command);
    void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
    void states();
    void initState();
    void idleState();
    void runState();
    void detectBlobs();

private:
    enum States
    {
        INIT,
        IDLE,
        RUNNING
    };

private:
    ros::NodeHandle node_handler_;
    dynamic_reconfigure::Server<mcr_blob_detection::BlobDetectionConfig> dynamic_reconfig_server_;
    ros::Subscriber event_sub_;
    ros::Publisher event_pub_;
    ros::Publisher blob_pub_;
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;
    bool image_sub_status_;
    sensor_msgs::ImageConstPtr image_message_;
    bool debug_mode_;
    bool start_blob_detection_;
    BlobDetection bd_;
    int blob_detection_status_ ;
    std::vector<vector<double> > blobs_;
    mcr_perception_msgs::BlobList blob_list_;
    mcr_perception_msgs::Blob blob_;
    std_msgs::String event_out_msg_;
    States run_state_;

};

#endif /* BLOBDETECTIONNODE_H_ */
