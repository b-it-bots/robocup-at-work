#ifndef BLOBTRACKINGNODE_H_
#define BLOBTRACKINGNODE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <mcr_blob_tracking/BlobTrackingConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <mcr_perception_msgs/BlobList.h>
#include <mcr_perception_msgs/Blob.h>
#include <string>
#include <math.h>

using namespace cv;

class BlobTrackingNode
{

public:
    BlobTrackingNode(ros::NodeHandle &nh);
    virtual ~BlobTrackingNode();
    void dynamicReconfigCallback(mcr_blob_tracking::BlobTrackingConfig &config, uint32_t level);
    void eventCallback(const std_msgs::String &event_command);
    void imageCallback(const sensor_msgs::ImageConstPtr &image_message);
    void blobsCallback(const mcr_perception_msgs::BlobList::ConstPtr &blobs);
    void states();
    void initState();
    void idleState();
    void runState();
    void trackBlob();
    void trackFirstLargesBlob();

private:
    enum States
    {
        INIT,
        IDLE,
        RUNNING
    };

private:
    ros::NodeHandle node_handler_;
    dynamic_reconfigure::Server<mcr_blob_tracking::BlobTrackingConfig> dynamic_reconfig_server_;
    ros::Subscriber event_sub_;
    ros::Publisher event_pub_;
    ros::Publisher blob_pose_pub_;
    ros::Subscriber blobs_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::ImageTransport image_transporter_;
    image_transport::Publisher image_pub_;
    bool image_sub_status_;
    bool blobs_sub_status_;
    sensor_msgs::ImageConstPtr image_message_;
    Mat debug_image_;
    bool debug_mode_;
    bool start_blob_tracking_;
    bool first_pass_;
    string tracking_type_;
    double blob_distance_threshold_;
    int blob_tracked_index_;
    vector<vector<double> > blobs_;
    mcr_perception_msgs::BlobList blob_list_;
    geometry_msgs::Pose2D blob_tracked_pose_;
    std_msgs::String event_out_msg_;
    States run_state_;

};

#endif /* BLOBTRACKINGNODE_H_ */
