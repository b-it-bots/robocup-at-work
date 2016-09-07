#include <mcr_blob_detection/blob_detection_node.h>

BlobDetectionNode::BlobDetectionNode(ros::NodeHandle &nh) : node_handler_(nh), image_transporter_(nh)
{
    dynamic_reconfig_server_.setCallback(boost::bind(&BlobDetectionNode::dynamicReconfigCallback, this, _1, _2));
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    event_sub_ = node_handler_.subscribe("event_in", 1, &BlobDetectionNode::eventCallback, this);
    blob_pub_ = node_handler_.advertise<mcr_perception_msgs::BlobList>("blobs", 1);
    image_pub_ = image_transporter_.advertise("debug_image", 1);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &BlobDetectionNode::imageCallback, this);
    run_state_ = INIT;
    start_blob_detection_ = false;
    image_sub_status_ = false;
}

BlobDetectionNode::~BlobDetectionNode()
{
    event_pub_.shutdown();
    event_sub_.shutdown();
    blob_pub_.shutdown();
    image_pub_.shutdown();
    image_sub_.shutdown();
}

void BlobDetectionNode::dynamicReconfigCallback(mcr_blob_detection::BlobDetectionConfig &config, uint32_t level)
{
    debug_mode_ = config.debug_mode;
    bd_.updateDynamicVariables(config.debug_mode, config.min_blob_area, config.max_blob_area);
}

void BlobDetectionNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start")
    {
        start_blob_detection_ = true;
        ROS_INFO("2D Blob Detection ENABLED");
    }
    else if (event_command.data == "e_stop")
    {
        start_blob_detection_ = false;
        ROS_INFO("2D Blob Detection DISABLED");
    }
}

void BlobDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    image_message_ = img_msg;
    image_sub_status_ = true;
}

void BlobDetectionNode::states()
{
    switch (run_state_)
    {
    case INIT:
        initState();
        break;
    case IDLE:
        idleState();
        break;
    case RUNNING:
        runState();
        break;
    default:
        initState();
    }
}

void BlobDetectionNode::initState()
{
    if (image_sub_status_)
    {
        run_state_ = IDLE;
        image_sub_status_ = false;
    }
}

void BlobDetectionNode::idleState()
{
    if (start_blob_detection_)
    {
        run_state_ = RUNNING;
    }
    else
    {
        run_state_ = INIT;
    }
}

void BlobDetectionNode::runState()
{
    detectBlobs();
    run_state_ = INIT;
}

void BlobDetectionNode::detectBlobs()
{

    cv_bridge::CvImagePtr cv_img_tmp_ptr;
    cv::Mat debug_image;

    try
    {
        cv_img_tmp_ptr = cv_bridge::toCvCopy(image_message_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image_message_->encoding.c_str());
    }

    blobs_.clear();
    blob_detection_status_ = bd_.detectBlobs(cv_img_tmp_ptr->image, debug_image, blobs_);

    if (blob_detection_status_ == 1)
    {
        event_out_msg_.data = "e_blobs_detected";

        blob_list_.blobs.clear();
        for (int i = 0; i < blobs_.size(); i++)
        {
            geometry_msgs::Pose2D pose;
            pose.x = blobs_.at(i).at(0);
            pose.y = blobs_.at(i).at(1);
            pose.theta = blobs_.at(i).at(2);
            blob_.blob_pose = pose;
            blob_.blob_area = blobs_.at(i).at(3);
            blob_list_.blobs.push_back(blob_);
        }
        blob_pub_.publish(blob_list_);
    }
    else if (blob_detection_status_ == -1)
    {
        event_out_msg_.data = "e_no_blobs_detected";
    }
    else if (blob_detection_status_ == -2)
    {
        event_out_msg_.data = "e_error";
    }
    else
    {
        event_out_msg_.data = "e_error";
    }

    event_pub_.publish(event_out_msg_);

    if (debug_mode_)
    {
        cv_bridge::CvImage cv_img_tmp;
        cv_img_tmp.encoding = sensor_msgs::image_encodings::BGR8;
        cv_img_tmp.image = debug_image;
        image_pub_.publish(cv_img_tmp.toImageMsg());
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_detection");
    ros::NodeHandle nh("~");
    ROS_INFO("Blob Detection Node Initialised");
    BlobDetectionNode bd(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        bd.states();
        rate.sleep();
    }

    return 0;
}
