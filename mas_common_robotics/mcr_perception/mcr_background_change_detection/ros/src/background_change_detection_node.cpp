#include <mcr_background_change_detection/background_change_detection_node.h>

BackgroundChangeDetectionNode::BackgroundChangeDetectionNode(ros::NodeHandle &nh) : node_handler_(nh), image_transporter_(nh)
{
    dynamic_reconfigre_server_.setCallback(boost::bind(&BackgroundChangeDetectionNode::dynamicReconfigCallback, this, _1, _2));
    event_sub_ = node_handler_.subscribe("event_in", 1, &BackgroundChangeDetectionNode::eventCallback, this);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &BackgroundChangeDetectionNode::imageCallback, this);
    image_pub_ = image_transporter_.advertise("debug_image", 1);
    current_state_ = INIT;
    has_image_data_ = false;
}

BackgroundChangeDetectionNode::~BackgroundChangeDetectionNode()
{
    event_sub_.shutdown();
    event_pub_.shutdown();
    image_sub_.shutdown();
    image_pub_.shutdown();
}


void BackgroundChangeDetectionNode::dynamicReconfigCallback(mcr_background_change_detection::BackgroundChangeConfig &config, uint32_t level)
{
    background_change_threshold_ = config.background_change_threshold;
    background_learning_rate_ = config.background_learning_rate;
    timeout_time_ = config.timeout_time;
    is_timeout_mode_ = config.is_timeout_mode;
    is_debug_mode_ = config.is_debug_mode;
    bcd_.updateDynamicVariables(is_debug_mode_, background_change_threshold_, background_learning_rate_);
}

void BackgroundChangeDetectionNode::eventCallback(const std_msgs::String &event_msg)
{
    event_in_msg_ = event_msg;
}

void BackgroundChangeDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
    image_msg_ = image_msg;
    has_image_data_ = true;
}

void BackgroundChangeDetectionNode::states()
{
    switch (current_state_)
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

void BackgroundChangeDetectionNode::initState()
{
    if (event_in_msg_.data == "e_start")
    {
        current_state_ = IDLE;
        event_in_msg_.data = "";
        is_first_pass_ = true;
        start_time_ = ros::Time::now();
    }
    else
    {
        current_state_ = INIT;
    }
}

void BackgroundChangeDetectionNode::idleState()
{
    if (event_in_msg_.data == "e_stop")
    {
        current_state_ = INIT;
        event_in_msg_.data = "";
    }
    else if (has_image_data_)
    {
        current_state_ = RUNNING;
        has_image_data_ = false;
    }
    else
    {
        current_state_ = IDLE;
    }
}

void BackgroundChangeDetectionNode::runState()
{
    if (is_timeout_mode_ && ((ros::Time::now() - start_time_).toSec() > timeout_time_))
    {
        event_out_msg_.data = "e_timeout";
        event_pub_.publish(event_out_msg_);
        current_state_ = INIT;
    }
    else
    {
        if (detectBackgroundChange())
        {
            event_out_msg_.data = "e_change";
            event_pub_.publish(event_out_msg_);
        }
        current_state_ = IDLE;
    }

    if (is_debug_mode_)
    {
        cv_bridge::CvImage debug_image_msg;
        debug_image_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_image_msg.image = debug_image_;
        image_pub_.publish(debug_image_msg.toImageMsg());
    }
}

bool BackgroundChangeDetectionNode::detectBackgroundChange()
{
    try
    {
        cv_bridge::CvImagePtr cv_img_tmp = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);
        cv::Mat current_frame = cv_img_tmp->image;
        if (is_first_pass_)
        {
            bcd_.initializeBackgroundModel(current_frame);
            is_first_pass_ = false;
        }
        if (bcd_.detectBackgroundChange(current_frame, debug_image_))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image_msg_->encoding.c_str());
        return false;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "background_change_detection");
    ros::NodeHandle nh("~");
    ROS_INFO("Background Change Detection Node Initialised");
    BackgroundChangeDetectionNode bcd(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        bcd.states();
        rate.sleep();
    }

    return 0;
}