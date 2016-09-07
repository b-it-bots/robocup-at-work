#include <mcr_image_filter/image_filter_node.h>

ImageFilterNode::ImageFilterNode(ros::NodeHandle &nh) :
    node_handler_(nh),
    image_transporter_(nh)
{
    dynamic_reconfig_server_.setCallback(boost::bind(&ImageFilterNode::dynamicReconfigCallback, this, _1, _2));
    event_sub_ = node_handler_.subscribe("event_in", 1, &ImageFilterNode::eventCallback, this);
    image_pub_ = image_transporter_.advertise("filtered_image", 1);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &ImageFilterNode::imageCallback, this);
    run_state_ = INIT;
    start_filter_image_ = false;
    image_sub_status_ = false;
}

ImageFilterNode::~ImageFilterNode()
{
    event_sub_.shutdown();
    image_pub_.shutdown();
    image_sub_.shutdown();
}

void ImageFilterNode::dynamicReconfigCallback(mcr_image_filter::ImageFilterConfig &config, uint32_t level)
{
    is_rotation_ = config.rotate_image;
    is_crop_ = config.crop_image;
    crop_factor_top_ = config.crop_factor_top;
    crop_factor_bottom_ = config.crop_factor_bottom;
    crop_factor_left_ = config.crop_factor_left;
    crop_factor_right_ = config.crop_factor_right;
}

void ImageFilterNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start")
    {
        start_filter_image_ = true;
        ROS_INFO("2D Image Filter ENABLED");
    }
    else if (event_command.data == "e_stop")
    {
        start_filter_image_ = false;
        ROS_INFO("2D Image Filter DISABLED");
    }
}

void ImageFilterNode::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    image_message_ = img_msg;
    image_sub_status_ = true;
}

void ImageFilterNode::states()
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

void ImageFilterNode::initState()
{
    if (image_sub_status_)
    {
        run_state_ = IDLE;
        image_sub_status_ = false;
    }
}

void ImageFilterNode::idleState()
{
    if (start_filter_image_)
    {
        run_state_ = RUNNING;
    }
    else
    {
        run_state_ = INIT;
    }
}

void ImageFilterNode::runState()
{
    filterImage();
    run_state_ = INIT;
}

void ImageFilterNode::filterImage()
{

    cv_bridge::CvImagePtr cv_img_ptr;

    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image_message_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_->encoding.c_str());
        return;
    }

    if (is_crop_)
    {
        Size cv_img_size = cv_img_ptr->image.size();
        int height = cv_img_size.height;
        int width = cv_img_size.width;
        Rect roi(width * crop_factor_left_, height * crop_factor_top_, (width - width * crop_factor_left_) - width * crop_factor_right_, (height - height * crop_factor_top_) - height * crop_factor_bottom_);
        cv_img_ptr->image = cv_img_ptr->image(roi);
    }

    if (is_rotation_)
    {
        flip(cv_img_ptr->image, cv_img_ptr->image, -1);
    }

    image_pub_.publish(cv_img_ptr->toImageMsg());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_filter");
    ros::NodeHandle nh("~");
    ROS_INFO("Image Filter Node Initialised");
    ImageFilterNode ifn(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ifn.states();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}