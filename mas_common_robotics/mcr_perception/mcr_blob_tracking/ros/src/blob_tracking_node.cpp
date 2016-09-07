#include <mcr_blob_tracking/blob_tracking_node.h>

BlobTrackingNode::BlobTrackingNode(ros::NodeHandle &nh) :
    node_handler_(nh),
    image_transporter_(nh),
    run_state_(INIT),
    start_blob_tracking_(false),
    image_sub_status_(false),
    blobs_sub_status_(false),
    first_pass_(true),
    blob_tracked_index_(0),
    blob_distance_threshold_(800),
    tracking_type_("First Largest"),
    debug_mode_(true)

{
    dynamic_reconfig_server_.setCallback(boost::bind(&BlobTrackingNode::dynamicReconfigCallback, this, _1, _2));
    event_sub_ = node_handler_.subscribe("event_in", 1, &BlobTrackingNode::eventCallback, this);
    blobs_sub_ = node_handler_.subscribe("blobs", 1, &BlobTrackingNode::blobsCallback, this);
    image_sub_ = image_transporter_.subscribe("input_image", 1, &BlobTrackingNode::imageCallback, this);
    blob_pose_pub_ = node_handler_.advertise<geometry_msgs::Pose2D>("blob_pose", 1);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);
    image_pub_ = image_transporter_.advertise("debug_image", 1);
    event_out_msg_.data = "";

}

BlobTrackingNode::~BlobTrackingNode()
{
    image_sub_.shutdown();
    event_sub_.shutdown();
    blobs_sub_.shutdown();
    blob_pose_pub_.shutdown();
    image_pub_.shutdown();
    event_pub_.shutdown();
}

void BlobTrackingNode::dynamicReconfigCallback(mcr_blob_tracking::BlobTrackingConfig &config, uint32_t level)
{
    debug_mode_ = config.debug_mode;
    tracking_type_ = config.tracking_type;
    blob_distance_threshold_ = config.blob_distance_threshold;
}

void BlobTrackingNode::eventCallback(const std_msgs::String &event_command)
{
    if (event_command.data == "e_start")
    {
        start_blob_tracking_ = true;
        ROS_INFO("2D Blob Tracking ENABLED");
    }
    else if (event_command.data == "e_stop")
    {
        start_blob_tracking_ = false;
        ROS_INFO("2D Blob Tracking DISABLED");
    }
}

void BlobTrackingNode::blobsCallback(const mcr_perception_msgs::BlobList::ConstPtr &blobs)
{
    blob_list_ = *blobs;
    blobs_sub_status_ = true;;
}

void BlobTrackingNode::imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    image_message_ = img_msg;
    image_sub_status_ = true;
}

void BlobTrackingNode::states()
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

void BlobTrackingNode::initState()
{
    if (blobs_sub_status_)
    {
        run_state_ = IDLE;
        blobs_sub_status_ = false;
    }
}

void BlobTrackingNode::idleState()
{
    if (start_blob_tracking_)
    {
        run_state_ = RUNNING;
    }
    else
    {
        run_state_ = INIT;
    }
}

void BlobTrackingNode::runState()
{
    trackBlob();
    run_state_ = INIT;
}

void BlobTrackingNode::trackBlob()
{

    cv_bridge::CvImagePtr cv_img_ptr;
    if (debug_mode_ && image_sub_status_)
    {
        try
        {
            cv_img_ptr = cv_bridge::toCvCopy(image_message_, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_->encoding.c_str());
        }
    }

    event_out_msg_.data = "";

    if (blob_list_.blobs.size() > 0)
    {

        blobs_.resize(blob_list_.blobs.size());

        for (int i = 0; i < blob_list_.blobs.size(); i++)
        {

            blobs_[i].resize(4);
            blobs_[i][0] = blob_list_.blobs.at(i).blob_pose.x;
            blobs_[i][1] = blob_list_.blobs.at(i).blob_pose.y;
            blobs_[i][2] = blob_list_.blobs.at(i).blob_pose.theta;
            blobs_[i][3] = blob_list_.blobs.at(i).blob_area;

        }

        if (tracking_type_.compare("First Largest") == 0)
        {
            trackFirstLargesBlob();
        }

        if (event_out_msg_.data.compare("e_blob_lost") == 0)
        {
            event_pub_.publish(event_out_msg_);
        }
        else
        {
            blob_pose_pub_.publish(blob_tracked_pose_);
            if (debug_mode_ && image_sub_status_)
            {
                Size cv_img_size = cv_img_ptr->image.size();
                int height = cv_img_size.height;
                int width = cv_img_size.width;
                line(cv_img_ptr->image, Point(0, height / 2), Point(width, height / 2), CV_RGB(0, 0, 255), 2);
                line(cv_img_ptr->image, Point(width / 2, 0), Point(width / 2, height), CV_RGB(0, 0, 255), 2);
                circle(cv_img_ptr->image, Point(blob_tracked_pose_.x, blob_tracked_pose_.y), 8, CV_RGB(255, 0, 0), 2);
                line(cv_img_ptr->image, Point(blob_tracked_pose_.x + cos(blob_tracked_pose_.theta * M_PI / 180) * 40, blob_tracked_pose_.y - sin(blob_tracked_pose_.theta * M_PI / 180) * 40), Point(blob_tracked_pose_.x - cos(blob_tracked_pose_.theta * M_PI / 180) * 40, blob_tracked_pose_.y + sin(blob_tracked_pose_.theta * M_PI / 180) * 40), CV_RGB(255, 0, 0), 2);
                line(cv_img_ptr->image, Point(blob_tracked_pose_.x - cos(blob_tracked_pose_.theta * M_PI / 180) * 40, blob_tracked_pose_.y - sin(blob_tracked_pose_.theta * M_PI / 180) * 40), Point(blob_tracked_pose_.x + cos(blob_tracked_pose_.theta * M_PI / 180) * 40, blob_tracked_pose_.y + sin(blob_tracked_pose_.theta * M_PI / 180) * 40), CV_RGB(255, 0, 0), 2);
            }
        }

    }
    else
    {
        event_out_msg_.data = "e_blob_lost";
        event_pub_.publish(event_out_msg_);
    }

    if (debug_mode_ && image_sub_status_)
    {
        image_pub_.publish(cv_img_ptr->toImageMsg());
    }

    image_sub_status_ = false;

}

void BlobTrackingNode::trackFirstLargesBlob()
{

    if (first_pass_)
    {

        double largest_blob_area = 0;
        int largest_blob_index = 0;

        for (int i = 0; i < blobs_.size(); i++)
        {
            if (blobs_.at(i).at(3) > largest_blob_area)
            {
                largest_blob_area = blobs_.at(i).at(3);
                largest_blob_index = i;
            }
        }

        blob_tracked_pose_.x = blobs_.at(largest_blob_index).at(0);
        blob_tracked_pose_.y = blobs_.at(largest_blob_index).at(1);
        blob_tracked_pose_.theta = blobs_.at(largest_blob_index).at(2);
        first_pass_ = false;

    }

    vector<double> distance_to_tracked_blob;
    distance_to_tracked_blob.resize(blobs_.size());

    for (int i = 0; i < blobs_.size(); i++)
    {
        double dist_x = blobs_.at(i).at(0) - blob_tracked_pose_.x;
        double dist_y = blobs_.at(i).at(1) - blob_tracked_pose_.y;
        distance_to_tracked_blob[i] = sqrt((dist_x * dist_x) + (dist_y * dist_y));
    }

    double minimum_blob_distance = distance_to_tracked_blob.at(0);
    blob_tracked_index_ = 0;

    for (int i = 0; i < blobs_.size(); i++)
    {
        if (distance_to_tracked_blob.at(i) < minimum_blob_distance)
        {
            minimum_blob_distance = distance_to_tracked_blob.at(i);
            blob_tracked_index_ = i;
        }
    }

    if (minimum_blob_distance < blob_distance_threshold_)
    {
        blob_tracked_pose_.x = blobs_.at(blob_tracked_index_).at(0);
        blob_tracked_pose_.y = blobs_.at(blob_tracked_index_).at(1);
        blob_tracked_pose_.theta = blobs_.at(blob_tracked_index_).at(2);
    }
    else
    {
        event_out_msg_.data = "e_blob_lost";
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_tracking");
    ros::NodeHandle nh("~");
    ROS_INFO("Blob Tracking Node Initialised");
    BlobTrackingNode bt(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        bt.states();
        rate.sleep();
    }

    return 0;
}