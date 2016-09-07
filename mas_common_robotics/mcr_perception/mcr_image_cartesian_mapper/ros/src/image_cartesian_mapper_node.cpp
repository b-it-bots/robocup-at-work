/**
 * @file image_cartesian_mapper_node.cpp
 * @author Ashok Meenakshi Sundaram (mashoksc@gmail.com)
 * @data June, 2015
 */

#include <mcr_image_cartesian_mapper/image_cartesian_mapper_node.h>

ImageCartesianMapperNode::ImageCartesianMapperNode(ros::NodeHandle &nh) : node_handler_(nh)
{
    event_sub_ = node_handler_.subscribe("event_in", 1, &ImageCartesianMapperNode::eventCallback, this);
    pose_sub_ = node_handler_.subscribe("pose", 1, &ImageCartesianMapperNode::poseCallback, this);

    cartesian_pub_ = node_handler_.advertise<geometry_msgs::PoseStamped>("cartesian_pose", 1);
    event_pub_ = node_handler_.advertise<std_msgs::String>("event_out", 1);

    node_handler_.getParam("camera_matrix/data", camera_intrinsic_list_);
    node_handler_.param<std::string>("target_frame", target_frame_, "base_link");
    node_handler_.param<std::string>("source_frame", source_frame_, "arm_cam3d_rgb_optical_frame");
    node_handler_.param<bool>("is_image_filter_enabled", is_image_filter_enabled_, false);
    node_handler_.param<bool>("crop_image", is_image_crop_enabled_, false);

    if (is_image_filter_enabled_ && is_image_crop_enabled_)
    {
        node_handler_.param<double>("image_width", image_width_, 640);
        node_handler_.param<double>("image_height", image_height_, 480);
        node_handler_.param<double>("crop_factor_top", crop_factor_top_, false);
        node_handler_.param<double>("crop_factor_left", crop_factor_left_, false);
    }

    current_state_ = INIT;
    has_pose_data_ = false;
}

ImageCartesianMapperNode::~ImageCartesianMapperNode()
{
    event_sub_.shutdown();
    pose_sub_.shutdown();
    cartesian_pub_.shutdown();
    event_pub_.shutdown();
}


void ImageCartesianMapperNode::eventCallback(const std_msgs::String &event_command)
{
    event_in_msg_ = event_command;
}

void ImageCartesianMapperNode::poseCallback(const geometry_msgs::Pose2D::ConstPtr &pose)
{
    pose_2d_ = *pose;
    has_pose_data_ = true;
}

void ImageCartesianMapperNode::states()
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

void ImageCartesianMapperNode::initState()
{
    if (event_in_msg_.data == "e_start")
    {
        current_state_ = IDLE;
        event_in_msg_.data == "";
        has_pose_data_ = false;
    }
    else
    {
        current_state_ = INIT;
    }
}

void ImageCartesianMapperNode::idleState()
{
    if (event_in_msg_.data == "e_stop")
    {
        current_state_ = INIT;
        event_in_msg_.data == "";
    }
    else if (has_pose_data_)
    {
        current_state_ = RUNNING;
        has_pose_data_ = false;
    }
    else
    {
        current_state_ = IDLE;
    }
}

void ImageCartesianMapperNode::runState()
{
    if (cameraOpticalToCameraMetrical() && cameraMetricalToCartesian())
    {
        cartesian_pub_.publish(cartesian_pose_);
        event_out_msg_.data = "e_done";
    }
    else
    {
        event_out_msg_.data = "e_error";
    }
    event_pub_.publish(event_out_msg_);
    current_state_ = IDLE;
}


bool ImageCartesianMapperNode::cameraOpticalToCameraMetrical()
{
    if (is_image_filter_enabled_ && is_image_crop_enabled_)
    {
        camera_coordinates_ << pose_2d_.x + image_width_*crop_factor_left_, pose_2d_.y + image_height_*crop_factor_top_, 1;
    }
    else
    {
        camera_coordinates_ << pose_2d_.x, pose_2d_.y, 1;
    }


    camera_intrinsic_matrix_ << camera_intrinsic_list_[0], camera_intrinsic_list_[1], camera_intrinsic_list_[2],
                             camera_intrinsic_list_[3], camera_intrinsic_list_[4], camera_intrinsic_list_[5],
                             camera_intrinsic_list_[6], camera_intrinsic_list_[7], camera_intrinsic_list_[8];

    camera_metrical_coordinates_ = camera_intrinsic_matrix_.inverse() * camera_coordinates_;

    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;

    point.x = camera_metrical_coordinates_(0, 0);
    point.y = camera_metrical_coordinates_(1, 0);
    point.z = camera_metrical_coordinates_(2, 0);
    quaternion = tf::createQuaternionMsgFromYaw(pose_2d_.theta * M_PI / 180);
    pose.position = point;
    pose.orientation = quaternion;

    camera_optical_pose_.header.stamp = ros::Time::now();
    camera_optical_pose_.header.frame_id = source_frame_;
    camera_optical_pose_.pose = pose;

    return true;
}


bool ImageCartesianMapperNode::cameraMetricalToCartesian()
{
    try
    {
        listener_.waitForTransform(target_frame_, camera_optical_pose_.header.frame_id, camera_optical_pose_.header.stamp, ros::Duration(3.0));
        listener_.transformPose(target_frame_, camera_optical_pose_, cartesian_pose_);
        return true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_cartesian_mapper");
    ros::NodeHandle nh("~");
    ROS_INFO("Initialised");
    ImageCartesianMapperNode icm(nh);

    int loop_rate = 30;
    nh.param<int>("loop_rate", loop_rate, 30);
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::spinOnce();
        icm.states();
        rate.sleep();
    }

    return 0;
}