/*********************************************************************
 * Software License Agreement (GPLv3 License)
 *
 *  Copyright (c) 2015, Hochschule Bonn-Rhein-Sieg.
 *  All rights reserved.
 *
 *********************************************************************/
/**
 * Author: Frederik Hegger
 */

#include <mcr_collision_velocity_filter/collision_velocity_filter_node.h>
#include <vector>
#include <string>

CollisionVelocityFilterNode::CollisionVelocityFilterNode() :
        footprint_msg_received(false), laser_scans_as_pcl_cloud_received(false), desired_twist_msg_received_(false),
        debug_mode_(false)
{
    std::string footprint_parameter_name = "footprint";
    std::string footprint_topic_name = "footprint";
    std::vector<std::string> scan_topics;

    ros::NodeHandle nh("~");

    nh.param<std::string>("target_frame", target_frame_, "/base_link");

    zero_velocities_.linear.x = zero_velocities_.linear.y = zero_velocities_.linear.z = 0.0;
    zero_velocities_.angular.x = zero_velocities_.angular.y = zero_velocities_.angular.z = 0.0;

    sub_footprint_ = nh.subscribe < geometry_msgs::PolygonStamped > (footprint_topic_name,
        1, &CollisionVelocityFilterNode::footprintCallback, this);

    collision_velocity_filter_ = new CollisionVelocityFilter();

    // try to read parameter from parameter server
    ROS_INFO("Reading parameter from parameter server ...");
    if (!getFootprintFromParameterServer(footprint_parameter_name))
    {
        ROS_WARN_STREAM("Could not read parameter <" << footprint_parameter_name
            << "> from the parameter server. Try to receive it from the specified topic ...");
        getFootprintFromTopic(footprint_topic_name);
    }
    ROS_INFO("Got footprint!");

    // Subscriber
    sub_twist_ = nh.subscribe < geometry_msgs::Twist > ("cmd_vel_in", 1,
        &CollisionVelocityFilterNode::twistCommadCallback, this);

    // read parameters
    scan_topics = readScanTopicsFromParameterServer();

    nh.param<bool>("debug_mode", debug_mode_, false);

    double soft_padding_front_rear, soft_padding_left_right;
    nh.param<double>("soft_padding_front_rear", soft_padding_front_rear, 0.15);
    nh.param<double>("soft_padding_left_right", soft_padding_left_right, 0.15);
    collision_velocity_filter_->setSoftPaddingParameter(soft_padding_front_rear, soft_padding_left_right);

    double hard_padding_front_rear, hard_padding_left_right;
    nh.param<double>("hard_padding_front_rear", hard_padding_front_rear, 0.02);
    nh.param<double>("hard_padding_left_right", hard_padding_left_right, 0.06);
    collision_velocity_filter_->setHardPaddingParameter(hard_padding_front_rear, hard_padding_left_right);

    double linear_velocity_in_soft_padding, angular_velocity_in_soft_padding;
    nh.param<double>("linear_velocity_in_soft_padding", linear_velocity_in_soft_padding, 0.02);
    nh.param<double>("angular_velocity_in_soft_padding", angular_velocity_in_soft_padding, 0.2);
    collision_velocity_filter_->setVelocitiesInSoftPadding(linear_velocity_in_soft_padding,
        angular_velocity_in_soft_padding);

    double angle_tolerance;
    nh.param<double>("angle_tolerance", angle_tolerance, 2.0);
    collision_velocity_filter_->setAngleTolerance(angle_tolerance);

    double min_linear_velocity;
    nh.param<double>("min_linear_velocity", min_linear_velocity, 0.02);
    collision_velocity_filter_->setMinimumLinearVelocity(min_linear_velocity);

    if (scan_topics.size() == 0)
    {
        ROS_ERROR("No scan topics specified. Exiting ...");
        exit(0);
    }
    else if (scan_topics.size() == 1)
    {
        ROS_INFO_STREAM("Subscribing to one scan topic: " << scan_topics[0]);
        sub_single_scan_ = nh.subscribe < sensor_msgs::LaserScan > (scan_topics[0], 10,
            &CollisionVelocityFilterNode::oneLaserScanCallback, this);
    }
    else
    {
        ROS_INFO("Subscribing to multiple scan topics: ");
        for (size_t i = 0; i < scan_topics.size(); ++i)
            ROS_INFO_STREAM("    " << scan_topics[i]);

        if (scan_topics.size() == 2)
        {
            sub_scan_1.subscribe(nh, scan_topics[0], 10);
            sub_scan_2.subscribe(nh, scan_topics[1], 10);

            two_synced_laser_scans_ = boost::make_shared<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
                sensor_msgs::LaserScan> > >(3);
            two_synced_laser_scans_->connectInput(sub_scan_1, sub_scan_2);
            two_synced_laser_scans_->registerCallback(boost::bind(
                &CollisionVelocityFilterNode::twoSynchronizedLaserscanCallback, this, _1, _2));
        }
        else if (scan_topics.size() == 3)
        {
            sub_scan_1.subscribe(nh, scan_topics[0], 10);
            sub_scan_2.subscribe(nh, scan_topics[1], 10);
            sub_scan_3.subscribe(nh, scan_topics[0], 10);

            three_synced_laser_scans_ = boost::make_shared<Synchronizer<ApproximateTime<sensor_msgs::LaserScan,
                sensor_msgs::LaserScan, sensor_msgs::LaserScan> > >(3);
            three_synced_laser_scans_->connectInput(sub_scan_1, sub_scan_2, sub_scan_3);
            three_synced_laser_scans_->registerCallback(boost::bind(
                &CollisionVelocityFilterNode::threeSynchronizedLaserscanCallback, this, _1, _2, _3));
        }
        else
        {
            ROS_ERROR_STREAM("The number of subscribed scan topics is not supported");
            exit(0);
        }
    }

    // Publisher
    pub_safe_twist_ = nh.advertise < geometry_msgs::Twist > ("cmd_vel_out", 1);
    pub_event_ = nh.advertise < std_msgs::String > ("event_out", 1);

    // Debug publisher
    if (debug_mode_)
    {
        pub_real_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("real_padding_footprint", 1, true);
        pub_soft_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("soft_padding_footprint", 1, true);
        pub_hard_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("hard_padding_footprint", 1, true);
        pub_front_right_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("front_right_footprint", 1, true);
        pub_front_left_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("front_left_footprint", 1, true);
        pub_rear_right_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("rear_right_footprint", 1, true);
        pub_rear_left_footprint_ = nh.advertise < geometry_msgs::PolygonStamped > ("rear_left_footprint", 1, true);
    }
}

CollisionVelocityFilterNode::~CollisionVelocityFilterNode()
{
    if (debug_mode_)
    {
        pub_hard_footprint_.shutdown();
        pub_soft_footprint_.shutdown();
        pub_real_footprint_.shutdown();
        pub_front_right_footprint_.shutdown();
        pub_front_left_footprint_.shutdown();
        pub_rear_right_footprint_.shutdown();
        pub_rear_left_footprint_.shutdown();
    }

    pub_event_.shutdown();
    pub_safe_twist_.shutdown();
    sub_twist_.shutdown();
    sub_footprint_.shutdown();
}

std::vector<std::string> CollisionVelocityFilterNode::readScanTopicsFromParameterServer()
{
    std::vector<std::string> scan_topics;
    ros::NodeHandle nh("~");

    if (!nh.getParam("scan_topics", scan_topics))
        ROS_ERROR_STREAM("No scan topics specified on the parameter server");

    return scan_topics;
}

bool CollisionVelocityFilterNode::getFootprintFromParameterServer(const std::string &parameter_name)
{
    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue footprint_param_list;
    geometry_msgs::PolygonStamped footprint;

    if (!nh.getParam(parameter_name, footprint_param_list))
        return false;

    footprint.header.frame_id = CollisionVelocityFilterNode::target_frame_;
    footprint.header.stamp = ros::Time::now();

    ROS_ASSERT(footprint_param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < footprint_param_list.size(); ++i)
    {
        ROS_ASSERT(footprint_param_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(footprint_param_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(footprint_param_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        geometry_msgs::Point32 point;
        point.x = static_cast<double>(footprint_param_list[i][0]);
        point.y = static_cast<double>(footprint_param_list[i][1]);

        footprint.polygon.points.push_back(point);
    }

    collision_velocity_filter_->updateRealFootprint(footprint);

    return true;
}

bool CollisionVelocityFilterNode::getFootprintFromTopic(const std::string &topic_name)
{
    footprint_msg_received = false;

    ros::Rate loop_rate(10);
    while (ros::ok() && !footprint_msg_received)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    collision_velocity_filter_->updateRealFootprint(footprint_msg_);

    footprint_msg_received = false;
}

void CollisionVelocityFilterNode::oneLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    fromROSMsg(getCloudFromLaserScan(*scan), laser_scans_as_pcl_cloud);

    laser_scans_as_pcl_cloud_received = true;
}
void CollisionVelocityFilterNode::twoSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
    const sensor_msgs::LaserScan::ConstPtr &scan_2)
{
    std::vector<sensor_msgs::LaserScan> scans;

    scans.push_back(*scan_1);
    scans.push_back(*scan_2);

    accumulateLaserScansToPointCloud(scans);
}

void CollisionVelocityFilterNode::threeSynchronizedLaserscanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_1,
    const sensor_msgs::LaserScan::ConstPtr &scan_2,
        const sensor_msgs::LaserScan::ConstPtr &scan_3)
{
    std::vector<sensor_msgs::LaserScan> scans;

    scans.push_back(*scan_1);
    scans.push_back(*scan_2);
    scans.push_back(*scan_3);

    accumulateLaserScansToPointCloud(scans);
}

void CollisionVelocityFilterNode::accumulateLaserScansToPointCloud(const std::vector<sensor_msgs::LaserScan> &scans)
{
    sensor_msgs::PointCloud2 accumulated_scans, scan_as_cloud;

    for (size_t i = 0; i < scans.size(); ++i)
    {
        // get laser scan as pcl pointcloud
        scan_as_cloud = getCloudFromLaserScan(scans[i]);

        // concatenate both clouds
        pcl::concatenatePointCloud(accumulated_scans, scan_as_cloud, accumulated_scans);
    }

    pcl::fromROSMsg(accumulated_scans, laser_scans_as_pcl_cloud);

    laser_scans_as_pcl_cloud_received = true;
}

sensor_msgs::PointCloud2 CollisionVelocityFilterNode::getCloudFromLaserScan(const sensor_msgs::LaserScan &scan)
{
    sensor_msgs::PointCloud2 cloud;
    try
    {
        transform_listener_.waitForTransform(scan.header.frame_id, target_frame_, scan.header.stamp,
            ros::Duration(0.1));
        laser_projector_.transformLaserScanToPointCloud(target_frame_, scan, cloud, transform_listener_);
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("Could not transform laser scan into target frame: " << e.what());
    }

    return cloud;
}

void CollisionVelocityFilterNode::footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
    footprint_msg_ = *msg;
    footprint_msg_received = true;
}

void CollisionVelocityFilterNode::twistCommadCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    desired_twist_msg_ = *msg;
    desired_twist_msg_received_ = true;
}

void CollisionVelocityFilterNode::update()
{
    if (desired_twist_msg_received_ && laser_scans_as_pcl_cloud_received)
    {
        safe_twist_ = collision_velocity_filter_->calculateSafeBaseVelocities(desired_twist_msg_,
            laser_scans_as_pcl_cloud);

        if (debug_mode_)
        {
            pub_real_footprint_.publish(collision_velocity_filter_->getRealFootprint());
            pub_soft_footprint_.publish(collision_velocity_filter_->getSoftPaddingFootprint());
            pub_hard_footprint_.publish(collision_velocity_filter_->getHardPaddingFootprint());
            pub_front_right_footprint_.publish(collision_velocity_filter_->getFrontRightFootprint());
            pub_front_left_footprint_.publish(collision_velocity_filter_->getFrontLeftFootprint());
            pub_rear_right_footprint_.publish(collision_velocity_filter_->getRearRightFootprint());
            pub_rear_left_footprint_.publish(collision_velocity_filter_->getRearLeftFootprint());
        }

        // publish safe velocities
        pub_safe_twist_.publish(safe_twist_);

        // publish events
        double sum_desired = fabs(desired_twist_msg_.linear.x) + fabs(desired_twist_msg_.linear.y)
            + fabs(desired_twist_msg_.angular.z);
        double sum_safe = fabs(safe_twist_.linear.x) + fabs(safe_twist_.linear.y) + fabs(safe_twist_.angular.z);

        if (sum_desired > 0.0 && sum_safe == 0.0)
        {
            event_out_.data = "e_zero_velocities_forwarded";
            pub_event_.publish(event_out_);
        }
        else if (sum_desired != sum_safe)
        {
            event_out_.data = "e_reduced_velocities_forwarded";
            pub_event_.publish(event_out_);
        }

        laser_scans_as_pcl_cloud_received = false;
        desired_twist_msg_received_ = false;
    }
}

int main(int argc, char **argv)
{
    // init ROS node
    ros::init(argc, argv, "collision_velocity_filter");

    CollisionVelocityFilterNode velocity_filter_node;

    ros::Rate loop_rate(100);

    ROS_INFO("Node started");
    while (ros::ok())
    {
        ros::spinOnce();

        velocity_filter_node.update();

        loop_rate.sleep();
    }
}
