/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 *
 */

#include <force_field_recovery/force_field_recovery.h>
#include <string>
#include <limits>

// Register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(force_field_recovery, ForceFieldRecovery, force_field_recovery::ForceFieldRecovery,
                        nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace force_field_recovery
{
ForceFieldRecovery::ForceFieldRecovery(): global_costmap_(NULL), local_costmap_(NULL),
    tf_(NULL), initialized_(NULL), is_oscillation_detection_initialized_(false),
    previous_angle_(0.0), allowed_oscillations_(0), number_of_oscillations_(0)
{}

void ForceFieldRecovery::initialize(std::string name, tf::TransformListener* tf,
                                    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap)
{
    // initialization, this code will be executed only once
    if (!initialized_)
    {
        // receiving move_base variables and copying them over to class variables
        tf_ = tf;
        global_costmap_ = global_costmap;
        local_costmap_ = local_costmap;
        // robot_base_frame_ commonly can be base_footprint frame
        robot_base_frame_ = local_costmap_->getBaseFrameID();
        // robot_global_frame_ commonly can be map frame
        robot_global_frame_ = local_costmap_->getGlobalFrameID();

        ros::NodeHandle private_nh("~/" + name);

        ROS_INFO("Initializing Force field recovery behavior...");

        // Getting values from parameter server and storing into class variables
        private_nh.param("velocity_scale_factor", velocity_scale_, 0.6);
        private_nh.param("obstacle_neighborhood", obstacle_neighborhood_, 0.6);
        private_nh.param("max_velocity", max_velocity_, 0.3);
        private_nh.param("timeout", timeout_, 3.0);
        private_nh.param("update_frequency", recovery_behavior_update_frequency_, 5.0);
        private_nh.param("oscillation_angular_tolerance", oscillation_angular_tolerance_, 2.8);
        private_nh.param("allowed_oscillations", allowed_oscillations_, 0);

        // Inform user about which parameters will be used for the recovery behavior
        ROS_INFO("Force field recovery behavior parameters : ");
        ROS_INFO("Velocity_scale parameter : %lf", velocity_scale_);
        ROS_INFO("Obstacle_neighborhood parameter : %lf", obstacle_neighborhood_);
        ROS_INFO("Max_velocity parameter : %lf", max_velocity_);
        ROS_INFO("Timeout parameter : %lf", timeout_);
        ROS_INFO("Recovery_behavior_update_frequency_ parameter : %lf", recovery_behavior_update_frequency_);
        ROS_INFO("Oscillation_angular_tolerance parameter : %lf", oscillation_angular_tolerance_);
        ROS_INFO("Allowed_oscillations parameter : %d", allowed_oscillations_);
        ROS_INFO("Robot base reference frame : %s", robot_base_frame_.c_str());
        ROS_INFO("Robot global reference frame : %s", robot_global_frame_.c_str());

        // set up cmd_vel publisher
        pub_twist_ = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // set up marker publishers
        pub_neighbourhood_ = private_nh.advertise<visualization_msgs::Marker>("obstacle_neighborhood", 1);
        pub_ff_marker_ = private_nh.advertise<visualization_msgs::Marker>("force_field_vector", 1);

        // set up cloud publishers topic
        pub_obstacle_cloud_ = private_nh.advertise<sensor_msgs::PointCloud2> ("obstacle_cloud", 1);

        // setting initialized flag to true, preventing this code to be executed twice
        initialized_ = true;
    }
    else
    {
        ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
}

void ForceFieldRecovery::runBehavior()
{
    // preventing the use of this code before initialization
    if (!initialized_)
    {
        ROS_ERROR("This object must be initialized before runBehavior is called");
        return;
    }

    // checking if the received costmaps are empty, if so exit
    if (global_costmap_ == NULL || local_costmap_ == NULL)
    {
        ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
        return;
    }

    ROS_INFO("Running force field recovery behavior");

    // Moving base away from obstacles
    moveBaseAwayFromObstacles(local_costmap_);
}

void ForceFieldRecovery::moveBaseAwayFromObstacles(costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::Time start_time = ros::Time::now();

    bool no_obstacles_in_radius = false;
    bool timeout = false;
    double cmd_vel_x = 0.0;
    double cmd_vel_y = 0.0;

    costmap_2d::Costmap2D* costmap_snapshot;
    pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
    sensor_msgs::PointCloud2 global_frame_obstacle_cloud;
    pcl::PointCloud<pcl::PointXYZ> robot_frame_obstacle_cloud;
    Eigen::Vector3f force_field;

    ros::Rate loop_rate(recovery_behavior_update_frequency_);

    // reset allowed_oscillations_ on each recovery behavior call
    allowed_oscillations_ = 0;

    // reset number of oscillations_ on each recovery behavior call
    number_of_oscillations_ = 0;

    // reset oscilation initialization en each recovery behavior call
    is_oscillation_detection_initialized_ = false;

    // checkStoppingConditions(...) is the function in charge of breaking the loop
    while (!checkStoppingConditions(force_field, start_time))
    {
        // 1. getting a snapshot of the costmap
        costmap_snapshot = costmap_ros->getCostmap();

        // 2. convert obstacles inside costmap into pointcloud
        obstacle_cloud = costmapToPointcloud(costmap_snapshot);

        // 3. publish global frame obstacle cloud
        global_frame_obstacle_cloud = publishCloud(obstacle_cloud, pub_obstacle_cloud_, robot_global_frame_);

        // 4. change cloud to the reference frame of the robot
        robot_frame_obstacle_cloud = changeCloudReferenceFrame(global_frame_obstacle_cloud, robot_base_frame_);

        // 5. compute force field
        force_field = computeForceField(robot_frame_obstacle_cloud);

        // 6. move base in the direction of the force field
        cmd_vel_x = force_field(0) * velocity_scale_;
        cmd_vel_y = force_field(1) * velocity_scale_;
        publishVelocities(cmd_vel_x, cmd_vel_y);

        // 7. publish markers (neighbourhood and force field vector) for visualization purposes
        publishObstacleNeighborhood();
        publishForceField(force_field);

        // 8. control the frequency update for costmap update
        loop_rate.sleep();
    }

    // 9. stop the base
    publishVelocities(0.0, 0.0);
}

pcl::PointCloud<pcl::PointXYZ> ForceFieldRecovery::costmapToPointcloud(const costmap_2d::Costmap2D* costmap)
{
    // for storing and return the pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int x_size_ = costmap->getSizeInCellsX();
    int y_size_ = costmap->getSizeInCellsY();

    int current_cost = 0;

    // for transforming map to world coordinates
    double world_x;
    double world_y;

    for (int i = 0; i < x_size_ ; i++)
    {
        for (int j = 0; j < y_size_ ; j++)
        {
            // getting each cost
            current_cost = costmap->getCost(i, j);

            // if cell is occupied by obstacle then add the centroid of the cell to the cloud
            if (current_cost == LETHAL_COST)
            {
                // get world coordinates of current occupied cell
                costmap->mapToWorld(i, j, world_x, world_y);

                // adding occupied cell centroid coordinates to cloud
                cloud.push_back(pcl::PointXYZ(world_x, world_y, 0));
            }
        }
    }

    return cloud;
}

sensor_msgs::PointCloud2 ForceFieldRecovery::publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                          ros::Publisher &cloud_pub, std::string frame_id)
{
    ROS_DEBUG("Publishing obstacle cloud");

    // Print points of the cloud in terminal
    pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();

    int numPoints = 0;

    while (cloud_iterator != cloud.end())
    {
        ROS_DEBUG("cloud [%d] = %lf, %lf, %lf ", numPoints, cloud_iterator->x, cloud_iterator->y, cloud_iterator->z);
        ++cloud_iterator;
        numPoints++;
    }

    ROS_DEBUG("total number of points in the cloud = %d", numPoints);

    // Creating a pointcloud2 data type
    pcl::PCLPointCloud2 cloud2;

    // Converting normal cloud to pointcloud2 data type
    pcl::toPCLPointCloud2(cloud, cloud2);

    // declaring a ros pointcloud data type
    sensor_msgs::PointCloud2 ros_cloud;

    // converting pointcloud2 to ros pointcloud
    pcl_conversions::fromPCL(cloud2, ros_cloud);

    // assigning a frame to ros cloud
    ros_cloud.header.frame_id = frame_id;

    // publish the cloud
    cloud_pub.publish(ros_cloud);

    // returning the cloud, it could be useful for other components
    return ros_cloud;
}

pcl::PointCloud<pcl::PointXYZ> ForceFieldRecovery::changeCloudReferenceFrame(sensor_msgs::PointCloud2 &ros_cloud,
        std::string target_reference_frame)
{
    // declaring the target ros pcl data type
    sensor_msgs::PointCloud2 target_ros_pointcloud;

    // changing pointcloud reference frame

    // declaring normal PCL clouds (not ros related)
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_trans;

    // convert from rospcl to pcl
    pcl::fromROSMsg(ros_cloud, cloud_in);

    // STEP 1 Convert xb3 message to center_bumper frame (i think it is better this way)
    tf::StampedTransform transform;
    try
    {
        tf_->lookupTransform(target_reference_frame, ros_cloud.header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Transform point cloud
    pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);

    return cloud_trans;
}

Eigen::Vector3f ForceFieldRecovery::computeForceField(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    Eigen::Vector3f force_vector(0, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();
    int numPoints = 0;

    while (cloud_iterator != cloud.end())
    {
        Eigen::Vector3f each_point(cloud_iterator->x, cloud_iterator->y, 0);

        ROS_DEBUG("Norm of the point : %f", each_point.norm());

        if (each_point.norm() < obstacle_neighborhood_)
        {
            force_vector -= each_point;
            numPoints++;
        }

        ++cloud_iterator;
    }

    if (numPoints == 0)
    {
        // Cloud is empty

        return Eigen::Vector3f(0, 0, 0);
    }

    force_vector.normalize();
    force_vector = force_vector * 1.0;

    ROS_DEBUG("Force vector = (%lf, %lf)", force_vector(0), force_vector(1));

    return force_vector;
}

bool ForceFieldRecovery::checkStoppingConditions(Eigen::Vector3f &force_field,
        ros::Time start_time)
{
    // A. no more obstacles in neighbourhood
    if (force_field(0) == 0 && force_field(1) == 0)
    {
        // force field = 0, 0 : means we are done and away from costmap obstacles

        ROS_INFO("No more obstacles in radius");
        ROS_INFO("Force field recovery succesfull");

        // stop the recovery behavior
        return true;
    }
    // B. recovery behavior oscillation detection
    else if (detectOscillations(force_field))
    {
        // this means the robot is stucked in a small area, causing the force field
        // to go back and forward -> oscillating, therefore we need to stop the recovery

        ROS_WARN("Oscillation detected! , will stop now...");

        // stop the recovery behavior
        return true;
    }
    // C. recovery behavior timeout
    else if (ros::Duration(ros::Time::now() - start_time).toSec() > timeout_)
    {
        // timeout, recovery behavior has been executed for timeout seconds and still no success, then abort
        ROS_WARN("Force field recovery behavior time out exceeded");

        // stop the recovery behavior
        return true;
    }

    // continue the execution of the recovery behavior
    return false;
}

bool ForceFieldRecovery::detectOscillations(Eigen::Vector3f &force_field)
{
    // do not initialize nor compare if force_field is zero
    if (std::abs(force_field(0)) < std::numeric_limits<double>::epsilon() &&
        std::abs(force_field(1)) < std::numeric_limits<double>::epsilon())
        return false;

    double current_angle = 0.0;
    double angle_difference = 0.0;

    // do not check for oscillations the first time, since there is no previous force to compare with
    if (is_oscillation_detection_initialized_)
    {
        // get the new force field angle
        current_angle = atan2(force_field(1) , force_field(0));

        ROS_DEBUG("previous angle : %lf (deg)", previous_angle_ * 180.0 / 3.14159265);
        ROS_DEBUG("current angle : %lf (deg)", current_angle * 180.0 / 3.14159265);

        // compare the angles
        angle_difference = atan2(sin(current_angle - previous_angle_), cos(current_angle - previous_angle_));

        ROS_INFO("angle_difference : %lf (deg)", angle_difference * 180.0 / 3.14159265);

        // detect if the force field angle has an abrupt angular change
        if (fabs(angle_difference) > oscillation_angular_tolerance_)
        {
            ROS_INFO("Change in direction of the force field detected");
            number_of_oscillations_++;
        }

        // making backup of the previous force field angle
        previous_angle_ = current_angle;
    }
    else
    {
        // compute angle of the first force field
        previous_angle_ = atan2(force_field(1) , force_field(0));

        ROS_DEBUG("force field initial x component : %lf (vel)", force_field(0));
        ROS_DEBUG("force field initial y component : %lf (vel)", force_field(1));

        ROS_DEBUG("initial angle : %lf", previous_angle_ * 180.0 / 3.14159265);

        // starting from second time, check for oscillations
        is_oscillation_detection_initialized_ = true;
    }

    if (number_of_oscillations_ > allowed_oscillations_)
    {
        // more than "n" allowed oscillations have been detected on the force field
        return true;
    }

    // no more than "n" allowed oscillations were detected in the force field so far
    return false;
}

void ForceFieldRecovery::publishVelocities(double x, double y)
{
    if (x != 0 && y != 0)  // Do not print if x and y are zero
    {
        ROS_DEBUG("Moving base into the direction of the force field x = %lf, y = %lf", x, y);
    }

    geometry_msgs::Twist twist_msg;

    // clamping x and y to maximum speed value
    if (x > max_velocity_) x = max_velocity_;
    else if (x < -max_velocity_) x = -max_velocity_;

    if (y > max_velocity_) y = max_velocity_;
    else if (y < -max_velocity_) y = -max_velocity_;

    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    pub_twist_.publish(twist_msg);
}

void ForceFieldRecovery::publishForceField(Eigen::Vector3f force_field)
{
    // variable declaration
    double force_field_angle = 0.0;
    visualization_msgs::Marker ff_marker;

    // filling the required data for the marker
    ff_marker.header.frame_id = robot_base_frame_.c_str();
    ff_marker.header.stamp = ros::Time::now();
    ff_marker.ns = "force_field";
    ff_marker.id = 1;
    ff_marker.type = visualization_msgs::Marker::ARROW;
    ff_marker.action = visualization_msgs::Marker::ADD;

    // origin of the marker is 0, 0, 0 by default, therefore not setting it

    // computing force field yaw angle
    force_field_angle = atan2(force_field(1), force_field(0));

    // transforming force field angle from radians to quaternion by setting yaw value
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, force_field_angle);

    // assign orientation of the quaternion to the marker
    ff_marker.pose.orientation.x = q.x();
    ff_marker.pose.orientation.y = q.y();
    ff_marker.pose.orientation.z = q.z();
    ff_marker.pose.orientation.w = q.w();

    // duration of the ff_marker : 0 - forever
    ff_marker.lifetime = ros::Duration(2.0);

    ff_marker.scale.x = 0.6;
    ff_marker.scale.y = 0.1;
    ff_marker.scale.z = 0.1;

    // alpha (transparency level)
    ff_marker.color.a = 0.9;

    // rgb for orange/yellow
    ff_marker.color.r = 255.0;
    ff_marker.color.g = 153.0;
    ff_marker.color.b = 0.0;

    // publish the force field vector as marker
    pub_ff_marker_.publish(ff_marker);
}

void ForceFieldRecovery::publishObstacleNeighborhood()
{
    // declaring a marker object
    visualization_msgs::Marker neighbourhood_marker;

    // filling the required data for the marker
    neighbourhood_marker.header.frame_id = robot_base_frame_.c_str();
    neighbourhood_marker.header.stamp = ros::Time::now();
    neighbourhood_marker.ns = "force_field";
    neighbourhood_marker.id = 0;
    neighbourhood_marker.type = visualization_msgs::Marker::CYLINDER;
    neighbourhood_marker.action = visualization_msgs::Marker::ADD;

    // neighbourhood_marker.pose.position already 0 by default, therefore not setting it

    // duration of the marker : forever
    neighbourhood_marker.lifetime = ros::Duration(5.0);

    neighbourhood_marker.scale.x = obstacle_neighborhood_ * 2.0;
    neighbourhood_marker.scale.y = obstacle_neighborhood_ * 2.0;
    neighbourhood_marker.scale.z = 0.1;

    neighbourhood_marker.color.a = 0.5;  // alpha (transparency level)
    neighbourhood_marker.color.r = 0.0;
    neighbourhood_marker.color.g = 1.0;
    neighbourhood_marker.color.b = 0.0;

    // publish the neighbourhood as cylinder marker
    pub_neighbourhood_.publish(neighbourhood_marker);
}
};  // namespace force_field_recovery
