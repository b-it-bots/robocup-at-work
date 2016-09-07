/*
 * nearest_object_detector_node.cpp
 *
 *  Created on: Mar 23, 2012
 *      Author: Frederik Hegger
 */

#include <math.h>
#include <float.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <laser_filters/angular_bounds_filter.h>

#include <mcr_algorithms/segmentation/laserscan_segmentation.h>
#include <mcr_algorithms/geometry/conversions.h>
#include <mcr_algorithms/wrapper/pcl_wrapper.hpp>

#include <mcr_perception_msgs/GetNearestObject.h>
#include <mcr_perception_msgs/LaserScanSegmentList.h>


ros::NodeHandle* nh_ptr = NULL;
ros::Publisher pub_markers;
ros::Publisher pub_obj_pose;
ros::Subscriber sub_scan;

laser_filters::LaserScanAngularBoundsFilter* laser_filter = NULL;
LaserScanSegmentation* scan_segmentation = NULL;
sensor_msgs::LaserScanConstPtr laser_scan;
geometry_msgs::PoseStamped last_observed_pose;

bool continues_mode_enabled = false;
bool subscribed_to_topic = false;
bool publish_marker = false;
bool scan_received = false;

geometry_msgs::PoseStamped calculateNearestObject(sensor_msgs::LaserScanConstPtr scan)
{
    geometry_msgs::PoseStamped observd_obj;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pcl_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    pcl_scan->header = pcl_conversions::toPCL(scan->header);
    pcl_with_normals->header = pcl_conversions::toPCL(scan->header);

    //reduce scan to a certain angle, see params
    sensor_msgs::LaserScanPtr scan_filtered(new sensor_msgs::LaserScan);
    laser_filter->update(*scan, *scan_filtered);

    mcr_perception_msgs::LaserScanSegmentList segment_list = scan_segmentation->getSegments(scan_filtered, true);

    // find nearest segment in the scan
    double min_distance = DBL_MAX, distance = 0, angle = 0;
    unsigned int min_segment_index = 0;
    for (unsigned int i = 0; i < segment_list.segments.size(); ++i)
    {
        Conversions::cartesian2polar2D(segment_list.segments[i].center.x, segment_list.segments[i].center.y, distance, angle);

        if (distance < min_distance)
        {
            min_distance = distance;
            min_segment_index = i;
        }
    }

    // calculate orientation of the segment
    // transform scan to point cloud (the same points is added three times in different heights because the normal estimation is only working then correctly)
    for (unsigned int i = 0; i < segment_list.segments[min_segment_index].data_points.size(); ++i)
    {
        pcl::PointXYZ p;

        p.x = segment_list.segments[min_segment_index].data_points[i].x;
        p.y = segment_list.segments[min_segment_index].data_points[i].y;
        p.z = 0.0;

        pcl_scan->points.push_back(p);

        p.x = segment_list.segments[min_segment_index].data_points[i].x;
        p.y = segment_list.segments[min_segment_index].data_points[i].y;
        p.z = 0.1;

        pcl_scan->points.push_back(p);

        p.x = segment_list.segments[min_segment_index].data_points[i].x;
        p.y = segment_list.segments[min_segment_index].data_points[i].y;
        p.z = 0.2;

        pcl_scan->points.push_back(p);
    }

    // calculate normals of the segment points
    PCLWrapper<pcl::PointXYZ>::computeNormals(pcl_scan, pcl_with_normals, 0.2);

    //make mean normal
    double normal_x = 0, normal_y = 0;
    for (unsigned int j = 0; j < pcl_with_normals->points.size(); ++j)
    {
        normal_x += pcl_with_normals->points[j].normal[0];
        normal_y += pcl_with_normals->points[j].normal[1];
    }

    normal_x /= pcl_with_normals->points.size();
    normal_y /= pcl_with_normals->points.size();

    //cout << "dist: " << min_distance << " angle: " << atan(normal_y / normal_x) << endl;

    observd_obj.header = scan->header;
    observd_obj.pose.position = segment_list.segments[min_segment_index].center;

    tf::Quaternion quat;
    quat = tf::createQuaternionFromYaw(atan(normal_y / normal_x));
    observd_obj.pose.orientation.x = quat[0];
    observd_obj.pose.orientation.y = quat[1];
    observd_obj.pose.orientation.z = quat[2];
    observd_obj.pose.orientation.w = quat[3];

    if (publish_marker)
    {
        visualization_msgs::MarkerArray marker_list;
        visualization_msgs::Marker marker_shape;
        visualization_msgs::Marker marker_orientation;

        marker_shape.header.frame_id = scan->header.frame_id;
        marker_shape.header.stamp = ros::Time::now();
        marker_orientation.header = marker_shape.header;

        marker_shape.ns = "nearest_object";
        marker_orientation.ns = marker_shape.ns;

        marker_shape.action = visualization_msgs::Marker::ADD;
        marker_orientation.action = marker_shape.action;

        marker_shape.id = 0;
        marker_orientation.id = 1;

        marker_orientation.type = visualization_msgs::Marker::ARROW;
        marker_shape.type = visualization_msgs::Marker::SPHERE;

        marker_shape.pose = observd_obj.pose;
        marker_orientation.pose = marker_shape.pose;

        marker_shape.scale.x = 0.1;
        marker_shape.scale.y = 0.1;
        marker_shape.scale.z = 0.1;
        marker_orientation.scale.x = 0.5;
        marker_orientation.scale.y = 0.5;
        marker_orientation.scale.z = 0.5;

        marker_shape.color.r = 1.0;
        marker_shape.color.g = 0.0;
        marker_shape.color.b = 0.0;
        marker_shape.color.a = 0.5;
        marker_orientation.color = marker_shape.color;

        marker_shape.lifetime = ros::Duration(1);
        marker_orientation.lifetime = marker_shape.lifetime;

        marker_list.markers.push_back(marker_orientation);
        marker_list.markers.push_back(marker_shape);
        pub_markers.publish(marker_list);
    }

    return observd_obj;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    laser_scan = scan_msg;

    if (continues_mode_enabled)
        last_observed_pose = calculateNearestObject(scan_msg);

    scan_received = true;

}

bool getNearestObject(mcr_perception_msgs::GetNearestObject::Request &req, mcr_perception_msgs::GetNearestObject::Response &res)
{
    // if not subscribe to laser scan topic, do it
    if (!subscribed_to_topic)
    {
        sub_scan = nh_ptr->subscribe < sensor_msgs::LaserScan > ("scan", 1, laserScanCallback);
        subscribed_to_topic = true;
    }

    // check if new laser scan data has arrived
    scan_received = false;
    while (!scan_received)
        ros::spinOnce();

    // calculate nearest object pose
    geometry_msgs::PoseStamped pose;
    pose = calculateNearestObject(laser_scan);

    res.pose = pose.pose;

    // unsubscribe from topic to save computational resources
    sub_scan.shutdown();
    subscribed_to_topic = false;

    return true;
}

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // if not subscribe to laser scan topic, do it
    if (!subscribed_to_topic)
    {
        sub_scan = nh_ptr->subscribe < sensor_msgs::LaserScan > ("scan", 1, laserScanCallback);
        subscribed_to_topic = true;
    }

    continues_mode_enabled = true;

    ROS_INFO("nearest object detector ENABLED");
    return true;
}

bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (subscribed_to_topic)
    {
        sub_scan.shutdown();
        subscribed_to_topic = false;
    }

    continues_mode_enabled = false;

    ROS_INFO("nearest object detector DISABLED");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mcr_nearest_object_detector");
    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    scan_segmentation = new LaserScanSegmentation(0.1, 3);
    laser_filter = new laser_filters::LaserScanAngularBoundsFilter();
    laser_filter->lower_angle_ = -(M_PI / 8);
    laser_filter->upper_angle_ = (M_PI / 8);

    // read params from parameter server file
    nh.getParam("publish_marker", publish_marker);
    ROS_INFO_STREAM("\tpublish marker: " << publish_marker);

    //either do a single call or ...
    ros::ServiceServer srv_nearest_obj = nh.advertiseService("get_nearest_object", getNearestObject);

    // ... let the node continusly publishing the nearest obj on a topic
    ros::ServiceServer srv_stop = nh.advertiseService("stop", stop);
    ros::ServiceServer srv_start = nh.advertiseService("start", start);

    //Publisher
    pub_obj_pose = nh.advertise < geometry_msgs::PoseStamped > ("nearest_object", 1);

    if (publish_marker)
        pub_markers = nh.advertise < visualization_msgs::MarkerArray > ("visualization_marker_array", 1);

    ROS_INFO("node successfully initialized");

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        ros::spinOnce();

        if (continues_mode_enabled)
        {
            if (scan_received)
            {
                pub_obj_pose.publish(last_observed_pose);
                scan_received = false;
            }

        }
        else
            loop_rate.sleep();
    }

    return 0;
}
