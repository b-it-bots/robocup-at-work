/* Copyright [2013] <Bonn-Rhein-Sieg University>
 *
 * body_detection_3d_node.cpp
 *
 *  Created on: 22.08.2010
 *      Author: Frederik Hegger
 */

#include <boost/random.hpp>
#include <Eigen/StdVector>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include <mcr_perception_msgs/GetPersonList.h>

#include <mcr_algorithms/geometry/conversions.h>

#include "mcr_body_detection_3d/BodyDetection3DConfig.h"
#include "mcr_body_detection_3d/impl/body_detection_3d.hpp"

ros::Subscriber sub_pointcloud2;

ros::Publisher pub_person_msg;
ros::Publisher pub_segmented_cloud;
ros::Publisher pub_visualization_marker;

sensor_msgs::PointCloud2 debug_cloud;

ros::NodeHandle* nh_ptr = NULL;
BodyDetection3D* body_detector = NULL;
tf::TransformListener *transform_listener = NULL;
bool is_pointcloud_received = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
vector<Person> person_list;
mcr_perception_msgs::PersonList ros_msg;
mcr_body_detection_3d::BodyDetection3DConfig dyn_recfg_parameters;


void publishVisualizationMarker(const mcr_perception_msgs::PersonList &person_list)
{
    visualization_msgs::MarkerArray marker_array;

    // convert person msgs into markers for rviz visualization
    for (unsigned int i = 0, j = 0; i < person_list.persons.size(); ++i)
    {
        visualization_msgs::Marker marker_text;
        visualization_msgs::Marker marker_shape;

        marker_shape.header.frame_id = marker_text.header.frame_id = person_list.persons[i].pose.header.frame_id;
        marker_shape.header.stamp = marker_text.header.stamp = ros::Time::now();
        marker_shape.ns = "3D body detections - pose";
        marker_text.ns = "3D body detections - probability";
        marker_shape.action = marker_text.action = visualization_msgs::Marker::ADD;

        marker_shape.id = ++j;
        marker_text.id = ++j;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_shape.type = visualization_msgs::Marker::CUBE;

        marker_shape.pose.position = marker_text.pose.position = person_list.persons[i].pose.pose.position;
        marker_shape.pose.orientation = marker_text.pose.orientation = person_list.persons[i].pose.pose.orientation;

        std::ostringstream s;
        if (person_list.persons[i].probability > 0)
            s << ceilf(person_list.persons[i].probability * 100) / 100;
        marker_text.text = s.str();

        marker_shape.scale.x = person_list.persons[i].depth;
        marker_shape.scale.y = person_list.persons[i].width;
        marker_shape.scale.z = person_list.persons[i].height;
        marker_shape.color.r = 1.0;
        marker_shape.color.g = 0.0;
        marker_shape.color.b = 0.0;
        marker_shape.color.a = 0.5;;

        marker_text.scale.x = 0.2;
        marker_text.scale.y = 0.2;
        marker_text.scale.z = 0.2;
        marker_text.color.r = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.b = 1.0;
        marker_text.color.a = 1.0;

        marker_shape.lifetime = marker_text.lifetime = ros::Duration(1);

        marker_array.markers.push_back(marker_shape);
        marker_array.markers.push_back(marker_text);
    }

    if (marker_array.markers.size() > 0)
        pub_visualization_marker.publish(marker_array);
}

mcr_perception_msgs::PersonList convertToRosMsg(const vector<Person> &person_list, const pcl::PCLHeader &pcl_header)
{
    mcr_perception_msgs::PersonList ros_msg;
    geometry_msgs::Quaternion quaternion;

    for (size_t i = 0; i < person_list.size(); ++i)
    {
        mcr_perception_msgs::Person person;

        pcl_conversions::fromPCL(pcl_header, person.header);

        person.pose.header = person.header;
        person.pose.pose.position.x = person_list[i].position_x;
        person.pose.pose.position.y = person_list[i].position_y;
        person.pose.pose.position.z = person_list[i].position_z;

        quaternion = tf::createQuaternionMsgFromYaw(person_list[i].orientation_yaw);
        person.pose.pose.orientation = quaternion;

        person.height = person_list[i].height;
        person.width = person_list[i].width;
        person.depth = person_list[i].depth;
        person.probability = person_list[i].probability;

        person.id = 0;
        person.is_occluded = false;
        person.is_tracked = false;

        ros_msg.persons.push_back(person);
    }

    return ros_msg;
}

void pointcloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud2_input)
{
    sensor_msgs::PointCloud2 cloud2_transformed;

    try
    {
        if (cloud2_input->header.frame_id != dyn_recfg_parameters.target_frame)
        {
            // transform point cloud to base link
            transform_listener->waitForTransform(dyn_recfg_parameters.target_frame, cloud2_input->header.frame_id,
                                                 cloud2_input->header.stamp, ros::Duration(1.0));
            pcl_ros::transformPointCloud(dyn_recfg_parameters.target_frame, *cloud2_input, cloud2_transformed,
                                         *transform_listener);
        }
        else
            cloud2_transformed = *cloud2_input;

        if (cloud2_transformed.width <= 0 || cloud2_transformed.height <= 0)
        {
            ROS_ERROR("pointcloud is empty");
            return;
        }
        pcl::fromROSMsg(cloud2_transformed, *pcl_cloud_input);

        // detect person
        person_list = body_detector->getPersonList(pcl_cloud_input);

        // convert
        ros_msg = convertToRosMsg(person_list, pcl_cloud_input->header);

        if (ros_msg.persons.size() > 0)
        {
            if (dyn_recfg_parameters.publish_visualization_markers)
                publishVisualizationMarker(ros_msg);
            if (dyn_recfg_parameters.publish_debug_topics)
            {
                pcl::toROSMsg(body_detector->getClassifiedCloud(), debug_cloud);
                pub_segmented_cloud.publish(debug_cloud);
            }

            // publish final person list
            pub_person_msg.publish(ros_msg);
        }

        ROS_DEBUG_STREAM("found " << person_list.size() << " person(s)");
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("pointcloud transformation failed: %s", ex.what());
        return;
    }
}

void dynamic_reconfig_callback(mcr_body_detection_3d::BodyDetection3DConfig &config, uint32_t level)
{
    body_detector->setMinimumClustersPerPerson(config.minimum_clusters_per_person);
    dyn_recfg_parameters = config;
}

void eventCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "e_start")
    {
        sub_pointcloud2 = nh_ptr->subscribe<sensor_msgs::PointCloud2> ("pointcloud_xyzrgb", 1, pointcloud2Callback);
        ROS_INFO("3D body detector ENABLED");
    }
    else if (msg->data == "e_stop")
    {
        sub_pointcloud2.shutdown();
        ROS_INFO("3D body detector DISABLED");
    }
}

int main(int argc, char** argv)
{
    std::string random_forest_model_filename = "";

    ros::init(argc, argv, "body_detection_3d");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    // get Parameter from server
    if (nh.getParam("model_filename", random_forest_model_filename) == false)
        ROS_WARN("\tparameter \"model_filename\" not specified in launch file, used default value: %s",
                 random_forest_model_filename.c_str());

    // People Detector
    body_detector = new BodyDetection3D();
    body_detector->loadModel(random_forest_model_filename);

    // dynamic reconfigure server
    dynamic_reconfigure::Server<mcr_body_detection_3d::BodyDetection3DConfig> dynamic_reconfig_server;
    dynamic_reconfig_server.setCallback(boost::bind(&dynamic_reconfig_callback, _1, _2));

    // Subscriber and Publisher
    ros::Subscriber sub_event = nh.subscribe<std_msgs::String>("event_in", 1, eventCallback);

    pub_person_msg = nh.advertise<mcr_perception_msgs::PersonList>("people_positions", 1, true);
    pub_segmented_cloud = nh.advertise<sensor_msgs::PointCloud2>("debug/segmented_cloud", 1, true);
    pub_visualization_marker = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

    // TF
    transform_listener = new tf::TransformListener();

    ROS_INFO("node successfully initialized");

    ros::spin();

    if (body_detector != NULL)
        delete body_detector;
    if (transform_listener != NULL)
        delete transform_listener;

    return 0;
}
