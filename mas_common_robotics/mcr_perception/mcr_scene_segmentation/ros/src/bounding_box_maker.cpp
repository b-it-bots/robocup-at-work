/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov, Santosh Thoduka
 *
 */

#include <mcr_scene_segmentation/bounding_box_maker.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>


void mcr_scene_segmentation::BoundingBoxMaker::onInit()
{
    nh_ = getPrivateNodeHandle();

    nh_.param<int>("max_queue_size", max_queue_size_, 3);
    nh_.param<bool>("approximate_sync", approximate_sync_, false);

    pub_bounding_boxes_ = nh_.advertise<mcr_perception_msgs::BoundingBoxList>("output", max_queue_size_);
    pub_pose_array_ = nh_.advertise<geometry_msgs::PoseArray>("output_poses", max_queue_size_);
    sub_clouds_filter_.subscribe(nh_, "input", max_queue_size_);
    sub_model_filter_.subscribe(nh_, "model", max_queue_size_);
    bounding_box_visualizer_ = boost::make_shared<mcr::visualization::BoundingBoxVisualizer>
                               (&nh_, "bounding_boxes",
                                mcr::visualization::Color(mcr::visualization::Color::SEA_GREEN));
    if (approximate_sync_)
    {
        sync_input_a_ = boost::make_shared<message_filters::Synchronizer
                        <message_filters::sync_policies::ApproximateTime
                        <mcr_perception_msgs::PointCloud2List, pcl_msgs::ModelCoefficients> > > (max_queue_size_);
        sync_input_a_->connectInput(sub_clouds_filter_, sub_model_filter_);
        sync_input_a_->registerCallback(
                       boost::bind(&mcr_scene_segmentation::BoundingBoxMaker::syncInputCallback, this, _1, _2));
    }
    else
    {
        sync_input_e_ = boost::make_shared<message_filters::Synchronizer
                        <message_filters::sync_policies::ExactTime
                        <mcr_perception_msgs::PointCloud2List, pcl_msgs::ModelCoefficients> > > (max_queue_size_);
        sync_input_e_->connectInput(sub_clouds_filter_, sub_model_filter_);
        sync_input_e_->registerCallback(
                       boost::bind(&mcr_scene_segmentation::BoundingBoxMaker::syncInputCallback, this, _1, _2));
    }
}

void mcr_scene_segmentation::BoundingBoxMaker::syncInputCallback(
        const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds,
        const pcl_msgs::ModelCoefficients::ConstPtr &model)
{
    if (pub_bounding_boxes_.getNumSubscribers() <= 0 && bounding_box_visualizer_->getNumSubscribers() <=0)
    {
        return;
    }

    if (clouds->pointclouds.empty() || model->values.empty())
    {
        NODELET_WARN("[%s::syncInputCallback] Clouds or model empty!", getName().c_str());
        return;
    }

    if (model->values.size() != 4)
    {
        NODELET_ERROR("[%s::syncInputCallback] Invalid model input!", getName().c_str());
        return;
    }

    const Eigen::Vector3f normal(model->values[0], model->values[1], model->values[2]);
    mcr_perception_msgs::BoundingBoxList bounding_boxes;
    bounding_boxes.bounding_boxes.resize(clouds->pointclouds.size());
    geometry_msgs::PoseArray pose_array;
    pose_array.poses.resize(clouds->pointclouds.size());
    for (size_t i = 0; i < clouds->pointclouds.size(); i++)
    {
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(clouds->pointclouds[i], *cloud);
        BoundingBox box = BoundingBox::create(cloud->points, normal);
        getPose(box, normal, pose_array.poses[i]);
        convertBoundingBox(box, bounding_boxes.bounding_boxes[i]);
    }
    bounding_boxes.header = clouds->header;
    pose_array.header = clouds->header;
    bounding_box_visualizer_->publish(bounding_boxes.bounding_boxes, clouds->header.frame_id);
    pub_bounding_boxes_.publish(bounding_boxes);
    pub_pose_array_.publish(pose_array);
}

void mcr_scene_segmentation::BoundingBoxMaker::getPose(
        const BoundingBox &box,
        const Eigen::Vector3f &axis,
        geometry_msgs::Pose &pose)
{
    BoundingBox::Points vertices = box.getVertices();
    Eigen::Vector3f n1;
    Eigen::Vector3f n2;
    Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
    // set n1 based on longest edge
    if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm())
    {
        n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
    }
    else
    {
        n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
    }
    n2 = n3.cross(n1);
    Eigen::Matrix3f m;
    m << n1 , n2 , n3;
    Eigen::Quaternion<float> q(m);
    q.normalize();


    Eigen::Vector3f centroid = box.getCenter();
    pose.position.x = centroid(0);
    pose.position.y = centroid(1);
    pose.position.z = centroid(2);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
}
