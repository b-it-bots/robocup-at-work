/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov, Santosh Thoduka
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_BOUNDING_BOX_MAKER_H_
#define MCR_SCENE_SEGMENTATION_BOUNDING_BOX_MAKER_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_msgs/ModelCoefficients.h>

#include <mcr_perception_msgs/PointCloud2List.h>
#include <mcr_perception_msgs/BoundingBox.h>
#include "mcr_scene_segmentation/bounding_box_visualizer.h"
#include "mcr_scene_segmentation/impl/helpers.hpp"
#include "mcr_scene_segmentation/bounding_box.h"

namespace mcr_scene_segmentation
{

namespace sync_policies = message_filters::sync_policies;

/**
 * Subscribes to PointCloud2List and publishes BoundingBoxList and PoseArray
 * corresponding to each point cloud
 */
class BoundingBoxMaker : public nodelet::Nodelet
{
    public:
        BoundingBoxMaker() {}

    protected:
        virtual void onInit();

        void syncInputCallback(const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds,
                               const pcl_msgs::ModelCoefficients::ConstPtr &model);

        void getPose(const BoundingBox &bounding_box, const Eigen::Vector3f &axis, geometry_msgs::Pose &pose);

    protected:
        ros::NodeHandle nh_;
        ros::Publisher pub_bounding_boxes_;
        ros::Publisher pub_pose_array_;

        message_filters::Subscriber<mcr_perception_msgs::PointCloud2List> sub_clouds_filter_;
        message_filters::Subscriber<pcl_msgs::ModelCoefficients> sub_model_filter_;

        boost::shared_ptr<message_filters::Synchronizer
            <sync_policies::ExactTime
            <mcr_perception_msgs::PointCloud2List, pcl_msgs::ModelCoefficients> > > sync_input_e_;
        boost::shared_ptr<message_filters::Synchronizer
            <sync_policies::ApproximateTime
            <mcr_perception_msgs::PointCloud2List, pcl_msgs::ModelCoefficients> > > sync_input_a_;

        boost::shared_ptr<mcr::visualization::BoundingBoxVisualizer> bounding_box_visualizer_;

        int max_queue_size_;
        bool approximate_sync_;
};
PLUGINLIB_DECLARE_CLASS(mcr_scene_segmentation, BoundingBoxMaker,
                        mcr_scene_segmentation::BoundingBoxMaker, nodelet::Nodelet);
}  // namespace mcr_scene_segmentation

#endif  // MCR_SCENE_SEGMENTATION_BOUNDING_BOX_MAKER_H_
