/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#ifndef MCR_SCENE_SEGMENTATION_IMAGE_ROI_EXTRACTION_H_
#define MCR_SCENE_SEGMENTATION_IMAGE_ROI_EXTRACTION_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/String.h>
#include <mcr_perception_msgs/BoundingBox.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <mcr_perception_msgs/ImageList.h>

#include <opencv2/opencv.hpp>

namespace mcr_scene_segmentation
{

/**
 * Subscribes to BoundingBoxList and camera image
 * and publishes an ImageList.
 * The images in ImageList correspond to the 2D ROIs retreived by projecting the 3D
 * bounding box onto the image plane
 *
 * only subscribes to image when e_start is received on event_in
 * stops subscription to image with e_stop is received
 */
class ImageROIExtraction : public nodelet::Nodelet
{
    public:
        ImageROIExtraction() {}

    protected:
        virtual void onInit();

        void imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                           const sensor_msgs::CameraInfoConstPtr &camera_info_msg);

        void boundingBoxCallback(const mcr_perception_msgs::BoundingBoxList::ConstPtr &bounding_boxes);
        void eventCallback(const std_msgs::String::ConstPtr &event);

    protected:
        ros::NodeHandle nh_;

        boost::shared_ptr<image_transport::ImageTransport> it_;
        ros::Publisher pub_bounding_boxes_;

        image_transport::CameraSubscriber sub_image_;
        ros::Subscriber sub_bounding_box_;

        ros::Subscriber sub_event_;

        ros::Publisher pub_image_list_;

        image_geometry::PinholeCameraModel camera_model_;
        tf::TransformListener tf_listener_;

        sensor_msgs::ImageConstPtr image_;
        sensor_msgs::CameraInfoConstPtr camera_info_;

        int max_queue_size_;
};
PLUGINLIB_DECLARE_CLASS(mcr_scene_segmentation, ImageROIExtraction,
                        mcr_scene_segmentation::ImageROIExtraction, nodelet::Nodelet);
}  // namespace mcr_scene_segmentation

#endif  // MCR_SCENE_SEGMENTATION_IMAGE_ROI_EXTRACTION_H_
