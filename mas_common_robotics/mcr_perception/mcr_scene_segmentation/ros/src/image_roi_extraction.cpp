/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */

#include <mcr_scene_segmentation/image_roi_extraction.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>


void mcr_scene_segmentation::ImageROIExtraction::onInit()
{
    nh_ = getPrivateNodeHandle();
    it_ = boost::make_shared<image_transport::ImageTransport>(nh_);

    nh_.param<int>("max_queue_size", max_queue_size_, 3);

    sub_event_ = nh_.subscribe("event_in", 1, &mcr_scene_segmentation::ImageROIExtraction::eventCallback, this);
    pub_image_list_ = nh_.advertise<mcr_perception_msgs::ImageList> ("output", max_queue_size_);
    sub_bounding_box_ = nh_.subscribe("bounding_boxes", 1,
                        &mcr_scene_segmentation::ImageROIExtraction::boundingBoxCallback, this);
}

void mcr_scene_segmentation::ImageROIExtraction::imageCallback(const sensor_msgs::ImageConstPtr &image_msg,
                                                              const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
    camera_info_ = camera_info_msg;
    image_ = image_msg;
}

void mcr_scene_segmentation::ImageROIExtraction::eventCallback(const std_msgs::String::ConstPtr &event)
{
    if (event->data == "e_start")
    {
        sub_image_ = it_->subscribeCamera("image", 1, &mcr_scene_segmentation::ImageROIExtraction::imageCallback, this);
    }
    else if (event->data == "e_stop")
    {
        sub_image_.shutdown();
    }
}

void mcr_scene_segmentation::ImageROIExtraction::boundingBoxCallback(
        const mcr_perception_msgs::BoundingBoxList::ConstPtr &bounding_boxes)
{
    if (!image_)
    {
        NODELET_WARN("[%s::boundingBoxCallback] No image received!", getName().c_str());
        return;
    }

    if (pub_image_list_.getNumSubscribers() <= 0)
    {
        return;
    }

    if (bounding_boxes->bounding_boxes.empty())
    {
        NODELET_WARN("[%s::boundingBoxCallback] bounding boxes empty!", getName().c_str());
        return;
    }
    camera_model_.fromCameraInfo(camera_info_);

    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(image_, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &ex)
    {
        NODELET_ERROR("Failed to convert image");
        return;
    }
    std::string object_frame_id = bounding_boxes->header.frame_id;
    try
    {
        tf_listener_.waitForTransform(object_frame_id, image_->header.frame_id,
                                      image_->header.stamp, ros::Duration(1.0));
    }
    catch (tf::TransformException ex)
    {
        NODELET_WARN("[%s::boundingBoxCallback] TF lookup error! %s", getName().c_str(), ex.what());
        return;
    }
    mcr_perception_msgs::ImageList image_list;
    for (int i = 0; i < bounding_boxes->bounding_boxes.size(); i++)
    {
        const mcr_perception_msgs::BoundingBox &bounding_box = bounding_boxes->bounding_boxes[i];
        std::vector<cv::Point2f> uv_vertices;
        for (int j = 0; j < bounding_box.vertices.size(); j++)
        {
            geometry_msgs::PointStamped point;
            geometry_msgs::PointStamped transformed_point;
            point.point = bounding_box.vertices[j];
            point.header.frame_id = object_frame_id;
            point.header.stamp = image_->header.stamp;
            try
            {
                tf_listener_.transformPoint(image_->header.frame_id, point, transformed_point);
            }
            catch (tf::TransformException ex)
            {
                NODELET_WARN("[%s::boundingBoxCallback] TF lookup error! %s", getName().c_str(), ex.what());
                return;
            }
            cv::Point3d point_xyz(transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
            cv::Point2f uv;
            uv = camera_model_.project3dToPixel(point_xyz);
            uv_vertices.push_back(uv);
        }
        cv::Rect roi_rectangle = cv::boundingRect(cv::Mat(uv_vertices));
        // expand rectangle a bit
        // (move top left by 5x5 pixels, and increase size by 10 x 10)
        roi_rectangle -= cv::Point(5, 5);
        roi_rectangle += cv::Size(10, 10);
        cv::Rect image_rect(0, 0, cv_image->image.cols, cv_image->image.rows);

        // check if roi is contained within image
        if (!((roi_rectangle & image_rect) == roi_rectangle))
        {
            if (roi_rectangle.x < 0)
            {
                roi_rectangle.x = 0;
            }
            if (roi_rectangle.y < 0)
            {
                roi_rectangle.y = 0;
            }
            if (roi_rectangle.x + roi_rectangle.width >= cv_image->image.cols)
            {
                roi_rectangle.width = cv_image->image.cols - roi_rectangle.x - 1;
            }
            if (roi_rectangle.y + roi_rectangle.height >= cv_image->image.rows)
            {
                roi_rectangle.height = cv_image->image.cols - roi_rectangle.y - 1;
            }
        }

        cv::Mat cropped_image(cv_image->image, roi_rectangle);

        cv_bridge::CvImage image_msg;
        image_msg.encoding = sensor_msgs::image_encodings::BGR8;
        image_msg.image = cropped_image;
        image_list.images.push_back(*image_msg.toImageMsg());
    }
    image_list.header = image_->header;
    pub_image_list_.publish(image_list);
}
