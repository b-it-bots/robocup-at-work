/*
 * waist_tracker_node.cpp
 *
 *  Created on: Jan 25, 2011
 *      Author: Frederik Hegger
 *
 *      TODO:  - some magic numbers left
 *             - cleanup
 */

#include "mcr_people_tracking/particle_filter.h"

#include <list>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <opencv/cv.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <mcr_perception_msgs/PersonList.h>
#include <mcr_perception_msgs/Person.h>
#include <mcr_perception_msgs/LaserScanSegmentList.h>
#include <mcr_perception_msgs/LaserScanSegment.h>
#include <mcr_algorithms/segmentation/laserscan_segmentation.h>
#include <mcr_algorithms/wrapper/pcl_wrapper.hpp>
#include <mcr_algorithms/projections/pointcloud_projections.hpp>
#include <mcr_algorithms/geometry/geometric_properties.hpp>
#include <mcr_algorithms/statistics/minmax.hpp>

#include "mcr_people_tracking/WaistTrackingConfig.h"

#define FIXED_FRAME                 "/base_link"
#define FIND_PERSON_IN_RANGE        0
#define FIND_PERSON_BY_HISTOGRAM    1

using namespace std;

LaserScanSegmentation* segmentor;
TrackingParticleFilter* tracker;
tf::TransformListener *transform_listener;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_input;
bool is_pointcloud_received = false;
cv::Mat initial_person_histogram;
double teached_person_min_z = 0;
double teached_person_height = 0;

unsigned int tracker_init_state = FIND_PERSON_IN_RANGE;

ros::Publisher pub_people_positions;
ros::Publisher pub_visualization_marker;

bool is_tracker_initialized = false;
bool is_person_occluded = false;

bool is_tracking_enabled = false;               // ToDo: change to false;
bool is_pointcloud_processing_enabled = false;

ros::Subscriber subWaistScan;
ros::Subscriber subPointCloud;
ros::NodeHandle* g_nh_ptr;

mcr_people_tracking::WaistTrackingConfig dyn_recfg_parameters;


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
        marker_shape.ns = marker_text.ns = "tracked person";
        marker_shape.action = marker_text.action = visualization_msgs::Marker::ADD;

        marker_shape.id = ++j;
        marker_text.id = ++j;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_shape.type = visualization_msgs::Marker::CUBE;

        marker_shape.pose.position = marker_text.pose.position = person_list.persons[i].pose.pose.position;
        marker_shape.pose.orientation = marker_text.pose.orientation = person_list.persons[i].pose.pose.orientation;

        if (person_list.persons[i].is_occluded)
            marker_text.text = "OCCLUDED";
        else
            marker_text.text = "";


        marker_shape.scale.x = 0.4;
        marker_shape.scale.y = 0.4;
        marker_shape.scale.z = 1.8;
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



bool is_in_vector(vector<unsigned int> vector_input, unsigned int value)
{
    for (unsigned int i = 0; i < vector_input.size(); ++i)
        if (vector_input[i] == value)
            return true;

    return false;
}

bool reinitialize_tracking_with_histogram_search(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    is_tracking_enabled = true;
    tracker_init_state = FIND_PERSON_BY_HISTOGRAM;
    is_tracker_initialized = false;
    is_pointcloud_processing_enabled = true;

    ROS_INFO("reinitialize tracking with searching for person by color histogram");

    return true;
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud2_input)
{
    is_pointcloud_received = false;

    if (!is_pointcloud_processing_enabled)
        return;

    sensor_msgs::PointCloud2 cloud2_transformed;

    try
    {
        transform_listener->waitForTransform(FIXED_FRAME, cloud2_input->header.frame_id, cloud2_input->header.stamp, ros::Duration(2.0));
        pcl_ros::transformPointCloud(FIXED_FRAME, *cloud2_input, cloud2_transformed, *transform_listener);

        if (cloud2_transformed.width <= 0 || cloud2_transformed.height <= 0)
        {
            ROS_WARN("Skip point cloud, because transformation for it is too old");
            return;
        }

        pcl::fromROSMsg(cloud2_transformed, *pcl_cloud_input);
        is_pointcloud_received = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Waiting for TF");
    }
}

void dynamic_reconfig_callback(mcr_people_tracking::WaistTrackingConfig &config, uint32_t level)
{
    dyn_recfg_parameters = config;
    dyn_recfg_parameters.angular_range_for_searching /= 2;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& inputScan)
{
    if (!is_tracking_enabled)
        return;

    mcr_perception_msgs::LaserScanSegmentList segmentList;
    segmentList = segmentor->getSegments(inputScan);

    //cout << "received segm: " <<  segmentList.segments.size() << std::endl;

    if (!is_tracker_initialized)
    {
        if (tracker_init_state == FIND_PERSON_IN_RANGE)
        {
            for (unsigned int i = 0; i < segmentList.segments.size(); ++i)
            {
                double dAngle = atan(segmentList.segments[i].center.y / segmentList.segments[i].center.x);
                double dDistance = sqrt(pow(segmentList.segments[i].center.x, 2.0) + pow(segmentList.segments[i].center.y, 2.0));

                if (dDistance < dyn_recfg_parameters.distance_range_for_searching && dAngle > -dyn_recfg_parameters.angular_range_for_searching && dAngle < dyn_recfg_parameters.angular_range_for_searching
                        && is_pointcloud_received)
                {
                    mcr_perception_msgs::LaserScanSegmentList temp;
                    temp.segments.push_back(segmentList.segments[i]);

                    tracker->initialize(temp);

                    /*
                          // cut down the pointcloud to the ROI
                          PCLWrapper<pcl::PointXYZRGB>::passThroughFilter(pcl_cloud_input, pcl_cloud_input, "x", (segmentList.segments[i].center.x - 0.2),
                                                                          (segmentList.segments[i].center.x + 0.2));
                          PCLWrapper<pcl::PointXYZRGB>::passThroughFilter(pcl_cloud_input, pcl_cloud_input, "y", (segmentList.segments[i].center.y - 0.35),
                                                                          (segmentList.segments[i].center.y + 0.35));
                          PCLWrapper<pcl::PointXYZRGB>::passThroughFilter(pcl_cloud_input, pcl_cloud_input, "z", 0.0, 2.0);

                          // get min z value for later recognition
                          double min_x, max_x, min_y, max_y, min_z, max_z;
                          MinMax::determineMinMax3D(*pcl_cloud_input, min_x, max_x, min_y, max_y, min_z, max_z);
                          teached_person_min_z = min_z;
                          teached_person_height = max_z;

                          ROS_INFO_STREAM("Z-Min: " << teached_person_min_z << " Z-Max: " << max_z);

                          // get fixed set of random points
                          pcl::PointCloud < pcl::PointXYZRGB > pcl_random_point_set;
                          vector<unsigned int> used_indices;

                          pcl_random_point_set.header = pcl_cloud_input->header;

                          for (unsigned int k = 0; k < 5000;)
                          {
                              unsigned int rand_index = rand() % pcl_cloud_input->points.size();

                              //cout << "index: " << rand_index << " is in vector: " << is_in_vector(used_indices, rand_index) << endl;

                              if (!is_in_vector(used_indices, rand_index))
                              {
                                  pcl_random_point_set.points.push_back(pcl_cloud_input->points[rand_index]);
                                  used_indices.push_back(rand_index);
                                  ++k;
                              }
                          }

                          //cout << "SIZE: " << pcl_random_point_set.points.size() << endl;

                          *pcl_cloud_input = pcl_random_point_set;

                          //copy only ROI points to an opencv image
                          IplImage* ipl_roi_person_bgr = cvCreateImage(cvSize(pcl_cloud_input->points.size(), 1), IPL_DEPTH_8U, 3);
                          for (unsigned int j = 0; j < pcl_cloud_input->points.size(); ++j)
                          {
                              cvSet2D(ipl_roi_person_bgr,
                                      0,
                                      j,
                                      cvScalar((0x0000ff & *reinterpret_cast<int*>(&(pcl_cloud_input->points[j].rgb))),
                                               ((0x00ff00 & *reinterpret_cast<int*>(&(pcl_cloud_input->points[j].rgb))) >> 8),
                                               ((0xff0000 & *reinterpret_cast<int*>(&(pcl_cloud_input->points[j].rgb))) >> 16)));

                              const unsigned char *color = reinterpret_cast<const unsigned char *>(&pcl_cloud_input->points[j].rgb);
                              cvSet2D(ipl_roi_person_bgr, 0, j, cvScalar(color[0], color[1], color[2]));  // cvScalar(b, g, r)
                          }

                          // calculate the histogram for the detected person
                          cv::Mat mat_roi_person_bgr(ipl_roi_person_bgr);
                          cv::Mat mat_roi_person_hsv;

                          cv::cvtColor(mat_roi_person_bgr, mat_roi_person_hsv, CV_BGR2HSV);

                          cv::Mat histogram;
                          int hue_bin_size = 30, sat_bin_size = 32;
                          int histogram_size[] =
                          { hue_bin_size, sat_bin_size };
                          float hue_range[] =
                          { 0, 180 }, sat_range[] =
                          { 0, 256 };
                          const float *ranges[] =
                          { hue_range, sat_range };
                          int channels[] =
                          { 0, 1 };

                          cv::calcHist(&mat_roi_person_hsv, 1, channels, cv::Mat(), initial_person_histogram, 1, histogram_size, ranges);

                          cvReleaseImage(&ipl_roi_person_bgr);
                    */

                    is_tracker_initialized = true;
                    is_pointcloud_processing_enabled = false;
                    ROS_INFO("Person in range FOUND!");

                    break;
                }
            }
        }

        if (tracker_init_state == FIND_PERSON_BY_HISTOGRAM && is_pointcloud_received)
        {
            pcl::PointCloud < pcl::PointXYZRGB > pcl_projected_cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_2d_segment(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_3d_segment(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_2d_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
            vector<PointIndices> vec_2d_segment_indices;

            // preprocessing: cropping and downsampling
            PCLWrapper<pcl::PointXYZRGB>::passThroughFilter(pcl_cloud_input, pcl_cloud_input, "x", 0, 3.0);
            PCLWrapper<pcl::PointXYZRGB>::passThroughFilter(pcl_cloud_input, pcl_passthrough, "z", teached_person_min_z, 2.0);
            PCLWrapper<pcl::PointXYZRGB>::downsampling(pcl_passthrough, pcl_cloud_input, 0.03);

            // project cloud to 2d
            PointCloudProjections::projection2_5D(*pcl_cloud_input, pcl_projected_cloud, "z");

            // cluster 2d projection
            PCLWrapper<pcl::PointXYZRGB>::clustering(pcl_projected_cloud.makeShared(), vec_2d_segment_indices, 0.10, 0, 100000);
            double min_hist_error = 9999;
            bool found_owner = false;
            mcr_perception_msgs::LaserScanSegment owner_pos;

            for (unsigned int i = 0; i < vec_2d_segment_indices.size(); ++i)
            {
                //convert indices to single point clouds
                copyPointCloud(pcl_projected_cloud, vec_2d_segment_indices[i], *pcl_2d_segment);

                // Todo: delete
                /*
                 sensor_msgs::PointCloud2 temp;
                 pcl::toROSMsg(*pcl_2d_segment, temp);
                 pubTemp.publish(temp);
                 */

                //get convex hull of the 2d projected segment
                PCLWrapper<pcl::PointXYZRGB>::extractConvexHull(pcl_2d_segment, pcl_2d_hull);

                if (pcl_2d_hull->size() < 2)
                    continue;

                PCLWrapper<pcl::PointXYZRGB>::get3DPointsWithinHull(pcl_passthrough, pcl_2d_hull, teached_person_min_z, 2.0, pcl_3d_segment);

                if (pcl_3d_segment->points.size() < 5000)
                    continue;

                // get fixed set of random points
                pcl::PointCloud < pcl::PointXYZRGB > pcl_random_point_set;
                vector<unsigned int> used_indices;

                pcl_random_point_set.header = pcl_3d_segment->header;

                for (unsigned int k = 0; k < 5000;)
                {
                    unsigned int rand_index = rand() % pcl_3d_segment->points.size();

                    //cout << "index: " << rand_index << " is in vector: " << is_in_vector(used_indices, rand_index) << endl;

                    if (!is_in_vector(used_indices, rand_index))
                    {
                        pcl_random_point_set.points.push_back(pcl_3d_segment->points[rand_index]);
                        used_indices.push_back(rand_index);
                        ++k;
                    }
                }

                *pcl_3d_segment = pcl_random_point_set;

                // get min z value for later recognition
                double min_x, max_x, min_y, max_y, min_z, max_z;
                MinMax::determineMinMax3D(*pcl_3d_segment, min_x, max_x, min_y, max_y, min_z, max_z);

                //ToDo:: height feature disabled
                //cout << "max_z: " << max_z << "(teached_person_height - 0.02): " << (teached_person_height - 0.02) << " (teached_person_height + 0.02): " << (teached_person_height + 0.02) << endl;

                //if(max_z < (teached_person_height - 0.02) || max_z > (teached_person_height + 0.02))
                //continue;

                //copy only ROI points to an opencv image
                IplImage* ipl_roi_segment = cvCreateImage(cvSize(pcl_3d_segment->points.size(), 1), IPL_DEPTH_8U, 3);
                for (unsigned int j = 0; j < pcl_3d_segment->points.size(); ++j)
                {
                    const unsigned char *color = reinterpret_cast<const unsigned char *>(&pcl_3d_segment->points[j].rgb);
                    cvSet2D(ipl_roi_segment, 0, j, cvScalar(color[0], color[1], color[2]));  // cvScalar(b, g, r)
                }

                // calculate the histogram for the detected person
                cv::Mat mat_roi_segment_bgr(ipl_roi_segment);
                cv::Mat mat_roi_segment_hsv;

                cv::cvtColor(mat_roi_segment_bgr, mat_roi_segment_hsv, CV_BGR2HSV);

                cv::Mat histogram, equalized_histogram;
                int hue_bin_size = 30, sat_bin_size = 32;
                int histogram_size[] =
                { hue_bin_size, sat_bin_size };
                float hue_range[] =
                { 0, 180 }, sat_range[] =
                { 0, 256 };
                const float *ranges[] =
                { hue_range, sat_range };
                int channels[] =
                { 0, 1 };

                cv::calcHist(&mat_roi_segment_hsv, 1, channels, cv::Mat(), histogram, 1, histogram_size, ranges);

                // CV_COMP_CORREL, CV_COMP_INTERSECT, CV_COMP_BHATTACHARYYA, CV_COMP_CHISQR
                float hist_error = cv::compareHist(histogram, initial_person_histogram, CV_COMP_BHATTACHARYYA);

                if (hist_error < 0.5)
                    ROS_INFO_STREAM("Hist-Error: " << hist_error << " HEIGHT: " << max_z);
                //ROS_INFO_STREAM("Points:" << pcl_3d_segment.points.size());

                if (hist_error < 0.5)
                {

                    if (hist_error < min_hist_error)
                    {
                        min_hist_error = hist_error;
                        Eigen::Vector4f centroid;
                        GeometricProperties::getCentroid3D<pcl::PointXYZRGB>(*pcl_3d_segment, centroid);

                        owner_pos.center.x = centroid[0];
                        owner_pos.center.y = centroid[1];
                        found_owner = true;
                    }
                }

                cvReleaseImage(&ipl_roi_segment);
            }
            if (found_owner)
            {

                mcr_perception_msgs::LaserScanSegmentList owner_list;
                owner_list.segments.push_back(owner_pos);
                tracker->initialize(owner_list);

                is_tracker_initialized = true;
                is_pointcloud_processing_enabled = false;

                ROS_INFO("Owner RECOGNIZED!");
            }

        }
    }

    else
    {
        //find occlusion
        for (unsigned int i = 0; i < segmentList.segments.size(); ++i)
        {
            double angle = atan(segmentList.segments[i].center.y / segmentList.segments[i].center.x);
            double distance = sqrt(pow(segmentList.segments[i].center.x, 2.0) + pow(segmentList.segments[i].center.y, 2.0));

            double yaw_threshold = 0;
            if (tracker->getMostLikelyPosition()->dDistance > 2.0)
                yaw_threshold = M_PI / 8;
            else
                yaw_threshold = M_PI / 6;

            if (distance < (tracker->getMostLikelyPosition()->dDistance - 0.3)
                    && (angle >= (tracker->getMostLikelyPosition()->dYaw - yaw_threshold) && angle <= (tracker->getMostLikelyPosition()->dYaw + yaw_threshold)))
            {
                //ToDo: comment in

                //cout << "occl dist: " <<  distance << " target dist: " << tracker->getMostLikelyPosition()->dDistance << endl;
                //cout << "occl yaw: " <<  angle << " target dYaw: " << tracker->getMostLikelyPosition()->dYaw << endl;

                ROS_INFO("OCCLUSION!");
                cout << "####### OCCLUSION ###########" << endl;

                is_person_occluded = true;
                break;
            }

            is_person_occluded = false;
        }

        if (!is_person_occluded || !dyn_recfg_parameters.occlusion_handling_enabled)
        {
            //ToDo: comment in
            //cout << "update tracker" << endl;
            tracker->predict();
            tracker->predict();
            tracker->update(segmentList);
        }

        StrPoint* mostLikelyPoint;
        mostLikelyPoint = tracker->getMostLikelyPosition();

        mcr_perception_msgs::PersonList trackedPersonList;
        mcr_perception_msgs::Person trackedPerson;

        trackedPerson.pose.header = inputScan->header;
        trackedPerson.pose.header.stamp = ros::Time::now();
        trackedPerson.pose.pose.position.x = mostLikelyPoint->dX;
        trackedPerson.pose.pose.position.y = mostLikelyPoint->dY;
        trackedPerson.pose.pose.position.z = 0.0;
        trackedPerson.id = 0;
        trackedPerson.is_tracked = true;
        trackedPerson.is_occluded = is_person_occluded;

        trackedPersonList.persons.push_back(trackedPerson);

        pub_people_positions.publish(trackedPersonList);

        if (dyn_recfg_parameters.publish_visualization_markers)
            publishVisualizationMarker(trackedPersonList);

        delete mostLikelyPoint;

        //std::cout << "x: " << mostLikelyPoint->dX << " y: " << mostLikelyPoint->dY << endl;
    }

    //cout << "###############################" << endl;
}

bool start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    is_tracking_enabled = true;
    is_tracker_initialized = false;
    is_pointcloud_processing_enabled = true;
    tracker_init_state = FIND_PERSON_IN_RANGE;

    subWaistScan = g_nh_ptr->subscribe < sensor_msgs::LaserScan > ("scan", 1, laserScanCallback);
    subPointCloud = g_nh_ptr->subscribe < sensor_msgs::PointCloud2 > ("pointcloud_xyzrgb", 1, pointCloudCallback);      // Todo: set back to cam3d

    ROS_INFO("tracking ENABLED");

    return true;
}

bool stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    is_tracking_enabled = false;
    subWaistScan.shutdown();
    subPointCloud.shutdown();
    ROS_INFO("tracking DISABLED");

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waist_tracking");
    ros::NodeHandle nh("~");
    g_nh_ptr = &nh;

    segmentor = new LaserScanSegmentation(0.20, 3);
    tracker = new TrackingParticleFilter(100);
    transform_listener = new tf::TransformListener();

    ros::ServiceServer service_start, service_stop;
    service_start = nh.advertiseService("start", &start);
    service_stop = nh.advertiseService("stop", &stop);

    ros::ServiceServer service_reinitialize_tracking;
    service_reinitialize_tracking = nh.advertiseService("reinitialize_tracking_with_histogram_search", &reinitialize_tracking_with_histogram_search);

    ros::Subscriber subWaistScan;
    ros::Subscriber subPointCloud;

    pub_people_positions = nh.advertise < mcr_perception_msgs::PersonList > ("people_positions", 1);
    pub_visualization_marker = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud_input = tmp_ptr;

    // dynamic reconfigure server
    dynamic_reconfigure::Server<mcr_people_tracking::WaistTrackingConfig> dynamic_reconfig_server;
    dynamic_reconfig_server.setCallback(boost::bind(&dynamic_reconfig_callback, _1, _2));

    if (dyn_recfg_parameters.occlusion_handling_enabled)
        ROS_INFO("occlusion handling ENABLED");
    else
        ROS_INFO("occlusion handling DISABLED");

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // CLEAN UP
    if (tracker != NULL)
        delete tracker;
    if (segmentor != NULL)
        delete segmentor;
    if (transform_listener != NULL)
        delete transform_listener;

    return 0;
}
