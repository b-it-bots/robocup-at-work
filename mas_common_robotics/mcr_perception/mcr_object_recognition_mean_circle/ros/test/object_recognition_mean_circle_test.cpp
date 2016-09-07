/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <mcr_perception_msgs/RecognizeObject.h>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

TEST(object_recognition_mean_circle_test, test_recognition_rate)
{
    ros::NodeHandle nh("~");
    double desired_recog_rate = 0.91;
    std::vector<std::string> object_list;

    nh.getParam("desired_recog_rate", desired_recog_rate);
    nh.getParam("object_list", object_list);

    ros::service::waitForService("/mcr_perception/object_recognizer/recognize_object", ros::Duration(5.0));

    std::string test_data_path;

    std::string default_dataset_path = ros::package::getPath("mds_pointclouds");

    nh.getParam("dataset", test_data_path);
    default_dataset_path.append(test_data_path);

    bfs::path p(test_data_path);

    int total_count = 0;
    int correct_count = 0;

    for (size_t i = 0; i < object_list.size(); i++)
    {
        std::string object_name = object_list.at(i);
        std::string full_path = default_dataset_path;
        full_path.append(object_name);

        bfs::path p(full_path);

        bfs::directory_iterator end_iter;

        if (bfs::exists(p))
        {
            for (bfs::directory_iterator obj_dir_it(p); obj_dir_it != end_iter; ++obj_dir_it)
            {
                mcr_perception_msgs::RecognizeObject::Request obj_rec_srv_req;

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

                if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (obj_dir_it->path().string(), *full_cloud) == -1)
                {
                    continue;
                }

                sensor_msgs::PointCloud2 ros_cloud;
                pcl::toROSMsg(*full_cloud, ros_cloud);
                obj_rec_srv_req.cloud = ros_cloud;

                mcr_perception_msgs::RecognizeObject::Response res;

                if (ros::service::call("/mcr_perception/object_recognizer/recognize_object", obj_rec_srv_req, res))
                {
                    total_count++;

                    if (object_name.compare(res.name) == 0)
                    {
                        correct_count++;
                    }
                }
            }
        }
    }

    double prediction_rate = static_cast<double>(correct_count) / total_count;
    std::cout << prediction_rate << std::endl;
    ASSERT_TRUE(prediction_rate > desired_recog_rate);
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "object_recognition_mean_circle_test");
    return RUN_ALL_TESTS();
}
