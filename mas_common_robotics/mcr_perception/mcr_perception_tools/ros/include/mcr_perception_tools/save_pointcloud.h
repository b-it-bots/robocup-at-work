#ifndef SAVE_POINTCLOUD_H_
#define SAVE_POINTCLOUD_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>

/**
 * This class saves pointclouds from a given topic as a .pcd file.
 *
 * Specify the input topic when launching the node and publish the file path
 * when you want to save the next published PointCloud on the input topic.
 *
 * Subscribes:
 * ~/input/file_path - full path to .pcd file including file extension
 * <input_topic> - (specified as parameter) - the sensor_msgs/PointCloud2 topic from where the pointcloud is saved
 *
 * Publishes:
 * ~/event_out - 'e_done' if saved successfully, 'e_failed' otherwise
 *
 * Parameters:
 * input_topic - pointcloud topic to subscribe to
 * file_type - save file as 'ascii', 'binary' or 'compressed'
 */
class SavePointCloud
{
    public:
        SavePointCloud();
        virtual ~SavePointCloud();


    private:
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void filePathCallback(const std_msgs::String &msg);


    private:
        ros::NodeHandle nh_;

        ros::Subscriber sub_pointcloud_;
        ros::Subscriber sub_file_path_;

        ros::Publisher pub_event_out_;

        /**
         * Full path to .pcd file including extension
         * eg: /home/demo/Desktop/1.pcd
         */
        std::string file_path_;

        std::string input_topic_;

        pcl::PCDWriter writer_;
};
#endif
