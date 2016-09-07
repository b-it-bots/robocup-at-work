/* 
 * Copyright [2016] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Unit test for run script node.
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <mcr_task_planning_tools/run_script_node.h>
#include <string>

std::string event_out_response;
bool callback_received = false;

void event_out_cb(const std_msgs::StringConstPtr &event_out)
{
    ROS_INFO("callback received !");
    callback_received = true;
    event_out_response = event_out->data;
}

TEST(run_script_node_test, success_test)
{
    ros::NodeHandle nh("~");
    ros::Publisher event_in_pub = nh.advertise<std_msgs::String>("event_in", 1);
    ros::Subscriber event_out_sub = nh.subscribe<std_msgs::String>("event_out", 1, &event_out_cb);

    // give some time for publishers/ subscribers to register in the network
    sleep(1.0);

    // create ros msg from string
    std_msgs::String event_in_ros_msg;
    event_in_ros_msg.data = std::string("e_trigger");

    // publish event_in msg
    ROS_INFO("publishing event_in msg");
    event_in_pub.publish(event_in_ros_msg);

    // give some time for the publisher to send message
    sleep(1.0);

    // listen for 1 callback
    ros::spinOnce();

    // compare result, should be e_success
    ASSERT_EQ(std::string("e_success"), event_out_response);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "run_script_node_test");
    ROS_INFO("Test initialized");

    return RUN_ALL_TESTS();
}
