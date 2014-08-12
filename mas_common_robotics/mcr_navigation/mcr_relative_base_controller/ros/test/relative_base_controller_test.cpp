#include <gtest/gtest.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <iostream>


bool event_done = false;
std::string event_done_str;
bool is_moving = false;

void cmd_vel_cb(const geometry_msgs::TwistConstPtr &cmd_vel)
{
  if (cmd_vel->linear.x != 0.0 || cmd_vel->linear.y != 0.0 || cmd_vel->angular.z != 0.0)
  {
    is_moving = true;
  }
  else
  {
    is_moving = false;
  }
}

void event_out_cb(const std_msgs::StringConstPtr &event_out)
{
  event_done_str = event_out->data;
  event_done = true;
}

TEST(relative_base_controller_test, test_move_without_event)
{
  ros::NodeHandle nh("~");
  ros::Publisher move_command_pub = nh.advertise<geometry_msgs::Twist>("/mcr_navigation/relative_base_controller/command", 1);
  ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &cmd_vel_cb);

  geometry_msgs::Twist move_command;
  move_command.linear.x = 0.1;  
  
  move_command_pub.publish(move_command);
  ros::Rate(0.1).sleep();
  ros::spinOnce();
  ASSERT_EQ(is_moving, false);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "relative_base_controller_test");
    return RUN_ALL_TESTS();
}
