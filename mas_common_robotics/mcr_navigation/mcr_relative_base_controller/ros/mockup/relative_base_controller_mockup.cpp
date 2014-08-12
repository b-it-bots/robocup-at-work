#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

std::string event_msg = "";

void eventCallback(const std_msgs::String::ConstPtr& msg)
{
  event_msg = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relative_base_controller");
  ros::NodeHandle n("~");

  std_msgs::String msg;

  ros::Publisher pub_event_out = n.advertise<std_msgs::String>("event_out", 10, true);
  ros::Subscriber sub_event_in = n.subscribe("event_in", 1, eventCallback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(event_msg == "e_start")
    {
      sleep(1);

      msg.data = "e_done";
      pub_event_out.publish(msg);

      event_msg = "";
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
