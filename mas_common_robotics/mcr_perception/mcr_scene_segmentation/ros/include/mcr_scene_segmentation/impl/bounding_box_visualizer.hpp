#ifndef BOUNDING_BOX_VISUALIZER_HPP
#define BOUNDING_BOX_VISUALIZER_HPP

#include <visualization_msgs/Marker.h>

namespace mcr
{

namespace visualization
{

BoundingBoxVisualizer::BoundingBoxVisualizer(const std::string& topic_name, Color color, bool check_subscribers)
: color_(color)
, check_subscribers_(check_subscribers)
{
  ros::NodeHandle nh("~");
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(topic_name, 10);
}

void BoundingBoxVisualizer::publish(const mcr_perception_msgs::BoundingBox& box, const std::string& frame_id)
{
  std::vector<mcr_perception_msgs::BoundingBox> boxes;
  boxes.push_back(box);
  publish(boxes, frame_id);
}

void BoundingBoxVisualizer::publish(const std::vector<mcr_perception_msgs::BoundingBox>& boxes, const std::string& frame_id)
{
  if (check_subscribers_ && marker_publisher_.getNumSubscribers() == 0) return;
  visualization_msgs::Marker lines;
  lines.header.frame_id = frame_id;
  lines.header.stamp = ros::Time::now();
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.001;
  lines.scale.y = 0.001;
  lines.color = color_;
  lines.ns = "bounding_boxes";
  lines.id = 1;
  for (const auto& box : boxes)
  {
    const auto& pt = box.vertices;
    lines.points.push_back(pt[0]); lines.points.push_back(pt[1]);
    lines.points.push_back(pt[0]); lines.points.push_back(pt[3]);
    lines.points.push_back(pt[0]); lines.points.push_back(pt[4]);
    lines.points.push_back(pt[1]); lines.points.push_back(pt[2]);
    lines.points.push_back(pt[1]); lines.points.push_back(pt[5]);
    lines.points.push_back(pt[2]); lines.points.push_back(pt[3]);
    lines.points.push_back(pt[2]); lines.points.push_back(pt[6]);
    lines.points.push_back(pt[3]); lines.points.push_back(pt[7]);
    lines.points.push_back(pt[4]); lines.points.push_back(pt[5]);
    lines.points.push_back(pt[4]); lines.points.push_back(pt[7]);
    lines.points.push_back(pt[5]); lines.points.push_back(pt[6]);
    lines.points.push_back(pt[6]); lines.points.push_back(pt[7]);
  }
  marker_publisher_.publish(lines);
}

}

}

#endif /* BOUNDING_BOX_VISUALIZER_HPP */

