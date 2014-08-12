#ifndef BOUNDING_BOX_VISUALIZER_H
#define BOUNDING_BOX_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <mcr_perception_msgs/BoundingBox.h>
#include "color.h"

namespace mcr
{

namespace visualization
{

class BoundingBoxVisualizer
{

public:

  BoundingBoxVisualizer(const std::string& topic_name,
                        Color color,
                        bool check_subscribers = true);

  void publish(const mcr_perception_msgs::BoundingBox& box, const std::string& frame_id);

  void publish(const std::vector<mcr_perception_msgs::BoundingBox>& boxes, const std::string& frame_id);

private:

  ros::Publisher marker_publisher_;

  const Color color_;
  bool check_subscribers_;

};

}

}

#include "impl/bounding_box_visualizer.hpp"

#endif /* BOUNDING_BOX_VISUALIZER_H */

