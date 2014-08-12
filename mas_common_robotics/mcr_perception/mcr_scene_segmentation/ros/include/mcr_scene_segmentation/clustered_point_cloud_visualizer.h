#ifndef CLUSTERED_POINT_CLOUD_VISUALIZER_H
#define CLUSTERED_POINT_CLOUD_VISUALIZER_H

#include <string>

#include <ros/ros.h>

#include <pcl/point_cloud.h>

#include "color.h"

namespace mcr
{

namespace visualization
{

class ClusteredPointCloudVisualizer
{

public:

  ClusteredPointCloudVisualizer(const std::string& topic_name,
                                bool check_subscribers = true);

  template<typename PointT>
  void publish(const std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters,
               const std::string& frame_id);

private:

  ros::Publisher cloud_publisher_;

  bool check_subscribers_;

  static const size_t COLORS_NUM = 32;
  float COLORS[COLORS_NUM];

};

}

}

#include "impl/clustered_point_cloud_visualizer.hpp"

#endif /* CLUSTERED_POINT_CLOUD_VISUALIZER_H */

