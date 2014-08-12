#ifndef ONLINE_TEST_BASE_HPP_
#define ONLINE_TEST_BASE_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "test_base.hpp"

class OnlineTestBase : public TestBase
{

public:

  OnlineTestBase()
  {
    ros::NodeHandle nh;
    cloud_subscriber_ = nh.subscribe("/camera/rgb/points", 5, &OnlineTestBase::cloudCallback, this);
  }

protected:

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &ros_cloud)
  {
    header_ = ros_cloud->header;
    cloud_.reset(new PointCloud);

    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(*ros_cloud, pc2);
    pcl::fromPCLPointCloud2(pc2, *cloud_);

    applyPassThroughFilter(cloud_);
    process();
  }

  /** Fill the fields of the marker object so that it visualizes the provided
    * vector of points by drawing a polyline through them.
    *
    * Polyline is drawn in salmon red by defalut and has id == 1. */
  void buildPolygonMarker(const PointCloud::VectorType& points,
                          visualization_msgs::Marker& marker,
                          float r = 1.0f,
                          float g = 0.5f,
                          float b = 0.5f,
                          int id = 1)
  {
    if (!points.size()) return;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "/openni_rgb_optical_frame";
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.color.a = 1.0;
    marker.ns = "polygon";
    marker.id = id;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    geometry_msgs::Point first_point;
    first_point.x = points[0].x;
    first_point.y = points[0].y;
    first_point.z = points[0].z;
    marker.points.push_back(first_point);

    for (size_t i = 1; i < points.size(); i++)
    {
      const auto& point = points[i];
      geometry_msgs::Point pt;
      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;
      marker.points.push_back(pt);
      marker.points.push_back(pt);
    }

    marker.points.push_back(first_point);
  }

  void buildPolygonMarkerArray(const PlanarPolygonVector& polygons,
                               visualization_msgs::MarkerArray& marker_array)
  {
    for (size_t i = 0; i < polygons.size(); i++)
    {
      float r = 1.0f - i * 0.02f;
      float b = 0.0f + i * 0.02f;
      visualization_msgs::Marker marker;
      buildPolygonMarker(polygons[i].getContour(), marker, r, 0.0f, b, i + 1);
      marker_array.markers.push_back(marker);
    }
  }

  ros::Subscriber cloud_subscriber_;
  PointCloud::Ptr cloud_;
  std_msgs::Header header_;

};

#endif

