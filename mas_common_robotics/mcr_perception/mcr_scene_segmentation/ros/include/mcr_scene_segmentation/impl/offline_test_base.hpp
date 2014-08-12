#ifndef OFFLINE_TEST_BASE_HPP_
#define OFFLINE_TEST_BASE_HPP_

#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "test_base.hpp"

/** Base class for offline perception test programs.
  * Provides PCLVisualizer initialization, infinite superloop (@ref run()),
  * and several helper functions to display different objects. */
class OfflineTestBase : public TestBase
{

public:

  OfflineTestBase()
  : viewer_("Viewer")
  {
    viewer_.setBackgroundColor(0, 0, 0);
    viewer_.addCoordinateSystem(1.0);
    viewer_.initCameraParameters();
  }

  void run()
  {
    process();
    while (!viewer_.wasStopped() && !ros::isShuttingDown())
    {
      viewer_.spinOnce(100);
      ros::spinOnce();
    }
  }

  void loadCloud(const char* name)
  {
    cloud_.reset(new PointCloud);
    pcl::io::loadPCDFile(name, *cloud_);
    applyPassThroughFilter(cloud_);
  }

protected:

  void displayCloud()
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
    viewer_.addPointCloud<PointT>(cloud_, single_color, "cloud");
  }

  void displayNormal(Eigen::Vector3f normal, Eigen::Vector3f start = Eigen::Vector3f::Zero())
  {
    pcl::PointXYZ pt1 = pcl::PointXYZ(start[0], start[1], start[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ(start[0] + (0.5 * normal[0]),
                                      start[1] + (0.5 * normal[1]),
                                      start[2] + (0.5 * normal[2]));
    viewer_.addArrow(pt2, pt1, 1.0, 0, 0, false, "normal");
  }

  /** For the given PlanarPolygon draw a polyline through its points and also
    * display the points themselves.
    *
    * Polygon is drawn in red by defalut and has an identifier "polygon0". */
  void displayPlanarPolygon(const PlanarPolygon& polygon, unsigned char r = 255, unsigned char g = 0, unsigned char b = 0, int id = 0)
  {
    PointCloud::Ptr contour(new PointCloud);
    contour->points = polygon.getContour();
    if (!contour->points.size())
      return;
    std::string polygon_name = "polygon" + boost::lexical_cast<std::string>(id);
    std::string points_name = polygon_name + "_points";
    viewer_.addPolygon<PointT>(contour, r, g, b, polygon_name);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color(contour, r, g, b);
    viewer_.addPointCloud(contour, color, points_name);
    // Temporary commented out to prevent compiler errors
    // viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, points_name);
  }

  void displayPlanarPolygons(const PlanarPolygonVector& polygons)
  {
    for (size_t i = 0; i < polygons.size(); i++)
    {
      unsigned char r = 255 - i * 5;
      unsigned char b = 0   + i * 5;
      displayPlanarPolygon(polygons[i], r, 0, b, i);
    }
  }

  pcl::visualization::PCLVisualizer viewer_;
  PointCloud::Ptr cloud_;

};

#endif

