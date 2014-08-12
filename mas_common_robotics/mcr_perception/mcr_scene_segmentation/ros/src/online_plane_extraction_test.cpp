#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "mcr_scene_segmentation/impl/reconfigurable_plane_extraction.hpp"
#include "mcr_scene_segmentation/impl/online_test_base.hpp"

class TestNode : public OnlineTestBase
{

public:

  TestNode()
  {
    ros::NodeHandle nh;
    marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("planes", 1);
  }

protected:

  void process()
  {
    if (!cloud_) return;

    PlanarPolygonVector planar_polygons;
    rpe_.plane_extraction_ptr->setInputCloud(cloud_);
    MEASURE_RUNTIME(rpe_.plane_extraction_ptr->extract(planar_polygons), "Plane extraction");

    for (size_t i = 0; i < planar_polygons.size(); i++)
    {
      const auto& c = planar_polygons[i].getCoefficients();
      ROS_INFO(" [%02zu] %.3f %.3f %.3f :: %.3f", i, c(0), c(1), c(2), c(3));
    }

    visualization_msgs::MarkerArray ma;
    buildPolygonMarkerArray(planar_polygons, ma);
    marker_publisher_.publish(ma);
  }

private:

    ReconfigurablePlaneExtraction rpe_;
    ros::Publisher marker_publisher_;

};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "plane_extraction");

  if (argc != 1 && argc != 3)
  {
    ROS_ERROR("Usage: %s [min_z max_z]", argv[0]);
    return 1;
  }

  TestNode tn;

  if (argc == 3)
    tn.setupPassThroughFilter(argv[1], argv[2]);

  ros::spin();

  return 0;
}
