#include <string>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <dynamic_reconfigure/server.h>

#include "mcr_scene_segmentation/PlaneExtractionTestConfig.h"
#include "plane_extraction.h"
#include "test_base.hpp"

class PlaneExtractionTest : public TestBase
{

public:

    PlaneExtractionTest(PointCloud::ConstPtr cloud)
    : TestBase()
    , cloud_(cloud)
    {
      server_.setCallback(boost::bind(&PlaneExtractionTest::reconfigure_callback, this, _1, _2));
      pe_.reset(new PlaneExtraction);
    }

protected:

    void process()
    {
      PlanarPolygonVector planar_polygons;

      pe_->setInputCloud(cloud_);

      MEASURE_RUNTIME(pe_->extract(planar_polygons), "Plane extraction");

      std::cout << "Number of planes extracted: " << planar_polygons.size() << std::endl;
      for (const auto& planar_polygon : planar_polygons)
      {
        std::cout << "---\n";
        std::cout << "Points in contour: " << planar_polygon.getContour().size() << std::endl;
        std::cout << "Plane coefficients:\n" << planar_polygon.getCoefficients() << std::endl;
      }

      viewer_.removeAllPointClouds(0);
      viewer_.removeAllShapes(0);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
      viewer_.addPointCloud<PointT>(cloud_, single_color, "cloud");

      if (plane_normal_)
        displayNormal(*plane_normal_);

      displayPlanarPolygons(planar_polygons);
    }

    void reconfigure_callback(mcr_scene_segmentation::PlaneExtractionTestConfig &config, uint32_t level)
    {
      pe_.reset(new PlaneExtraction(config.normal_max_depth_change_factor,
                                    config.normal_smoothing_size,
                                    config.min_inliers,
                                    pcl::deg2rad(config.angular_threshold),
                                    config.distance_threshold,
                                    config.maximum_curvature,
                                    config.refinement_threshold,
                                    config.refinement_depth_independent));
      if (config.apply_angular_constraints)
      {
        plane_normal_.reset(new Eigen::Vector3f(config.constraint_normal_x, config.constraint_normal_y, config.constraint_normal_z));
        plane_normal_->normalize();
        double threshold = pcl::deg2rad(config.constraint_angular_threshold);
        if (config.apply_distance_constraints)
        {
          pe_->setPlaneConstraints(*plane_normal_, threshold, config.constraint_distance, config.constraint_distance_threshold);
        }
        else
        {
          pe_->setPlaneConstraints(*plane_normal_, threshold);
        }
      }
      else
      {
        plane_normal_.reset();
      }
      process();
    }

    dynamic_reconfigure::Server<mcr_scene_segmentation::PlaneExtractionTestConfig> server_;

    std::unique_ptr<PlaneExtraction> pe_;
    PointCloud::ConstPtr cloud_;

    std::unique_ptr<Eigen::Vector3f> plane_normal_;

};

int main (int argc, char** argv)
{
  ros::init(argc, argv, "plane_extraction");

  if (argc != 2 && argc != 4)
  {
    ROS_ERROR("Usage: %s <filename.pcd> [min_z max_z]", argv[0]);
    return 1;
  }

  PointCloud::Ptr cloud(new PointCloud);
  pcl::io::loadPCDFile(argv[1], *cloud);

  if (argc == 4)
  {
    float min_z = boost::lexical_cast<float>(argv[3]);
    float max_z = boost::lexical_cast<float>(argv[4]);
    pcl::PassThrough<PointT> pass_through;
    pass_through.setFilterFieldName("z");
    pass_through.setFilterLimits(min_z, max_z);
    pass_through.setKeepOrganized(true);
    pass_through.setInputCloud(cloud);
    pass_through.filter(*cloud);
    ROS_INFO("Keep points in %.2f to %.2f range.", min_z, max_z);
  }

  PlaneExtractionTest pet(cloud);
  pet.run();

  return 0;
}
