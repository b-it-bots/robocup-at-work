#ifndef RECONFIGURABLE_PLANE_EXTRACTION_HPP
#define RECONFIGURABLE_PLANE_EXTRACTION_HPP

#include <functional>

#include <dynamic_reconfigure/server.h>

#include "mcr_scene_segmentation/PlaneExtractionConfig.h"
#include "mcr_scene_segmentation/plane_extraction.h"

/** This class encapsulates PlaneExtraction object and augments it with a
  * dynamic reconfigure server which could be used to easily tweak the
  * parameters. User-defined callback will be executed on every update from
  * dynamic reconfigure server. */
class ReconfigurablePlaneExtraction
{

public:

  typedef std::function<void(void)> ReconfigureCallback;

  ReconfigurablePlaneExtraction(ReconfigureCallback callback = ReconfigureCallback())
  : callback_(callback)
  {
    server_.setCallback(boost::bind(&ReconfigurablePlaneExtraction::serverReconfigureCallback, this, _1, _2));
    plane_extraction_ptr.reset(new PlaneExtraction);
  }

  std::unique_ptr<PlaneExtraction> plane_extraction_ptr;
  std::unique_ptr<Eigen::Vector3f> plane_normal_ptr;

private:

  void serverReconfigureCallback(mcr_scene_segmentation::PlaneExtractionConfig &config, uint32_t level)
  {
    ROS_INFO("Reconfiguring PlaneExtraction module...");
    plane_extraction_ptr.reset(new PlaneExtraction(config.normal_max_depth_change_factor,
                                                   config.normal_smoothing_size,
                                                   config.min_inliers,
                                                   pcl::deg2rad(config.angular_threshold),
                                                   config.distance_threshold,
                                                   config.maximum_curvature,
                                                   config.refinement_threshold,
                                                   config.refinement_depth_independent));
    if (config.apply_angular_constraints)
    {
      plane_normal_ptr.reset(new Eigen::Vector3f(config.constraint_normal_x, config.constraint_normal_y, config.constraint_normal_z));
      plane_normal_ptr->normalize();
      double threshold = pcl::deg2rad(config.constraint_angular_threshold);
      if (config.apply_distance_constraints)
      {
        plane_extraction_ptr->setPlaneConstraints(*plane_normal_ptr, threshold, config.constraint_distance, config.constraint_distance_threshold);
      }
      else
      {
        plane_extraction_ptr->setPlaneConstraints(*plane_normal_ptr, threshold);
      }
    }
    else
    {
      plane_normal_ptr.reset();
    }
    if (callback_)
      callback_();
  }

  dynamic_reconfigure::Server<mcr_scene_segmentation::PlaneExtractionConfig> server_;
  ReconfigureCallback callback_;

};

#endif /* RECONFIGURABLE_PLANE_EXTRACTION_HPP */

