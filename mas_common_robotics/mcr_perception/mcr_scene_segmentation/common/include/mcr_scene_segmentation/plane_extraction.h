#ifndef PLANE_EXTRACTION_H_
#define PLANE_EXTRACTION_H_

#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

#include "aliases.h"

/** This class performs multiplane segmentation and (optionally) filtering of
  * the extracted planes.
  *
  * The input point cloud is processed with OrganizedMultiplaneSegmentation
  * class from PCL, which outputs several planar regions. These regions are
  * further filtered to remove those that do not satisfy the constraints (such
  * as normal orientation or distance to the origin). Finally a convex hull
  * around each region is computed and the corresponding planar polygons are
  * output to the user. */
class PlaneExtraction
{

public:

  /** Default constructor will setup all involved PCL classes with sensible
    * default parameters. */
  PlaneExtraction();

  /** This constructor allows to set parameters of all involved PCL classs. */
  PlaneExtraction(double normal_max_depth_change_factor,
                  double normal_smoothing_size,
                  unsigned int min_inliers,
                  double angular_threshold,
                  double distance_threshold,
                  double maximum_curvature,
                  double refinement_threshold,
                  double refinement_depth_dependent);

  void extract(PlanarPolygonVector& planar_polygons);

  inline void setInputCloud(const PointCloud::ConstPtr &cloud)
  {
    input_ = cloud;
  }

  inline PointCloud::ConstPtr getInputCloud() const
  {
    return input_;
  }

  inline void setPlaneConstraints(const Eigen::Vector3f& normal, double angular_threshold)
  {
    apply_angular_constraints_ = true;
    apply_distance_constraints_ = false;
    plane_normal_ = normal;
    angular_threshold_ = angular_threshold;
  }

  inline void setPlaneConstraints(const Eigen::Vector3f& normal, double angular_threshold,
                                  double distance, double distance_threshold)
  {
    apply_angular_constraints_ = true;
    apply_distance_constraints_ = true;
    plane_normal_ = normal;
    angular_threshold_ = angular_threshold;
    distance_ = distance;
    distance_threshold_ = distance_threshold;
  }

  inline void removePlaneConstraints()
  {
    apply_angular_constraints_ = false;
    apply_distance_constraints_ = false;
  }

  inline void setSortByArea(bool sort)
  {
    sort_by_area_ = sort;
  }

  inline bool getSortByArea() const
  {
    return sort_by_area_;
  }

  virtual ~PlaneExtraction()
  {
    input_.reset();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  PointCloud::ConstPtr input_;

  pcl::IntegralImageNormalEstimation<PointT, PointNT> ne_;
  pcl::OrganizedMultiPlaneSegmentation<PointT, PointNT, PointLT> mps_;
  pcl::ProjectInliers<PointT> pi_;

  double angular_threshold_;
  double distance_threshold_;

  Eigen::Vector3f plane_normal_;
  double distance_;

  bool apply_angular_constraints_;
  bool apply_distance_constraints_;

  bool sort_by_area_;

};

#endif
