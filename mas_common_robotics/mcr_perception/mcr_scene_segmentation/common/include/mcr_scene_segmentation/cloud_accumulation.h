#ifndef CLOUD_ACCUMULATION_H
#define CLOUD_ACCUMULATION_H

#include <pcl/octree/octree_pointcloud_occupancy.h>

#include "aliases.h"
#include "octree_pointcloud_occupancy_colored.h"

/** This class accumulates input point clouds in the occupancy octree with a
  * given spatial resolution. */
class CloudAccumulation
{

public:

  typedef std::unique_ptr<CloudAccumulation> UPtr;

  explicit CloudAccumulation(double resolution = 0.0025);

  void addCloud(const PointCloud::ConstPtr& cloud);

  void getAccumulatedCloud(PointCloud& cloud);

  int getCloudCount() const { return cloud_count_; }

  void reset();

private:

  typedef OctreePointCloudOccupancyColored<PointT> Octree;
  typedef std::unique_ptr<Octree> OctreeUPtr;

  OctreeUPtr octree_;

  int cloud_count_;
  double resolution_;

};

#endif /* CLOUD_ACCUMULATION_H */

