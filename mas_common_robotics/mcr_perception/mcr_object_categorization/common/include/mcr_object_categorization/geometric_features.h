/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef SGEOMETRICFEATURES_H_
#define SGEOMETRICFEATURES_H_

#include <map>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

struct SGeometricFeatures
{
    pcl::PointCloud<pcl::PointXYZ> objectPointCloud; //
    std::map<node*, double> meanApspDistances; // mean allpair distance to each node
    std::map<node*, std::vector<double> > apspDistances; //all all pair
    int numPoints; //
    int numNodes; //
    int numEdges;
    pcl::PointXYZ objectPointCloudCentroid;
    double meanASPS;
    double distBtwMostDecentralNodes_ASPS;
    double distBtwMostDecentralNodes_Direct; // like euclidean distance
};

#endif /* SGEOMETRICFEATURES_H_ */
