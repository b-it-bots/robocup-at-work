/*
 * geometric_properties.h
 *
 *  Created on: Apr 29, 2011
 *      Author: fred
 */

#ifndef GEOMETRIC_PROPERTIES_H_
#define GEOMETRIC_PROPERTIES_H_

#include <pcl/filters/passthrough.h>

class GeometricProperties
{
public:
	template<typename PointT>
	static void getCentroid3D(const pcl::PointCloud<PointT> &pcl_cloud_input, Eigen::Vector4f &centroid)
	{
		// Initialize to 0
		centroid.setZero();
		if(pcl_cloud_input.points.empty())
			return;
		// For each point in the cloud
		int cp = 0;

		// If the data is dense, we don't need to check for NaN
		if(pcl_cloud_input.is_dense)
		{
			for(size_t i = 0; i < pcl_cloud_input.points.size(); ++i)
				centroid += pcl_cloud_input.points[i].getVector4fMap();
			centroid[3] = 0;
			centroid /= pcl_cloud_input.points.size();
		}
		// NaN or Inf values could exist => check for them
		else
		{
			for(size_t i = 0; i < pcl_cloud_input.points.size(); ++i)
			{
				// Check if the point is invalid
				if(!pcl_isfinite(pcl_cloud_input.points[i].x) || !pcl_isfinite(pcl_cloud_input.points[i].y) || !pcl_isfinite(pcl_cloud_input.points[i].z))
					continue;

				centroid += pcl_cloud_input.points[i].getVector4fMap();
				cp++;
			}
			centroid[3] = 0;
			centroid /= cp;
		}
	}
};

#endif /* GEOMETRIC_PROPERTIES_H_ */
