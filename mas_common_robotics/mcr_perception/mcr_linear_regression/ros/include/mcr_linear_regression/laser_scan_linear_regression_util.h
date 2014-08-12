/*
 * LaserScanLinearRegressionUtil.h
 *
 *  Created on: Jun 12, 2012
 *      Author: matthias
 */

#ifndef LASERSCANLINEARREGRESSIONUTIL_H_
#define LASERSCANLINEARREGRESSIONUTIL_H_

#include <vector>

#include <sensor_msgs/LaserScan.h>
#include "mcr_linear_regression/laser_scan_linear_regression.h"

class LaserScanLinearRegressionUtil
{
 public:
	LaserScanLinearRegressionUtil();
	virtual ~LaserScanLinearRegressionUtil();

	std::vector<LaserScanLinearRegression::ScanItem> convert(sensor_msgs::LaserScanConstPtr scan, double angle_offset);

};

#endif /* LASERSCANLINEARREGRESSIONUTIL_H_ */
