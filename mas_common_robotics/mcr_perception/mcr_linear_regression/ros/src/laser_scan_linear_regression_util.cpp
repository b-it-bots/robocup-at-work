#include "mcr_linear_regression/laser_scan_linear_regression_util.h"

LaserScanLinearRegressionUtil::LaserScanLinearRegressionUtil()
{

}

LaserScanLinearRegressionUtil::~LaserScanLinearRegressionUtil()
{

}

std::vector<LaserScanLinearRegression::ScanItem> LaserScanLinearRegressionUtil::convert(sensor_msgs::LaserScanConstPtr scan, double angle_offset)
{
	std::vector<LaserScanLinearRegression::ScanItem> data;

	if (scan == 0)
	{
		return data;
	}

	LaserScanLinearRegression::ScanItem tmpItem;
	for (unsigned int i = 0; i < scan->ranges.size(); i++)
	{
		tmpItem.angle = angle_offset + scan->angle_min + scan->angle_increment * i;
		tmpItem.distance = scan->ranges[i];
		data.push_back(tmpItem);
	}

	return data;
}
