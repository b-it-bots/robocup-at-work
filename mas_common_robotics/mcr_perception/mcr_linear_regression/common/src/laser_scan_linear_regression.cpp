#include <vector>
#include <math.h>
#include <iostream>

#include "mcr_linear_regression/laser_scan_linear_regression.h"

namespace LaserScanLinearRegression
{

	std::vector<ScanItem> ScanItemFilter::filterByDistance(std::vector<ScanItem> items, double minDistance, double maxDistance)
	{
		std::vector<ScanItem> filteredData;

		for (unsigned int i = 0; i < items.size(); i++)
		{
			if (items[i].distance >= minDistance && items[i].distance <= maxDistance)
			{
				filteredData.push_back(items[i]);
			}
		}

		return filteredData;
	}

	std::vector<ScanItem> ScanItemFilter::filterByAngle(std::vector<ScanItem> items, double minAngle, double maxAngle)
	{
		std::vector<ScanItem> filteredData;

		for (unsigned int i = 0; i < items.size(); i++)
		{
			if (items[i].angle >= minAngle && items[i].angle <= maxAngle)
			{
				filteredData.push_back(items[i]);
			}
		}

		return filteredData;
	}

	std::vector<ScanItem> ScanItemFilter::filterMidAngle(std::vector<ScanItem> items, double angleFromCenter)
	{
		std::vector<ScanItem> filteredData;

		for (unsigned int i = 0; i < items.size(); i++)
		{
			if (fabs(items[i].angle) >= angleFromCenter)
			{
				filteredData.push_back(items[i]);
			}
		}

		return filteredData;
	}

	bool RegressionAnalysis::calculateCoefficient(std::vector<ScanItem> items, double& center, double& a, double &b)
	{

		double Sxx = 0.0;
		double Sxy = 0.0;
		double Syy = 0.0;
		double xm = 0.0;
		double ym = 0.0;

		center = 0;
		a = 0;
		b = 0;

		if (items.size() == 0)
		{
			return false;
		}
		//std::cout << "Items: " << items.size() << std::endl;

		for (unsigned int i = 0; i < items.size(); i++)
		{
			ScanItem it = items[i];
			xm += it.x();
			ym += it.y();
			//std::cout << "Item : " << it.x() << ", " << it.y() << " - " << it.angle << " - " << it.distance << std::endl;
		}

		xm /= items.size();
		ym /= items.size();

		for (unsigned int i = 0; i < items.size(); i++)
		{
			ScanItem it = items[i];

			Sxx += (it.x() - xm) * (it.x() - xm);
			Sxy += (it.x() - xm) * (it.y() - ym);
			Syy += (it.y() - ym) * (it.y() - ym);

		}
		//std::cout << "Sxx: " << Sxx << std::endl;
		//std::cout << "Sxy: " << Sxy << std::endl;
		//std::cout << "Syy: " << Syy << std::endl;

		//b = Sxx / Sxy;
		b = Sxy / Syy;
		a = xm - b * ym;
		center = ym;

		//std::cout << "a: " << a << std::endl;
		//std::cout << "b: " << b << std::endl;
		//std::cout << "center: " << ym << std::endl;

		return true;
	}

}
