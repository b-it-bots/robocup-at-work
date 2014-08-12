/*
 * conversions.h
 *
 *  Created on: Apr 13, 2011
 *      Author: Frederik Hegger
 */

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <math.h>

class Conversions
{
public:
	static double rad2degree(const double radian);
	static double degree2rad(const double degree);
	static void polar2cartesian2D(const double distance, const double angle, double &x, double &y);
	static void cartesian2polar2D(const double x, const double y, double &distance, double &angle);
	static double correctAngle(const double angle);
};

#endif /* CONVERSIONS_H_ */
