/*
 * conversions.cpp
 *
 *  Created on: Apr 13, 2011
 *      Author: Frederik hegger
 */

#include "mcr_algorithms/geometry/conversions.h"

double Conversions::rad2degree(const double radian)
{
    return (radian * (180.0 / M_PI));
}

double Conversions::degree2rad(const double degree)
{
    return (degree * (M_PI / 180.0));
}

void Conversions::polar2cartesian2D(const double distance, const double angle, double &x, double &y)
{
    x = distance * cos(angle);
    y = distance * sin(angle);
}

void Conversions::cartesian2polar2D(const double x, const double y, double &distance, double &angle)
{
    distance = sqrt(pow(x, 2.0) + pow(y, 2.0));
    angle = atan(y / x);
}

double correctAngle(double angle)
{
    if (angle > M_PI)
        return (angle - (2.0 * M_PI));
    else if (angle < -M_PI)
        return (angle + (2.0 * M_PI));
    else
        return angle;
}

