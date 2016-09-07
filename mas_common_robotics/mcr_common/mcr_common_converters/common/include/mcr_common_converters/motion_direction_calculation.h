/*
 * Copyright [2015] <Bonn-Rhein-Sieg University>
 * motion_direction_calculation.h
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#ifndef MCR_COMMON_CONVERTERS_MOTION_DIRECTION_CALCULATION_H_
#define MCR_COMMON_CONVERTERS_MOTION_DIRECTION_CALCULATION_H_

#include <math.h>

/**
 * \brief compute direction of motion from a given linear (x, y) and angular velocity (z) in 2D.
 * \param linear_x linear velocity on the x axis in meter per second
 * \param linear_y linear velocity on the y axis in meter per second
 * \param angular_z angular velocity around the z axis in radians per second
 * return the angle of the motion direction in the x,y plane
 */
double getMotionDirectionFromTwist2D(const double &linear_x, const double &linear_y, const double &angular_z);

#endif /* MCR_COMMON_CONVERTERS_MOTION_DIRECTION_CALCULATION_H_ */
