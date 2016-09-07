/* 
 * Copyright [2015] <Bonn-Rhein-Sieg University>  
 * motion_direction_calculation.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: fred
 */

#include <mcr_common_converters/motion_direction_calculation.h>

#include <algorithm>

double getMotionDirectionFromTwist2D(const double &linear_x, const double &linear_y, const double &angular_z)
{
    const double max_allowed_angular_angle = M_PI;

    double motion_direction = 0.0;
    double linear_velocity = 0.0, angular_velocity = 0.0;
    double linear_distance = 0.0, linear_angle = 0.0;
    double angular_linear_velocity = 0.0, angular_linear_angle = 0.0;
    double angular_angle = 0.0;
    double radius = 0.0;

    // NOTE: the following calculation neglect the time, i.e. we consider the distance/angle traveled in one second.

    /* 
     * calculate the resulting motion vector for linear x and linear y velocities, 
     * d.h. the polar coordinates for the point (x,y)
     */
    linear_velocity = linear_distance = sqrt((pow(linear_x, 2) + pow(linear_y, 2)));
    linear_angle = atan2(linear_y, linear_x);

    angular_angle = angular_velocity = angular_z;

    // if there is no motion, return 0.0
    if ((linear_velocity == 0) && (angular_velocity == 0))
        return 0.0;

    /*
     * TODO: this is wrong. We need to account for the velocity and the maximum range here, 
     * i.e. how far we can look, e.g. only 90 or 180
     */
    if (angular_velocity == 0)
        return linear_angle;

    /*
     * take the actual angular angle and limit it to be max. the value specified 
     * by "max_allowed_angular_angle" and keep the original sign
     */
    if (linear_velocity == 0)
        return (std::min(fabs(angular_velocity), max_allowed_angular_angle) * (angular_angle / fabs(angular_angle)));

    else
    {
        // calculate the radius of the circle which is determined by the linear and angular velocity
        radius = linear_velocity / angular_velocity;

        // calculate the length of the hypotenuse of the isosceles traingle
        angular_linear_velocity = sqrt((2 * pow(radius, 2) * (1 - cos(angular_angle))));

        // get the opposite angle of the angle between the hypotenuse and one side
        angular_linear_angle = (M_PI / 2) - ((M_PI - angular_angle) / 2);

        /*
         * add the two angles, i.e. the angle calculated from the linear motion and the 
         * linear angle from the angular motion
         */
        motion_direction = angular_linear_angle + linear_angle;

        // check if calculated angle is with the range of the specified one (max_allowed_angular_angle)
        if (fabs(motion_direction) > max_allowed_angular_angle)
            motion_direction = max_allowed_angular_angle * (motion_direction / fabs(motion_direction));

        return motion_direction;
    }

    // should never reach this point
    return 0.0;
}
