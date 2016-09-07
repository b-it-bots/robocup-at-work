/*
 * geometric_distances.hpp
 *
 *  Created on: Mar 28, 2011
 *      Author: Frederik Hegger
 */

#ifndef GEOMETRIC_DISTANCES_HPP_
#define GEOMETRIC_DISTANCES_HPP_

#include <math.h>

class GeometricDistances
{
public:
    static double getEuclideanDistance2D(double point1_x, double point1_y, double point2_x, double point2_y)
    {
        return sqrt(pow(point1_x - point2_x, 2.0) + pow(point1_y - point2_y, 2.0));
    }

    template <typename PointT>
    static double getEuclideanDistance3D(PointT point1, PointT point2)
    {
        double delta_x = point1.x - point2.x;
        double delta_y = point1.y - point2.y;
        double delta_z = point1.z - point2.z;

        return (sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0) + pow(delta_z, 2.0)));
    }

    template <typename PointT>
    static double getManhattanDistance3D(PointT point1, PointT point2)
    {
        double delta_x = fabs(point1.x - point2.x);
        double delta_y = fabs(point1.y - point2.y);
        double delta_z = fabs(point1.z - point2.z);

        return (delta_x + delta_y + delta_z);
    };
};

#endif /* GEOMETRIC_DISTANCES_HPP_ */
