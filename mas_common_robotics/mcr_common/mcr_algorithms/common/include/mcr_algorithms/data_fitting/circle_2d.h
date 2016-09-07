/*
 * circle_2d.h
 *
 *  Created on: Jun 15, 2011
 *      Author: Frederik Hegger and Thomas Breuer
 */

#ifndef CIRCLE_2D_H_
#define CIRCLE_2D_H_

#include <vector>
#include <geometry_msgs/Point.h>
#include <opencv/cv.h>

using namespace std;

/**
 * @class Circle2D
 * @brief this class fits a circle to the data in vecstrDataPoints and returns the x and y coordinate of the center point, the radius of the fitted circle
 * and the residual sum of squares.
 */
class Circle2D
{
public:
    Circle2D(unsigned int max_iterations, double tolerance);
    void fitCircle(vector<geometry_msgs::Point>& data_points, double& radius, double& center_x, double& center_y, double& residual_sum_of_squares);

private:
    unsigned int max_iterations_;
    double tolerance_;
};

#endif /* CIRCLE_2D_H_ */
