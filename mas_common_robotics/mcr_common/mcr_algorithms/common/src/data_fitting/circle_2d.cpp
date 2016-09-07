/*
 * circle_2d.cpp
 *
 *  Created on: Jun 15, 2011
 *      Author: Frederik Hegger and Thomas Breuer
 */

#include "mcr_algorithms/data_fitting/circle_2d.h"

Circle2D::Circle2D(unsigned int max_iterations = 256, double tolerance = 1e-06)
{
    this->max_iterations_ = max_iterations;
    this->tolerance_ = tolerance;
}

void Circle2D::fitCircle(vector<geometry_msgs::Point>& data_points, double& radius, double& center_x, double& center_y, double& residual_sum_of_squares)
{
    //fit circle
    CvMat* A = cvCreateMat(data_points.size(), 3, CV_64FC1);
    CvMat* B = cvCreateMat(data_points.size(), 1, CV_64FC1);

    int j = 0;
    for (unsigned int k = 0; k < data_points.size(); ++k)
    {
        float x = data_points.at(k).x;
        float y = data_points.at(k).y;

        cvmSet(A, j, 0, -2.0 * x);
        cvmSet(A, j, 1, -2.0 * y);
        cvmSet(A, j, 2, 1);

        cvmSet(B, j, 0, -pow(x, 2) - pow(y, 2));
        j++;
    }

    CvMat* sol = cvCreateMat(3, 1, CV_64FC1);

    cvSolve(A, B, sol, CV_SVD);

    float xc = cvmGet(sol, 0, 0);
    float yc = cvmGet(sol, 1, 0);
    radius = sqrt(pow(xc, 2) + pow(yc, 2) - cvmGet(sol, 2, 0));

    cvReleaseMat(&A);
    A = 0;
    cvReleaseMat(&B);
    B = 0;
    cvReleaseMat(&sol);
    sol = 0;

    residual_sum_of_squares = 0;

    for (unsigned int l = 0; l < data_points.size(); ++l)
        residual_sum_of_squares += pow(radius - sqrt(pow(xc - data_points.at(l).x, 2) + pow(yc - data_points.at(l).y, 2)), 2);
}
