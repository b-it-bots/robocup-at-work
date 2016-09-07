/* 
 * Copyright [2015] <Bonn-Rhein-Sieg University>  
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array  
 * of poses and calculates the path lenght based on the distance between two points of each pose.  
 * 
 */

#include <mcr_navigation_tools/path_length_calculator.h>

PathLengthCalculator::PathLengthCalculator()
{
    is_plan_set_ = false;
}

PathLengthCalculator::PathLengthCalculator(const nav_msgs::Path &plan)
{
    plan_ = plan;
    is_plan_set_ = true;
}

void PathLengthCalculator::setPath(const nav_msgs::Path &plan)
{
    plan_.poses.clear();
    plan_ = plan;
    is_plan_set_ = true;
}

double PathLengthCalculator::computeLength()
{
    if (!is_plan_set_)
    {
        // plan not set
        return -1.0;
    }

    if (plan_.poses.size() == 0)
    {
        // empty plan received
        return -1.0;
    }

    double accumulated_sum = 0;
    Eigen::Vector4f previous(0, 0, 0, 0);
    Eigen::Vector4f actual(0, 0, 0, 0);

    // iterate over all plan poses to compute euclidean distance between all and sum them
    for (int i = 0; i < plan_.poses.size() - 1 ; i++)
    {
        previous[0] = plan_.poses[i].pose.position.x;
        previous[1] = plan_.poses[i].pose.position.y;

        actual[0] = plan_.poses[i+1].pose.position.x;
        actual[1] = plan_.poses[i+1].pose.position.y;

        // compute distance between two points using pcl library
        accumulated_sum += pcl::distances::l2(previous, actual);
    }

    return accumulated_sum;
}
