/*  
* Copyright [2015] <Bonn-Rhein-Sieg University>  
*  
* Author: Oscar Lima (olima_84@yahoo.com)
* 
* Listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array  
* of poses and calculates the path lenght based on the distance between two points of each pose.  
*   
*/ 

#ifndef MCR_NAVIGATION_TOOLS_PATH_LENGTH_CALCULATOR_H
#define MCR_NAVIGATION_TOOLS_PATH_LENGTH_CALCULATOR_H

#include <math.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <pcl/registration/distances.h>

class PathLengthCalculator
{
    public:
        PathLengthCalculator();

        explicit PathLengthCalculator(const nav_msgs::Path &plan);

        // to set the value of member variable plan_ from outside of the class
        void setPath(const nav_msgs::Path &plan);

        // calculates the length of a path
        double computeLength();

    private:
        // to store the plan path (array of poses) from which the distance will be calculated
        nav_msgs::Path plan_;

        // flag to control if computeLength function can be called
        bool is_plan_set_;
};

#endif  // MCR_NAVIGATION_TOOLS_PATH_LENGTH_CALCULATOR_H
