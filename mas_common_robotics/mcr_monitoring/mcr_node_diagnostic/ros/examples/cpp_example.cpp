/*
 * cpp_example.cpp
 *
 *  Created on: May 23, 2013
 *      Author: matthias
 */


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


#include <mcr_node_diagnostic/NodeDiagnostic.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "cpp_example");

    ros::NodeHandle nodeHandle;

    std::vector<diagnostic_msgs::KeyValue> keyValues;
    keyValues.push_back(mcr::NodeDiagnostic::keyValue("key", "value"));

    mcr::NodeDiagnostic::warn("IK", mcr::NodeDiagnostic::COMMUNICATION, "IK solver service is missing",
                              keyValues);

    usleep(5000000);

    mcr::NodeDiagnostic::reset("IK");

}

