#include <math.h>
#include <iostream>

#include <ros/ros.h>

#include <mcr_perception_msgs/BaseScanLinearRegression.h>
#include "mcr_linear_regression/laser_scan_linear_regression.h"
#include "mcr_linear_regression/laser_scan_linear_regression_util.h"

using namespace mcr_perception_msgs;

class LaserScanLinearRegressionService
{
protected:

    ros::NodeHandle nh;

    std::string basescan_topic;
    ros::Duration time_to_wait;

    LaserScanLinearRegression::ScanItemFilter scanfilter;
    LaserScanLinearRegression::RegressionAnalysis regAnalysis;
    LaserScanLinearRegressionUtil util;
public:

    LaserScanLinearRegressionService(ros::NodeHandle nh, std::string basescan_topic)
    {
        this->nh = nh;
        this->basescan_topic = basescan_topic;
        time_to_wait = ros::Duration(2.0);
    }

    bool baseScanLinearRegression(BaseScanLinearRegressionRequest& req, BaseScanLinearRegressionResponse& res)
    {

        if (fabs(req.filter_maxAngle) <= 0.0001 && fabs(req.filter_minAngle) <= 0.0001)
        {
            req.filter_minAngle = -M_PI_4;
            req.filter_maxAngle = M_PI_4;
        }

        if (fabs(req.filter_maxDistance) <= 0.0001 && fabs(req.filter_minDistance) <= 0.0001)
        {
            req.filter_minDistance = 0.02;
            req.filter_maxDistance = 0.8;
        }

        //FIXME: check if the angle object is youBot/hokoyu dependend
        double angle_offset = -0.025;

        ROS_DEBUG("Wait for base_scan message");

        double sum_center = 0.0;
        double sum_a = 0.0;
        double sum_b = 0.0;

        int iterations = 3;

        for (int i = 0; i < iterations; i++)
        {
            sensor_msgs::LaserScanConstPtr scan = ros::topic::waitForMessage < sensor_msgs::LaserScan > (basescan_topic, nh, time_to_wait);

            if (scan == 0)
            {
                ROS_ERROR("No scan received within timeframe on topic %s", basescan_topic.c_str());
                return false;
            }

            std::vector<LaserScanLinearRegression::ScanItem> data = util.convert(scan, angle_offset);

            std::vector<LaserScanLinearRegression::ScanItem> filtered_data = scanfilter.filterByDistance(data, req.filter_minDistance, req.filter_maxDistance);
            filtered_data = scanfilter.filterByAngle(filtered_data, req.filter_minAngle, req.filter_maxAngle);
            filtered_data = scanfilter.filterMidAngle(filtered_data, 0.2);

            double center, a, b;

            ROS_DEBUG("Calculating Coeffs");

            regAnalysis.calculateCoefficient(filtered_data, center, a, b);

            sum_center += center;
            sum_a += a;
            sum_b += b;

        }

        sum_center /= iterations;
        sum_a /= iterations;
        sum_b /= iterations;

        ROS_DEBUG("a: %f, b: %f, c: %f", sum_a, sum_b, sum_center);

        res.center = sum_center;
        res.a = sum_a;
        res.b = sum_b;

        return true;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear_regression");
    ros::NodeHandle n;

    std::string topic;
    ros::param::param<std::string>("~scan_topic", topic, "/scan_front");

    ROS_DEBUG("Listen to scan_topic: %s", topic.c_str());


    LaserScanLinearRegressionService lslrs(n, topic);

    std::string service_name = topic + std::string("_linearregression");

    ros::ServiceServer service = n.advertiseService(service_name.c_str(), &LaserScanLinearRegressionService::baseScanLinearRegression, &lslrs);

    ROS_DEBUG("LaserScanLinearRegressionService is ready");

    ros::Rate loop_rate(15);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
