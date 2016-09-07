#ifndef LASERSCANLINEARREGRESSION_H_
#define LASERSCANLINEARREGRESSION_H_

namespace LaserScanLinearRegression
{

class ScanItem
{
public:
    ScanItem()
    {
    }
    virtual ~ScanItem()
    {
    }

    double angle;
    double distance;

    double x()
    {
        return distance * cos(angle);
    }

    double y()
    {
        return distance * sin(angle);
    }
};

class ScanItemFilter
{

public:
    ScanItemFilter()
    {
    }
    virtual ~ScanItemFilter()
    {
    }

    std::vector<ScanItem> filterByDistance(std::vector<ScanItem> items, double minDistance, double maxDistance);

    std::vector<ScanItem> filterByAngle(std::vector<ScanItem> items, double minAngle, double maxAngle);

    std::vector<ScanItem> filterMidAngle(std::vector<ScanItem> items, double angleFromCenter);

};

class RegressionAnalysis
{

public:
    RegressionAnalysis()
    {
    }
    virtual ~RegressionAnalysis()
    {
    }

    bool calculateCoefficient(std::vector<ScanItem> items, double& meanX, double& meanY, double &slope);

};
}

#endif /* LASERSCANLINEARREGRESSION_H_ */
