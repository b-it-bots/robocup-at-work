/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef __CKEYPOINT_H__
#define __CKEYPOINT_H__

#include <iostream>
#include <vector>

class CKeyPoint
{
private:
    double x_cord;
    double y_cord;
    double a;
    double b;
    double c;
    std::vector<double> siftData;
    int featureDimension;
    int clusterLabel;
    int assignedToRegion;
    std::vector<double> clusterDistances;
    std::vector<int> rankedClusterDistancesIdx; //sorted indicies of ClusterDistances
public:
    CKeyPoint(int param_featDim);
    double get_x_cord(void);
    double get_y_cord(void);
    double get_a(void)
    {
        return a;
    };
    double get_b(void)
    {
        return b;
    };
    double get_c(void)
    {
        return c;
    };
    void set_x_cord(double param_x_cord);
    void set_y_cord(double param_y_cord);
    void set_a(double param_a);
    void set_b(double param_b);
    void set_c(double param_c);
    void add_feature(int index, double value);
    void set_clusterLabel(int param_clusterLabel);
    void setAssignedToRegion(int assignedToRegion);
    int getAssignedToRegion();

    std::vector<double> &get_siftData()
    {
        return siftData;
    };
    int& getClusterLabel()
    {
        return clusterLabel;
    }
    double getFeature(int index)
    {
        return  siftData[index];
    }

    void setClusterDistances(std::vector<double> clusterDistances);
    std::vector<double>& getClusterDistances();

    void setRankedClusterDistancesIdx(std::vector<int> rankedClusterDistancesIdx);
    std::vector<int>& getRankedClusterDistancesIdx();

    double getClusterDistances(int &index);
    int getRankedClusterDistancesIdx(int &index);

    ~CKeyPoint();
};

#endif
