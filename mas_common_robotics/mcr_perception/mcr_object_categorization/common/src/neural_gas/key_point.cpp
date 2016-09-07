/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include "neural_gas/key_point.h"


using namespace std;

CKeyPoint::CKeyPoint(int param_featDim)
{
    featureDimension = param_featDim;
    siftData.resize(param_featDim);
}

void CKeyPoint::add_feature(int index, double value)
{
    siftData[index] = value;
}


void CKeyPoint::set_x_cord(double param_x_cord)
{
    x_cord = param_x_cord;
}

void CKeyPoint::set_y_cord(double param_y_cord)
{
    y_cord = param_y_cord;
}

void CKeyPoint::set_a(double param_a)
{
    a = param_a;
}

void CKeyPoint::set_b(double param_b)
{
    b = param_b;
}

void CKeyPoint::set_c(double param_c)
{
    c = param_c;
}
double CKeyPoint::get_x_cord(void)
{
    return x_cord;
}

double CKeyPoint::get_y_cord(void)
{
    return y_cord;
}

void CKeyPoint::set_clusterLabel(int param_clusterLabel)
{
    clusterLabel = param_clusterLabel;
}

CKeyPoint::~CKeyPoint()
{
    //commented out since it causes troubles in ICE implementation
    ;//this->siftData.~CDataVector();
}
void CKeyPoint::setClusterDistances(std::vector<double> clusterDistances)
{
    this->clusterDistances = clusterDistances;
}
std::vector<double>& CKeyPoint::getClusterDistances()
{
    return this->clusterDistances;
}

double CKeyPoint::getClusterDistances(int &index)
{
    return this->clusterDistances.at(index);
}

void CKeyPoint::setRankedClusterDistancesIdx(std::vector<int> rankedClusterDistancesIdx)
{
    this->rankedClusterDistancesIdx = rankedClusterDistancesIdx;
}
std::vector<int>& CKeyPoint::getRankedClusterDistancesIdx()
{
    return this->rankedClusterDistancesIdx;
}
