/*
 * File:   CSAEToolBox.h
 * Author: Christian Mueller
 *
 * Created on May 31, 2011, 11:27 PM
 */

#ifndef CSAETOOLBOX_H
#define CSAETOOLBOX_H

#include<cstdlib>
#include<vector>

class CSAEToolBox
{
public:
    CSAEToolBox();
    static std::vector<double> computeMeanVector(std::vector<std::vector<double> > vectorSet);
    static std::vector<double> computeVarianceVector(std::vector<std::vector<double> > vectorSet, std::vector<double> meanVector);
    static double randDouble(double low, double high);
    ~CSAEToolBox();
private:

};

#endif  /* CSAETOOLBOX_H */

