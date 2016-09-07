/*
 * File:   CSAEToolBox.cpp
 * Author: Christian Mueller
 *
 * Created on May 31, 2011, 11:27 PM
 */

#include "sparse_auto_encoder/sae_tool_box.h"
#include <assert.h>
#include <cmath>

CSAEToolBox::CSAEToolBox()
{
}

std::vector<double> CSAEToolBox::computeMeanVector(std::vector<std::vector<double> > vectorSet)
{
    std::vector<double> meanVector;
    assert(vectorSet.size() > 0);
    assert(vectorSet[0].size() > 0);

    meanVector.resize(vectorSet[0].size());
    for (unsigned int i = 0; i < vectorSet.size(); ++i)
    {
        for (unsigned int j = 0; j < vectorSet[i].size(); ++j)
        {
            meanVector[j] += vectorSet[i][j];
        }
    }

    for (unsigned int i = 0; i < meanVector.size(); ++i)
    {

        meanVector[i] =  meanVector[i] / (double)vectorSet.size();

    }
    return meanVector;
}


std::vector<double> CSAEToolBox::computeVarianceVector(std::vector<std::vector<double> > vectorSet, std::vector<double> meanVector)
{

    std::vector<double> varianceVector;

    assert(vectorSet.size() > 0);
    assert(vectorSet[0].size() > 0);
    assert(vectorSet[0].size() == meanVector.size());

    varianceVector.resize(vectorSet[0].size());
    for (unsigned int i = 0; i < vectorSet.size(); ++i)
    {
        for (unsigned int j = 0; j < vectorSet[i].size(); ++j)
        {
            varianceVector[j] += pow(vectorSet[i][j] - meanVector[j], 2);
        }
    }


    for (unsigned int i = 0; i < varianceVector.size(); ++i)
    {
        varianceVector[i] =  varianceVector[i] / (double)vectorSet.size();
    }

    return varianceVector;
}
double CSAEToolBox::randDouble(double low, double high)
{
    double temp;

    /* swap low & high around if the user makes no sense */
    if (low > high)
    {
        temp = low;
        low = high;
        high = temp;
    }

    /* calculate the random number & return it */
    temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0)) * (high - low)
           + low;
    return temp;
}


CSAEToolBox::~CSAEToolBox()
{
}

