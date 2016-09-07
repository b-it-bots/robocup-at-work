/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef CPCA_H
#define CPCA_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <opencv/cv.h>

class CPca
{
private:
    std::vector<std::vector<double> > inputFeatureVectors;

    CvMat* featureVector;
    CvMat* avgVector;
    CvMat* eigenVectors;
    CvMat* eigenValues;

    std::vector<std::vector<double> >eigenVectorsVec;
    std::vector<double> eigenValuesVec;


    unsigned int numDim;

public:
    CPca() {};
    void init(std::vector<std::vector<double> > &featureVectors, unsigned int &numDim);
    void computePca(std::vector<std::vector<double> > &eigenVectors, std::vector<double> &eigenValues);
    ~CPca();
};
#endif
