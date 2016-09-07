/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CPCACV_H
#define CPCACV_H

#include <malloc.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <opencv/cv.h>

class CPcaCv
{
private:

    int numClasses;

    std::vector< std::vector<double> > featureVectors;
    cv::Mat inputVectors;
    cv::Mat avgVector;
    cv::Mat eigenVectors;
    cv::Mat eigenValues;
    cv::Mat numEigenVectors;
    cv::Mat numRequiredEigenValues;
    cv::PCA pcaComputer;
    unsigned int numDim;

public:
    CPcaCv()
    {
        ;
    };
    CPcaCv(std::vector< std::vector<double> > &featureVectors, unsigned int &numDim);
    void init();
    void reduceDim(std::vector<double> &input, std::vector<double> &output);
    void printOutMat(cv::Mat mat, bool isCol);

    cv::Mat getEigenValues();
    cv::Mat getEigenVectors();
};
#endif
