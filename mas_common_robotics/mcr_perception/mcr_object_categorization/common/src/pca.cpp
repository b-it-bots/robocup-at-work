/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#include <limits>
#include <assert.h>

#include "pca.h"

void CPca::init(std::vector<std::vector<double> > &inputFeatureVectors, unsigned int &numDim)
{
    assert(inputFeatureVectors.size() > 0);
    assert(inputFeatureVectors[0].size() > 0);
    this->inputFeatureVectors = inputFeatureVectors;
    this->numDim = numDim;

    featureVector = cvCreateMat(inputFeatureVectors.size(), inputFeatureVectors[0].size(), CV_32FC1);
    avgVector = cvCreateMat(1, inputFeatureVectors[0].size(), CV_32FC1);
    eigenVectors = cvCreateMat(inputFeatureVectors[0].size(), inputFeatureVectors[0].size(), CV_32FC1);
    eigenValues = cvCreateMat(1, inputFeatureVectors[0].size(), CV_32FC1);
}

void CPca::computePca(std::vector<std::vector<double> > &retEigenVectors, std::vector<double> &retEigenValues)
{
    for (int iExample = 0; iExample < inputFeatureVectors.size(); iExample++)
    {
        for (int iFeature = 0; iFeature < inputFeatureVectors[0].size(); iFeature++)
        {
            *((float*) CV_MAT_ELEM_PTR(*featureVector, iExample, iFeature)) = inputFeatureVectors[iExample][iFeature];
        }
    }

    cvCalcPCA(featureVector, avgVector, eigenValues, eigenVectors, CV_PCA_DATA_AS_ROW);

    assert(eigenValues->cols == inputFeatureVectors[0].size() && eigenValues->rows == 1);
    assert(eigenVectors->cols == inputFeatureVectors[0].size() && eigenVectors->rows == inputFeatureVectors[0].size());

    retEigenVectors.clear();
    retEigenVectors.resize(eigenVectors->rows);
    for (int iExample = 0; iExample < eigenVectors->rows; iExample++)
    {
        for (int iFeature = 0; iFeature < eigenVectors->cols; iFeature++)
        {
            retEigenVectors[iExample].push_back(CV_MAT_ELEM(*eigenVectors, float, iExample, iFeature));
        }
    }
    this->eigenVectorsVec = retEigenVectors;

    retEigenValues.clear();
    for (unsigned int i = 0; i < inputFeatureVectors[0].size(); ++i)
    {
        retEigenValues.push_back(CV_MAT_ELEM(*eigenValues, float, 0, i));
    }

    this->eigenValuesVec = retEigenValues;

    cvReleaseMat(&featureVector);
    cvReleaseMat(&avgVector);
    cvReleaseMat(&eigenVectors);
    cvReleaseMat(&eigenValues);
}

CPca::~CPca()
{
    /*cvReleaseMat(&featureVector);
     cvReleaseMat(&avgVector);
     cvReleaseMat(&eigenVectors);
     cvReleaseMat(&eigenValues);*/
}

