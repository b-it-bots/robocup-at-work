/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */


#include <limits>
#include <assert.h>

#include "neural_gas/pca_cv.h"

CPcaCv::CPcaCv(std::vector<std::vector<double> > &featureVectors, unsigned int &numDim)
{
    assert(numDim > 0);
    assert(featureVectors.size() > 0);
    assert(featureVectors[0].size() > 0);

    this->numDim = numDim;
    this->featureVectors = featureVectors;
    //this->avgVector = cv::Mat();


    this->inputVectors = cv::Mat(featureVectors.size(), featureVectors[0].size(), CV_64FC1);

}

void CPcaCv::init()
{

    for (unsigned int iVector = 0; iVector < this->featureVectors.size(); iVector++)
    {
        //  std::cout << iVector << " " << featureVectors.size() << std::endl;
        for (unsigned int iFeature = 0; iFeature < this->featureVectors[iVector].size(); iFeature++)
        {
            this->inputVectors.at<double> (iVector, iFeature) = this->featureVectors[iVector][iFeature];
        }
    }

    std::cout << std::endl << "Values" << this->inputVectors.cols << " " << this->inputVectors.rows << std::endl;
    //  this->printOutMat(inputVectors, 0);

    this->pcaComputer(this->inputVectors, this->avgVector, CV_PCA_DATA_AS_ROW, this->numDim);//CV_PCA_DATA_AS_COL ); //CV_PCA_DATA_AS_ROW);

    //      cv::Mat vect = this->getEigenVectors();
    //  std::cout << std::endl << "Values" << vect.rows << std::endl;
}

cv::Mat CPcaCv::getEigenValues()
{
    return this->pcaComputer.eigenvalues;
}
cv::Mat CPcaCv::getEigenVectors()
{
    return this->pcaComputer.eigenvectors;
}

void CPcaCv::reduceDim(std::vector<double> &input, std::vector<double> &output)
{


    cv::Mat matInput(input);
    matInput = matInput.t();


    cv::Mat matOutput = cv::Mat();
    this->pcaComputer.project(matInput, matOutput);


    output.clear();
    for (unsigned int i = 0; i < matOutput.cols; i++)
    {
        output.push_back((double) matOutput.at<double> (0, i));
    }

}

void CPcaCv::printOutMat(cv::Mat mat, bool isCol)
{

    if (!isCol)
        for (unsigned int i = 0; i < mat.cols; i++)
            std::cout << (double) mat.at<double> (0, i) << " ";
    if (isCol)
        for (unsigned int i = 0; i < mat.rows; i++)
            std::cout << (double) mat.at<double> (i, 0) << " ";
}

