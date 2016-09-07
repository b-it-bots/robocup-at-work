/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include <assert.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "sparse_auto_encoder/layer.h"


CLayer::CLayer()
{
    this->logger = &CLogger::getInstance();
};

CLayer::CLayer(int numInputs, int numNeurons, double bias, double sparseParameter)
{
    this->logger = &CLogger::getInstance();
    this->initLayer(numInputs, numNeurons, bias, sparseParameter);
}

CLayer::CLayer(int numInputs, int numNeurons, double bias, double sparseParameter, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector)
{
    this->logger = &CLogger::getInstance();
    this->initLayer(numInputs, numNeurons, bias, sparseParameter, initValueMeanVector, initValueVarianceVector);
}

void CLayer::initLayer(int numInputs, int numNeurons, double bias, double sparseParameter)
{
    assert(numNeurons > 0);

    this->logger->log->debug("CLayer::initLayer...NUMINPUT=%d NUMNEURON=%d BIAS=%lf", numInputs, numNeurons, bias);
    this->neurons.clear();
    this->numNeurons = numNeurons;
    this->bias = bias;
    this->numInputs = numInputs;

    this->neurons.resize(this->numNeurons);

    for (unsigned int iterNeurons = 0; iterNeurons < this->numNeurons; ++iterNeurons)
    {
        this->neurons[iterNeurons] = CNeuron(this->numInputs, bias, sparseParameter);
        this->neurons[iterNeurons].init();
    }
}

void CLayer::initLayer(int numInputs, int numNeurons, double bias, double sparseParameter, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector)
{
    assert(numNeurons > 0);

    this->logger->log->debug("CLayer::initLayer...NUMINPUT=%d NUMNEURON=%d BIAS=%lf", numInputs, numNeurons, bias);
    this->neurons.clear();
    this->numNeurons = numNeurons;
    this->bias = bias;
    this->numInputs = numInputs;

    this->neurons.resize(this->numNeurons);

    for (unsigned int iterNeurons = 0; iterNeurons < this->numNeurons; ++iterNeurons)
    {
        this->neurons[iterNeurons] = CNeuron(this->numInputs, bias, sparseParameter);
        this->neurons[iterNeurons].init(initValueMeanVector, initValueVarianceVector);
    }
}

void CLayer::input(std::vector<double> inputVector)
{
    assert(inputVector.size() == this->numInputs);
    this->inputVector = inputVector;
    this->errorVector.clear();
    this->responseVector.clear();

}

std::vector<double> CLayer::feedForward(std::vector<double> inputVector)
{
    std::vector<double> response;
    this->input(inputVector);
    assert(this->inputVector.size() == this->numInputs);
    logger->log->debug("CLayer::feedForward...this->inputVector.size()=%d  this->numInputs=%d", this->inputVector.size(), this->numInputs);
    response = this->computeFeedForward();
    logger->log->debug("CLayer::feedForward...response=%d neurons=%d", response.size(), this->getNumNeurons());
    inputVector.clear();

    return response;
}

std::vector<double> CLayer::feedForward()
{
    assert(this->inputVector.size() == this->numInputs);
    return feedForward(this->inputVector);
}

std::vector<double> CLayer::computeFeedForward()
{
    std::vector<double> fed;
    assert(this->inputVector.size() == this->numInputs);

    fed.resize(this->numNeurons);
    for (unsigned int iterNeuron = 0; iterNeuron < this->numNeurons; ++iterNeuron)
    {
        fed[iterNeuron] = this->neurons[iterNeuron].fire(inputVector);
    }

    return fed;
}

CLayer::~CLayer()
{
    ;

}
