/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <assert.h>
#include <cmath>
#include <algorithm>
#include <functional>
#include <vector>
#include <ctime>

#include "sparse_auto_encoder/auto_encoder.h"
#include "sparse_auto_encoder/sae_tool_box.h"
#include "file_settings.h"
#include "logger.h"

//#include <c++/4.5/bits/stl_vector.h>

CAutoEncoder::CAutoEncoder()
{
    this->logger = &CLogger::getInstance();
    srand(unsigned(time(NULL)));
    this->numInputs = -1;
    this->numOutputs = -1;
    this->numHiddenNeurons = 0.5f; //0.5
}

CAutoEncoder::CAutoEncoder(int numInputs, int numHiddenLayer, int numOutputs) //, double biasHiddenLayer,double biasOutputLayer) {
{
    srand(unsigned(time(NULL)));
    this->logger = &CLogger::getInstance();
    this->logger->log->debug("CAutoEncoder::CAutoEncoder...");
    this->numHiddenNeurons = 0.5f; //0.5
    this->init(numInputs, numHiddenLayer, numOutputs);//, biasHiddenLayer,biasOutputLayer);
}

void CAutoEncoder::setNumHiddenNeuron(float numHiddenNeurons)
{
    this->numHiddenNeurons = numHiddenNeurons;
}

void CAutoEncoder::init(int numInputs, int numHiddenLayer, int numOutputs) //, double biasHiddenLayer,double biasOutputLayer)
{
    assert(numInputs == numOutputs);
    assert(numHiddenLayer == 1);

    this->numInputs = numInputs;
    this->numOutputs = numOutputs;
    //    int numLayerNeurons = (int) this->numInputs * 0.5f;
    int numLayerNeurons = (int) this->numInputs * this->numHiddenNeurons;

    if (numLayerNeurons < 1)
    {
        numLayerNeurons = 1;
    }

//  std::cout << "CAutoEncoder::init--->" << numLayerNeurons << "\n";
    assert(numLayerNeurons > 0);

    this->nn = CNeuralNetwork(numInputs, numInputs);
    //One hidden layer
    this->nn.addLayer(this->numInputs, numLayerNeurons, 0); // 1 //, biasHiddenLayer);
    //
    //nn.addLayer(numLayerNeurons,5,1);
    //nn.addLayer(5,numLayerNeurons,1);
    //
    this->nn.addLayer(numLayerNeurons, this->numOutputs, 0);// biasOutputLayer; //OUTPUT!
    //nn.addLayer(numOutputs,this->numOutputs,1);
}

void CAutoEncoder::init(int numInputs, int numHiddenLayer, int numOutputs, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector)
{
    assert(numInputs == numOutputs);
    assert(numHiddenLayer == 1);

    this->numInputs = numInputs;
    this->numOutputs = numOutputs;
    int numLayerNeurons = (int) this->numInputs * this->numHiddenNeurons;

    if (numLayerNeurons < 1)
    {
        numLayerNeurons = 1;
    }
    assert(numLayerNeurons > 0);

    //std::cout << "CAutoEncoder::init--->" << numLayerNeurons << "\n";
    this->nn = CNeuralNetwork(numInputs, numInputs);
    //One hidden layer
    this->nn.addLayer(this->numInputs, numLayerNeurons, 0.0, initValueMeanVector, initValueVarianceVector);
    //
    //nn.addLayer(numLayerNeurons,5,1);
    //nn.addLayer(5,numLayerNeurons,1);
    //
    this->nn.addLayer(numLayerNeurons, this->numOutputs, 0); //OUTPUT!
    //nn.addLayer(numOutputs,this->numOutputs,1);
}

void CAutoEncoder::input(std::vector<double> inputVector)
{

    if (this->numInputs > 0)
    {
        assert(inputVector.size() == this->numInputs);
        this->inputVector = inputVector;
    }
    else if (this->numInputs == -1)
    {
        this->init(inputVector.size(), 1, inputVector.size());
        this->inputVector = inputVector;
    }
    else
    {
        this->logger->log->error("CAutoEncoder::input...this->numInputs==no correct defined");
    }
    this->inputVectorSet.clear();
}

void CAutoEncoder::input(std::vector<std::vector<double> > inputVectorSet)
{

    assert(inputVectorSet.size() > 0);
    assert(inputVectorSet[0].size() > 0);

    if (this->numInputs > 0)
    {
        assert(inputVectorSet[0].size() == this->numInputs);
        this->inputVectorSet = inputVectorSet;
    }
    else if (this->numInputs == -1)
    {
        std::vector<double> meanVector = CSAEToolBox::computeMeanVector(inputVectorSet);
        std::vector<double> varianceVector = CSAEToolBox::computeVarianceVector(inputVectorSet, meanVector);
        this->init(inputVectorSet[0].size(), 1, inputVectorSet[0].size(), meanVector, varianceVector);
        this->inputVectorSet = inputVectorSet;

    }
    else
    {
        this->logger->log->error("CAutoEncoder::input...this->numInputs==no correct defined");
    }

    this->logger->log->debug("CAutoEncoder::input...TrainSetSize=%d", this->inputVectorSet.size());
    this->inputVector.clear();
}

void CAutoEncoder::input(std::string inputFile)
{
    this->input(CFileSettings::loadStdVector(inputFile));
}

std::vector<std::vector<double> > CAutoEncoder::encode(std::string inputFile)
{
    std::vector<std::vector<double> > toEncode = CFileSettings::loadStdVector(inputFile);

    return this->encode(toEncode);
}
std::vector<std::vector<double> > CAutoEncoder::encode(std::vector<std::vector<double> > inputVectorSetToEncode)
{
    std::vector<std::vector<double> > inputVectorSetEncoded;
    for (unsigned int i = 0; i < inputVectorSetToEncode.size(); ++i)
    {
        inputVectorSetEncoded.push_back(this->encode(inputVectorSetToEncode[i]));
    }

    return inputVectorSetEncoded;
}

std::vector<double> CAutoEncoder::encode(std::vector<double> inputVectorToEncode)
{
    //we return the hidden layer output not the outputlayer!!!
    return nn.feedForward(inputVectorToEncode)[0];
}

std::vector<double> CAutoEncoder::decode(std::vector<double> inputVectorToEncode)
{
    //we return the output layer which is
    // std::cout<<nn.feedForward(inputVectorToEncode).size()<<"ii ";
    return nn.feedForwardOnce(inputVectorToEncode, 1);
}

std::vector<double> CAutoEncoder::encodeDecode(std::vector<double> inputVectorToEncode)
{
    //we return the hidden layer output not the outputlayer!!!
    this->logger->log->debug("CAutoEncoder::encodeDecode...");
    return nn.feedForward(inputVectorToEncode)[1];
}

std::vector<double> CAutoEncoder::feedVector()
{
    assert(this->inputVector.size() == this->numInputs);
    return this->feedVector(this->inputVector);
}

std::vector<std::vector<double> > CAutoEncoder::feedVectorSet()
{
    assert(this->inputVectorSet.size() > 0);
    return this->feedVectorSet(this->inputVectorSet);

}

std::vector<double> CAutoEncoder::feedVector(std::vector<double> inputVector)
{
    assert(this->inputVector.size() == this->numInputs);
    std::vector<double> response;
    this->input(inputVector);

    response = this->nn.feed(this->inputVector, this->inputVector);
    assert(this->inputVector.size() == response.size());
    this->nn.backPropagation();

    inputVector.clear();
    return response;
}

std::vector<std::vector<double> > CAutoEncoder::randomizeVectorSet(std::vector<std::vector<double> > inputVectorSet)
{
    assert(inputVectorSet.size() > 0);

    std::vector<int> randIdx;
    std::vector<std::vector<double> > randInputVector;

    for (unsigned int i = 0; i < inputVectorSet.size(); ++i)
    {
        randIdx.push_back(i);
    }

    random_shuffle(randIdx.begin(), randIdx.end());

    randInputVector.resize(inputVectorSet.size());
    assert(inputVectorSet.size() == randIdx.size());
    for (unsigned int i = 0; i < inputVectorSet.size(); ++i)
    {
        randInputVector[i] = inputVectorSet[randIdx[i]];
    }

    return randInputVector;
}

std::vector<std::vector<double> > CAutoEncoder::feedVectorSet(std::vector<std::vector<double> > inputVectorSet)
{
    assert(inputVectorSet.size() > 0);
    std::vector<std::vector<double> > responseSet;
    std::vector<double> response;
    this->input(inputVectorSet);

    int wait;
    std::vector<std::vector<double> > randInputVectorSet = this->randomizeVectorSet(this->inputVectorSet);
    for (unsigned int i = 0; i < randInputVectorSet.size(); ++i)
    {
        this->logger->log->debug("CAutoEncoder::feedVectorSet...feed Pattern=%d", i);

        //   CFileSettings::coutStdVector(randInputVectorSet[i], true);
        response = this->nn.feed(this->inputVectorSet[i], this->inputVectorSet[i]);
        assert(this->inputVectorSet[i].size() == response.size());
        this->nn.backPropagation();
        responseSet.push_back(response);
        //std::cin>>wait;
    }
    inputVectorSet.clear();
    return responseSet;
}

double CAutoEncoder::train(double allowedMSE, int maxIteration)
{
    if (this->inputVector.size() > 0)
    {
        return this->trainVector(allowedMSE, maxIteration);
    }
    else
    {
        return this->trainVectorSet(allowedMSE, maxIteration);
    }
}

double CAutoEncoder::trainVector(double allowedMSE, int maxIteration)
{

    std::vector<double> response;
    double curMSE;
    unsigned int i = 0;
    int t;
    do
    {
        ++i;
        logger->log->debug("CAutoEncoder::train...begin Epoch=%d\n", i);
        response = this->feedVector();

        for (unsigned int iterRes = 0; iterRes < response.size(); ++iterRes)
        {
            logger->log->debug("CAutoEncoder::train...Epoch=%d...Response[%d]=%lf", i, iterRes, response[iterRes]);
        }
        curMSE = this->getMeanSquaredError(this->inputVector, response);
        std::cout << "\r Epoch=" << i << " MSE=" << curMSE;
        logger->log->debug("CAutoEncoder::train...Epoch=%d...Current MSE = %lf\n", i, curMSE);

    }
    while (curMSE > allowedMSE && i < maxIteration);

    for (unsigned int i = 0; i < this->nn.getNumLayers(); ++i)
    {
        logger->log->debug("CAutoEncoder::train...learned Weights for layer %d", i);
        std::vector<std::vector<double> > weightVectors = this->nn.getWeightVector(i);

        for (unsigned int iterNeuron = 0; iterNeuron < weightVectors.size(); ++iterNeuron)
        {
            for (unsigned int iterWeight = 0; iterWeight < weightVectors[iterNeuron].size(); ++iterWeight)
            {
                logger->log->debug("CAutoEncoder::train....Layer=%d Neuron=%d weight[%d]=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);

            }
            //logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d bias=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);
        }
    }

    for (unsigned int i = 0; i < this->nn.getNumLayers(); ++i)
    {
        logger->log->debug("CAutoEncoder::train...Responses for layer %d", i);
        std::vector<double> responseVector = this->nn.getResponseVector(i);
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {
            logger->log->debug("CAutoEncoder::train....Layer=%d Neuron=%d Response=%lf", i, iterNeuron, responseVector[iterNeuron]);
            //logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d bias=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);
        }
    }

    return curMSE;
}

double CAutoEncoder::trainVectorSet(double allowedMSE, int maxIteration)
{

    std::vector<std::vector<double> > response;

    double curMSE;
    unsigned int i = 0;
    int wait;
    do
    {
        ++i;
        logger->log->debug("CAutoEncoder::train...begin Epoch=%d\n", i);
        response = this->feedVectorSet();

        for (unsigned int iterSet = 0; iterSet < response.size(); ++iterSet)
        {
            for (unsigned int iterRes = 0; iterRes < response[iterSet].size(); ++iterRes)
            {
                logger->log->debug("CAutoEncoder::train...Epoch=%d...Train=%d Response[%d]=%lf", i, iterSet, iterRes, response[iterSet][iterRes]);
            }
        }
        curMSE = this->getMeanSquaredError(this->inputVectorSet, response);
        logger->log->info("CAutoEncoder::train...Epoch=%d...Current MSE = %lf\n", i, curMSE);
        std::cout << "\r Epoch=" << i << " MSE=" << curMSE;
        // std::cin>>wait;

    }
    while (curMSE > allowedMSE && i < maxIteration);
    //exit(1);

    logger->log->info("CAutoEncoder::train...Epoch=%d...Last MSE = %lf\n", i, curMSE);
    for (unsigned int i = 0; i < this->nn.getNumLayers(); ++i)
    {
        logger->log->info("CAutoEncoder::train...learned Weights for layer %d", i);
        std::vector<std::vector<double> > weightVectors = this->nn.getWeightVector(i);

        for (unsigned int iterNeuron = 0; iterNeuron < weightVectors.size(); ++iterNeuron)
        {
            for (unsigned int iterWeight = 0; iterWeight < weightVectors[iterNeuron].size(); ++iterWeight)
            {
                logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d weight[%d]=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);

            }
            //logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d bias=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);
        }
    }

    for (unsigned int i = 0; i < this->nn.getNumLayers(); ++i)
    {
        logger->log->info("CAutoEncoder::train...Responses for layer %d", i);
        std::vector<double> responseVector = this->nn.getResponseVector(i);
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {
            logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d Response=%lf", i, iterNeuron, responseVector[iterNeuron]);
            //logger->log->info("CAutoEncoder::train....Layer=%d Neuron=%d bias=%lf", i, iterNeuron, iterWeight, weightVectors[iterNeuron][iterWeight]);
        }
    }

    return curMSE;
}

double CAutoEncoder::getMeanSquaredError(std::vector<std::vector<double> > inputVector, std::vector<std::vector<double> > response)
{
    assert(inputVector.size() == response.size());

    double sumError = 0.0;
    for (unsigned int i = 0; i < inputVector.size(); ++i)
    {
        sumError += this->getMeanSquaredError(inputVector[i], response[i]);
    }
    return sqrt(sumError / inputVector.size());
}

double CAutoEncoder::getMeanSquaredError(std::vector<double> inputVector, std::vector<double> response)
{
    assert(inputVector.size() == response.size());

    double sumError = 0.0;
    for (unsigned int i = 0; i < inputVector.size(); ++i)
    {
        sumError += pow(inputVector[i] - response[i], 2);
    }
    return sqrt(sumError / (double) inputVector.size());
}

CAutoEncoder::~CAutoEncoder()
{
    ;
}
