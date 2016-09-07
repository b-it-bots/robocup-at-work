/*
 * File:   CNeuralNetwork.mm
 * Author: Christian Mueller
 *
 * Created on May 28, 2011, 6:15 PM
 */

#include <iostream>
#include <map>
#include <assert.h>
#include <vector>
#include <cmath>

#include "sparse_auto_encoder/neural_network.h"


CNeuralNetwork::CNeuralNetwork()
{
    this->logger = &CLogger::getInstance();
}

CNeuralNetwork::CNeuralNetwork(int numInputs, int numLabels)
{
    this->logger = &CLogger::getInstance();
    this->numInputs = numInputs;
    this->numLabels = numLabels;
    this->sparseParameter = 0.001f;
    this->learningRate = 0.25f;
    this->learningRateSparsity = 0.6f;
    this->weightDecay = 0.7f;

}

void CNeuralNetwork::addLayer(int numLayerInputs, int numNeurons, double bias)
{
    //this->logger->log->debug("CNeuralNetwork::addLayer...");
    this->layers.push_back(CLayer(numLayerInputs, numNeurons, bias, this->sparseParameter));
}

void CNeuralNetwork::addLayer(int numLayerInputs, int numNeurons, double bias, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianzVector)
{
    //this->logger->log->debug("CNeuralNetwork::addLayer...");
    this->layers.push_back(CLayer(numLayerInputs, numNeurons, bias, this->sparseParameter, initValueMeanVector, initValueVarianzVector));
}

void CNeuralNetwork::initNeuralNetwork()
{
    /*for(unsigned int iterLayer = 0; iterLayer  < this->layers.size(); ++iterLayer)
    {
        this->layers[iterLayer].initLayer();
    }*/

    std::cout << "CNeuralNetwork::initNeuralNetwork ... Not implemented!!!" << std::endl;
}

void CNeuralNetwork::input(std::vector<double> inputVector, std::vector<double> labelVector)
{
    this->inputVector = inputVector;
    this->labelVector = labelVector;
}

std::vector<double> CNeuralNetwork::feed()
{
    assert(this->inputVector.size() == this->numInputs);
    assert(this->labelVector.size() == this->numLabels);
    return this->feed(this->inputVector, this->labelVector);
}

//lastResponse
std::vector<double> CNeuralNetwork::feed(std::vector<double> inputVector, std::vector<double> labelVector)
{
    std::vector<double> response;
    this->input(inputVector, labelVector);
    this->logger->log->debug("CNeuralNetwork::feed...inputVec=%d  numInputs=%d", this->inputVector.size(), this->numInputs);
    assert(this->inputVector.size() == this->numInputs);

    response = inputVector;
    for (unsigned int iterLayer = 0; iterLayer < this->layers.size(); ++iterLayer)
    {
        response = this->layers[iterLayer].feedForward(response);
        this->layers[iterLayer].setResponseVector(response);
        this->logger->log->debug("CNeuralNetwork::feed...this->layers[iterLayer].getNumNeurons()=%d  response.size()=%d", this->layers[iterLayer].getNumNeurons(), response.size());
        this->logger->log->debug("CNeuralNetwork::feed...Result of Layer %d...", iterLayer);
        for (unsigned int i = 0; i < response.size(); ++i)
        {
            this->logger->log->debug("CNeuralNetwork::feed...Layer=%d, Response=%d = %lf", iterLayer, i, response[i]);
            //   if (response[i] == -1)
            //      exit(1);
        }
        assert(this->layers[iterLayer].getNumNeurons() == response.size());
    }
    return response;
}

std::vector< std::vector<double> > CNeuralNetwork::feedForward(std::vector<double> inputVector)
{
    std::vector<double> response;
    std::vector< std::vector<double> > responseVector;

    this->logger->log->debug("CNeuralNetwork::feedForward...inputVec=%d  numInputs=%d", this->inputVector.size(), this->numInputs);
    assert(this->inputVector.size() == this->numInputs);

    response = inputVector;
    for (unsigned int iterLayer = 0; iterLayer < this->layers.size(); ++iterLayer)
    {
        //response = this->layers[iterLayer].feedForward(response);
        response = this->feedForwardOnce(response , iterLayer);
        responseVector.push_back(response);
        this->layers[iterLayer].setResponseVector(response);
        this->logger->log->debug("CNeuralNetwork::feedForward...this->layers[iterLayer].getNumNeurons()=%d  response.size()=%d", this->layers[iterLayer].getNumNeurons(), response.size());
        this->logger->log->debug("CNeuralNetwork::feedForward...Result of Layer %d...", iterLayer);

        for (unsigned int i = 0; i < response.size(); ++i)
        {
            this->logger->log->debug("CNeuralNetwork::feedForward...Layer=%d, Response=%d = %lf", iterLayer, i, response[i]);
            //   if (response[i] == -1)
            //      exit(1);
        }
    }
    return responseVector;
}

std::vector<double> CNeuralNetwork::feedForwardOnce(std::vector<double> inputVector, int layer)
{
    std::vector<double> response;
    std::vector< std::vector<double> > responseVector;
    assert(layer < this->layers.size());
    //this->logger->log->debug("CNeuralNetwork::feedForwardOnce...inputVec=%d ", this->inputVector.size());

    response = this->layers[layer].feedForward(inputVector);

    assert(this->layers[layer].getNumNeurons() == response.size());

    return response;
}

void CNeuralNetwork::backPropagation()
{
    this->logger->log->debug("CNeuralNetwork::backPropagation...");

    // std::vector<std::vector<double> > layerError;
    // layerError.resize(this->layers.size());
    for (int iterLayer = this->layers.size() - 1; iterLayer >= 0; --iterLayer)
    {
        this->logger->log->debug("CNeuralNetwork::backPropagation..try(%d)", iterLayer);
        this->layers[iterLayer].setErrorVector(this->computeError2(iterLayer));
        this->updateWeights2(iterLayer);
        //assert(this->layers[iterLayer].getNumNeurons() == response.size());
        this->logger->log->debug("CNeuralNetwork::backPropagation...done");
    }
    this->logger->log->debug("CNeuralNetwork::backPropagation...out");
}

void CNeuralNetwork::updateWeights(int layer)
{

    this->logger->log->debug("CNeuralNetwork::updateWeights...");

    //if(layer < this->layers.size()-1)
    //{
    std::vector<double> &postErrorVector = this->layers[layer + 1].getErrorVector();
    std::vector<CNeuron>&neurons = this->layers[layer].getNeurons();
    std::vector<double> &errorVector = this->layers[layer].getErrorVector();
    std::vector<double> &inputVector = this->layers[layer].getInputVector();

    for (unsigned int iterNeuron = 0; iterNeuron < neurons.size(); ++iterNeuron)
    {
        std::vector<double> &weightVector = neurons[iterNeuron].getWeightVector();

        assert(weightVector.size() == inputVector.size());
        for (unsigned int iterNeuronWeight = 0; iterNeuronWeight < weightVector.size(); ++iterNeuronWeight)
        {
            this->logger->log->debug("CNeuralNetwork::updateWeights...Layer=%d: Neuron=%d weightOLD=%lf learningRate=%lf, NeuronError=%lf, input=%lf", layer, iterNeuron, weightVector[iterNeuronWeight], (learningRate), errorVector[iterNeuron], inputVector[iterNeuronWeight]);
            //1.Method
            weightVector[iterNeuronWeight] += (this->learningRate) * errorVector[iterNeuron] * inputVector[iterNeuronWeight];

            //2.Method
            //weightVector[iterNeuronWeight] = weightVector[iterNeuronWeight] - (this->learningRate) *(postErrorVector[iterNeuron] * inputVector[iterNeuronWeight]);
            this->logger->log->debug("CNeuralNetwork::updateWeights...............: Neuron=%d weightNEW=%lf", iterNeuron, weightVector[iterNeuronWeight]);
            //we iterate over weightVector(all weights of neuron)
            //we iterator over inputVector(all inputs of neuron)
            //neuron error is fixed, sice it is the same

        }
        //double &neuronBias = neurons[iterNeuron].getBias();
        // this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
        // neuronBias += this->learningRate * errorVector[iterNeuron];
        // neurons[iterNeuron].setBias(neuronBias);
        //this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());

        /*   double &neuronBias = neurons[iterNeuron].getBias();
            this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
            neuronBias = neuronBias - this->learningRate * postErrorVector[iterNeuron];
            neurons[iterNeuron].setBias(neuronBias);
            this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());
        */
        //NOW update bias! Sparcity
        /*if(layer <this->layers.size()-1) //only hidden layers
        {
            double &neuronBias = neurons[iterNeuron].getBias();
             this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
             neuronBias += this->learningRate * this->learningRateSparsity * (neurons[iterNeuron].getSparsityEstimate() - 0.9);
             neurons[iterNeuron].setBias(neuronBias);
             this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());
        }*/
    }

    //}
}


void CNeuralNetwork::updateWeights2(int layer)
{

    this->logger->log->debug("CNeuralNetwork::updateWeights...");

    //if(layer < this->layers.size()-1)
    //{
    std::vector<double> &postErrorVector = this->layers[layer + 1].getErrorVector();
    std::vector<CNeuron>&neurons = this->layers[layer].getNeurons();
    std::vector<double> &errorVector = this->layers[layer].getErrorVector();
    std::vector<double> &inputVector = this->layers[layer].getInputVector();

    for (unsigned int iterNeuron = 0; iterNeuron < neurons.size(); ++iterNeuron)
    {
        std::vector<double> &weightVector = neurons[iterNeuron].getWeightVector();

        assert(weightVector.size() == inputVector.size());
        for (unsigned int iterNeuronWeight = 0; iterNeuronWeight < weightVector.size(); ++iterNeuronWeight)
        {
            this->logger->log->debug("CNeuralNetwork::updateWeights...Layer=%d: Neuron=%d weightOLD=%lf learningRate=%lf, NeuronError=%lf, input=%lf", layer, iterNeuron, weightVector[iterNeuronWeight], (learningRate), errorVector[iterNeuron], inputVector[iterNeuronWeight]);
            //1.Method
            weightVector[iterNeuronWeight] = weightVector[iterNeuronWeight] - (this->learningRate) * (errorVector[iterNeuron] * inputVector[iterNeuronWeight]);// + this->weightDecay*weightVector[iterNeuronWeight]); //here is missing lambda W
            //weightVector[iterNeuronWeight] = weightVector[iterNeuronWeight]/(double)weightVector.size();
            //2.Method
            //weightVector[iterNeuronWeight] = weightVector[iterNeuronWeight] - (this->learningRate) *(postErrorVector[iterNeuron] * inputVector[iterNeuronWeight]);
            this->logger->log->debug("CNeuralNetwork::updateWeights...............: Neuron=%d weightNEW=%lf", iterNeuron, weightVector[iterNeuronWeight]);
            //we iterate over weightVector(all weights of neuron)
            //we iterator over inputVector(all inputs of neuron)
            //neuron error is fixed, sice it is the same

        }
        //    double &neuronBias = neurons[iterNeuron].getBias();
        //   this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
        //  neuronBias = neuronBias - this->learningRate * errorVector[iterNeuron];
        //  neuronBias = neuronBias - this->learningRate * neuronBias;
        //  neurons[iterNeuron].setBias(neuronBias);
        // this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());

        /*   double &neuronBias = neurons[iterNeuron].getBias();
            this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
            neuronBias = neuronBias - this->learningRate * postErrorVector[iterNeuron];
            neurons[iterNeuron].setBias(neuronBias);
            this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());
        */
        //NOW update bias! Sparcity
        /*  if(layer <this->layers.size()-1) //only hidden layers
          {
              double &neuronBias = neurons[iterNeuron].getBias();
               this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasOLD=%lf", iterNeuron, neuronBias);
               neuronBias = neuronBias - this->learningRate * this->learningRateSparsity * (neurons[iterNeuron].getSparsityEstimate() - 0.9);
               neurons[iterNeuron].setBias(neuronBias);
               this->logger->log->debug("CNeuralNetwork::updateBias...Hidden Layer: Neuron=%d biasNEW=%lf", iterNeuron, neurons[iterNeuron].getBias());
          }*/
    }

    //}
}


std::vector<double> CNeuralNetwork::computeError(int layer)
{

    std::vector<double> error;
    assert(layer >= 0);
    std::vector<double> responseVector = this->layers[layer].getResponseVector();
    assert(responseVector.size() > 0);
    error.resize(responseVector.size());

    this->logger->log->debug("CNeuralNetwork::computeError...layerResponses[layer].size()=%d", responseVector.size());
    //This is the output layer case;
    if (layer == this->layers.size() - 1)
    {
        std::vector<double> desired = labelVector;
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {
            this->logger->log->debug("CNeuralNetwork::computeError...try");
            //1. Method
            error[iterNeuron] = responseVector[iterNeuron] * (1 - pow(responseVector[iterNeuron], 2)) * (desired[iterNeuron] - responseVector[iterNeuron]);

            //2. Method
            // error[iterNeuron] = -(desired[iterNeuron] - responseVector[iterNeuron]) * (1.0 - pow(responseVector[iterNeuron], 2.0));
            this->logger->log->debug("CNeuralNetwork::computeError...Layer=%d Neuron=%d Error=%lf", layer, iterNeuron, error[iterNeuron]);
        }
    }
    else   // We know that is is a hidden neuron
    {
        assert(this->layers.size() - 1 >= layer + 1); // check whether there is a postLayer.
        CLayer &postLayer = this->layers[layer + 1];
        std::vector<CNeuron> &postNeurons = this->layers[layer + 1].getNeurons();
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {

            double deltaSum = 0;
            for (unsigned int iterPostNeuron = 0; iterPostNeuron < postNeurons.size(); ++iterPostNeuron)
            {
                double delta = postLayer.getResponseVector()[iterNeuron];
                double weight = postNeurons[iterPostNeuron].getWeightVector()[iterNeuron];
                deltaSum += (delta * weight);
            }

            this->logger->log->debug("CNeuralNetwork::computeError...try");
            //1.Method
            error[iterNeuron] = responseVector[iterNeuron] * (1 - pow(responseVector[iterNeuron], 2)) * deltaSum;

            //2.Method
            //error[iterNeuron] = deltaSum * (1 - pow(responseVector[iterNeuron], 2));
            this->logger->log->debug("CNeuralNetwork::computeError...Layer=%d Neuron=%d Error=%lf", layer, iterNeuron, error[iterNeuron]);
        }
    }

    //error[iterNeuron]=(desired[iterNeuron] - layerResponses[iterLayer][iterNeuron]);


    this->logger->log->debug("CNeuralNetwork::computeError...out");
    return error;
}

std::vector<double> CNeuralNetwork::computeError2(int layer)
{

    std::vector<double> error;
    assert(layer >= 0);
    std::vector<double> responseVector = this->layers[layer].getResponseVector();
    assert(responseVector.size() > 0);
    error.resize(responseVector.size());

    this->logger->log->debug("CNeuralNetwork::computeError...layerResponses[layer].size()=%d", responseVector.size());
    //This is the output layer case;
    if (layer == this->layers.size() - 1)
    {
        std::vector<double> desired = labelVector;
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {
            this->logger->log->debug("CNeuralNetwork::computeError...try");
            //1. Method
            // error[iterNeuron] = responseVector[iterNeuron] * (1 - pow(responseVector[iterNeuron],2))*(desired[iterNeuron] - responseVector[iterNeuron]);

            //2. Method
            error[iterNeuron] = -(desired[iterNeuron] - responseVector[iterNeuron]) * (1.0 - pow(responseVector[iterNeuron], 2.0));
            this->logger->log->debug("CNeuralNetwork::computeError...Layer=%d Neuron=%d Error=%lf", layer, iterNeuron, error[iterNeuron]);
        }
    }
    else   // We know that is is a hidden neuron
    {
        assert(this->layers.size() - 1 >= layer + 1); // check whether there is a postLayer.
        CLayer &postLayer = this->layers[layer + 1];
        std::vector<CNeuron> &postNeurons = this->layers[layer + 1].getNeurons();
        for (unsigned int iterNeuron = 0; iterNeuron < responseVector.size(); ++iterNeuron)
        {

            double deltaSum = 0;
            for (unsigned int iterPostNeuron = 0; iterPostNeuron < postNeurons.size(); ++iterPostNeuron)
            {
                double delta = postLayer.getErrorVector()[iterNeuron];
                double weight = postNeurons[iterPostNeuron].getWeightVector()[iterNeuron];
                deltaSum += (delta * weight);
            }

            this->logger->log->debug("CNeuralNetwork::computeError...try");
            //1.Method
            //error[iterNeuron] = responseVector[iterNeuron] * (1 - pow(responseVector[iterNeuron],2)) * deltaSum;

            //2.Method
            error[iterNeuron] = deltaSum * (1 - pow(responseVector[iterNeuron], 2));
            this->logger->log->debug("CNeuralNetwork::computeError...Layer=%d Neuron=%d Error=%lf", layer, iterNeuron, error[iterNeuron]);
        }
    }

    //error[iterNeuron]=(desired[iterNeuron] - layerResponses[iterLayer][iterNeuron]);


    this->logger->log->debug("CNeuralNetwork::computeError...out");
    return error;
}

std::vector<std::vector<double> > CNeuralNetwork::getWeightVector(unsigned int layer)
{
    assert(layer < this->layers.size());


    std::vector<std::vector<double> > weightVectors;
    std::vector<double> weightVector;
    std::vector<CNeuron> neurons = this->layers[layer].getNeurons();
    for (unsigned int i = 0; i < neurons.size(); ++i)
    {
        weightVector = neurons[i].getWeightVector();
        weightVectors.push_back(weightVector);
    }

    return weightVectors;
}


std::vector<double> CNeuralNetwork::getResponseVector(unsigned int layer)
{
    assert(layer < this->layers.size());

    std::vector<double> responseVector;
    std::vector<CNeuron> neurons = this->layers[layer].getNeurons();
    for (unsigned int i = 0; i < neurons.size(); ++i)
    {
        double response = neurons[i].getResponse();
        responseVector.push_back(response);
    }
    return responseVector;
}

CNeuralNetwork::~CNeuralNetwork()
{
}

