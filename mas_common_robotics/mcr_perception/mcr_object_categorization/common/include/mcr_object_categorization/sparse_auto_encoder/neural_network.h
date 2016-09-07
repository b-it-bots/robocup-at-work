/*
 * File:   CNeuralNetwork.h
 * Author: Christian Mueller
 *
 * Created on May 28, 2011, 6:15 PM
 */

#ifndef CNEURALNETWORK_H
#define CNEURALNETWORK_H

#include <iostream>
#include <stdio.h>
#include <vector>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "sparse_auto_encoder/layer.h"
#include "sparse_auto_encoder/neuron.h"
#include "logger.h"

class CNeuralNetwork
{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->layers;
        ar & this->numInputs;
        ar & this->numLabels;
        ar & this->sparseParameter;
        ar & this->learningRate;
        ar & this->learningRateSparsity;
        ar & this->weightDecay;
        ar & this->inputVector;
        ar & this->labelVector;
    }

    CLogger *logger;
    std::vector<CLayer> layers;
    int numInputs;
    int numLabels;
    double sparseParameter;
    double learningRate;
    double learningRateSparsity;
    double weightDecay;
    std::vector<double> inputVector;
    std::vector<double> labelVector;

    std::vector<double> computeError(int layer);
    std::vector<double> computeError2(int layer);

    void updateWeights(int layer);
    void updateWeights2(int layer);

public:
    CNeuralNetwork();
    CNeuralNetwork(int numInputs, int numLabels);
    void addLayer(int numLayerInputs, int numNeurons, double bias);
    void addLayer(int numLayerInputs, int numNeurons, double bias, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianzVector);
    void initNeuralNetwork();
    void input(std::vector<double> inputVector, std::vector<double> labelVector);
    std::vector<double> feed();
    std::vector<double> feed(std::vector<double> inputVector, std::vector<double> labelVector);
    std::vector<std::vector<double> > feedForward(std::vector<double> inputVector);
    std::vector<double> feedForwardOnce(std::vector<double> inputVector, int layer);

    void backPropagation();

    std::vector<std::vector<double> > getWeightVector(unsigned int layer);
    std::vector<double> getResponseVector(unsigned int layer);
    int getNumLayers()
    {
        return this->layers.size();
    }

    ~CNeuralNetwork();

};

//BOOST_CLASS_VERSION(CNeuralNetwork, 1)

#endif  /* CNEURALNETWORK_H */

