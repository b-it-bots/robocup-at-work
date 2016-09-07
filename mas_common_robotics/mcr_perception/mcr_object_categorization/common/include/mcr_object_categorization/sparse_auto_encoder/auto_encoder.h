/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */


#ifndef _CAUTOENCODER_H
#define _CAUTOENCODER_H

#include <stdio.h>
#include <iostream>
#include <string>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "sparse_auto_encoder/neural_network.h"
#include "logger.h"

class CAutoEncoder
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->nn;
        ar & this->numInputs;
        ar & this->numOutputs;
        ar & this->numHiddenNeurons;
        ar & this->inputVector;
        ar & this->inputVectorSet;
    }

    CLogger *logger;
    CNeuralNetwork nn;

    int numInputs;
    int numOutputs;
    float numHiddenNeurons; //percentage of inputneurons!
    std::vector<double> inputVector;
    std::vector<std::vector<double> > inputVectorSet;

    double getMeanSquaredError(std::vector<double> inputVector, std::vector<double> response);
    double getMeanSquaredError(std::vector<std::vector<double> > inputVector, std::vector<std::vector<double> > response);
public:
    CAutoEncoder();
    CAutoEncoder(int numInputs, int numHiddenLayer, int numOutputs);
    void init(int numInputs, int numHiddenLayer, int numOutputs);//, double biasHiddenLayer,double biasOutputLayer);
    void init(int numInputs, int numHiddenLayer, int numOutputs, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector);
    void input(std::vector<double> inputVector);
    void input(std::vector<std::vector<double> > inputVector);
    void input(std::string inputFile);

    void setNumHiddenNeuron(float numHiddenNeurons);

    std::vector<double> feedVector();
    std::vector<std::vector<double> > feedVectorSet();

    std::vector<double> feedVector(std::vector<double> inputVector);
    std::vector<std::vector<double> > feedVectorSet(std::vector<std::vector<double> > inputVectorSet);

    double train(double allowedMSE, int maxIteration);
    double train(double allowedMSE, int maxIteration, float numHiddenNeurons);

    std::vector<std::vector<double> > encode(std::string inputFile);
    std::vector<std::vector<double> > encode(std::vector<std::vector<double> > toEncode);
    std::vector<double> encode(std::vector<double> toEncode);
    std::vector<double> decode(std::vector<double> toEncode);
    std::vector<double> encodeDecode(std::vector<double> inputVectorToEncode);

    double trainVector(double allowedMSE, int maxIteration);
    double trainVectorSet(double allowedMSE, int maxIteration);
    std::vector<std::vector<double> > randomizeVectorSet(std::vector<std::vector<double> > inputVectorSet);

    ~CAutoEncoder();
};

//BOOST_CLASS_VERSION(CAutoEncoder, 1)

#endif
