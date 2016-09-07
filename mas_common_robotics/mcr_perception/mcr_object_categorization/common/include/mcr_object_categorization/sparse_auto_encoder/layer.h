/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CLAYER_H
#define CLAYER_H

#include <iostream>
#include <map>
#include <vector>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "logger.h"
#include "sparse_auto_encoder/neuron.h"

class CLayer
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->neurons;
        ar & this->bias;
        ar & this->inputVector;
        ar & this->errorVector;
        ar & this->responseVector;
        ar & this->numInputs;
        ar & this->numNeurons;
    }

    CLogger *logger;
    std::vector<CNeuron> neurons;
    double bias;

    std::vector<double> inputVector;
    std::vector<double> errorVector;
    std::vector<double> responseVector;
    int numInputs;
    int numNeurons;

    std::vector<double> computeFeedForward();
public:
    CLayer();
    CLayer(int numInputs, int numNeurons, double bias, double sparseParameter);
    CLayer(int numInputs, int numNeurons, double bias, double sparseParameter, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector);
    void initLayer(int numInputs, int numNeurons, double bias, double sparseParameter);
    void initLayer(int numInputs, int numNeurons, double bias, double sparseParameter, std::vector<double> initValueMeanVector, std::vector<double> initValueVarianceVector);

    void input(std::vector<double> inputVector);

    std::vector<double> feedForward();
    std::vector<double> feedForward(std::vector<double> inputVector);
    int getNumNeurons()
    {
        return numNeurons;
    }
    std::vector<CNeuron> &getNeurons()
    {
        return this->neurons;
    }

    std::vector<double> &getResponseVector()
    {
        return this->responseVector;
    }
    void setResponseVector(std::vector<double> responseVector)
    {
        this->responseVector = responseVector;
    }

    std::vector<double> &getErrorVector()
    {
        return this->errorVector;
    }
    std::vector<double> &getInputVector()
    {
        return this->inputVector;
    }
    void setErrorVector(std::vector<double> errorVector)
    {
        this->errorVector = errorVector;
    }

    ~CLayer();

};
BOOST_CLASS_VERSION(CLayer, 1)

#endif
