/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CNEURON_H
#define CNEURON_H

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

class CNeuron
{
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->inputVector;
        ar & this->weightVector;
        ar & this->bias;
        ar & this->numInputs;
        ar & this->response;
        ar & this->sparseParameter;
        ar & this->sparsity;
    }

    CLogger *logger;
    std::vector<double> inputVector;
    std::vector<double> weightVector;
    double bias;
    int numInputs;
    double response;
    double sparseParameter;
    double sparsity;
    double activationFunctionTanh();
    double activationFunctionSig();
public:
    CNeuron();
    CNeuron(int numInputs, double bias = 1.0, double sparseParameter = 0.001);
    void init();
    void init(std::vector<double> mean, std::vector<double> variance);
    void input(std::vector<double> inputVector);
    double fire();
    double fire(std::vector<double> inputVector);
    double &getBias()
    {
        return this->bias;
    }
    void setBias(double bias)
    {
        this->bias = bias;
    }
    double &getSparsityEstimate()
    {
        return this->sparsity;
    }
    double &getSparseParameter()
    {
        return this->sparseParameter;
    }
    std::vector<double> &getWeightVector()
    {
        return weightVector;
    }
    double getResponse()
    {
        return this->response;
    }
    ~CNeuron();
};

BOOST_CLASS_VERSION(CNeuron, 1)

#endif
