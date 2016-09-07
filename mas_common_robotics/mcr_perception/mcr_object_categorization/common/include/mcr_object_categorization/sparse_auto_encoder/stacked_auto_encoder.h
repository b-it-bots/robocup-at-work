/*
 * File:   CStackedAutoEncoder.h
 * Author: Christian Mueller
 *
 * Created on June 1, 2011, 5:59 PM
 */

#ifndef CSTACKEDAUTOENCODER_H
#define CSTACKEDAUTOENCODER_H

#include <iostream>
#include <cstdlib>
#include <vector>
#include <map>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "logger.h"
#include "sparse_auto_encoder/auto_encoder.h"

class CStackedAutoEncoder
{
public:
    CStackedAutoEncoder();
    CStackedAutoEncoder(int numStacks);
    void init(int numStacks);

    void input(std::string inputFile);
    void input(std::vector<std::vector<double> > inputVectorSet);

    void train(double allowedMSE, int maxIteration);
    void trainOpt(double allowedMSE, int maxIteration);

    std::vector<double> encode(std::vector<double> query);
    std::vector<double> decode(std::vector<double> query);
    std::vector<double> encodeDecode(std::vector<double> query);
    double encodeDecode(std::vector<std::vector<double> > query);
    std::vector<double> encodeDecode(std::vector<double> query, double &getMSEError);

    double getMeanSquaredError(std::vector<double> inputVector, std::vector<double> response);
    virtual ~CStackedAutoEncoder();
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->numStacks;
        ar & this->stackedAE;
        ar & this->inputVectorSet;
        ar & this->inputVector;
    }

    CLogger *logger;
    int numStacks;
    std::vector<CAutoEncoder> stackedAE;
    std::vector<std::vector<double> > inputVectorSet;
    std::vector<double> inputVector;

    void trainVectorSet(double allowedMSE, int maxIteration);
    void trainVectorSetOpt(double allowedMSE, int maxIteration);

    void trainVector(double allowedMSE, int maxIteration);

    CAutoEncoder paramGridSearch(double allowedMSE, int maxIteration, std::vector<std::vector<double> > trainSet);
};

BOOST_CLASS_VERSION(CStackedAutoEncoder, 1)

#endif  /* STACKEDAUTOENCODER_H */

