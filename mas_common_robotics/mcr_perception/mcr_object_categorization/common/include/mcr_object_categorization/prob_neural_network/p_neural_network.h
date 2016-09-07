/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef CPNEURALNETWORK_H_
#define CPNEURALNETWORK_H_

#include <iostream>
#include <stdio.h>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <utility>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include "object_description.h"
#include "object_decomposition.h"
#include "logger.h"

struct SPNeuralNetworkNeuron
{
    int label;
    std::map<int, std::vector<double> > pattern;
    SPNeuralNetworkNeuron() :
        label(0)
    {
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & label;
        ar & pattern;
    }
};

class CPNeuralNetwork
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & model;
        ar & trainModel;
        ar & testModel;
        ar & stdDeviation;
        ar & trainTestSetRatio;
    }

protected:
    //neuron(label), pattern(objects), featureVector
    std::map<int, std::map<int, std::vector<double> > > model;
    std::map<int, std::map<int, std::vector<double> > > trainModel;
    std::map<int, std::map<int, std::vector<double> > > testModel;
    double stdDeviation;

    double computeResponse(std::map<int, std::vector<double> > queryPattern, std::map<int, std::vector<double> > modelPattern);
    double kernelGaussian(double u, double stdDerivation);
    double mean(std::vector<double> histo);
    double variance(std::vector<double> histo, double mean);
    double getKLD(std::vector<double> a, std::vector<double> b);
    double getEuclideanDistance(std::vector<double> a, std::vector<double> b);
    double getJensenShannonDivergence(std::vector<double> a, std::vector<double> b);
    double diffEntropy(std::vector<double> a);
    double getCosineDistance(std::vector<double> a, std::vector<double> b);
    double log_2(double n);
    void verifyPNNModel(std::map<int, std::map<int, std::vector<double> > > toVerifyModel);

    float trainTestSetRatio; //0.7 -> 0.7 train , 0.3 test
    CLogger *logger;

public:
    CPNeuralNetwork();

    //neuron(label), model(objects), featureVector
    void addModel(std::map<int, std::map<int, std::vector<double> > > &model);
    void addModel(int label, std::map<int, std::vector<double> > &model);
    std::pair<int, double> evaluate(std::map<int, std::map<int, std::vector<double> > > query);

    //label, measure of similarity
    std::pair<int, double> evaluate(std::map<int, std::map<int, std::vector<double> > > query, int labelToCompareWith);

    std::vector<double> normalizeHist(std::vector<double> histo);

    void init(float trainTestSetRatio = -1);

    void verifyPNN(std::map<int, CObjectDecomposition> objects);
    void verifyPNN();
    ~CPNeuralNetwork();
};
/*
 namespace boost
 {
 namespace serialization
 {

 template<class Archive>
 void serialize(Archive & ar, CPNeuralNetwork & pnn, SPNeuralNetworkNeuron & pnnn,
 const unsigned int version)
 {
 ar & pnn.model;
 ar & pnn.trainModel;
 ar & pnn.testModel;
 ar & pnn.stdDeviation;
 ar & pnn.trainTestSetRatio;
 ar & pnnn.label;
 ar & pnnn.pattern;
 }

 } // namespace serialization
 } // namespace boost

 */
#endif /* CPNEURALNETWORK_H_ */
