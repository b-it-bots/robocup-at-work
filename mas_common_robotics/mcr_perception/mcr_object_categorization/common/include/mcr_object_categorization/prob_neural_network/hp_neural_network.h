/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef CHPNEURALNETWORK_H_
#define CHPNEURALNETWORK_H_

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/archive/shared_ptr_helper.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

#include "prob_neural_network/p_neural_network.h"
#include "logger.h"

//this is a node in the network
struct SHPNeuralNetworkNeuron
{
    SPNeuralNetworkNeuron neuron;
    boost::shared_ptr<SHPNeuralNetworkNeuron> leftNeuron;
    boost::shared_ptr<SHPNeuralNetworkNeuron> rightNeuron;

    //this are labels which has been sorted to this Neuron
    //however it must be now decided whether they are sorted to leftNeuron or right neuron.
    //so again the most dissimilar labels are found and then the rest of them are sortet between left and right
    std::vector<int> labelToSort;

    SHPNeuralNetworkNeuron()
    {
        leftNeuron.reset();
        rightNeuron.reset();
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & leftNeuron;
        ar & rightNeuron;
        ar & neuron;
        ar & labelToSort;
    }
};
BOOST_SERIALIZATION_SHARED_PTR(SHPNeuralNetworkNeuron)

struct SVerficationResult
{
    double totalCorrect;
    double totalFalse;
    //label
    std::map<int, double> correctLabel;
    std::map<int, double> falseLabel;

    int numberOfExamples;

    SVerficationResult() :
        totalCorrect(-1), totalFalse(-1), numberOfExamples(0)
    {
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & totalCorrect;
        ar & totalFalse;
        ar & correctLabel;
        ar & falseLabel;
        ar & numberOfExamples;
    }

};

class CHPNeuralNetwork: public CPNeuralNetwork
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<CPNeuralNetwork>(*this);
        //  ar & hModel;
        //  ar & hTrainModel;
        //  ar & hTestModel;
        ar & rootNeuron;
    }

protected:

    boost::shared_ptr<SHPNeuralNetworkNeuron> rootNeuron;
    //neuron(label), patterns(objects), label'(accordinf all labels), pattern'(all patterns from label'), featureVector (pattern feature vector according label' and pattern' cfg)
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hModel;

    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hTrainModel;

    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hTestModel;

    int findValue(std::vector<int> vec, int value);

    double getSimilarity(int label1, int label2);

    //label, label of highest error between labels
    std::pair<int, int> findMaxDissimilarity(std::map<int, std::map<int, std::vector<double> > > neurons);
    std::pair<int, int> findMaxDissimilarity(std::vector<int> neurons);
    std::map<int, std::vector<int> > findSimilarity(std::vector<int> &neurons, int label1, int label2);

    void divideAndSort(std::vector<int> &neuronsToEvaluate, boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron);

    std::pair<int, double> getMaxResponse(std::vector<std::pair<int, double> > responses);

    float trainTestSetRatio; //0.7 -> 0.7 train , 0.3 test
    CLogger *logger;
public:
    CHPNeuralNetwork();
    void initHierarchy(float trainTestSetRatio = -1);

    //label, pattern
    void addHModel(int label, int pattern, std::map<int, std::map<int, std::vector<double> > > &model);

    void setHModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > &model);

    std::pair<int, double> hEvaluate(std::map<int, std::map<int, std::vector<double> > > query);

    void verifyHPNN();
    SVerficationResult verifyHPNNModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > toVerifyModel);

    ~CHPNeuralNetwork()
    {

    }
    ;
};
/*
 namespace boost
 {
 namespace serialization
 {

 template<class Archive>
 void serialize(Archive & ar, SPNeuralNetworkNeuron & pnnn,
 const unsigned int version)
 {
 ar & pnnn.label;
 ar & pnnn.pattern;
 }

 } // namespace serialization
 } // namespace boost


 */

#endif /* CPNEURALNETWORK_H_ */
