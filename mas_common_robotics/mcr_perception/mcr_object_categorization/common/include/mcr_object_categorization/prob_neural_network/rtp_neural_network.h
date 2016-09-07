/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef CRTPNEURALNETWORK_H_
#define CRTPNEURALNETWORK_H_

#include <boost/shared_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>

#include "prob_neural_network/hp_neural_network.h"
#include "file_settings.h"
#include "logger.h"

#define NUM_RAND_ATTEMPT 10
//3

class CRTPNeuralNetwork: public CHPNeuralNetwork
{

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<CHPNeuralNetwork>(*this);
        ar & numRandomAttempt;
        ar & instanceNum;
    }
protected:
    int numRandomAttempt;

    static int instanceCounter;
    int instanceNum;

public:
    CRTPNeuralNetwork();
    void initHierarchy(float trainTestSetRatio = -1);
    std::pair<int, int> findRandomMaxDissimilarity(std::vector<int> neurons, unsigned int attempt);
    std::vector<std::pair<int, int> > getRandomPairs(std::vector<int> neurons, unsigned int numPairs);
    std::pair<int, int> getRandomPair(std::vector<int> neurons);
    void divideAndSort(std::vector<int> &neuronsToEvaluate, boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron);

    ~CRTPNeuralNetwork();
};

#endif /* CRTPNEURALNETWORK_H_ */
