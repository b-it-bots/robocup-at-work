/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef CPNEURALNETWORKT_H_
#define CPNEURALNETWORKT_H_

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <utility>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/threadpool.hpp>

#include "neural_network/p_neural_network.h>

class CPNeuralNetworkT: public CPNeuralNetwork {
private:

public:
    CPNeuralNetworkT();
    static double stdDeviationT;
    static std::map<int, double> evalutionResponses;
    static boost::mutex evalResp_mutex;
    static double getJensenShannonDivergenceT(std::vector<double> a,
            std::vector<double> b);
    static std::vector<double> normalizeHistT(
        std::vector<double> histo);
    static double diffEntropyT(std::vector<double> a);
    static double kernelGaussianT(double u, double stdDerivation);
    static double log_2(double n);
    int evaluate(std::map<int, std::map<int, std::vector<double> > > query);
    static void computeResponse(int label,std::map<int, std::vector<double> > queryPattern,
                                std::map<int, std::vector<double> > modelPattern);

    ~CPNeuralNetworkT(){};
};

#endif /* CPNEURALNETWORK_H_ */
