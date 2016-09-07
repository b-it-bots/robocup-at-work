/*
 * CPNeuralNetwork.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author:  Christian Mueller
 */


#include <algorithm>

#include "prob_neural_network/p_neural_network_t.h"
#include "prob_neural_network/file_settings.h"


std::map<int, double> CPNeuralNetworkT::evalutionResponses;
boost::mutex CPNeuralNetworkT::evalResp_mutex;
double CPNeuralNetworkT::stdDeviationT = 0.1;

CPNeuralNetworkT::CPNeuralNetworkT()
{
    CPNeuralNetwork();
}

double CPNeuralNetworkT::getJensenShannonDivergenceT(std::vector<double> a,
        std::vector<double> b)
{

    std::vector<double> delta;

    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (unsigned int m = 0; m < a.size(); m++)
        {
            delta[m] = ((a[m] + b[m]) / 2.0f);
        }

        double firstTerm = CPNeuralNetworkT::diffEntropyT(delta);
        double aVal, bVal;

        aVal = CPNeuralNetworkT::diffEntropyT(a);

        bVal = CPNeuralNetworkT::diffEntropyT(b);

        //std::cout << "aVal " << aVal << "          " << "bVal " << bVal << "\n";
        double secondTerm = (aVal + bVal) / 2.0f;

        return (firstTerm - secondTerm);
    }
    else
        return -1;
}

double CPNeuralNetworkT::diffEntropyT(std::vector<double> a)
{
    double entropy = 0;
    for (unsigned int m = 0; m < a.size(); m++)
    {
        double res;
        res = a[m] * log_2(a[m]);
        //std::cout<<"entropy"<< a[m]<<" "<< entropy<<"\n";
        if (res != res) // check for nan
        {
            //std::cout<<" Y "<<std::endl;
            res = 0;
        }
        entropy += res;
    }

    return (entropy * (-1));
}

double CPNeuralNetworkT::kernelGaussianT(double u, double stdDerivation)
{
    /*this->kernelGaussian(((query[i] - dd[d]) / h));*/
    //return (1.0f / sqrt(2.0f * PI)) * exp((-0.5f) * pow(u, 2));

    return exp((double) pow(u, 2.0) / (-2.0 * pow(stdDerivation, 2.0)));
}

std::vector<double> CPNeuralNetworkT::normalizeHistT(std::vector<double> histo)
{
    //Normalize histogram to 1 !
    if (histo.size() > 0)
    {
        double sum = 0;
        for (unsigned k = 0; k < (unsigned) histo.size(); k++)
        {
            sum += histo[k];
        }

        if (sum > 0)
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = (double) histo[k] / (double) sum;
            }
        }
        else
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = 0;
            }
        }
    }
    else
    {
        histo.clear();
        histo.push_back(0);
    }

    return histo;
}

void CPNeuralNetworkT::computeResponse(int label, std::map < int, std::vector <
                                       double > > queryPattern,
                                       std::map<int, std::vector<double> > modelPattern)
{
    std::cout << " CPNeuralNetwork::computeResponse ....\n";
    std::map<int, std::vector<double> >::iterator iterPatterns;

    double totalResponse = 0.0;
    std::cout << "   Pattern responses:\n";
    for (iterPatterns = modelPattern.begin(); iterPatterns
            != modelPattern.end(); ++iterPatterns)
    {
        double response;

        //Eucl
        //response = this->kernelGaussian(getEuclideanDistance(
        //      iterPatterns->second, queryPattern[iterPatterns->first]), this->stdDeviation);

        //JS
        response = CPNeuralNetworkT::kernelGaussianT(
                       CPNeuralNetworkT::getJensenShannonDivergenceT(
                           CPNeuralNetworkT::normalizeHistT(iterPatterns->second),
                           CPNeuralNetworkT::normalizeHistT(
                               queryPattern[iterPatterns->first])),
                       CPNeuralNetworkT::stdDeviationT);

        //  CFileSettings::coutStdVector(queryPattern[iterPatterns->first],true);
        std::cout << "   Pattern " << iterPatterns->first << " = " << response
                  << "\n";
        totalResponse += response;
    }

    CPNeuralNetworkT::evalResp_mutex.lock();
    CPNeuralNetworkT::evalutionResponses[label] = ((double) totalResponse
            / (double) modelPattern.size());
    CPNeuralNetworkT::evalResp_mutex.unlock();
}

/* network consists of neurons
 * neurons consists of submodels
 */

bool value_comparer(std::map<int, double>::value_type &i1,
                    std::map<int, double>::value_type &i2)
{
    return i1.second < i2.second;
}

int CPNeuralNetworkT::evaluate(std::map < int,
                               std::map<int, std::vector<double> > > query)
{

    CPNeuralNetworkT::evalutionResponses.clear();

    std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons;
    std::map<int, std::vector<double> >::iterator iterPattern;
    std::cout << " CPNeuralNetwork::evaluate ....\n";

    //neuron, responseValue;
    //std::map<int, double> response;

    std::pair<int, double> maxResponse;
    maxResponse.first = 0;
    maxResponse.second = 0;

    boost::threadpool::pool tp(8);

    for (iterNeurons = model.begin(); iterNeurons != model.end(); ++iterNeurons)
    {
        std::map<int, std::vector<double> > modelPattern = iterNeurons->second;
        std::map<int, std::vector<double> > queryPattern =
            query[iterNeurons->first];
        //response[iterNeurons->first] = computeResponse(queryPattern,
        //  modelPattern);

        tp.schedule(boost::bind(CPNeuralNetworkT::computeResponse,
                                iterNeurons->first, queryPattern, modelPattern));

    }
    tp.wait();

    /*

     if (response.second > maxResponse.second) {
     maxResponse = response;
     }*/

    std::map<int, double>::iterator iterMax = std::max_element(
                CPNeuralNetworkT::evalutionResponses.begin(),
                CPNeuralNetworkT::evalutionResponses.end(), value_comparer);

    std::cout << "   Final response to label T " << iterMax->first << " = "
              << iterMax->second << "\n";

    return iterMax->first;
}

double CPNeuralNetworkT::log_2(double n)
{
    // log(n)/log(2) is log2.
    return log((double) n) / log((double) 2);
}
