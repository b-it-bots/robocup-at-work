/*
 * File:   StackedAutoEncoder.cpp
 * Author: Christian Mueller
 *
 * Created on June 1, 2011, 5:59 PM
 */

#include <assert.h>
#include <cmath>
#include <climits>
#include <time.h>
#include <algorithm>
#include <vector>
#include <cstdlib>

#include "sparse_auto_encoder/stacked_auto_encoder.h"
#include "file_settings.h"


//#include <c++/4.5/limits>

CStackedAutoEncoder::CStackedAutoEncoder()
{
    this->logger = &CLogger::getInstance();
    this->numStacks = 1;
    this->init(this->numStacks);
}

CStackedAutoEncoder::CStackedAutoEncoder(int numStacks)
{
    this->logger = &CLogger::getInstance();
    this->numStacks = numStacks;
    this->init(this->numStacks);
}

void CStackedAutoEncoder::init(int numStacks)
{
    this->logger->log->debug("CStackedAutoEncoder::init...");
    assert(numStacks > 0);
    this->stackedAE.clear();
    this->numStacks = numStacks;
    this->stackedAE.resize(this->numStacks);
    for (unsigned int i = 0; i < this->numStacks; ++i)
    {
        this->stackedAE[i] = CAutoEncoder();
    }
    this->inputVectorSet.clear();
    this->inputVector.clear();
}

void CStackedAutoEncoder::input(std::vector<std::vector<double> > inputVectorSet)
{

    assert(inputVectorSet.size() > 0);
    assert(inputVectorSet[0].size() > 0);

    this->inputVector.clear();
    this->inputVectorSet = inputVectorSet;

    this->logger->log->debug("CStackedAutoEncoder::input...TrainSetSize=%d", this->inputVectorSet.size());
    this->inputVector.clear();
}

void CStackedAutoEncoder::input(std::string inputFile)
{
    this->input(CFileSettings::loadStdVector(inputFile));
}

void CStackedAutoEncoder::train(double allowedMSE, int maxIteration)
{
    assert((this->inputVector.size() == 0 && this->inputVectorSet.size() > 0) || this->inputVector.size() > 0 && this->inputVectorSet.size() == 0);
    if (this->inputVectorSet.size() > 0)
    {
        this->trainVectorSet(allowedMSE, maxIteration);
    }

}

void CStackedAutoEncoder::trainOpt(double allowedMSE, int maxIteration)
{
    assert((this->inputVector.size() == 0 && this->inputVectorSet.size() > 0) || this->inputVector.size() > 0 && this->inputVectorSet.size() == 0);
    if (this->inputVectorSet.size() > 0)
    {
        this->trainVectorSetOpt(allowedMSE, maxIteration);
    }
}

void CStackedAutoEncoder::trainVectorSetOpt(double allowedMSE, int maxIteration)
{
    this->logger->log->debug("CStackedAutoEncoder::trainVectorSetOpt...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(this->inputVectorSet.size() > 0);

    std::vector<std::vector<double> > trainSet = this->inputVectorSet;
    std::cout << "this->stackedAE.size()" << this->stackedAE.size() << "\n";
    for (unsigned iterAE = 0; iterAE < this->stackedAE.size(); ++iterAE)
    {
        this->logger->log->info("\nCStackedAutoEncoder::trainVectorSetOpt...%d STACK", iterAE);
        std::cout << "\nCStackedAutoEncoder::trainVectorSetOpt..." << iterAE << " STACK => NumTrainSet" << trainSet.size() << " NumDim=" << trainSet[0].size() << "\n";

        //trainSet = this->inputVectorSet;
        this->stackedAE[iterAE] = this->paramGridSearch(allowedMSE, maxIteration, trainSet);

        trainSet = this->stackedAE[iterAE].encode(trainSet);
        //  std::cout << "Best  " << trainSet[0].size() << "\n";
        //  std::cout<<"\nCStackedAutoEncoder::trainVectorSet--->>"<<trainSet[0].size()<<"\n";

        /* for(unsigned int ii=0; ii < trainSet.size(); ++ii)
         {

         for(unsigned int iii=0; iii < trainSet[ii].size(); ++iii)
         {
         std::cout<<" "<<trainSet[ii][iii];
         }
         std::cout<<"\n";
         }*/
        //this is a chain!! the encoded trainSet is the input for the next stack!
    }
}

CAutoEncoder CStackedAutoEncoder::paramGridSearch(double allowedMSE, int maxIteration, std::vector<std::vector<double> > trainSet)
{

    assert(trainSet.size() > 0);

    double minHNeurons = 1.0f / (double)trainSet[0].size(); // at least one neuron
    double maxHNeurons = 1; // max neurons == 1 == as much neurons as feature vector length

    double bestHNeurons = 1;
    double steps = 0.1f; // ten step at least
    CAutoEncoder aeOptBest;
    unsigned int iter = 0;
    unsigned int gridSearchIter = 2; //3 10
    float mse = 1.0f;
    do
    {
        mse = 1.0f;
        double currErr = 1;

        std::cout << "\nCStackedAutoEncoder::paramGridSearch...Iteration No =  " << iter << "\n";
        for (double hNeurons = minHNeurons; hNeurons <= (maxHNeurons + std::numeric_limits<double>::epsilon()); hNeurons += steps)
        {

            std::vector<std::vector<double> > trainSetOpt = trainSet;
            CAutoEncoder aeOpt;
            aeOpt.setNumHiddenNeuron(hNeurons);
            aeOpt.input(trainSetOpt);
            currErr = aeOpt.train(allowedMSE, maxIteration);
            std::cout << "\nCStackedAutoEncoder::paramGridSearch...try " << hNeurons << " => err = " << currErr << "\n";

            if (currErr <= mse)
            {
                mse = currErr;
                bestHNeurons = hNeurons;
                aeOptBest = aeOpt;
            }

            //+-step size since max dist between previous min/max
        }

        minHNeurons = (bestHNeurons - ((double) steps));
        maxHNeurons = (bestHNeurons + ((double) steps));

        steps = steps * 0.5;

        iter++;
    }
    while (iter < gridSearchIter);

    std::cout << "\nFINAL MSE " << mse << " with " << bestHNeurons << " hidden neurons\n";
    return aeOptBest;
}
void CStackedAutoEncoder::trainVectorSet(double allowedMSE, int maxIteration)
{
    this->logger->log->debug("CStackedAutoEncoder::trainVectorSet...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(this->inputVectorSet.size() > 0);

    std::vector<std::vector<double> > trainSet = this->inputVectorSet;
    for (unsigned iterAE = 0; iterAE < this->stackedAE.size(); ++iterAE)
    {
        this->logger->log->info("\nCStackedAutoEncoder::trainVectorSet...%d STACK", iterAE);
        std::cout << "\nCStackedAutoEncoder::trainVectorSet..." << iterAE << " STACK => NumTrainSet" << trainSet.size() << " NumDim=" << trainSet[0].size() << "\n";

        this->stackedAE[iterAE].input(trainSet);
        this->stackedAE[iterAE].train(allowedMSE, maxIteration);

        trainSet = this->stackedAE[iterAE].encode(trainSet);
        // std::cout<<"\nCStackedAutoEncoder::trainVectorSet--->>"<<trainSet[0].size()<<"\n";

        /* for(unsigned int ii=0; ii < trainSet.size(); ++ii)
         {

         for(unsigned int iii=0; iii < trainSet[ii].size(); ++iii)
         {
         std::cout<<" "<<trainSet[ii][iii];
         }
         std::cout<<"\n";
         }*/
        //this is a chain!! the encoded trainSet is the input for the next stack!
    }
}

std::vector<double> CStackedAutoEncoder::encode(std::vector<double> query)
{
    this->logger->log->debug("CStackedAutoEncoder::feedforward...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(query.size() > 0);

    std::vector<double> toEncode = query;
    std::vector<double> output;
    for (unsigned int iterAE = 0; iterAE < this->stackedAE.size(); ++iterAE)
    {
        toEncode = this->stackedAE[iterAE].encode(toEncode);
        //this is a chain!! the encoded trainSet is the input for the next stack!
    }

    return toEncode;
}

double CStackedAutoEncoder::encodeDecode(std::vector<std::vector<double> > query)
{
    double totalError = 0.0;
    for (unsigned int i = 0; i < query.size(); ++i)
    {

        std::vector<double> cur = this->encodeDecode(query[i]);
        totalError += this->getMeanSquaredError(query[i], cur);
    }

    return totalError / (double) query.size();
}

std::vector<double> CStackedAutoEncoder::encodeDecode(std::vector<double> query)
{
    this->logger->log->debug("CStackedAutoEncoder::feedforward...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(query.size() > 0);

    std::vector<double> encoded;
    std::vector<double> decoded;

    //Encode query through the stack of Autoencoders
    encoded = this->encode(query);

    //Encode query through the stack of Autoencoders
    decoded = this->decode(encoded);

    this->logger->log->debug("CStackedAutoEncoder::encodeDecode...MSE = &lf", this->getMeanSquaredError(query, decoded));

    return decoded;
}

std::vector<double>  CStackedAutoEncoder::encodeDecode(std::vector<double> query , double &getMSEError)
{
    this->logger->log->debug("CStackedAutoEncoder::feedforward...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(query.size() > 0);

    std::vector<double> encoded;
    std::vector<double> decoded;

    //Encode query through the stack of Autoencoders
    encoded = this->encode(query);

    //Encode query through the stack of Autoencoders
    decoded = this->decode(encoded);

    getMSEError = this->getMeanSquaredError(query, decoded);
    this->logger->log->debug("CStackedAutoEncoder::encodeDecode...MSE = %lf", getMSEError);

    return decoded;
}

std::vector<double> CStackedAutoEncoder::decode(std::vector<double> query)
{
    this->logger->log->debug("CStackedAutoEncoder::feedforward...");
    assert(this->numStacks > 0);
    assert(this->numStacks == this->stackedAE.size());
    assert(query.size() > 0);

    std::vector<double> toDecode = query;

    for (int iterAE = this->stackedAE.size() - 1; iterAE >= 0; --iterAE)
    {
        toDecode = this->stackedAE[iterAE].decode(toDecode);
    }

    return toDecode;
}

/*
 std::vector<double> CStackedAutoEncoder::encode(std::vector<double> query)
 {
 this->logger->log->debug("CStackedAutoEncoder::feedforward...");
 assert(this->numStacks>0);
 assert(this->numStacks == this->stackedAE.size());
 assert(query.size()>0);

 std::vector<double> toEncode = query;
 std::vector<double> output;
 for(unsigned int iterAE=0; iterAE < this->stackedAE.size(); ++iterAE)
 {
 if(iterAE < this->stackedAE.size()-1)
 {
 toEncode = this->stackedAE[iterAE].encode(toEncode);
 }
 else if(iterAE == this->stackedAE.size()-1)
 {
 //here we can apply some outlayer like softmax or logistic regression
 // output = this->stackedAE[iterAE].encodeDecode(toEncode);
 output = this->stackedAE[iterAE].encode(toEncode);
 }
 else
 {
 this->logger->log->debug("CStackedAutoEncoder::feedforward...something is wrong!!!");
 }
 //this is a chain!! the encoded trainSet is the input for the next stack!
 }

 return output;
 }
 */

double CStackedAutoEncoder::getMeanSquaredError(std::vector<double> inputVector, std::vector<double> response)
{
    assert(inputVector.size() == response.size());

    double sumError = 0.0;
    for (unsigned int i = 0; i < inputVector.size(); ++i)
    {
        sumError += pow(inputVector[i] - response[i], 2);
    }
    return sqrt(sumError / (double) inputVector.size());
}

CStackedAutoEncoder::~CStackedAutoEncoder()
{
}

