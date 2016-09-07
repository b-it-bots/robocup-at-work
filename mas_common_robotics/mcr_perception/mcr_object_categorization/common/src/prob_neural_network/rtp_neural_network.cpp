/*
 * CRTPNeuralNetwork.cpp
 *
 *  Created on: May 15, 2011
 *      Author:  Christian Mueller
 */


#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <utility>

#include "prob_neural_network/rtp_neural_network.h"

int CRTPNeuralNetwork::instanceCounter = 0;

CRTPNeuralNetwork::CRTPNeuralNetwork()
{
    logger->log->debug("CRTPNeuralNetwork::CRTPNeuralNetwork...");
    // TODO Auto-generated constructor stub
    CHPNeuralNetwork();
    srand((unsigned) time(0));
    this->numRandomAttempt = NUM_RAND_ATTEMPT;

    this->instanceNum = CRTPNeuralNetwork::instanceCounter;
    CRTPNeuralNetwork::instanceCounter++;
}

CRTPNeuralNetwork::~CRTPNeuralNetwork()
{

    // TODO Auto-generated destructor stub
}

void CRTPNeuralNetwork::initHierarchy(float trainTestSetRatio)
{
    this->trainTestSetRatio = trainTestSetRatio;
    this->logger->log->debug("CRTPNeuralNetwork::initHierachy init...(trainTestSetRatio=%lf)\n", this->trainTestSetRatio);

    if (this->hModel.size() == 0)
        return;
    //since it is the root neuron
    this->rootNeuron->neuron.label = 0;
    std::vector<int> neuronsToEvaluate;
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > >::iterator iterHModel;
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPattern;

    /////////////////////////////////////
    //Divide hModel regarding  trainTestRatio;
    if (this->trainTestSetRatio < (0 + std::numeric_limits<float>::epsilon()))
    {
        this->hTrainModel = hModel;
        this->logger->log->debug("CRTPNeuralNetwork::initHierarchy...complete examples used as trainExamples\n");
    }
    else
    {
        for (iterHModel = this->hModel.begin(); iterHModel != this->hModel.end(); ++iterHModel)
        {
            std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern;
            pattern = iterHModel->second;

            int numberOfTrainExamples = (int) ceil(float((float) pattern.size() * this->trainTestSetRatio));
            if (numberOfTrainExamples <= 0)
            {
                this->hTrainModel = hModel;

                this->logger->log->info("CRTPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                        this->hTrainModel[iterHModel->first].size(), this->hModel[iterHModel->first].size());

            }
            else //create Train and test examples
            {
                int currNumberofTrainExamples = 0;
                for (iterPattern = pattern.begin(); iterPattern != pattern.end(); ++iterPattern)
                {
                    if (currNumberofTrainExamples < numberOfTrainExamples)
                    {
                        this->hTrainModel[iterHModel->first][iterPattern->first] = iterPattern->second;
                    }
                    else
                    {
                        this->hTestModel[iterHModel->first][iterPattern->first] = iterPattern->second;
                    }
                    currNumberofTrainExamples++;
                }
                this->logger->log->info("CRTPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                        this->hTrainModel[iterHModel->first].size(), this->hModel[iterHModel->first].size());
            }
        }

    }
    /////////////////////////////////////

    for (iterHModel = this->hTrainModel.begin(); iterHModel != this->hTrainModel.end(); ++iterHModel)
    {
        neuronsToEvaluate.push_back(iterHModel->first);
    }
    boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron = this->rootNeuron;

    //  int i = 1;
    //do {
    this->divideAndSort(neuronsToEvaluate, parentNeuron);

    this->verifyHPNN();
    /*
     std::cout << "LEFT ::" << this->rootNeuron->leftNeuron->neuron.label
     << "\n";
     std::cout << "RIGHT ::" << this->rootNeuron->rightNeuron->neuron.label
     << "\n";

     //std::cout<<"LEFT ::" << this->rootNeuron->leftNeuron<<"\n";
     std::cout << "LEFT RIGHT ::"
     << this->rootNeuron->leftNeuron->rightNeuron->neuron.label << "\n";
     std::cout << "LEFT RIGHT RIGHT ::"
     << this->rootNeuron->leftNeuron->rightNeuron->rightNeuron->neuron.label
     << "\n";
     if (this->rootNeuron->leftNeuron->rightNeuron->rightNeuron->rightNeuron
     == NULL) {
     std::cout << "SUPER!! \n";
     } else {
     std::cout << "LEFT RIGHT RIGHT RIGHT::"
     << this->rootNeuron->leftNeuron->rightNeuron->rightNeuron->rightNeuron->neuron.label
     << "\n";
     }*/
    //  } while (neuronsToEvaluate.size() > 1);
}

std::pair<int, int> CRTPNeuralNetwork::findRandomMaxDissimilarity(std::vector<int> neurons, unsigned int attempt)
{

    // cat , patid, cat' paid', feat
    //std::map<int, std::map<int, std::map<int,  std::map<int, std::vector<double> > > > > neurons

    this->logger->log->debug("CRTPNeuralNetwork::findRandomMaxDissimilarity...\n");

    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPat1, iterPat2;

    //get random selected pairs (number of paris == attempts)
    std::vector < std::pair<int, int> > randomSelectedParis = this->getRandomPairs(neurons, attempt);

    //label, label
    std::pair<int, int> minSimilarLabels;
    minSimilarLabels.first = 0;
    minSimilarLabels.second = 0;
    double minDissimilarity = std::numeric_limits<double>::infinity();
    double currSimilarity = std::numeric_limits<double>::infinity();

    for (unsigned int iterPairs = 0; iterPairs < randomSelectedParis.size(); ++iterPairs)
    {
        unsigned int iterNeurons1 = randomSelectedParis[iterPairs].first;
        unsigned int iterNeurons2 = randomSelectedParis[iterPairs].second;

        this->logger->log->debug("CRTPNeuralNetwork::findRandomMaxDissimilarity...Check Pair %d : %s -- %s \n", iterPairs, CFileSettings::labels[neurons[iterNeurons1]].c_str(),
                                 CFileSettings::labels[neurons[iterNeurons2]].c_str());
        currSimilarity = 0;
        // Do not compare the same label
        if (neurons[iterNeurons1] != neurons[iterNeurons2])
        {
            std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern1 = this->hTrainModel[neurons[iterNeurons1]];
            std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern2 = this->hTrainModel[neurons[iterNeurons2]];

            //std::cout<<"CHPNeuralNetwork::findMaxDissimilarity should be same;


            //now campare the pattern according their representation to the required category
            int numComparison = 0;
            for (iterPat1 = pattern1.begin(); iterPat1 != pattern1.end(); ++iterPat1)
            {
                for (iterPat2 = pattern2.begin(); iterPat2 != pattern2.end(); ++iterPat2)
                {

                    //Compare pat1 with pat2 regarding the label cfg from pat1
                    //std::cout << "Comparing " << iterNeurons1->first<<"("<<iterPat1->first<<")"
                    //      << "--" << iterNeurons2->first<<"("<<iterPat2->first<<")" << "\n";
                    currSimilarity += this->computeResponse(iterPat1->second[neurons[iterNeurons1]], iterPat2->second[neurons[iterNeurons1]]);
                    //std::cout << "currDissimilarityError "
                    //      << currSimilarity << "\n";
                    ++numComparison;
                }
            }
            currSimilarity /= ((double) numComparison);
            if (currSimilarity < minDissimilarity)
            {
                minDissimilarity = currSimilarity;
                minSimilarLabels.first = neurons[iterNeurons1];
                minSimilarLabels.second = neurons[iterNeurons2];
            }

        }
    }

    return minSimilarLabels;
}

void CRTPNeuralNetwork::divideAndSort(std::vector<int> &neuronsToEvaluate, boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron)
{
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPatterns;
    std::map<int, std::vector<int> >::iterator iiiter;

    std::pair<int, int> maxDissimilarLabels;

    if (neuronsToEvaluate.size() <= 1)
    {
        this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...stop neuronsToEvaluate=%d \n", neuronsToEvaluate.size());
        //parentNeuron->leftNeuron=NULL;
        //parentNeuron->rightNeuron=NULL;
        if (neuronsToEvaluate.size() == 1)
        {

            this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...Single Label %s added as left to %s \n", CFileSettings::labels[neuronsToEvaluate[0]].c_str(),
                                     CFileSettings::labels[parentNeuron->neuron.label].c_str());

            boost::shared_ptr<SHPNeuralNetworkNeuron> leftNeuron(new SHPNeuralNetworkNeuron);
            leftNeuron->neuron.label = neuronsToEvaluate[0];
            for (iterPatterns = this->hTrainModel[neuronsToEvaluate[0]].begin(); iterPatterns != this->hTrainModel[neuronsToEvaluate[0]].end(); ++iterPatterns)
            {
                leftNeuron->neuron.pattern[iterPatterns->first] = iterPatterns->second[neuronsToEvaluate[0]][iterPatterns->first];
            }
            parentNeuron->leftNeuron = leftNeuron;
        }

        return;
    }

    maxDissimilarLabels = this->findRandomMaxDissimilarity(neuronsToEvaluate, this->numRandomAttempt);

    this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...Most Dissim are %s <> %s \n", CFileSettings::labels[maxDissimilarLabels.first].c_str(),
                             CFileSettings::labels[maxDissimilarLabels.second].c_str());

    boost::shared_ptr<SHPNeuralNetworkNeuron> leftNeuron(new SHPNeuralNetworkNeuron);
    leftNeuron->neuron.label = maxDissimilarLabels.first;
    for (iterPatterns = this->hTrainModel[maxDissimilarLabels.first].begin(); iterPatterns != this->hTrainModel[maxDissimilarLabels.first].end(); ++iterPatterns)
    {
        leftNeuron->neuron.pattern[iterPatterns->first] = iterPatterns->second[maxDissimilarLabels.first][iterPatterns->first];
    }

    boost::shared_ptr<SHPNeuralNetworkNeuron> rightNeuron(new SHPNeuralNetworkNeuron);
    rightNeuron->neuron.label = maxDissimilarLabels.second;
    for (iterPatterns = this->hTrainModel[maxDissimilarLabels.second].begin(); iterPatterns != this->hTrainModel[maxDissimilarLabels.second].end(); ++iterPatterns)
    {
        rightNeuron->neuron.pattern[iterPatterns->first] = iterPatterns->second[maxDissimilarLabels.second][iterPatterns->first];
    }

    parentNeuron->leftNeuron = leftNeuron;
    parentNeuron->rightNeuron = rightNeuron;

    neuronsToEvaluate.erase(neuronsToEvaluate.begin() + findValue(neuronsToEvaluate, maxDissimilarLabels.first));
    neuronsToEvaluate.erase(neuronsToEvaluate.begin() + findValue(neuronsToEvaluate, maxDissimilarLabels.second));

    //Now sort the left neurons to the left and right neurons;
    std::map<int, std::vector<int> > maxSimilarityLabel = this->findSimilarity(neuronsToEvaluate, maxDissimilarLabels.first, maxDissimilarLabels.second);

    for (iiiter = maxSimilarityLabel.begin(); iiiter != maxSimilarityLabel.end(); ++iiiter)
    {
        this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...Similar %s: \n ", CFileSettings::labels[iiiter->first].c_str());
        for (unsigned int ii = 0; ii < iiiter->second.size(); ++ii)
        {
            this->logger->log->debug("  %s\n", CFileSettings::labels[iiiter->second[ii]].c_str());
        }
    }
    std::vector<int> leftNeuronsToEvaluate = maxSimilarityLabel[maxDissimilarLabels.first];
    std::vector<int> rightNeuronsToEvaluate = maxSimilarityLabel[maxDissimilarLabels.second];

    this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...start left(parent=%s)...\n", CFileSettings::labels[parentNeuron->leftNeuron->neuron.label].c_str());
    this->divideAndSort(leftNeuronsToEvaluate, parentNeuron->leftNeuron);

    this->logger->log->debug("CRTPNeuralNetwork::divideAndSort...start right(parent=%s)...\n", CFileSettings::labels[parentNeuron->rightNeuron->neuron.label].c_str());
    this->divideAndSort(rightNeuronsToEvaluate, parentNeuron->rightNeuron);
    //remove labels of maxDiss from neuros Toeval

    //check if neuron.size == 1 add it to the most similar parent node
}

std::vector<std::pair<int, int> > CRTPNeuralNetwork::getRandomPairs(std::vector<int> neurons, unsigned int numPairs)
{
    this->logger->log->debug("CRTPNeuralNetwork::getRandomPairs...Num neurons %d Num pairs %d...\n", neurons.size(), numPairs);
    std::vector < std::pair<int, int> > pairs;
    for (unsigned int iterPairs = 0; iterPairs < numPairs; ++iterPairs)
    {
        pairs.push_back(this->getRandomPair(neurons));
    }

    return pairs;
}

std::pair<int, int> CRTPNeuralNetwork::getRandomPair(std::vector<int> neurons)
{
    std::pair<int, int> randPairSelection;
    if (neurons.size() == 2)
    {
        randPairSelection.first = 0;
        randPairSelection.second = 1;
    }
    else if (neurons.size() > 2)
    {
        int randSelectionIdx1;
        int randSelectionIdx2;
        do
        {
            randSelectionIdx1 = rand() % neurons.size();
            randSelectionIdx2 = rand() % neurons.size();

        }
        while (randSelectionIdx1 == randSelectionIdx2);

        randPairSelection.first = randSelectionIdx1;
        randPairSelection.second = randSelectionIdx2;
    }
    else
    {
        this->logger->log->warn("CRTPNeuralNetwork::getRandomPair...Less than two neurons to randomly select, return -1,-1!!!\n");
        randPairSelection.first = -1;
        randPairSelection.second = -1;
    }
    return randPairSelection;
}

