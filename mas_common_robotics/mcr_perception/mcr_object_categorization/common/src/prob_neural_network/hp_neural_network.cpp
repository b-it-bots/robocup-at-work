/*
 * CPNeuralNetwork.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author: Christian Mueller
 */



#include <assert.h>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>

#include "file_settings.h"
#include "prob_neural_network/hp_neural_network.h"


#define PI 3.14159265

CHPNeuralNetwork::CHPNeuralNetwork()
{
    this->logger = &CLogger::getInstance();
    this->logger->log->debug("CHPNeuralNetwork::CHPNeuralNetwork...\n");
    CFileSettings::init();
    this->rootNeuron.reset(new SHPNeuralNetworkNeuron);
    this->trainTestSetRatio = -1;
    this->stdDeviation = 0.05f;//0.05 is STD!!! // worked fine 0.2f; //smoothness parameter!

}

/*
 * Note 3_0 == 3_0_3_0 , 3_1 == 3_1_3_1
 * */

void CHPNeuralNetwork::addHModel(int label, int pattern, std::map<int, std::map<int, std::vector<double> > > &model)
{
    //model represents the featVec regarding all other labels and pattern cfg according the pattern of label.
    this->hModel[label][pattern] = model;
}

void CHPNeuralNetwork::setHModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > &model)
{
    //model represents the featVec regarding all other labels and pattern cfg according the pattern of label.
    this->hModel = model;
}

// feat vec same length
std::pair<int, int> CHPNeuralNetwork::findMaxDissimilarity(std::map<int, std::map<int, std::vector<double> > > neurons)
{
    this->logger->log->debug("CHPNeuralNetwork::findMaxDissimilarity find...\n");
    std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons1, iterNeurons2;
    //label, label
    std::pair<int, int> minSimilarLabels;
    minSimilarLabels.first = 0;
    minSimilarLabels.second = 0;
    double minDissimilarity = std::numeric_limits<double>::infinity();
    double currSimilarity = std::numeric_limits<double>::infinity();

    for (iterNeurons1 = neurons.begin(); iterNeurons1 != neurons.end(); ++iterNeurons1)
    {
        std::map<int, std::vector<double> > pattern1 = iterNeurons1->second;
        for (iterNeurons2 = neurons.begin(); iterNeurons2 != neurons.end(); ++iterNeurons2)
        {
            if (iterNeurons1->first != iterNeurons2->first)
            {
                std::map<int, std::vector<double> > pattern2 = iterNeurons2->second;

                this->logger->log->debug("Comparing %d -- %d \n", iterNeurons1->first, iterNeurons2->first);
                currSimilarity = computeResponse(pattern1, pattern2);

                if (currSimilarity < minDissimilarity)
                {
                    minDissimilarity = currSimilarity;
                    minSimilarLabels.first = iterNeurons1->first;
                    minSimilarLabels.second = iterNeurons2->first;
                }
            }
        }
    }

    return minSimilarLabels;
}

std::pair<int, int> CHPNeuralNetwork::findMaxDissimilarity(std::vector<int> neurons)
{

    // cat , patid, cat' paid', feat
    //std::map<int, std::map<int, std::map<int,  std::map<int, std::vector<double> > > > > neurons

    this->logger->log->debug("CHPNeuralNetwork::findMaxDissimilarity find...\n");

    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPat1, iterPat2;

    //label, label
    std::pair<int, int> minSimilarLabels;
    minSimilarLabels.first = 0;
    minSimilarLabels.second = 0;
    double minDissimilarity = std::numeric_limits<double>::infinity();
    double currSimilarity = std::numeric_limits<double>::infinity();

    for (unsigned int iterNeurons1 = 0; iterNeurons1 < neurons.size(); ++iterNeurons1)
    {
        for (unsigned int iterNeurons2 = 0; iterNeurons2 < neurons.size(); ++iterNeurons2)
        {
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
    }

    return minSimilarLabels;
}

double CHPNeuralNetwork::getSimilarity(int label1, int label2)
{

    double currSimilarity = 0;

    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPat1, iterPat2;

    std::map<int, std::map<int, std::map<int, std::vector<double> > > > p1 = this->hTrainModel[label1];
    std::map<int, std::map<int, std::map<int, std::vector<double> > > > p2 = this->hTrainModel[label2];

    int numComparison = 0;
    for (iterPat1 = p1.begin(); iterPat1 != p1.end(); ++iterPat1)
    {
        for (iterPat2 = p2.begin(); iterPat2 != p2.end(); ++iterPat2)
        {
            currSimilarity += this->computeResponse(iterPat1->second[label1], iterPat2->second[label1]);
            ++numComparison;
        }
    }

    return currSimilarity / (double) numComparison;
}

std::map<int, std::vector<int> > CHPNeuralNetwork::findSimilarity(std::vector<int> &neurons, int label1, int label2)
{

    //label of all neurons, corrsponding label;
    std::map<int, std::vector<int> > maxSimilarityLabel;
    // cat , patid, cat' paid', feat
    //std::map<int, std::map<int, std::map<int,  std::map<int, std::vector<double> > > > > neurons


    //  std::map<int, std::map<int, std::map<int, std::vector<double> > > >
    //          patternLabel1 = this->hTrainModel[label1];
    //  std::map<int, std::map<int, std::map<int, std::vector<double> > > >
    //          patternLabel2 = this->hTrainModel[label2];

    double currSimilarity1;
    double currSimilarity2;

    for (unsigned int iterNeurons = 0; iterNeurons != neurons.size(); ++iterNeurons)
    {

        currSimilarity1 = this->getSimilarity(label1, neurons[iterNeurons]);

        //  std::cout<<"1  = "<<currSimilarity1<<std::endl;
        currSimilarity2 = this->getSimilarity(label2, neurons[iterNeurons]);
        //std::cout<<"2  = "<<currSimilarity2<<std::endl;

        if (currSimilarity1 > currSimilarity2)
        {
            maxSimilarityLabel[label1].push_back(neurons[iterNeurons]);
        }
        else
        {
            maxSimilarityLabel[label2].push_back(neurons[iterNeurons]);
        }
    }

    return maxSimilarityLabel;
}

void CHPNeuralNetwork::initHierarchy(float trainTestSetRatio)
{
    this->trainTestSetRatio = trainTestSetRatio;
    this->logger->log->debug("CHPNeuralNetwork::initHierachy init...(trainTestSetRatio=%lf)\n", this->trainTestSetRatio);

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
        this->logger->log->debug("CHPNeuralNetwork::initHierarchy...complete examples used as trainExamples\n");
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

                this->logger->log->info("CHPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
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
                this->logger->log->info("CHPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
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

    //this->verifyHPNN();
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

int CHPNeuralNetwork::findValue(std::vector<int> vec, int value)
{
    for (unsigned int i = 0; i < vec.size(); i++)
    {
        if (vec[i] == value)
        {
            return i;
        }
    }

    return -1;
}

void CHPNeuralNetwork::divideAndSort(std::vector<int> &neuronsToEvaluate, boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron)
{
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPatterns;
    std::map<int, std::vector<int> >::iterator iiiter;

    std::pair<int, int> maxDissimilarLabels;

    if (neuronsToEvaluate.size() <= 1)
    {
        this->logger->log->debug("CHPNeuralNetwork::divideAndSort...stop neuronsToEvaluate=%d \n", neuronsToEvaluate.size());
        //parentNeuron->leftNeuron=NULL;
        //parentNeuron->rightNeuron=NULL;
        if (neuronsToEvaluate.size() == 1)
        {

            this->logger->log->debug("CHPNeuralNetwork::divideAndSort...Single Label %s added as left to", CFileSettings::labels[neuronsToEvaluate[0]].c_str(),
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

    maxDissimilarLabels = this->findMaxDissimilarity(neuronsToEvaluate);

    this->logger->log->debug("CHPNeuralNetwork::divideAndSort...Most Dissim are %s <> %s \n", CFileSettings::labels[maxDissimilarLabels.first].c_str(),
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
        this->logger->log->debug("CHPNeuralNetwork::divideAndSort...Similar %s: \n ", CFileSettings::labels[iiiter->first].c_str());
        for (unsigned int ii = 0; ii < iiiter->second.size(); ++ii)
        {
            this->logger->log->debug("  %s\n", CFileSettings::labels[iiiter->second[ii]].c_str());
        }
    }
    std::vector<int> leftNeuronsToEvaluate = maxSimilarityLabel[maxDissimilarLabels.first];
    std::vector<int> rightNeuronsToEvaluate = maxSimilarityLabel[maxDissimilarLabels.second];

    this->logger->log->debug("CHPNeuralNetwork::divideAndSort...start left(parent=%s)...\n", CFileSettings::labels[parentNeuron->leftNeuron->neuron.label].c_str());
    this->divideAndSort(leftNeuronsToEvaluate, parentNeuron->leftNeuron);

    this->logger->log->debug("CHPNeuralNetwork::divideAndSort...start right(parent=%s)...\n", CFileSettings::labels[parentNeuron->rightNeuron->neuron.label].c_str());
    this->divideAndSort(rightNeuronsToEvaluate, parentNeuron->rightNeuron);
    //remove labels of maxDiss from neuros Toeval

    //check if neuron.size == 1 add it to the most similar parent node
}

void CHPNeuralNetwork::verifyHPNN()
{
    this->logger->log->info("CHPNeuralNetwork::verifyHPNN....\n");
    if (this->rootNeuron == NULL)
    {
        return;
    }

    if (this->hTrainModel.size() > 0)
    {
        this->logger->log->info("CHPNeuralNetwork::verifyHPNN...TrainData:\n");
        this->verifyHPNNModel(this->hTrainModel);
    }
    if (this->hTestModel.size() > 0)
    {
        this->logger->log->info("CHPNeuralNetwork::verifyHPNN...TestData:\n");
        this->verifyHPNNModel(this->hTestModel);
    }

}

SVerficationResult CHPNeuralNetwork::verifyHPNNModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > toVerifyModel)
{
    SVerficationResult vr;
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > >::iterator iterNeurons;
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPattern;

    //label, sum
    std::map<int, int> correctAll;

    //label, sum
    std::map<int, int> incorrectAll;

    std::map<int, int>::iterator iterResp;

    int correct = 0;
    int incorrect = 0;
    int total = 0;
    for (iterNeurons = toVerifyModel.begin(); iterNeurons != toVerifyModel.end(); ++iterNeurons)
    {
        std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern;
        pattern = iterNeurons->second;
        for (iterPattern = pattern.begin(); iterPattern != pattern.end(); ++iterPattern)
        {
            if (iterNeurons->first == this->hEvaluate(iterPattern->second).first)
            {
                this->logger->log->debug("CHPNeuralNetwork::verifyHPNNModel...Correct(%s)\n", CFileSettings::labels[iterNeurons->first].c_str());
                correct++;
                correctAll[iterNeurons->first]++;
            }
            else
            {
                this->logger->log->debug("CHPNeuralNetwork::verifyHPNNModel...Oh Oh(%s)\n", CFileSettings::labels[iterNeurons->first].c_str());
                incorrect++;
                incorrectAll[iterNeurons->first]++;
            }
            total++;
        }
    }

    if (total > 0)
    {
        vr.numberOfExamples = total;
        vr.totalCorrect = (double) correct / (double) total;
        vr.totalFalse = (double) incorrect / (double) total;

        this->logger->log->info("CHPNeuralNetwork::verifyHPNNModel...Total Correct: %lf Incorrect: %lf\n", vr.totalCorrect, vr.totalFalse);

        for (iterResp = correctAll.begin(); iterResp != correctAll.end(); ++iterResp)
        {

            vr.correctLabel[iterResp->first] = (double) correctAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]);

            this->logger->log->info("CHPNeuralNetwork::verify...Label %s Correct: %lf\n ", CFileSettings::labels[iterResp->first].c_str(), vr.correctLabel[iterResp->first]);
        }

        for (iterResp = incorrectAll.begin(); iterResp != incorrectAll.end(); ++iterResp)
        {
            vr.falseLabel[iterResp->first] = (double) incorrectAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]);

            this->logger->log->info("CHPNeuralNetwork::verify...Label %s Incorrect: %lf\n ", CFileSettings::labels[iterResp->first].c_str(), vr.falseLabel[iterResp->first]);

        }
    }
    else
    {
        this->logger->log->warn("CHPNeuralNetwork::verify... example set is empty!!\n");
    }

    return vr;
}

std::pair<int, double> CHPNeuralNetwork::getMaxResponse(std::vector<std::pair<int, double> > responses)
{
    std::vector<std::pair<int, double> >::iterator iter;
    std::pair<int, double> maxResponse;

    maxResponse.first = 0;
    maxResponse.second = 0;

    for (iter = responses.begin(); iter != responses.end(); ++iter)
    {

        if (iter->second >= maxResponse.second)
        {
            maxResponse.first = iter->first;
            maxResponse.second = iter->second;
        }
    }

    return maxResponse;
}

std::pair<int, double> CHPNeuralNetwork::hEvaluate(std::map<int, std::map<int, std::vector<double> > > query)
{
    std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons;
    std::map<int, std::vector<double> >::iterator iterPattern;
    this->logger->log->debug(" CHPNeuralNetwork::hEvaluate ....\n");

    //neuron, responseValue;
    //std::map<int, double> response;

    std::pair<int, double> maxResponse;
    maxResponse.first = 0;
    maxResponse.second = 0;

    std::pair<int, double> lastResponse;
    lastResponse.first = 0;
    lastResponse.second = 0;

    boost::shared_ptr<SHPNeuralNetworkNeuron> parentNeuron = this->rootNeuron;
    std::pair<int, double> responseLeft;
    std::pair<int, double> responseRight;
    std::pair<int, double> responseResult;

    std::vector < std::pair<int, double> > responseResults;
    do
    {
        if (parentNeuron->leftNeuron == NULL && parentNeuron->rightNeuron == NULL)
        {
            break;
        }
        std::map<int, std::vector<double> > modelPattern;
        std::map<int, std::vector<double> > queryPattern;

        responseLeft.first = 0;
        responseRight.first = 0;
        responseLeft.second = 0;
        responseRight.second = 0;

        if (parentNeuron->leftNeuron != NULL)
        {
            responseLeft.first = parentNeuron->leftNeuron->neuron.label;
            modelPattern = parentNeuron->leftNeuron->neuron.pattern;
            queryPattern = query[parentNeuron->leftNeuron->neuron.label];
            responseLeft.second = computeResponse(queryPattern, modelPattern);
        }

        if (parentNeuron->rightNeuron != NULL)
        {
            responseRight.first = parentNeuron->rightNeuron->neuron.label;
            modelPattern = parentNeuron->rightNeuron->neuron.pattern;
            queryPattern = query[parentNeuron->rightNeuron->neuron.label];
            responseRight.second = computeResponse(queryPattern, modelPattern);
        }

        if (responseLeft.second > responseRight.second)
        {
            parentNeuron = parentNeuron->leftNeuron;
            responseResult = responseLeft;
            this->logger->log->debug("CHPNeuralNetwork::evaluate...Branching left\n");
        }
        else
        {
            parentNeuron = parentNeuron->rightNeuron;
            responseResult = responseRight;
            this->logger->log->debug("CHPNeuralNetwork::hEvaluate...Branching right\n");
        }

        //if a continously decreasing error is wanted
        if (responseResult.second < lastResponse.second)
        {
            this->logger->log->debug("CHPNeuralNetwork::hEvaluate...STOP! Eror increased!\n");
            return lastResponse;
        }

        //if all results are stored and later the highest probabable is selected
        //responseResults.push_back(responseResult);

        this->logger->log->debug("CHPNeuralNetwork::hEvaluate...Response to %s = %lf", CFileSettings::labels[parentNeuron->neuron.label].c_str(), responseResult.second);

        lastResponse = responseResult;
    }
    while (parentNeuron->leftNeuron != NULL || parentNeuron->rightNeuron != NULL);

    //if all results are stored and later the highest probabable is selected
    //lastResponse = this->getMaxResponse(responseResults);

    return lastResponse;
}
