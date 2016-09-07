/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include <limits>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include <cmath>
#include <ctime>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "neural_gas/neural_gas_classifier.h"
#include "neural_gas/mds.h"
#include "neural_gas/neural_gas.h"
#include "neural_gas/tool_box_ng.h"

CNeuralGasClassifier::CNeuralGasClassifier()
{
    cvInitFont(&this->font, CV_FONT_VECTOR0, 0.5, 0.5, 0.0, 1);
    ;
}

CNeuralGasClassifier::CNeuralGasClassifier(int positionVectorSize, int positionValueRange, float nearestNeighborWeight, float nNeighborWeight, float maxAddNodeError, int maxEdgeAge,
        int intervalAddNode, float errorNewNodeDecrease, float errorGeneralDecrease, int stoppingCriterionSelection, float stoppingCriterionParam, int distanceMeasureType, int initHitStrength,
        double minHitDeleteNodeThreshold)
{

    this->positionVectorSize = positionVectorSize;
    this->positionValueRange = positionValueRange;
    this->nearestNeighborWeight = nearestNeighborWeight;
    this->nNeighborWeight = nNeighborWeight;
    this->maxAddNodeError = maxAddNodeError;
    this->maxEdgeAge = maxEdgeAge;
    this->intervalAddNode = intervalAddNode;
    this->errorNewNodeDecrease = errorNewNodeDecrease;
    this->errorGeneralDecrease = errorGeneralDecrease;
    this->stoppingCriterionSelection = stoppingCriterionSelection;
    this->stoppingCriterionParam = stoppingCriterionParam;
    this->distanceMeasureType = distanceMeasureType;
    this->initHitStrength = initHitStrength;
    this->minHitDeleteNodeThreshold = minHitDeleteNodeThreshold;

    this->neuralGas = new CNeuralGas(positionVectorSize, positionValueRange, nearestNeighborWeight, nNeighborWeight, maxAddNodeError, maxEdgeAge, intervalAddNode, errorNewNodeDecrease,
                                     errorGeneralDecrease, stoppingCriterionSelection, stoppingCriterionParam, distanceMeasureType, initHitStrength, minHitDeleteNodeThreshold);
    cvInitFont(&this->font, CV_FONT_VECTOR0, 0.5, 0.5, 0.0, 1);
}

void CNeuralGasClassifier::reset()
{

    this->neuralGas->reset();
    delete neuralGas;

    this->neuralGas = new CNeuralGas(positionVectorSize, positionValueRange, nearestNeighborWeight, nNeighborWeight, maxAddNodeError, maxEdgeAge, intervalAddNode, errorNewNodeDecrease,
                                     errorGeneralDecrease, stoppingCriterionSelection, stoppingCriterionParam, distanceMeasureType, initHitStrength, minHitDeleteNodeThreshold);
}

double CNeuralGasClassifier::randDouble(double low, double high)
{
    double temp;

    /* swap low & high around if the user makes no sense */
    if (low > high)
    {
        temp = low;
        low = high;
        high = temp;
    }

    /* calculate the random number & return it */
    temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0)) * (high - low) + low;
    return temp;
}

double CNeuralGasClassifier::train(std::vector<std::vector<double> > trainData, int maxNumSignals, bool display, int offSet, double maxDistanceConsistency, bool normalizeToOne, float addNoise)
{
    double meanSqaredErrorToModel = 0;
    unsigned int startIndexTraining = 0;

    //std::cout<<trainData.size()<<"ng <- \n";
    assert(trainData.size() > 0);

    if (display)
    {
        this->displayOffset = offSet;
        cvNamedWindow("GAS", 1);
    }

    std::vector<CNeuralGasNode> inputSignals;

    if (this->neuralGas->getNumSignalInputs() == 0)
    {
        ///std::cout << "Start training..." << std::endl;
        CNeuralGasNode* initInputSignal1 = new CNeuralGasNode();
        //  std::cout<< "First init (" << initInputSignal1->id << ")" << std::endl;
        initInputSignal1->setPosition(trainData[0]);
        initInputSignal1->setLabel(65);
        //  std::cout << "First init" << std::endl;
        this->neuralGas->addNeuron(initInputSignal1);

        CNeuralGasNode* initInputSignal2 = new CNeuralGasNode();
        //  std::cout << "Second init (" << initInputSignal2->id << ")"
        //  << std::endl;
        initInputSignal2->setPosition(trainData[1]);
        initInputSignal2->setLabel(65);
        //      std::cout << "Second init" << std::endl;
        this->neuralGas->addNeuron(initInputSignal2);

        startIndexTraining = 2;
    }
    else
    {
        ;//std::cout << "additional Training..." << std::endl;
    }
    int max = maxNumSignals;
    if (maxNumSignals == -1 || maxNumSignals > trainData.size())
        max = trainData.size();
    std::vector<bool> used;
    used.resize(max);

    //boost::threadpool::pool threadPoolUpdateNeuralGas(2);
    for (int iter = startIndexTraining; iter < max; iter++)
    {

        CNeuralGasNode inputSignal = CNeuralGasNode(false);
        int random = iter;
        do
        {
            random = (rand() % max);
        }
        while (used[random] == true);

        used[random] = true;

        //Add noise
        if (addNoise > 0)
        {
            for (int dimTrainData = 0; dimTrainData < trainData[random].size(); ++dimTrainData)
            {
                trainData[random][dimTrainData] += randDouble(-(trainData[random][dimTrainData] * addNoise), trainData[random][dimTrainData] * addNoise);
            }
        }

        inputSignal.setPosition(trainData[random]);
        inputSignal.setLabel(65);

        this->neuralGas->updateNeuralGas(&inputSignal);
        //this->neuralGas->updateNeuralGasT(&inputSignal, this->neuralGas);
        //threadPoolUpdateNeuralGas.schedule(boost::bind(this->neuralGas->updateNeuralGasT,&inputSignal, this->neuralGas));


        inputSignals.push_back(inputSignal);

        if (display)
            updateDisplay(inputSignals);
    }
    //threadPoolUpdateNeuralGas.wait();


    if (this->neuralGas->getNumberNodes() > 0)
    {
        if (maxDistanceConsistency != -1)
        {
            meanSqaredErrorToModel = this->neuralGas->checkModelConsistency(inputSignals, maxDistanceConsistency);
        }
        //
        //this->neuralGas->checkTriangulation();
        //}


        if (normalizeToOne)
        {
            //this normalizes the gas that the most distance neuron is 1 far
            this->neuralGas->normalizeNeuralGas();
        }

        if (display)
        {
            updateDisplay(inputSignals);
        }
    }

    return meanSqaredErrorToModel;
}

double CNeuralGasClassifier::train(std::vector<std::vector<double> > trainData, std::vector<int> labels, int maxNumSignals, bool display, int offSet, double maxDistanceConsistency,
                                   bool normalizeToOne, float addNoise)
{
    double meanSqaredErrorToModel = 0;
    unsigned int startIndexTraining = 0;

    //std::cout<<trainData.size()<<"ng <- \n";
    assert(trainData.size() > 0);
    assert(trainData.size() == labels.size());

    if (display)
    {
        this->displayOffset = offSet;
        cvNamedWindow("GAS", 1);
    }

    std::vector<CNeuralGasNode> inputSignals;

    if (this->neuralGas->getNumSignalInputs() == 0)
    {
        ///std::cout << "Start training..." << std::endl;
        CNeuralGasNode* initInputSignal1 = new CNeuralGasNode();
        //  std::cout<< "First init (" << initInputSignal1->id << ")" << std::endl;
        initInputSignal1->setPosition(trainData[0]);
        initInputSignal1->setLabel(labels[0]);
        //  std::cout << "First init" << std::endl;
        this->neuralGas->addNeuron(initInputSignal1);

        CNeuralGasNode* initInputSignal2 = new CNeuralGasNode();
        //  std::cout << "Second init (" << initInputSignal2->id << ")"
        //  << std::endl;
        initInputSignal2->setPosition(trainData[1]);
        initInputSignal2->setLabel(labels[1]);
        //      std::cout << "Second init" << std::endl;
        this->neuralGas->addNeuron(initInputSignal2);

        startIndexTraining = 2;
    }
    else
    {
        ;//std::cout << "additional Training..." << std::endl;
    }
    int max = maxNumSignals;
    if (maxNumSignals == -1 || maxNumSignals > trainData.size())
        max = trainData.size();
    std::vector<bool> used;
    used.resize(max);

    //boost::threadpool::pool threadPoolUpdateNeuralGas(2);
    for (int iter = startIndexTraining; iter < max; iter++)
    {

        CNeuralGasNode inputSignal = CNeuralGasNode(false);
        int random = iter;
        do
        {
            random = (rand() % max);
        }
        while (used[random] == true);

        used[random] = true;

        //Add noise
        if (addNoise > 0)
        {
            for (int dimTrainData = 0; dimTrainData < trainData[random].size(); ++dimTrainData)
            {
                trainData[random][dimTrainData] += randDouble(-(trainData[random][dimTrainData] * addNoise), trainData[random][dimTrainData] * addNoise);
            }
        }

        inputSignal.setPosition(trainData[random]);
        //std::cout<<"CNeuralGasNode::setLabel................set to"<<labels[random]<<std::endl;
        inputSignal.setLabel(labels[random]);

        this->neuralGas->updateNeuralGas(&inputSignal);
        //this->neuralGas->updateNeuralGasT(&inputSignal, this->neuralGas);
        //threadPoolUpdateNeuralGas.schedule(boost::bind(this->neuralGas->updateNeuralGasT,&inputSignal, this->neuralGas));


        inputSignals.push_back(inputSignal);

        if (display)
            updateDisplay(inputSignals);
    }
    //threadPoolUpdateNeuralGas.wait();


    if (this->neuralGas->getNumberNodes() > 0)
    {
        if (maxDistanceConsistency != -1)
        {
            meanSqaredErrorToModel = this->neuralGas->checkModelConsistency(inputSignals, maxDistanceConsistency);
        }
        //
        //this->neuralGas->checkTriangulation();
        //}


        if (normalizeToOne)
        {
            //this normalizes the gas that the most distance neuron is 1 far
            this->neuralGas->normalizeNeuralGas();
        }

        if (display)
        {
            updateDisplay(inputSignals);
        }
    }

    return meanSqaredErrorToModel;
}

void CNeuralGasClassifier::verifyModel(std::vector<std::vector<double> > testVectors, std::vector<int> labels)
{
    assert(this->neuralGas->getNeuralGasNodes().size() > 0);

    int error = 0;
    for (unsigned int i = 0; i < testVectors.size(); ++i)
    {
        //std::cout<<"\nLabel to test "<<labels[i]<<":";
        if (labels[i] != this->neuralGas->measureKNearestNodeDistance(testVectors[i], 1, 1))
        {
            error++;
        }
    }

    std::cout << "Error = " << (double) error / (double) testVectors.size() << "\n";
    //exit(1);
}

void CNeuralGasClassifier::verifyModel2(std::vector<std::vector<double> > testVectors, std::vector<int> labels)
{
    assert(this->neuralGas->getNeuralGasNodes().size() > 0);

    double error = 0;
    std::map<int, double> meanDist;
    std::map<int, int> counts;

    for (unsigned int i = 0; i < testVectors.size(); ++i)
    {
        std::cout << "\nLabel to test " << labels[i] << ":";

        meanDist[labels[i]] += this->neuralGas->measureDistance(testVectors[i], 1);
        counts[labels[i]]++;
    }

    std::map<int, double>::iterator iter;

    for (iter = meanDist.begin(); iter != meanDist.end(); ++iter)
    {
        std::cout << "Error = " << iter->first << " = " << (double) meanDist[iter->first] / (double) counts[iter->first] << "\n";
    }
    //exit(1);
}

void CNeuralGasClassifier::saveNeuralGas(std::string filename)
{
    this->neuralGas->saveNeuralGas(filename);
    ///his->neuralGas->createJohnsonAllPairShortestPathGraph(filename);
}

apsp_graph CNeuralGasClassifier::computeJohnsonAPSP(std::string filename)
{
    return this->neuralGas->createJohnsonAllPairShortestPathGraph(filename);
}

apsp_graph CNeuralGasClassifier::computeJohnsonAPSP()
{
    return this->neuralGas->createJohnsonAllPairShortestPathGraph();
}

double CNeuralGasClassifier::query(std::vector<std::vector<double> > query, int type)
{
    return this->neuralGas->measureDistance(query, type);
}

double CNeuralGasClassifier::query(std::vector<double> query, int type)
{
    return this->neuralGas->measureDistance(query, type);
}

CNeuralGas* CNeuralGasClassifier::getNeuralGas()
{
    return this->neuralGas;
}

void CNeuralGasClassifier::updateDisplay(std::vector<CNeuralGasNode> inputSignals)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    float factor = 150; //300 PNN;// 150 OCS; //50;
    CvScalar yellow;
    CvScalar pink;
    CvScalar red;
    CvScalar white;

    CvScalar colors[8];
    colors[0] = cvScalar(100, 100, 100);
    colors[1] = cvScalar(255, 0, 0);
    colors[2] = cvScalar(0, 255, 0);
    colors[3] = cvScalar(255, 255, 0);
    colors[4] = cvScalar(0, 0, 255);
    colors[5] = cvScalar(255, 0, 255);
    colors[6] = cvScalar(0, 255, 255);
    colors[7] = cvScalar(255, 255, 255);

    std::cout << "Display update" << std::endl;
    std::map<int, CNeuralGasNode*> gas = this->neuralGas->getNeuralGasNodes();
    //cv::Mat space = cv::Mat(cv::Size(this->neuralGas->getPositionValueRange(), this->neuralGas->getPositionValueRange()), CV_8UC3);

    IplImage *space = cvCreateImage(cvSize(this->neuralGas->getPositionValueRange(), this->neuralGas->getPositionValueRange()), 8, 3);
    cvZero(space);

    //************************GET XY
    unsigned int numDim = 2;
    std::vector<std::vector<double> > siftData;

    for (nodeIter it = gas.begin(); it != gas.end(); it++)
    {
        siftData.push_back(it->second->position);
    }

    CMds mds = CMds(siftData, numDim);

    mds.init();
    for (nodeIter it = gas.begin(); it != gas.end(); it++)
    {
        mds.reduceDim(it->second->position, it->second->visualXYPosition);
    }

    for (int i = 0; i < inputSignals.size(); i++)
    {
        mds.reduceDim(inputSignals[i].position, inputSignals[i].visualXYPosition);
    }

    /////////////////////////////////////////
    //space.zeros(space.size(), CV_8UC3);
    //cvPutText(space, boost::lexical_cast<std::string>(static_cast<int> (this->neuralGas->getNumSignalInputs())), cvPoint(20, 25), 4, 1.0f, cvScalar(0, 255, 0));
    //cvPutText(space, boost::lexical_cast<std::string>(static_cast<int> (gas.size())), cvPoint(20, 50), 4, 1.0f, cvScalar(0, 255, 0));
    //cvPutText(space, boost::lexical_cast<std::string>(static_cast<double> (this->neuralGas->getDecayAdaptation())), cvPoint(20, 75), 4, 1.0f, cvScalar(0, 255, 0));
    //cvPutText(space, boost::lexical_cast<std::string>(static_cast<double> (this->neuralGas->getNearestNeighborWeight())), cvPoint(20, 100), 4, 1.0f, cvScalar(0, 255, 0));

    cvPutText(space, boost::lexical_cast<std::string>(static_cast<int>(this->neuralGas->getNumSignalInputs())).c_str(), cvPoint(20, 25), &this->font, cvScalar(0, 255, 0));
    cvPutText(space, boost::lexical_cast<std::string>(static_cast<int>(gas.size())).c_str(), cvPoint(20, 50), &this->font, cvScalar(0, 255, 0));
    cvPutText(space, boost::lexical_cast<std::string>(static_cast<double>(this->neuralGas->getDecayAdaptation())).c_str(), cvPoint(20, 75), &this->font, cvScalar(0, 255, 0));
    cvPutText(space, boost::lexical_cast<std::string>(static_cast<double>(this->neuralGas->getNearestNeighborWeight())).c_str(), cvPoint(20, 100), &this->font, cvScalar(0, 255, 0));

    for (unsigned int u = 0; u < 8; ++u)
    {
        cvPutText(space, boost::lexical_cast<std::string>(static_cast<int>(u)).c_str(), cvPoint(u * 10, 125), &this->font, colors[u]);
    }

    std::cout << "input signal (" << inputSignals.back().position[0] << "," << inputSignals.back().position[1] << ")" << std::endl;

    //66 157
    /*for (int k = 0; k < inputSignals.size(); k++)
     {
     // if (inputSignals[k].getLabel() == 65)
     // {
     cv::circle(space, cv::Point(inputSignals[k].visualXYPosition[0] * factor + this->displayOffset, inputSignals[k].visualXYPosition[1] * factor + this->displayOffset), 1, yellow, 1);
     // std::cout << " 0 ";
     }
     if (inputSignals[k].getLabel() == 66)
     {
     cv::circle(space, cv::Point(inputSignals[k].visualXYPosition[0] * factor + this->displayOffset, inputSignals[k].visualXYPosition[1] * factor + this->displayOffset), 1, pink, 1);

     // std::cout << " 1 ";
     }
     if (inputSignals[k].getLabel() == 67)
     {
     cv::circle(space, cv::Point(inputSignals[k].visualXYPosition[0] * factor + this->displayOffset, inputSignals[k].visualXYPosition[1] * factor + this->displayOffset), 1, red, 1);
     // std::cout << " 2 ";
     }
     if (inputSignals[k].getLabel() == 68)
     {
     cv::circle(space, cv::Point(inputSignals[k].visualXYPosition[0] * factor + this->displayOffset, inputSignals[k].visualXYPosition[1] * factor + this->displayOffset), 1, white, 1);
     // std::cout << " 3 ";
     }
     }*/

    //  cv::imshow("GAS", space);
    //  cv::waitKey(5);

    for (nodeIter it = gas.begin(); it != gas.end(); it++)
    {
        std::cout << "Node " << it->second->id << "(" << it->second->visualXYPosition[0] * factor << "," << it->second->visualXYPosition[1] * factor << "," << it->second->getLabel() << ") "
                  << std::endl << " Error:" << it->second->error << std::endl;
        cvCircle(space, cvPoint(it->second->visualXYPosition[0] * factor + this->displayOffset, it->second->visualXYPosition[1] * factor + this->displayOffset), 5, colors[2],
                 CV_FILLED); //colors[it->second->getLabel()]
        //  cvPutText(space, boost::lexical_cast<std::string>(static_cast<int> (it->second->id)).c_str(),
        //          cvPoint(it->second->visualXYPosition[0] * factor + this->displayOffset, it->second->visualXYPosition[1] * factor + this->displayOffset), &this->font, cvScalar(255, 255, 255));
        //  cvPutText(
        //          space,
        //          std::string(
        //                  std::string("(") + std::string(boost::lexical_cast<std::string>(static_cast<int> (((float) it->second->hits / (float) this->neuralGas->getNumSignalInputs()) * 100)))
        //                          + std::string(")")).c_str(),
        //          cvPoint(it->second->visualXYPosition[0] * factor + this->displayOffset + 15, it->second->visualXYPosition[1] * factor + this->displayOffset), &this->font, cvScalar(255, 255, 255));

        std::cout << " EdgeSize " << it->second->edgeId.size() << " -- " << this->neuralGas->edges.size() << std::endl;
        for (int j = 0; j < it->second->edgeId.size(); j++)
        {

            CNeuralGasEdge* edge = this->neuralGas->edges[it->second->edgeId[j]];
            cvLine(space, cvPoint(edge->nodes[0]->visualXYPosition[0] * factor + this->displayOffset, edge->nodes[0]->visualXYPosition[1] * factor + this->displayOffset),
                   cv::Point(edge->nodes[1]->visualXYPosition[0] * factor + this->displayOffset, edge->nodes[1]->visualXYPosition[1] * factor + this->displayOffset), cvScalar(100, 100, 100), 1);
            std::cout << " Edge age (" << edge->id << "): " << edge->age << std::endl;
        }
    }
    cvShowImage("GAS", space);
    cvWaitKey(1); //10
    cvReleaseImage(&space);
    //std::cout << "update done" << std::endl;
}

CNeuralGasClassifier::~CNeuralGasClassifier()
{
    delete this->neuralGas;
}
