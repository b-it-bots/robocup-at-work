/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef _CNGC_H__
#define _CNGC_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "neural_gas/neural_gas.h"
#include "johnson_apsp/apsp_graph.hpp"

class CNeuralGasClassifier
{
private:
    CNeuralGas *neuralGas;
    int displayOffset;
    int positionVectorSize;
    int positionValueRange;
    float nearestNeighborWeight; //move the nearest node to target
    float nNeighborWeight; //move other neigbors to target
    float maxAddNodeError;
    int maxEdgeAge;
    int intervalAddNode;
    float errorNewNodeDecrease;  // nearedt node : factor to decrease error
    float errorGeneralDecrease; // general factor to decrease error of all nodes
    int stoppingCriterionSelection;
    float stoppingCriterionParam;
    int distanceMeasureType;
    int initHitStrength;
    double minHitDeleteNodeThreshold;
    std::vector<int> labels;
    CvFont font;
public:
    CNeuralGasClassifier();
    CNeuralGasClassifier(int positionVectorSize, int positionValueRange,
                         float nearestNeighborWeight, float nNeighborWeight,
                         float maxAddNodeError, int maxEdgeAge, int intervalAddNode,
                         float errorNewNodeDecrease, float errorGeneralDecrease,
                         int stoppingCriterionSelection, float stoppingCriterionParam,
                         int distanceMeasureType, int initHitStrength,
                         double minHitDeleteNodeThreshold);
    double train(std::vector<std::vector<double> > trainData, int maxNumSignals,
                 bool display, int offSet, double maxDistanceConsistency, bool normalizeToOne = false, float addNoise = -1.0f);
    double train(std::vector<std::vector<double> > trainData, std::vector<int> labels, int maxNumSignals,
                 bool display, int offSet, double maxDistanceConsistency, bool normalizeToOne = false, float addNoise = -1.0f);

    void verifyModel(std::vector<std::vector<double> > testVectors, std::vector<int> labels);
    void verifyModel2(std::vector<std::vector<double> > testVectors, std::vector<int> labels);
    void reset();
    double query(std::vector<std::vector<double> > query, int type);
    double query(std::vector<double> query, int type);

    void saveNeuralGas(std::string filename);
    apsp_graph computeJohnsonAPSP(std::string filename);
    apsp_graph computeJohnsonAPSP();
    void updateDisplay(std::vector<CNeuralGasNode> inputSignals);
    double randDouble(double low, double high);
    CNeuralGas* getNeuralGas();
    ~CNeuralGasClassifier();

};

#endif
