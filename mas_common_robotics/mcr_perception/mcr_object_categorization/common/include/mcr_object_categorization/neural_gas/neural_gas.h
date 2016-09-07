/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CNEURALGAS_H
#define CNEURALGAS_H


#include <iostream>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <cmath>
#include <map>

#include "neural_gas/tool_box_ng.h"
#include "neural_gas/neural_gas_node.h"
#include "neural_gas/neural_gas_edge.h"
#include "johnson_apsp/apsp_graph.hpp"

class CNeuralGas
{
private:
    //std::vector<CNeuralGasNode*> nodes;
    std::map<int, CNeuralGasNode*> nodes;
    CToolBoxNG toolBox;
    int positionVectorSize;
    int positionValueRange;
    int maxEdgeAge;
    int intervalAddNode; // -1 or maxAddNodeError -1

    float nearestNeighborWeight;
    float nNeighborWeight;
    float maxAddNodeError; //-1 or intervalAddNode -1
    float errorNewNodeDecrease;
    float errorGeneralDecrease;
    float initHitStrength;
    int numSignalInputs;
    int stoppingCriterionSelection;
    float stoppingCriterionParam;
    int distanceMeasureType;
    double minHitDeleteNodeThreshold;
    double decayAdaptation;

public:
    CNeuralGas() {};
    std::map<int, CNeuralGasEdge*> edges; // id of edge, edge
    CNeuralGas(int positionVectorSize, int positionValueRange,
               float nearestNeighborWeight, float nNeighborWeight,
               float maxAddNodeError, int maxEdgeAge, int intervalAddNode,
               float errorNewNodeDecrease, float errorGeneralDecrease,
               int stoppingCriterionSelection, float stoppingCriterionParam,
               int distanceMeasureType, int initHitStrength, double minHitDeleteNodeThreshold);
    void initRandom(int nNodes);
    void addNeuron(CNeuralGasNode* inputSignal);

    void updateNodePositions(CNeuralGasNode* inputSignal);
    void updateNeuralGas(CNeuralGasNode* inputSignal);
    static void updateNeuralGasT(CNeuralGasNode* inputSignal, CNeuralGas *ng);
    void updateNodeNumber();
    void updateEdges();
    void updateNodeError();
    void updateDecayLearnining();

    void createConnection(CNeuralGasNode &a, CNeuralGasNode &b);
    std::vector<double> doRandomPosition();
    void
    getNearestNeighbor(CNeuralGasNode &a, int &neighbour, double &distance);
    void getNearestNeighbor(CNeuralGasNode &a, int &neighbour,
                            double &distance, int &secondNearestNeighbor);

    int getMaxErrorNode();

    void deleteEdge(CNeuralGasEdge *edge);
    void deleteNode(CNeuralGasNode *node);
    double measureDistance(std::vector<std::vector<double> > query, int type);
    double measureDistance(std::vector<double> query, int type) ;
    int measureKNearestNodeDistance(std::vector<double> query, int type, int k);
    double measureKNearestNodeDistanceError(std::vector<double> query, int type, int k);

    std::map<int, CNeuralGasNode*>& getNeuralGasNodes()
    {
        return this->nodes;
    }
    double getDistance(std::vector<double> a, std::vector<double> b);
    double getEuclideanDistance(std::vector<double> a, std::vector<double> b);
    double getCosineDistance(std::vector<double> a, std::vector<double> b);
    bool stoppingCriterion(float stoppingCriterionSelection);
    double checkModelConsistency(std::vector<CNeuralGasNode> &inputNodes, double maxDistance);
    double getDistanceBetweenNodes(unsigned int nodeId1, unsigned int nodeId2);

    void saveNeuralGas(std::string fileName);

    int getPositionVectorSize()
    {
        return this->positionVectorSize;
    }
    int getNumberNodes()
    {
        return this->nodes.size();
    }
    int getNumberEdges()
    {
        return this->edges.size();
    }
    int getPositionValueRange()
    {
        return this->positionValueRange;
    }
    int getMaxEdgeAge()
    {
        return this->maxEdgeAge;
    }
    int getIntervalAddNode()
    {
        return this->intervalAddNode;
    }
    float getNearestNeighborWeight()
    {
        return this->nearestNeighborWeight;
    }
    float getNNeighborWeight()
    {
        return this->nNeighborWeight;
    }
    float getMaxAddNodeError()
    {
        return this->maxAddNodeError;
    }
    float getErrorNewNodeDecrease()
    {
        return this->errorNewNodeDecrease;
    }
    float getErrorGeneralDecrease()
    {
        return this->errorGeneralDecrease;
    }
    int getNumSignalInputs()
    {
        return this->numSignalInputs;
    }
    int getStoppingCriterionSelection()
    {
        return this->stoppingCriterionSelection;
    }
    float getStoppingCriterionParam()
    {
        return this->stoppingCriterionParam;
    }
    double getDecayAdaptation()
    {
        return this->decayAdaptation;
    }

    //get returns the distance from nearest input signal  the node given by the node id.
    double getNearestNodeDistance(std::vector<CNeuralGasNode> &inputSignals, int &nodeId, int &nearestInputId);

    void checkTriangulation();

    apsp_graph createJohnsonAllPairShortestPathGraph(std::string fileName);
    apsp_graph createJohnsonAllPairShortestPathGraph();

    //this normalizes the gas that the most distance neuron is 1 far
    void normalizeNeuralGas();
    std::vector<double> computeCentroid();
    void reset();

    ~CNeuralGas();
};

#endif
