/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CNEURALGASNODE_H
#define CNEURALGASNODE_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <cmath>
#include <map>

class CNeuralGasNode
{
private:
    static int next_id;
public:

    int id;
    unsigned int age;
    unsigned int hits;
    std::vector<int> edgeId;
    std::vector<double> position;
    std::vector<double> visualXYPosition;
    double error;

    //label,counts
    std::map<int, int> label; //distribution divide by labelCounts!
    int labelCounts;

    float epsB;
    std::vector<CNeuralGasNode*> neighbors;

    bool updated;
    unsigned int iteration;
    double activity;

    CNeuralGasNode(bool isNode = true);

    void moveToNode(CNeuralGasNode &neighbour, double distance, float weight);

    int getId()
    {
        return id;
    };
    void setLabel(int label);
    int getLabel();
    std::map<int, int> getLabelCounts();
    void setPosition(std::vector<double> position);
    void adaptID(int id);
    void deleteEdge(int id);
    void reset();
    ~CNeuralGasNode();
};

#endif
