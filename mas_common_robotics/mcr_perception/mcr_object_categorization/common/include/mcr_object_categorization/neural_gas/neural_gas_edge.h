/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef CNEURALGASEDGE_H
#define CNEURALGASEDGE_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <cmath>

#include "neural_gas/neural_gas_node.h"

class CNeuralGasEdge
{
private:
    static int next_id;

public:
    CNeuralGasEdge();
    int id;
    int age;
    std::vector<CNeuralGasNode*> nodes;
    bool isAged();
    CNeuralGasNode* getNeighbor(CNeuralGasNode* node);
    void reset();
};

#endif
