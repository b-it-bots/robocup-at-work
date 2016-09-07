/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include "neural_gas/neural_gas_edge.h"

int CNeuralGasEdge::next_id = 0;

CNeuralGasEdge::CNeuralGasEdge()
{
    this->id = CNeuralGasEdge::next_id++;
    this->age = 0;
}

bool CNeuralGasEdge::isAged()
{
    return false;
}

CNeuralGasNode* CNeuralGasEdge::getNeighbor(CNeuralGasNode* node)
{
    //std::cout<<"Edge: getNeighbor: 0="<< this->nodes[0]->id<< " 1="<<this->nodes[1]->id<<std::endl;

    if (node->id == this->nodes[0]->id)
        return this->nodes[1];
    else
        return this->nodes[0];
}

void CNeuralGasEdge::reset()
{
    CNeuralGasEdge::next_id = 0;
    this->id = 0;
    this->age = 0;
    nodes.clear();

    //std::cout<<"\nCNeuralGasEdge::reset----------> "<<CNeuralGasEdge::next_id<<"\n";
}

