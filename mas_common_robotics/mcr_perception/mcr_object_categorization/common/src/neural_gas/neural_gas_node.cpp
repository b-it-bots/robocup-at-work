/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */


#include <limits>

#include "neural_gas/neural_gas_node.h"

int CNeuralGasNode::next_id = 0;

CNeuralGasNode::CNeuralGasNode(bool isNode)
{
    //if (isNode)
    this->id = CNeuralGasNode::next_id++;
    //  else
    //      this->id = -1;

    this->age = 0;
    this->error = 0.0f;
    //this->label = 0;
    this->labelCounts = 0;
    this->hits = 1;
    this->activity = 0.0f;

}

void CNeuralGasNode::reset()
{
    CNeuralGasNode::next_id = 0;
    this->id = 0;
    this->age = 0;
    this->error = 0.0f;
    //this->label = 0;
    this->labelCounts = 0;
    this->hits = 1;
    this->activity = 0.0f;
    this->edgeId.clear();
    this->position.clear();
    this->neighbors.clear();
    this->visualXYPosition.clear();

    // std::cout<<"\nNeuralGasNode::reset----- -----> "<<CNeuralGasNode::next_id<<"\n";
}

void CNeuralGasNode::setPosition(std::vector<double> position)
{
    this->position = position;
}

void CNeuralGasNode::setLabel(int label)
{
    this->label[label]++;
    this->labelCounts++;
}

int CNeuralGasNode::getLabel()
{

    std::map<int, int>::iterator iterlabels;
    int highCount = 0;
    int highCountLabel = 0;
    for (iterlabels = this->label.begin(); iterlabels != this->label.end(); ++iterlabels)
    {
        if (iterlabels->second >= highCount)
        {
            highCountLabel = iterlabels->first;
            highCount = iterlabels->second;
        }
    }

    return highCountLabel;
}

std::map<int, int> CNeuralGasNode::getLabelCounts()
{
    return this->label;
}

void CNeuralGasNode::moveToNode(CNeuralGasNode &neighbour, double distance, float weight)
{

    //std::cout << "Distance " << distance << std::endl;
    if (distance > std::numeric_limits<double>::epsilon()) // the neighbor and node are at the same position, so dont move
    {

        weight = weight * distance;

        std::vector<float> result;

        result.resize(this->position.size());

        for (unsigned int i = 0; i < this->position.size(); i++)
        {
            result[i] = weight * ((float) neighbour.position[i] - (float) this->position[i]) / (float) distance;
            this->position[i] += result[i];
        }

    }

}

void CNeuralGasNode::adaptID(int id)
{
    this->id = id;
}

void CNeuralGasNode::deleteEdge(int id)
{
    for (unsigned int i = 0; i < this->edgeId.size(); i++)
    {
        if (this->edgeId[i] == id)
            this->edgeId.erase(this->edgeId.begin() + i);
    }
}

CNeuralGasNode::~CNeuralGasNode()
{

    for (unsigned int i = 0; i < this->neighbors.size(); i++)
    {
        delete[] this->neighbors[i];
    }
}

