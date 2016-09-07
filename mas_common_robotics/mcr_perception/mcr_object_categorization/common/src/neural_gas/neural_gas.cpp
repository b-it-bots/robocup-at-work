/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */


#include <limits>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <ctime>
#include <fstream>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <omp.h>

#include "neural_gas/mds.h"
#include "neural_gas/tool_box_ng.h"
#include "neural_gas/neural_gas.h"
#include "neural_gas/neural_gas_node.h"
#include "neural_gas/neural_gas_edge.h"

//#include <boost/thread.hpp>
//#include <boost/config.hpp>
//#include <boost/multi_array.hpp>
//#include <boost/lexical_cast.hpp>
//#include <boost/property_map/property_map.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/graphviz.hpp>
//#include <boost/graph/graph_traits.hpp>
//#include <boost/graph/johnson_all_pairs_shortest.hpp>


#define MIN_NODE_SET 2

CNeuralGas::CNeuralGas(int positionVectorSize, int positionValueRange, float nearestNeighborWeight, float nNeighborWeight, float maxAddNodeError, int maxEdgeAge, int intervalAddNode,
                       float errorNewNodeDecrease, float errorGeneralDecrease, int stoppingCriterionSelection, float stoppingCriterionParam, int distanceMeasureType, int initHitStrength,
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
    this->numSignalInputs = 0;
    this->initHitStrength = initHitStrength;
    this->minHitDeleteNodeThreshold = minHitDeleteNodeThreshold;
}

//It checks if neurons are close to the given inputsignals!
//since the gas should be close to the model -> if note delete node!
double CNeuralGas::checkModelConsistency(std::vector<CNeuralGasNode> &inputNodes, double maxDistance)
{
    //  std::cout<<"NeuralGas checkModelConsistency ...\n";
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    int nearestInputIndex;
    double totalMeanError = 0;
    int numberComparisons = 0;
    if (inputNodes.size() > 0 && this->nodes.size() > 0)
    {

        double distance;
        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            nearestInputIndex = -1;
            int nodeID = it->second->getId();
            distance = this->getNearestNodeDistance(inputNodes, nodeID, nearestInputIndex);
            totalMeanError += distance;
            numberComparisons++;

            //  std::cout << "DISTANCE input index :" << nearestInputIndex << "(" << distance << "/" << maxDistance << ")\n";
            if (distance > maxDistance)
            {
                //  std::cout << "**Delete Node ID:" << nodeID << "(" << distance << "/" << maxDistance << ")\n";
                this->deleteNode(this->nodes[nodeID]);
            }
        }
        assert(numberComparisons > 0);
        assert(this->nodes.size() > 0);
        totalMeanError = totalMeanError / ((double) numberComparisons);
    }
    return totalMeanError;
}

void CNeuralGas::normalizeNeuralGas()
{
    //  std::cout<<"NeuralGas normalizeNeuralGas ...\n";
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    if (this->nodes.size() > 0)
    {
        int posDim = 0;
        std::vector<double> centroid;
        centroid = this->computeCentroid();

        nodeIter it = this->nodes.begin();
        posDim = it->second->position.size();

        //do translation that centroid is coord frame origin!
        std::vector<double> distances;
        double maxDistance = 0;
        double t = 0;

        std::vector<double> normalizedCentroid;
        for (unsigned int d = 0; d < posDim; ++d)
        {
            normalizedCentroid.push_back(0);
        }

        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            for (unsigned int d = 0; d < posDim; ++d)
            {
                it->second->position[d] = it->second->position[d] - centroid[d];
            }
            distances.push_back(this->getDistance(it->second->position, normalizedCentroid));
        }

        maxDistance = *std::max_element(distances.begin(), distances.end());
        t = ((double) 1 / maxDistance);

        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            for (unsigned int d = 0; d < posDim; ++d)
            {
                //line vector equation
                it->second->position[d] = normalizedCentroid[d] + (it->second->position[d] - normalizedCentroid[d]) * t;
            }
        }

    }

}

//slow must be optimized!!!
void CNeuralGas::checkTriangulation()
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        //std::cout << "nodeiter!\n";
        CNeuralGasNode* parentNode = it->second;
        if (parentNode->edgeId.size() > 1)
        {
            for (int neigIter = 0; neigIter < parentNode->edgeId.size(); neigIter++)
            {
                bool triangle = false;
                std::cout << "checking neighbourhood!\n";
                CNeuralGasNode* neigbor = this->edges[parentNode->edgeId[neigIter]]->getNeighbor(parentNode);
                for (int neighsneigh = 0; neighsneigh < neigbor->edgeId.size(); neighsneigh++)
                {
                    //std::cout << "triangelsearch\n";
                    for (int parentneigh = 0; parentneigh < parentNode->edgeId.size(); parentneigh++)
                    {
                        //      std::cout << "triangelsearch1 "<<
                        //      this->edges[neigbor->edgeId[neighsneigh]]->getNeighbor(neigbor)->getId()<<"\n";
                        //     //" "<<this->edges[parentNode->edgeId[parentneigh]]->getNeighbor(parentNode)->getId() <<" \n";

                        if (this->edges[neigbor->edgeId[neighsneigh]]->getNeighbor(neigbor)->getId() == this->edges[parentNode->edgeId[parentneigh]]->getNeighbor(parentNode)->getId())
                        {
                            //then triangle
                            std::cout << "triangle found for neig->  " << neigbor->getId() << "\n";
                            triangle = true;
                            break;
                        }

                    }
                    if (triangle)
                        break;

                }
                if (!triangle)
                {
                    //  std::cout << "triangle NOT found for neig->  " << neigbor->getId() << "\n";
                    float minAngle = 361.0f;
                    CNeuralGasNode* minparentNeighNode2;
                    for (int parentneigh = 0; parentneigh < parentNode->edgeId.size(); parentneigh++)
                    {
                        CNeuralGasNode* parentNeighNode2 = this->edges[parentNode->edgeId[parentneigh]]->getNeighbor(parentNode);

                        std::vector<double> parentneighPos = parentNeighNode2->position;

                        float angle;
                        if (neigbor->getId() != parentNeighNode2->getId())
                        {
                            // take the one with the shortest distance
                            std::cout << "   Angle (neigborID=" << neigbor->getId() << "/" << parentNeighNode2->getId() << ") :" << angle << "\n";
                            //this->getDistance(parentneighPos, neigbor->position);
                            angle = toolBox.angleBetweenPoints(parentneighPos, neigbor->position);
                            if (angle < minAngle)
                            {
                                minAngle = angle;
                                minparentNeighNode2 = parentNeighNode2;
                                triangle = true;
                            }
                        }

                        ///     float angle;
                        ///      if (neigbor->getId() != parentNeighNode2->getId() && toolBox.angleBetweenPoints(parentNode->position, parentneighPos) < toolBox.angleBetweenPoints(parentNode->position, neigbor->position))
                        ///      {// take the one with the shortest distance
                        ///      // std::cout << "   Angle (neigborID=" << neigbor->getId() << "/" << parentNeighNode2->getId() << ") :" << angle << "\n";
                        ///      this->createConnection(*neigbor, *parentNeighNode2);
                        ///      triangle = true;
                        ///      //return;
                        ///      break;
                        ///
                        ///      }
                    }
                    if (triangle)
                    {
                        this->createConnection(*neigbor, *minparentNeighNode2);
                    }
                }
            }
        }
    }
}

void CNeuralGas::reset()
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    typedef std::map<int, CNeuralGasEdge*>::iterator edgeIter;

    //std::cout << "before numSignalInputs" << numSignalInputs << std::endl;
    for (nodeIter it = this->nodes.begin(); it != (this->nodes.end()); it++)
    {
        it->second->reset();
    }

    for (edgeIter it = this->edges.begin(); it != this->edges.end(); it++)
    {
        it->second->reset();
    }

    this->edges.clear();
    this->nodes.clear();
    this->numSignalInputs = 0;
    //std::cout << "after numSignalInputs" << numSignalInputs << std::endl;
}

void CNeuralGas::initRandom(int nNodes)
{
    for (int iter = 0; iter < nNodes; iter++)
    {

        CNeuralGasNode *node = new CNeuralGasNode(); /// !!!!! POINTER
        node->setPosition(this->doRandomPosition());
        this->nodes[node->id] = node;
        // std::cout << "(" << node->position[0] << "," << node->position[1] << ") created" << std::endl;

        if (iter > 0)
        {
            this->createConnection(*nodes[node->id], *nodes[node->id - 1]);
        }
    }
}

void CNeuralGas::addNeuron(CNeuralGasNode* inputSignal)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    // std::cout << "Add neuron " << this->nodes.size() << std::endl;

    if (this->nodes.size() == 0)
    {
        CNeuralGasNode *node; // = new CNeuralGasNode();
        node = inputSignal;

        this->nodes[node->id] = node;
        //       std::cout << "(" << node->position[0] << "," << node->position[1] << ") created (empty)" << std::endl;
    }
    else
    {
        CNeuralGasNode *node;// = new CNeuralGasNode();
        node = inputSignal;
        //    std::cout << "(" << node->position[0] << "," << node->position[1] << ") created" << std::endl;
        for (nodeIter it = this->nodes.begin(); it != (this->nodes.end()); it++)
        {
            this->createConnection(*it->second, *node);
        }
        this->nodes[node->id] = node;
    }

    numSignalInputs++;
    //   std::cout << "node added" << std::endl;
}

void CNeuralGas::updateNeuralGasT(CNeuralGasNode* inputSignal, CNeuralGas *ng)
{
    ng->updateNeuralGas(inputSignal);
}

void CNeuralGas::updateNeuralGas(CNeuralGasNode* inputSignal)
{
    if (!this->stoppingCriterion(this->stoppingCriterionSelection))
    {

        //   begin_time = clock();
        this->updateNodePositions(inputSignal);
        //   std::cout <<"CNeuralGas::updateNeuralGas1..."<< float(clock() - begin_time) / (CLOCKS_PER_SEC/1000)<<"\n";

        // begin_time = clock();
        this->updateEdges();
        //std::cout <<"CNeuralGas::updateNeuralGas2..."<< float(clock() - begin_time) / (CLOCKS_PER_SEC/1000)<<"\n";

        //   begin_time = clock();
        this->updateNodeNumber();
        //  std::cout <<"CNeuralGas::updateNeuralGas3..."<< float(clock() - begin_time) / (CLOCKS_PER_SEC/1000)<<"\n";

        //       begin_time = clock();
        this->updateNodeError();
        //  std::cout <<"CNeuralGas::updateNeuralGas4..."<< float(clock() - begin_time) / (CLOCKS_PER_SEC/1000)<<"\n";
        //this->updateDecayLearnining();
        numSignalInputs++;
        //  std::cout << "done";
    }

    //std::cout <<"CNeuralGas::updateNeuralGas..."<< float(clock() - begin_time) / CLOCKS_PER_SEC<<"\n";
    //
}

void CNeuralGas::updateDecayLearnining()
{
    //plot 0.05*exp(-1*0.005*x));
    //Std decrease rate
    //  this->nearestNeighborWeight = (nearestNeighborWeight*exp((double)-1*0.00001f*(double)numSignalInputs));
    //  this->nNeighborWeight =  (nNeighborWeight*exp((double)-1*0.00001f*(double)numSignalInputs));
    //std::cout<<"Decay "<<nearestNeighborWeight << " " << nNeighborWeight <<std::endl;
    double decayfactor = 100.0f;

    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    double meanHits = 0;
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        meanHits += it->second->error; //hits;
    }
    meanHits /= this->nodes.size();
    meanHits *= ((double) numSignalInputs);
    decayAdaptation = (double) 1 / (meanHits * decayfactor);

    this->nearestNeighborWeight = (nearestNeighborWeight * exp((double) - 1 * (decayAdaptation)));
    this->nNeighborWeight = (nNeighborWeight * exp((double) - 1 * (decayAdaptation)));
    std::cout << "Decay " << (decayAdaptation) << " " << nearestNeighborWeight << " " << nNeighborWeight << std::endl;
    //getchar();
}
void CNeuralGas::updateNodeError()
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        it->second->error = it->second->error * this->errorGeneralDecrease;
    }
}

void CNeuralGas::updateEdges()
{
    typedef std::map<int, CNeuralGasEdge*>::iterator edgeIter;

    //  std::cout << "Edge update " << maxEdgeAge << std::endl;

    //  for (edgeIter it = this->edges.begin(); it != this->edges.end(); it++) {
    //      std::cout << "ID Edge" << (it)->second->id << std::endl;
    //  }

    if (this->edges.size() == 1)
    {
        return;
    }
    for (edgeIter it = this->edges.begin(); it != this->edges.end(); it++)
    {
        //  std::cout << "Edge deleting" << std::endl;
        if ((it)->second->age > this->maxEdgeAge)
        {
            int id = (it)->second->id;

            //////////
            this->deleteEdge(this->edges[id]);
        }
        //  std::cout << "Edge deleted done out" << std::endl;
    }
}

void CNeuralGas::updateNodePositions(CNeuralGasNode* inputSignal)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    int nearestNeighbourId;
    int secondNearestNeighborId;
    double nearestNeighbourDistance;
    int is2ndNeig = 0;
    //  is2ndNeig = false; //id 2nd nn is in nn neighborhood

    //5 6
    this->getNearestNeighbor(*inputSignal, nearestNeighbourId, nearestNeighbourDistance, secondNearestNeighborId);

    //The farther the node the higher the error the higher the activity
    //Never count down the hits!!! since the hits should be independent to the sequence of input signals
    this->nodes[nearestNeighbourId]->hits++;
    this->nodes[nearestNeighbourId]->error += nearestNeighbourDistance;
    this->nodes[nearestNeighbourId]->activity = exp(nearestNeighbourDistance * (-1));
    this->nodes[nearestNeighbourId]->label = inputSignal->label;

    this->nodes[nearestNeighbourId]->moveToNode(*inputSignal, this->nodes[nearestNeighbourId]->error, this->nearestNeighborWeight);


    for (int iterNeighbors = 0; iterNeighbors < this->nodes[nearestNeighbourId]->edgeId.size(); iterNeighbors++)
    {

        CNeuralGasNode *nn;
        CNeuralGasEdge *edge;

        edge = (this->edges[this->nodes[nearestNeighbourId]->edgeId[iterNeighbors]]);
        //3
        edge->age++;

        nn = edge->getNeighbor(this->nodes[nearestNeighbourId]);
        ///////// Here even the direct neigbours are getting error not like in paper where just the nearest point gets
        nn->error += getDistance(inputSignal->position, nn->position);

        //  std::cout<<">>>>>>>>>>>>>>>>>moveTonode "<<inputSignal->id<<" "<<std::endl;
        nn->moveToNode(*inputSignal, nn->error, this->nNeighborWeight);
        ///////////
        //6
        if (nn->id == this->nodes[secondNearestNeighborId]->id)
        {
            edge->age = 0;
            is2ndNeig = 1;
        }
    }

    //6
    if (this->nodes.size() > 2 && is2ndNeig == 0)
    {
        this->createConnection(*this->nodes[nearestNeighbourId], *this->nodes[secondNearestNeighborId]);
        this->nodes[secondNearestNeighborId]->label = inputSignal->label;
        //  std::cout << "new created Connection xx" << std::endl;
    }
}

/*this->maxAddNodeError==-1 then just adding in interval else if reached this->maxAddNodeError */
void CNeuralGas::updateNodeNumber()
{

    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    //Adding?
    //////////////DO ADDING NODES IN AN INTERVAL // before his->maxAddNodeError != -1
    if ((this->numSignalInputs % this->intervalAddNode == 0 && this->maxAddNodeError == -1) || this->maxAddNodeError > 0)
    {
        typedef std::map<int, CNeuralGasEdge*>::iterator edgeIter;
        int maxErrorId = this->getMaxErrorNode();

        //  std::cout << "MaxError Node " << maxErrorId << std::endl;
        /////////////DO ADDING NODES IF MAX ERROR is high enough
        if (maxErrorId == -1 || this->nodes[maxErrorId]->error < this->maxAddNodeError)
        {
            if (this->maxAddNodeError == -1)
            {
                std::cout << "should not be here! since intervalAddNode mode";
            }

            return;
        }
        else
        {
            CNeuralGasNode* maxErrorNode = this->nodes[maxErrorId];

            if (maxErrorNode->edgeId.size() == 0) // can not happen since we delete such kind of nodes
                return;
            else
            {
                double maxNeighError = std::numeric_limits<double>::infinity() * (-1);
                int maxNeighErrorId = -1;
                int maxEdgeErrorId = -1;
                //       std::cout << "MaxError Node " << maxErrorNode->id << std::endl;
                CNeuralGasNode* maxNeighErrorNode;
                for (int iterNeighbors = 0; iterNeighbors < maxErrorNode->edgeId.size(); iterNeighbors++)
                {

                    CNeuralGasNode *nn;
                    CNeuralGasEdge *edge;

                    edge = this->edges[maxErrorNode->edgeId[iterNeighbors]];
                    nn = edge->getNeighbor(maxErrorNode);

                    if (nn->error > maxNeighError)
                    {
                        maxNeighError = nn->error;
                        maxNeighErrorId = nn->id;
                        maxNeighErrorNode = nn;
                        maxEdgeErrorId = edge->id;
                        //     std::cout << "NN error " << nn->id << std::endl;
                    }

                }
                if (maxNeighError == 0 && maxErrorNode->edgeId.size() > 1)
                {
                    ;// std::cout << "Something is wrong!!!" << maxNeighError << " and " << maxErrorNode->edgeId.size() << std::endl;
                }
                //    std::cout << "MaxNeigError Node " << maxNeighErrorNode->id << std::endl;
                //  std::cout << "Neigbours-->[0]  " << this->edges[maxEdgeErrorId]->nodes[0]->id << this->edges[maxEdgeErrorId]->nodes[0]->position[0]
                //          << this->edges[maxEdgeErrorId]->nodes[0]->position[1] << std::endl;
                // std::cout << "Neigbours-->[1]  " << this->edges[maxEdgeErrorId]->nodes[1]->id << this->edges[maxEdgeErrorId]->nodes[1]->position[0]
                //          << this->edges[maxEdgeErrorId]->nodes[1]->position[1] << std::endl;

                //   std::cout << "distances!!!" << maxErrorNode->id << " " << maxNeighErrorNode->id << std::endl;
                float distanceBtwMax = this->getDistance(maxErrorNode->position, maxNeighErrorNode->position);
                //      if (maxErrorNode->id == maxNeighErrorNode->id) {
                //  std::cout << "distances!!!" << std::endl;
                // std::cout << "----------------------------------updateNodeNumber : Nodes have same IDs!!!";
                //              << std::endl;
                //      exit(1);
                //  }
                if (distanceBtwMax == 0)
                    distanceBtwMax = 0.000001f;

                if (maxErrorNode->position[0] < 0 || maxErrorNode->position[1] < 0)
                {
                    ;//std::cout << "updateNodeNumber----***********************************STH IS WRONG WITH maxErrorNode " << maxErrorNode->id << std::endl;

                }
                if (maxNeighErrorNode->position[0] < 0 || maxNeighErrorNode->position[1] < 0)
                {
                    ;// std::cout << "updateNodeNumber----***********************************STH IS WRONG WITH maxNeighErrorNode " << maxNeighErrorNode->id << std::endl;
                }

                CNeuralGasNode* newNode = new CNeuralGasNode();
                //std::cout << "created new Node " << newNode->id << std::endl;

                std::vector<float> newNodePos;
                newNodePos.resize(maxErrorNode->position.size());
                newNode->position.resize(maxErrorNode->position.size());
                for (int i = 0; i < maxErrorNode->position.size(); i++)
                {
                    newNodePos[i] = (distanceBtwMax * 0.5) * ((float) maxNeighErrorNode->position[i] - (float) maxErrorNode->position[i]) / (float) distanceBtwMax;

                    newNode->position[i] = ((maxErrorNode->position[i] + newNodePos[i]));
                }

                if (newNode->position[0] < 0 || newNode->position[1] < 0)
                {
                    ;//std::cout << "updateNodeNumber----***********************************STH IS WRONG WITH NEW NODE " << newNode->id << std::endl;
                }
                newNode->error = ((maxErrorNode->error + maxNeighErrorNode->error) * 0.5f);
                newNode->label = maxErrorNode->label;
                this->nodes[newNode->id] = newNode;

                //adpat error var

                //   std::cout << "\n*MAXERRORNode error" << maxErrorNode->error << "\n";
                maxErrorNode->error = maxErrorNode->error * this->errorNewNodeDecrease; // * maxErrorNode->hits;
                //    std::cout << "\n*MAXERRORNode error" << maxErrorNode->error << " " << maxErrorNode->activity << "\n";
                //      std::cout << "\n*maxNeighErrorNode error" << maxNeighErrorNode->error << "\n";
                maxNeighErrorNode->error = maxNeighErrorNode->error * this->errorNewNodeDecrease;// * maxNeighErrorNode->hits;
                newNode->error = maxErrorNode->error;
                //      std::cout << "\n*maxNeighErrorNode error" << maxNeighErrorNode->error << " " << maxNeighErrorNode->activity << "\n";
                //      getchar();
                //createConnection between new node
                this->createConnection(*newNode, *maxErrorNode);
                this->createConnection(*newNode, *maxNeighErrorNode);

                //delete old connection.
                //        std::cout << "Delte1 old connections" << std::endl;
                this->edges.erase(this->edges.find(maxEdgeErrorId));
                //          std::cout << "Delte2 old connections" << std::endl;
                maxErrorNode->deleteEdge(maxEdgeErrorId);
                //            std::cout << "Delte3 old connections" << std::endl;
                maxNeighErrorNode->deleteEdge(maxEdgeErrorId);
                //              std::cout << "Delte4 old connections" << std::endl;

                //when requires paper;
                //  maxErrorNode->hits = this->initHitStrength - (1-exp(-1.05f/3.33));
            }
        }
    }

    //removing Nodes
    if (this->minHitDeleteNodeThreshold != -1)
    {
        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            if ((((double) it->second->hits / (double) this->numSignalInputs) < minHitDeleteNodeThreshold))
            {
                this->deleteNode(it->second);
            }
        }
    }

}

std::vector<double> CNeuralGas::doRandomPosition()
{
    std::vector<double> rPos;
    for (int iter = 0; iter < positionVectorSize; iter++)
        rPos.push_back(rand() % positionValueRange);

    return rPos;
}

void CNeuralGas::createConnection(CNeuralGasNode &a, CNeuralGasNode &b)
{

    CNeuralGasEdge *edge = new CNeuralGasEdge();

    // std::cout << "Connection create between Node " << a.id << " and " << b.id << "  Total:" << this->edges.size() << std::endl;

    edge->nodes.push_back(&a);
    edge->nodes.push_back(&b);

    a.edgeId.push_back(edge->id);
    b.edgeId.push_back(edge->id);

    this->edges[edge->id] = edge;

    // std::cout << "Connection " << edge->id << " created between Node " << a.id << " and " << b.id << "  Total:" << this->edges.size() << std::endl;
}

void CNeuralGas::getNearestNeighbor(CNeuralGasNode &a, int &neighbour, double &distance)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    distance = std::numeric_limits<double>::infinity();
    double curDistance = 0;

    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        curDistance = this->getDistance(a.position, it->second->position);
        if (curDistance < distance)
        {
            distance = curDistance;
            neighbour = it->first;
        }
    }
}

void CNeuralGas::getNearestNeighbor(CNeuralGasNode &a, int &neighbour, double &distance, int &secondNearestNeighbor)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    distance = std::numeric_limits<double>::infinity();
    double secondDistance = std::numeric_limits<double>::infinity();
    double curDistance = 0;

    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        curDistance = this->getDistance(a.position, it->second->position);
        if (curDistance < distance)
        {
            secondDistance = distance;
            secondNearestNeighbor = neighbour;

            distance = curDistance;
            neighbour = it->first;
        }
        else if (curDistance < secondDistance)
        {
            secondDistance = curDistance;
            secondNearestNeighbor = it->first;
        }
    }
}

double CNeuralGas::getDistance(std::vector<double> a, std::vector<double> b)
{
    if (distanceMeasureType == 1)
        return this->getEuclideanDistance(a, b);
    else if (distanceMeasureType == 2)
        return this->getCosineDistance(a, b);
    else
        return std::numeric_limits<double>::infinity();
}

double CNeuralGas::getEuclideanDistance(std::vector<double> a, std::vector<double> b)
{

    std::vector<double> delta;
    double deltaSum = 0;
    assert(a.size() == b.size());

    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (int m = 0; m < a.size(); m++)
        {
            delta.at(m) = (a.at(m) - b.at(m));
        }

        for (int m = 0; m < a.size(); m++)
        {

            deltaSum += (delta.at(m) * delta.at(m)); //(pow(delta.at(m), 2));
        }

        //return deltaSum;//sqrt(deltaSum);
        return sqrt(deltaSum);
    }
    else
        return -1;
}

double CNeuralGas::getCosineDistance(std::vector<double> a, std::vector<double> b)
{

    double nominator = 0.0f;
    double aProduct = 0.0f;
    double bProduct = 0.0f;
    double denominator = 0.0f;

    if (a.size() == b.size())
    {
        for (int m = 0; m < a.size(); m++)
        {
            nominator += (a[m] * b[m]);
        }

        for (int m = 0; m < a.size(); m++)
        {

            aProduct += pow(a.at(m), 2);
            bProduct += pow(b.at(m), 2);
        }
        denominator = (sqrt(aProduct) * sqrt(bProduct));
        //  std::cout<<"Cosine..."<<nominator << "/"<<denominator<<"="<<(nominator / denominator)<<std::endl;
        if (denominator == 0)
        {
            //  std::cout << "!!! DIV/0" << std::endl;
            denominator = std::numeric_limits<double>::epsilon();
        }

        return (1 - (nominator / denominator));
    }
    else
    {
        std::cout << "Something is wrong with the vector length" << std::endl;
        return -1;
    }
}

int CNeuralGas::getMaxErrorNode()
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    double maxError = std::numeric_limits<double>::infinity() * (-1);
    int maxErrorNode = -1;

    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        if (it->second->error > maxError)
        {
            maxError = it->second->error;
            maxErrorNode = it->first;
        }
    }

    // std::cout << "MAY" << std::endl;
    return maxErrorNode;
}

//THIS IS A CIRCLE between DELETEEDGE AND DELETE NODE!!!
void CNeuralGas::deleteEdge(CNeuralGasEdge *edge)
{
    CNeuralGasNode* node1 = edge->nodes[0];
    CNeuralGasNode* node2 = edge->nodes[1];

    // std::cout << "+++++++++++NEW Edge delete.. " << edge->id << " " << node1->edgeId.size() << " " << node2->edgeId.size() << " " << " total:" << this->edges.size()
    //         << std::endl;
    this->edges.erase(this->edges.find(edge->id));
    node1->deleteEdge(edge->id);
    node2->deleteEdge(edge->id);
    //  std::cout << "+++++++++++Edge delete.. " << edge->id << " " << node1->edgeId.size() << " " << node2->edgeId.size() << " " << " total:" << this->edges.size() << std::endl;
    //  std::cout << "Edge deleted done" << std::endl;

    if (node1->edgeId.size() == 0 && this->nodes.size() > MIN_NODE_SET)
    {
        // std::cout << "Node w/o edges -> erasing..." << edge->id << std::endl;
        this->deleteNode(node1);
        //  std::cout << "DELETE NODE IN SINCE OF MISSING EDGE1" << std::endl;
        //  exit(1);
    }

    if (node2->edgeId.size() == 0 && this->nodes.size() > MIN_NODE_SET)
    {
        //   std::cout << "Node w/o edges -> erasing..." << edge->id << std::endl;
        this->deleteNode(node2);
        //    std::cout << "DELETE NODE IN SINCE OF MISSING EDGE2" << std::endl;
        //  exit(1);
    }
}

void CNeuralGas::deleteNode(CNeuralGasNode *node)
{

    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    typedef std::vector<int>::iterator edgeIdIter;

    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {

        if (it->first == node->id)
        {
            int idToDelete = it->first;
            ///OWEI hier you must allways work with find!!!! the node with the correct ID!!! since you might delte
            //the node in the for loop!!
            //            std::cout << "Number of Edges " << it->second->edgeId.size() << std::endl;

            for (edgeIdIter itEdge = it->second->edgeId.begin(); itEdge != it->second->edgeId.end(); itEdge)
            {

                //     std::cout << "*******delete edge with id " << *itEdge << std::endl;

                //   std::cout << "*******delete edge with size " << it->second->edgeId.size() << std::endl;
                this->deleteEdge(this->edges[*itEdge]);
                // std::cout << "*******delete edge with size " << it->second->edgeId.size() << std::endl;
                //exit(1);
            }

            if (this->nodes.find(idToDelete) != this->nodes.end() && this->nodes.size() > MIN_NODE_SET)
                nodes.erase(this->nodes.find(idToDelete));

        }
    }

}

bool CNeuralGas::stoppingCriterion(float stoppingCriterionSelection)
{
    if (this->stoppingCriterionSelection == 1)
    {
        if (this->nodes.size() >= (int) this->stoppingCriterionParam)
            return true;
    }
    return false;
}

double CNeuralGas::measureDistance(std::vector<std::vector<double> > query, int type)
{

    double error = 0.0f;
    for (int iter = 0; iter < query.size(); iter++)
    {
        error += measureDistance(query[iter], type);
    }
    return ((double) error / (double) query.size());
}

double CNeuralGas::measureDistance(std::vector<double> query, int type)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    int nearestNodes;
    double nearestDistances;

    double error = 0.0f;
    this->distanceMeasureType = type;
    int nearestNodeId = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    double meanDistance = 0;
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        double distance;
        distance = this->getDistance(query, it->second->position);
        meanDistance += distance;
        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestNodeId = it->first;
        }
    }
    nearestNodes = nearestNodeId;
    nearestDistances = nearestDistance;
    std::cout << "CNeuralGas::measureDistance....Nearest Node has Label :" << this->nodes[nearestNodes]->getLabel() << std::endl;
    error = (nearestDistances * ((double) 1 - ((double) this->nodes[nearestNodes]->hits / (double) this->numSignalInputs)));
    //error = (nearestDistances);
    //error = (double)meanDistance/(double)this->nodes.size();
    return error;
}


int CNeuralGas::measureKNearestNodeDistance(std::vector<double> query, int type, int k)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    int nearestNodes;
    double nearestDistances;

    double error = 0.0f;
    this->distanceMeasureType = type;
    int nearestNodeId = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        double distance;

        distance = this->getDistance(query, it->second->position);

        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestNodeId = it->first;
        }
    }
    nearestNodes = nearestNodeId;
    nearestDistances = nearestDistance;
    //std::cout<<"CNeuralGas::measureDistance....Nearest Node has Label :" << this->nodes[nearestNodes]->label<<std::endl;

    error = (nearestDistances * ((double) 1 - ((double) this->nodes[nearestNodes]->hits / (double) this->numSignalInputs)));
    return this->nodes[nearestNodes]->getLabel();
}

double CNeuralGas::measureKNearestNodeDistanceError(std::vector<double> query, int type, int k)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    int nearestNodes;
    double nearestDistances;

    double error = 0.0f;
    this->distanceMeasureType = type;
    int nearestNodeId = 0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
    {
        double distance;

        distance = this->getDistance(query, it->second->position);

        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestNodeId = it->first;
        }
    }
    nearestNodes = nearestNodeId;
    nearestDistances = nearestDistance;
    //std::cout<<"CNeuralGas::measureDistance....Nearest Node has Label :" << this->nodes[nearestNodes]->label<<std::endl;

    //error = (nearestDistances * ((double) 1 - ((double) this->nodes[nearestNodes]->hits / (double) this->numSignalInputs)));
    return nearestDistance;
}

double CNeuralGas::getNearestNodeDistance(std::vector<CNeuralGasNode> &inputSignals, int &nodeId, int &nearestInputId)
{
    nearestInputId = 0;

    assert(inputSignals.size() > 0);
    assert(this->nodes.size() > 0);
    assert(this->nodes.find(nodeId) != this->nodes.end());

    double nearestDistance = std::numeric_limits<double>::infinity();
    for (unsigned int iterInput = 0; iterInput < inputSignals.size(); iterInput++)
    {
        double distance = 0.0;

        assert(inputSignals[iterInput].position.size() > 0);
        assert(this->nodes[nodeId]->position.size() > 0);
        distance = this->getDistance(inputSignals[iterInput].position, this->nodes[nodeId]->position);

        if (distance < nearestDistance)
        {
            nearestDistance = distance;
            nearestInputId = iterInput;
        }
    }

    assert(nearestDistance > 0);
    assert(nearestDistance != std::numeric_limits<double>::infinity());
    return nearestDistance;
}

apsp_graph CNeuralGas::createJohnsonAllPairShortestPathGraph(std::string fileName)
{
    typedef std::map<int, CNeuralGasEdge*>::iterator edgeIter;

    apsp_graph johnsonAPSPGraph;

    std::string filenameA = fileName;
    filenameA.append("_JohnAPSP.ng");

    for (edgeIter it = this->edges.begin(); it != this->edges.end(); it++)
    {
        int node1 = it->second->nodes[0]->getId();
        int node2 = it->second->nodes[1]->getId();
        double distance = this->getDistance(it->second->nodes[0]->position, it->second->nodes[1]->position);

///     fout << node1 << " -> " << node2 << "[label=" << distance << "]\n";

        johnsonAPSPGraph.add_undirectedEdge(node1, node2, distance);
    }
/// fout << "}\n";

/// std::cout << "... Graphivz graph saved.\n";

    johnsonAPSPGraph.apspJohnson();
    //johnsonAPSPGraph.print_matrix();
    johnsonAPSPGraph.print_matrix(filenameA);
    std::cout << "... Johnson ASPS Matrix saved.(dot johnson-eg.dot -Tps -o johnson-eg.ps)\n";
///fout.close();

    return johnsonAPSPGraph;
}

//Only graph without any saving stuff
apsp_graph CNeuralGas::createJohnsonAllPairShortestPathGraph()
{
    typedef std::map<int, CNeuralGasEdge*>::iterator edgeIter;

    apsp_graph johnsonAPSPGraph;

    for (edgeIter it = this->edges.begin(); it != this->edges.end(); it++)
    {
        int node1 = it->second->nodes[0]->getId();
        int node2 = it->second->nodes[1]->getId();
        double distance = this->getDistance(it->second->nodes[0]->position, it->second->nodes[1]->position);

        //fout << node1 << " -> " << node2 << "[label=" << distance << "]\n";

        johnsonAPSPGraph.add_undirectedEdge(node1, node2, distance);
    }

    johnsonAPSPGraph.apspJohnson();

    return johnsonAPSPGraph;
}

double CNeuralGas::getDistanceBetweenNodes(unsigned int nodeId1, unsigned int nodeId2)
{
    assert(this->nodes.find(nodeId1) != this->nodes.end());
    assert(this->nodes.find(nodeId2) != this->nodes.end());
    return this->getDistance(this->nodes[nodeId1]->position, this->nodes[nodeId2]->position);
}

void CNeuralGas::saveNeuralGas(std::string fileName)
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    std::fstream outNodes, outEdges;
    std::cout << "Save NeuralGas..." << std::endl;
    std::string filenameNodes = fileName;
    std::string filenameEdges = fileName;
    filenameNodes.append("_nodes.ng");
    filenameEdges.append("_edges.ng");
    outNodes.open(filenameNodes.c_str(), std::ios::out | std::ios::binary);

    if (this->nodes.size() > 0 && outNodes.is_open())
    {

        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            outNodes << it->second->getId() << " ";
            for (int j = 0; j < it->second->position.size(); j++)
            {
                outNodes << it->second->position[j] << " ";
            }
            outNodes << std::endl;
        }

        outNodes.close();
        std::cout << "...Neural Gas Nodes saved." << std::endl;
    }
    else
    {
        std::cout << "...could not save NeuralGas Nodes!" << std::endl;
    }

    outEdges.open(filenameEdges.c_str(), std::ios::out | std::ios::binary);
    if (this->nodes.size() > 0 && outEdges.is_open())
    {

        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            outEdges << it->second->getId() << " ";

            for (int j = 0; j < it->second->edgeId.size(); j++)
            {
                //outEdges<< it->second->edgeId[j]<< " ";

                //  std::cout<<"node id"<<it->second->getId()<<"\n";

                if (this->edges.find(it->second->edgeId[j]) != this->edges.end())
                {
                    //      std::cout<<"  edge id"<<it->second->edgeId[j]<<"\n";
                    //  std::cout<<"      neig id"<<this->edges[it->second->edgeId[j]]->getNeighbor(it->second)->getId()<<"\n";
                    //outEdges<<this->edges.find(it->second->edgeId[j])->second->getNeighbor(it->second)->getId()<<" ";
                    outEdges << this->edges[it->second->edgeId[j]]->getNeighbor(it->second)->getId() << " ";
                }
                else
                {
                    std::cout << "...Neural Gas Edges saving: some edge not found." << std::endl;
                }
            }
            outEdges << std::endl;
        }

        outEdges.close();
        std::cout << "...Neural Gas Edges saved." << std::endl;
    }
    else
    {
        std::cout << "...could not save NeuralGas Edges!" << std::endl;
    }
}

std::vector<double> CNeuralGas::computeCentroid()
{
    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;

    std::vector<double> centroid;
    if (this->nodes.size() > 0)
    {
        nodeIter it = this->nodes.begin();

        centroid.resize(it->second->position.size());

        for (nodeIter it = this->nodes.begin(); it != this->nodes.end(); it++)
        {
            for (unsigned int d = 0; d < centroid.size(); ++d)
            {
                centroid[d] += it->second->position[d];
            }
        }

        for (unsigned int d = 0; d < centroid.size(); ++d)
        {
            centroid[d] = (double) centroid[d] / (double) this->nodes.size();
        }

    }
    return centroid;
}

CNeuralGas::~CNeuralGas()
{
    /*
     std::map<int, CNeuralGasNode*>::iterator iterNodes;
     for(iterNodes = this->nodes.begin() ; iterNodes != this->nodes.end(); iterNodes++)
     {
     delete iterNodes->second;
     }*/
}

