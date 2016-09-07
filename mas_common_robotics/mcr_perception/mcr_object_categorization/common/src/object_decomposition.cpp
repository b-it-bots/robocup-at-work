/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


// comboine shell estimaiton
#include <cstdlib>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>

#include "emd/emd.h"
#include "object_decomposition.h"

CObjectDecomposition::CObjectDecomposition()
{
    // TODO Auto-generated constructor stub
    this->isGeoShellMode = false;
    this->isInit = false;
    this->isNodeMode = false;
    this->isNormalMode = false;
    this->isGeoMode = false;
    this->isGeoNodeMode = false;
    this->isNormalShellMode = false;
    this->isNodeToCenterMode = false;
    this->nodeKdeEstimations.clear();
    this->logger = &CLogger::getInstance();
}
void CObjectDecomposition::setManualInitShell(bool isInit)
{
    this->logger->log->debug("!!CObjectDecomposition::manualSetInitShell!!\n");
    this->isGeoShellMode = isInit;
    this->isInit = isInit;
}

void CObjectDecomposition::setManualInitNormal(bool isInit)
{
    this->logger->log->debug("!!CObjectDecomposition::manualSetInitNormal!!\n");
    this->isInit = isInit;
    this->isNormalMode = isInit;
}

void CObjectDecomposition::setManualInitShellNormal(bool isInit)
{
    this->logger->log->debug(
        "!!CObjectDecomposition::manualSetInitShellNormal!!\n");
    this->isInit = isInit;
    this->isNormalMode = isInit;
    this->isGeoShellMode = isInit;
}

void CObjectDecomposition::addObject(SObjectDescription &object)
{
    this->logger->log->debug("CObjectDecomposition::addObject... \n");
    this->objects.push_back(object);
}

std::map<int, std::vector<double> > CObjectDecomposition::initCombinedGeoShellNormalEstimation(
    int numShell, bool isCommon)
{

    this->logger->log->debug(
        "CObjectDecomposition::initCombinedGeoShellNormalEstimation....\n");

    if (isCommon)
    {
        this->initGeoShellEstimationCommon(numShell);
        this->initNormalEstimationCommon();
        //  std::cout<<" initCombinedGeoShellNormalEstimation uncomment here!!!";
    }
    else
    {
        this->initGeoShellEstimation(numShell);
        this->initNormalEstimation();
    }

    std::map<int, std::vector<double> > estimations;

    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {

        std::vector<std::vector<double> > comb;
        std::vector<double> s =
            this->objects[iterObject].combinedNormalizedKdeEstimations;
        std::vector<double>
        n =
            this->objects[iterObject].combinedNormalizedKdeNormalEstimations;
        //this->logger->log->error("CObjectDecomposition::initCombinedGeoShellNormalEstimation... Sth wrong...size=%d %d \n ",s.size(),n.size());
        //something is worng!!!
        if (s.size() == 0 || n.size() == 0)
        {
            this->logger->log->error(
                "CObjectDecomposition::initCombinedGeoShellNormalEstimation... Sth wrong...size==0\n");
            return estimations;
        }
        comb.push_back(s);
        comb.push_back(n);

        this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations
        = this->combineShellEstimations(comb);
        estimations[iterObject]
        = this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations;
    }

    return estimations;
}

std::map<int, std::vector<double> > CObjectDecomposition::initCombinedGeoShellNodeToCenterEstimation(
    int numShell, bool isCommon)
{

    this->logger->log->debug(
        "CObjectDecomposition::initCombinedGeoShellNodeToCenterEstimation....\n");

    if (isCommon)
    {
        assert(false);
        //this->initGeoShellEstimationCommon(numShell);
        //  this->initNormalEstimationCommon();
        //  std::cout<<" initCombinedGeoShellNormalEstimation uncomment here!!!";
    }
    else
    {
        this->initGeoShellEstimation(numShell);
        this->initNodeToCenterEstimation();
    }

    std::map<int, std::vector<double> > estimations;

    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {

        std::vector<std::vector<double> > comb;
        std::vector<double> s =
            this->objects[iterObject].combinedNormalizedKdeEstimations;
        std::vector<double>
        n =
            this->objects[iterObject].kdeNodeToCenterDistanceEstimations;
        //this->logger->log->error("CObjectDecomposition::initCombinedGeoShellNormalEstimation... Sth wrong...size=%d %d \n ",s.size(),n.size());
        //something is worng!!!
        if (s.size() == 0 || n.size() == 0)
        {
            this->logger->log->error(
                "CObjectDecomposition::initCombinedGeoShellNodeToCenterEstimation... Sth wrong...size==0\n");
            assert(s.size() == 0 || n.size() == 0);
            return estimations;
        }
        comb.push_back(s);
        comb.push_back(n);

        this->objects[iterObject].combinedNormalizedKdeShellNodeToCenterEstimations
        = this->combineShellEstimations(comb);
        estimations[iterObject]
        = this->objects[iterObject].combinedNormalizedKdeShellNodeToCenterEstimations;
    }

    return estimations;
}

std::map<int, std::vector<double> > CObjectDecomposition::initCombinedGeoShellNormalNormalShellEstimation(
    int numGeoShell, int numNormalShell, bool isCommon)
{

    this->logger->log->debug(
        "CObjectDecomposition::initCombinedGeoShellNormalNormalShellEstimation....\n");

    if (isCommon)
    {
        //      this->initGeoShellEstimationCommon(numShell);
        //  this->initNormalEstimationCommon();
        this->logger->log->warn(
            "CObjectDecomposition::initCombinedGeoShellNormalNormalShellEstimation  Not implemented yet!!!\n");
    }
    else
    {
        this->initGeoShellEstimation(numGeoShell);
        this->initNormalEstimation();
        this->initNormalShellEstimation(numNormalShell);
    }

    std::map<int, std::vector<double> > estimations;

    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {

        std::vector<std::vector<double> > comb;
        std::vector<double> s =
            this->objects[iterObject].combinedNormalizedKdeEstimations;
        std::vector<double>
        n =
            this->objects[iterObject].combinedNormalizedKdeNormalEstimations;
        std::vector<double> ns =
            this->objects[iterObject].combinedNormalShellEstimation;

        //something is worng!!!
        if (s.size() == 0 || n.size() == 0 || ns.size() == 0)
        {
            this->logger->log->error(
                "CObjectDecomposition::initCombinedGeoShellNormalNormalShellEstimation... number of estimation are different...size==0\n");
            return estimations;
        }
        comb.push_back(s);
        comb.push_back(n);
        comb.push_back(ns);

        this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations
        = this->combineShellEstimations(comb);
        estimations[iterObject]
        = this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations;
    }

    return estimations;
}

/*Normal estimation
 * for each object an own estimation that is optimized for each densitiy -> x, y, z*/
void CObjectDecomposition::initNormalEstimation()
{
    this->logger->log->debug(
        "CObjectDecomposition::initNormalEstimation  !!INDIVIDUAL OPT!!...\n");

    if (!this->isNormalMode)
    {
        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {

            // x y z
            for (unsigned int j = 0; j < this->objects[i].pointNormals.size(); ++j)
            {
                CDensityEstimation kde;
                kde.addData(this->objects[i].pointNormals[j],
                            this->objects[i].label, this->objects[i].filename);
                this->objects[i].kdeNormals[j] = kde;
            }
        }

        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {

            //  std::cout << "**InitEstimationhistogram KDE for Object " << iterObject
            //      << "\n";
            std::map<int, CDensityEstimation> &kdeObject =
                this->objects[iterObject].kdeNormals;
            //Now iter each shell of an object for init histogram
            for (unsigned int iterKde = 0; iterKde < kdeObject.size(); ++iterKde)
            {
                //  std::cout << "***InitEstimationhistogrm for Object " << iterObject
                //      << " Shell " << iterKdeShell->first << "\n";
                //[0] since we only add one shell distribution!

                //  std::cout << "SIZE shell "
                //      << iterKdeShell->second.initEstimationHistogram().size();
                this->objects[iterObject].kdeNormalEstimations.push_back(
                    kdeObject[iterKde].initEstimation()[0]);
            }

            this->objects[iterObject].combinedNormalizedKdeNormalEstimations
            = this->combineShellEstimations(
                  this->objects[iterObject].kdeNormalEstimations);
        }

        this->isNormalMode = true;
    }
    else
    {
        this->logger->log->warn(
            "CObjectDecomposition::initNormalEstimation...Normal estimation is already initialized\n");
    }
}

void CObjectDecomposition::initNodeToCenterEstimation()
{
    this->logger->log->debug(
        "CObjectDecomposition::initNodeToCenterEstimation  !!INDIVIDUAL OPT!!...\n");

    if (!this->isNodeToCenterMode)
    {

        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {

            CDensityEstimation kde;
            kde.addData(this->objects[i].nodeToCenterDistances, this->objects[i].label,
                        this->objects[i].filename);
            this->objects[i].kdeNodeToCenterDistanceEstimations = kde.initEstimation()[0];
            this->objects[i].kdeNodeToCenterDistance = kde;
            //Since we just have a single shell!
            this->isInit = true;
            this->isNodeToCenterMode = true;
        }
    }
    else
    {
        this->logger->log->debug(
            "CObjectDecomposition::initNodeToCenterEstimation...Error Mode! Current Mode is not std mode\n");
    }
}

//New
void CObjectDecomposition::initNormalShellEstimation(int numShell)
{
    this->logger->log->debug(
        "CObjectDecomposition::initNormalShellEstimation...\n");
    if (!this->isNormalShellMode)
    {
        std::vector<std::vector<int> > decomposedNodes;

        this->logger->log->debug(
            "CObjectDecomposition::initNormalShellEstimation...InitEstimationNormalShell....\n");

        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {
            std::map<int, CDensityEstimation> decompX;
            std::map<int, CDensityEstimation> decompY;
            std::map<int, CDensityEstimation> decompZ;

            //  std::cout << "**Object " << iObject << " NumNodes"
            //      << this->objects[iObject].nodePosition.size() << "\n";
            //  std::cout << "**Object " << iObject << " OptimalShellSize \n";
            this->objects[iObject].normalShellConfig = this->optimalShellSize(
                        objects[iObject], numShell);

            //std::cout << "**Object " << iObject << " DecomposedNodes \n";
            this->objects[iObject] = this->decomposeNormalShell(
                                         this->objects[iObject].normalShellConfig, objects[iObject]);

            int shellSize = this->objects[iObject].normalShellConfig.shellSize;

            //node id, xyz, values
            std::map<int, std::map<int, std::vector<double> > >
            nodePointNormals = this->objects[iObject].nodePointNormals;
            int label = this->objects[iObject].label;
            std::string filename = this->objects[iObject].filename;

            //  std::cout << "**Object " << iObject
            //      << " Add DecomposedNodes to KDE densities\n";
            for (int iterDecomp = 0; iterDecomp < shellSize; ++iterDecomp)
            {
                std::map<int, std::vector<double> > allXYZNormalNodesInShell;
                std::vector<int>
                nodesInShell =
                    this->objects[iObject].decomposedNormalShellNodes[iterDecomp];
                std::vector<double>
                nodesInShellWeights =
                    this->objects[iObject].decomposedNormalShellNodesWeights[iterDecomp];

                std::map<int, std::vector<double> >::iterator iterPointNormals;

                for (unsigned int iterNodesInShell = 0; iterNodesInShell
                        < nodesInShell.size(); ++iterNodesInShell)
                {

                    //xyz, normals (all normals close to node)
                    std::map<int, std::vector<double> > currNode =
                        nodePointNormals[nodesInShell[iterNodesInShell]];

                    ///X in shell 0 from point iterNodeInShell
                    std::vector<double> normalsAxis = currNode[0];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        //  std::cout<<"X"<<normalsAxis[iterNormalAxis]<<"\n";
                        allXYZNormalNodesInShell[0].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }

                    ///Y
                    normalsAxis = currNode[1];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        //  std::cout<<"Y"<<normalsAxis[iterNormalAxis]<<"\n";
                        allXYZNormalNodesInShell[1].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }

                    ///Z
                    normalsAxis = currNode[2];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        //  std::cout<<"Z"<<normalsAxis[iterNormalAxis]<<"\n";
                        allXYZNormalNodesInShell[2].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }
                }
                std::string currentShell;
                currentShell = boost::lexical_cast<std::string>(
                                   static_cast<unsigned int>(iterDecomp));
                //if (allApspDistanceOfNodesInShell.size() > 0) {

                //All X normals of ALl nodes which are in shell iterDecomp
                decompX[iterDecomp].addData(allXYZNormalNodesInShell[0], label,
                                            filename + "_NormalShellX" + currentShell);
                decompY[iterDecomp].addData(allXYZNormalNodesInShell[1], label,
                                            filename + "_NormalShellY" + currentShell);
                decompZ[iterDecomp].addData(allXYZNormalNodesInShell[2], label,
                                            filename + "_NormalShellZ" + currentShell);
            }

            this->objects[iObject].kdeNormalXShell = decompX;
            this->objects[iObject].kdeNormalYShell = decompY;
            this->objects[iObject].kdeNormalZShell = decompZ;
        }

    }

    //Now init all initEstimationHistogram of all shells and objects!

    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {

        std::vector<double> combNorm;

        this->logger->log->debug(
            "CObjectDecomposition::initNormalShellEstimation...Init Estimation for Object %d",
            iterObject);
        std::map<int, CDensityEstimation> &kdeObjectX =
            this->objects[iterObject].kdeNormalXShell;
        std::map<int, CDensityEstimation> &kdeObjectY =
            this->objects[iterObject].kdeNormalYShell;
        std::map<int, CDensityEstimation> &kdeObjectZ =
            this->objects[iterObject].kdeNormalZShell;

        //Now iter each shell of an object for init histogram
        for (unsigned int iterKdeShell = 0; iterKdeShell < kdeObjectX.size(); ++iterKdeShell)
        {
            this->logger->log->debug(
                "CObjectDecomposition::initNormalShellEstimation...Normal X %d Shell Estimation...\n",
                iterKdeShell);
            this->objects[iterObject].kdeNormalXEstimations.push_back(
                kdeObjectX[iterKdeShell].initEstimation()[0]);
        }

        for (unsigned int iterKdeShell = 0; iterKdeShell < kdeObjectY.size(); ++iterKdeShell)
        {
            this->logger->log->debug(
                "CObjectDecomposition::initNormalShellEstimation...Normal Y %d Shell Estimation...\n",
                iterKdeShell);
            this->objects[iterObject].kdeNormalYEstimations.push_back(
                kdeObjectY[iterKdeShell].initEstimation()[0]);
        }

        for (unsigned int iterKdeShell = 0; iterKdeShell < kdeObjectZ.size(); ++iterKdeShell)
        {
            this->logger->log->debug(
                "CObjectDecomposition::initNormalShellEstimation...Normal Z %d Shell Estimation...\n",
                iterKdeShell);
            this->objects[iterObject].kdeNormalZEstimations.push_back(
                kdeObjectZ[iterKdeShell].initEstimation()[0]);
        }

        //  check og größe inzeszimazion zid mehr als 1
        //in density estimation sind 13 object drine 1 darfg sind

        //combine all X Normal estimation from all shells
        this->objects[iterObject].combinedNormalXShellEstimation
        = this->combineShellEstimations(
              this->objects[iterObject].kdeNormalXEstimations);

        this->objects[iterObject].combinedNormalYShellEstimation
        = this->combineShellEstimations(
              this->objects[iterObject].kdeNormalYEstimations);

        this->objects[iterObject].combinedNormalZShellEstimation
        = this->combineShellEstimations(
              this->objects[iterObject].kdeNormalZEstimations);

        this->objects[iterObject].combinedNormalShellEstimation.clear();

        combNorm = this->objects[iterObject].combinedNormalXShellEstimation;
        for (unsigned int iterComb = 0; iterComb < combNorm.size(); ++iterComb)
        {
            this->objects[iterObject].combinedNormalShellEstimation.push_back(
                combNorm[iterComb]);
        }

        combNorm = this->objects[iterObject].combinedNormalYShellEstimation;
        for (unsigned int iterComb = 0; iterComb < combNorm.size(); ++iterComb)
        {
            this->objects[iterObject].combinedNormalShellEstimation.push_back(
                combNorm[iterComb]);
        }

        combNorm = this->objects[iterObject].combinedNormalZShellEstimation;
        for (unsigned int iterComb = 0; iterComb < combNorm.size(); ++iterComb)
        {
            this->objects[iterObject].combinedNormalShellEstimation.push_back(
                combNorm[iterComb]);
        }
        /*
         for (unsigned int iiii = 0; iiii
         < this->objects[iterObject].combinedNormalShellEstimation.size(); ++iiii)
         {
         std::cout<<"***"
         << this->objects[iterObject].combinedNormalShellEstimation[iiii]
         << " ";
         }*/

    }

    //Debug combined
    this->logger->log->debug(
        "CObjectDecomposition::initNormalShellEstimation....final combined Histogram  NormalsX(shells) NormalsY(shells) NormalsZ(shells)\n");
    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {
        this->logger->log->debug(
            "CObjectDecomposition::initNormalShellEstimation...Object %d  - Histogram Size %d\n",
            iterObject,
            this->objects[iterObject].combinedNormalShellEstimation.size());
        for (unsigned int iiii = 0; iiii
                < this->objects[iterObject].combinedNormalShellEstimation.size(); ++iiii)
        {
            this->logger->log->debug(
                "%lf ",
                this->objects[iterObject].combinedNormalShellEstimation[iiii]);
        }
        this->logger->log->debug("\n");
    }

    this->isNormalShellMode = true;
    this->isInit = true;
}

/*
 * For all object an common Estimation and binsize etc
 */
void CObjectDecomposition::initNormalEstimationCommon()
{
    this->logger->log->debug(
        "CObjectDecomposition::initNormalEstimation2 !!Common OPT!!...\n");

    if (!this->isNormalMode)
    {

        CDensityEstimation kde[3];
        unsigned int kdeSize = 3;

        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {
            // x y z
            for (unsigned int j = 0; j < this->objects[i].pointNormals.size(); ++j)
            {
                kde[j].addData(this->objects[i].pointNormals[j],
                               this->objects[i].label, this->objects[i].filename);
            }
        }

        //
        for (unsigned int iterKde = 0; iterKde < kdeSize; ++iterKde)
        {
            kde[iterKde].optimizeCommonBinSize();
        }

        //


        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {
            // x y z
            for (unsigned int j = 0; j < this->objects[i].pointNormals.size(); ++j)
            {
                this->objects[i].kdeNormals[j] = kde[j];
            }
        }

        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {

            //  std::cout << "**InitEstimationhistogram KDE for Object " << iterObject
            //      << "\n";
            std::map<int, CDensityEstimation> &kdeObject =
                this->objects[iterObject].kdeNormals;
            //Now iter each shell of an object for init histogram
            for (unsigned int iterKde = 0; iterKde < kdeObject.size(); ++iterKde)
            {
                //  std::cout << "***InitEstimationhistogrm for Object " << iterObject
                //      << " Shell " << iterKdeShell->first << "\n";
                //[0] since we only add one shell distribution!

                //  std::cout << "SIZE shell "
                //      << iterKdeShell->second.initEstimationHistogram().size();
                this->objects[iterObject].kdeNormalEstimations.push_back(
                    kdeObject[iterKde].initEstimation(true)[iterObject]);
            }

            this->objects[iterObject].combinedNormalizedKdeNormalEstimations
            = this->combineShellEstimations(
                  this->objects[iterObject].kdeNormalEstimations);
        }
        this->isInit = true;
        this->isNormalMode = true;
    }
    else
    {
        this->logger->log->debug("Normal estimation is already initialized\n");
    }
}

//init estimation via the entire apsp distance
void CObjectDecomposition::initGeoEstimation()
{
    this->logger->log->debug("CObjectDecomposition::initGeoEstimation....\n");
    if (!this->isGeoMode)
    {
        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {

            CDensityEstimation kde;
            kde.addData(this->objects[i].apspDistances, this->objects[i].label,
                        this->objects[i].filename);
            this->objects[i].kdeEstimations = kde.initEstimation();
            this->objects[i].combinedNormalizedKdeEstimations
            = this->combineShellEstimations(
                  this->objects[i].kdeEstimations);
            //Since we just have a single shell!
            this->objects[i].kdeShell[0] = kde;
            this->isInit = true;
            this->isGeoMode = true;
        }
    }
    else
    {
        this->logger->log->debug(
            "CObjectDecomposition::initGeoEstimation...Error Mode! Current Mode is not std mode\n");
    }
}

//Init estimation via object nodes distance density distribution
void CObjectDecomposition::initNodeEstimation()
{
    this->logger->log->debug("CObjectDecomposition::initEstimationNode....\n");
    std::map<int, std::vector<double> >::iterator iterNodeApspDist;

    if (!this->isGeoNodeMode)
    {

        CDensityEstimation kdeNode;
        std::vector<double> totalApspDistancs;
        if (this->objects.size() > 0)
        {

            //concat all apsp distance of all nodes
            for (unsigned int i = 0; i < this->objects.size(); ++i)
            {

                for (unsigned int j = 0; j
                        < this->objects[i].apspDistances.size(); ++j)
                {
                    totalApspDistancs.push_back(
                        this->objects[i].apspDistances[j]);
                }
            }

            kdeNode.addData(totalApspDistancs, this->objects[0].label,
                            "no-name");
            kdeNode.initEstimation();

            //now take each apsp of a node and estimate the density
            for (unsigned int i = 0; i < this->objects.size(); ++i)
            {
                std::map<int, std::vector<double> > nodeApspDists =
                    this->objects[i].nodeApspDistance;
                this->objects[i].kdeNode = kdeNode;

                std::map<int, std::vector<double> > nodeKdeEstimations;
                for (iterNodeApspDist = nodeApspDists.begin(); iterNodeApspDist
                        != nodeApspDists.end(); ++iterNodeApspDist)
                {

                    this->objects[i].nodeKdeEstimations[iterNodeApspDist->first]
                    = this->objects[i].kdeNode.estimateDensityDistribution(
                          iterNodeApspDist->second)[0]; //[0] since we only add single data -> addData
                }

                this->isInit = true;
                this->isGeoNodeMode = true;
            }
        }
    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::initEstimationNode...Error Mode! Current Mode is shell mode so use initEstimationShell\n");
    }
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateNodeEstimation(
    SObjectDescription &query)
{
    this->logger->log->debug(
        "CObjectDecomposition::estimateNodeEstimation...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    if (this->isGeoNodeMode)
    {
        std::vector<double> estimate;

        // we take the object 0 since all kde node estimates have the same configuration!!
        estimate = this->objects[0].kdeNode.estimateDensityDistribution(
                       query.apspDistances)[0];
        allCombNormHistos[0] = estimate;
    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::estimateNodeEstimation...not init\n");
    }
    return allCombNormHistos;
}

/*
 * estimate bin size
 * create weights
 * generation of db
 */
void CObjectDecomposition::initGeoShellEstimation(int numShell)
{
    this->logger->log->debug(
        "CObjectDecomposition::initEstimationGeoShell....\n");

    if (!this->isGeoShellMode)
    {
        std::vector<std::vector<int> > decomposedNodes;

        this->logger->log->debug(
            "CObjectDecomposition::initEstimationGeoShell...InitEstimationGeoShell....\n");
        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {

            this->logger->log->debug(
                "CObjectDecomposition::initEstimationGeoShell... Object ID: %d, NumNodes %d\n",
                iObject, this->objects[iObject].nodePosition.size());
            //  std::cout << "**Object " << iObject << " OptimalShellSize \n";
            this->objects[iObject].shellConfig = this->optimalShellSize(
                    objects[iObject], numShell);

            //std::cout << "**Object " << iObject << " DecomposedNodes \n";
            this->objects[iObject] = this->decomposeShell(
                                         this->objects[iObject].shellConfig, objects[iObject]);

            int shellSize = this->objects[iObject].shellConfig.shellSize;
            std::map<int, CDensityEstimation> decomp;
            std::map<int, std::vector<double> > nodeApspDistance =
                this->objects[iObject].nodeApspDistance;
            int label = this->objects[iObject].label;
            std::string filename = this->objects[iObject].filename;

            //  std::cout << "**Object " << iObject
            //      << " Add DecomposedNodes to KDE densities\n";
            for (int iterDecomp = 0; iterDecomp < shellSize; ++iterDecomp)
            {

                std::vector<int> nodesInShell =
                    this->objects[iObject].decomposedNodes[iterDecomp];
                std::vector<double>
                nodesInShellWeights =
                    this->objects[iObject].decomposedNodesWeights[iterDecomp];

                this->logger->log->debug(
                    "CObjectDecomposition::initEstimationGeoShell...Num Nodes %d in Shell %d \n",
                    nodesInShell.size(), iterDecomp);
                std::vector<double> allApspDistanceOfNodesInShell;
                for (unsigned int iterNodesInShell = 0; iterNodesInShell
                        < nodesInShell.size(); ++iterNodesInShell)
                {

                    std::vector<double> apspDistNode =
                        nodeApspDistance[nodesInShell[iterNodesInShell]];
                    for (unsigned int iterApspDistNode = 0; iterApspDistNode
                            < apspDistNode.size(); ++iterApspDistNode)
                    {

                        //without weigthing
                        //allApspDistanceOfNodesInShell.push_back(
                        //  apspDistNode[iterApspDistNode]);

                        //with weighting
                        allApspDistanceOfNodesInShell.push_back(
                            apspDistNode[iterApspDistNode]
                            * nodesInShellWeights[iterNodesInShell]);
                    }
                }
                std::string currentShell;
                currentShell = boost::lexical_cast<std::string>(
                                   static_cast<unsigned int>(iterDecomp));
                //if (allApspDistanceOfNodesInShell.size() > 0) {
                decomp[iterDecomp].addData(allApspDistanceOfNodesInShell,
                                           label, filename + "_GeoShell" + currentShell);
                //}
            }
            this->objects[iObject].kdeShell = decomp;
        }

        this->logger->log->debug(
            "CObjectDecomposition::initEstimationGeoShell...Init histogram estimation ...\n");
        //Now init all initEstimationHistogram of all shells and objects!

        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {

            //  std::cout << "**InitEstimationhistogram KDE for Object " << iterObject
            //      << "\n";
            std::map<int, CDensityEstimation> &kdeObject =
                this->objects[iterObject].kdeShell;
            //Now iter each shell of an object for init histogram
            for (unsigned int iterKdeShell = 0; iterKdeShell < kdeObject.size(); ++iterKdeShell)
            {
                //  std::cout << "***InitEstimationhistogrm for Object " << iterObject
                //      << " Shell " << iterKdeShell->first << "\n";
                //[0] since we only add one shell distribution!

                //  std::cout << "SIZE shell "
                //      << iterKdeShell->second.initEstimationHistogram().size();
                this->objects[iterObject].kdeEstimations.push_back(
                    kdeObject[iterKdeShell].initEstimation()[0]);
            }

            this->objects[iterObject].combinedNormalizedKdeEstimations
            = this->combineShellEstimations(
                  this->objects[iterObject].kdeEstimations);
        }

        //Debug combined

        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug(
                "CObjectDecomposition::initEstimationGeoShell...InitEstimationhistogrm comb norm for Object %d -- combinedNormalizedKdeEstimationsSIZE %d \n",
                iterObject,
                this->objects[iterObject].combinedNormalizedKdeEstimations.size());

            for (unsigned int iiii = 0; iiii
                    < this->objects[iterObject].combinedNormalizedKdeEstimations.size(); ++iiii)
            {
                this->logger->log->debug(
                    "%lf ",
                    this->objects[iterObject].combinedNormalizedKdeEstimations[iiii]);
            }
            this->logger->log->debug("\n");
        }

        this->isGeoShellMode = true;
        this->isInit = true;
    }
}

/*
 * estimate bin size
 * create weights
 * generation of db
 */
void CObjectDecomposition::initGeoShellEstimationCommon(int numShell)
{
    this->logger->log->debug(
        "*********CObjectDecomposition::initEstimationShellCommon....!!Common OPT!!\n");

    if (!this->isGeoShellMode)
    {
        std::vector<std::vector<int> > decomposedNodes;

        this->logger->log->debug(
            "CObjectDecomposition::initEstimationShellCommon.... initEstimationGeoShell....\n");

        std::map<int, CDensityEstimation> decomp;
        std::map<int, CDensityEstimation>::iterator iterD;

        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {

            //  std::cout << "**Object " << iObject << " NumNodes"
            //      << this->objects[iObject].nodePosition.size() << "\n";
            //  std::cout << "**Object " << iObject << " OptimalShellSize \n";
            this->objects[iObject].shellConfig = this->optimalShellSize(
                    objects[iObject], numShell);

            //std::cout << "**Object " << iObject << " DecomposedNodes \n";
            this->objects[iObject] = this->decomposeShell(
                                         this->objects[iObject].shellConfig, objects[iObject]);

            int shellSize = this->objects[iObject].shellConfig.shellSize;

            std::map<int, std::vector<double> > nodeApspDistance =
                this->objects[iObject].nodeApspDistance;
            int label = this->objects[iObject].label;
            std::string filename = this->objects[iObject].filename;

            //  std::cout << "**Object " << iObject
            //      << " Add DecomposedNodes to KDE densities\n";
            for (int iterDecomp = 0; iterDecomp < shellSize; ++iterDecomp)
            {

                std::vector<int> nodesInShell =
                    this->objects[iObject].decomposedNodes[iterDecomp];
                std::vector<double>
                nodesInShellWeights =
                    this->objects[iObject].decomposedNodesWeights[iterDecomp];

                //      std::cout << "***Object " << iObject << " Add Shell " << iterDecomp
                //          << " to KDE densities\n";
                std::vector<double> allApspDistanceOfNodesInShell;
                for (unsigned int iterNodesInShell = 0; iterNodesInShell
                        < nodesInShell.size(); ++iterNodesInShell)
                {

                    std::vector<double> apspDistNode =
                        nodeApspDistance[nodesInShell[iterNodesInShell]];
                    for (unsigned int iterApspDistNode = 0; iterApspDistNode
                            < apspDistNode.size(); ++iterApspDistNode)
                    {

                        //without weigthing
                        //allApspDistanceOfNodesInShell.push_back(
                        //  apspDistNode[iterApspDistNode]);

                        //with weighting
                        allApspDistanceOfNodesInShell.push_back(
                            apspDistNode[iterApspDistNode]
                            * nodesInShellWeights[iterNodesInShell]);
                    }
                }
                std::string currentShell;
                currentShell = boost::lexical_cast<std::string>(
                                   static_cast<unsigned int>(iterDecomp));
                //if (allApspDistanceOfNodesInShell.size() > 0) {
                decomp[iterDecomp].addData(allApspDistanceOfNodesInShell,
                                           label, filename + "_GeoShell" + currentShell);
                //}
            }
        }

        //init binsize config: since common we do it once and not for each object again and again in initEstimation()
        for (iterD = decomp.begin(); iterD != decomp.end(); ++iterD)
        {
            iterD->second.optimizeCommonBinSize();
        }

        //Add for each object the same data! for binsize opt -> so we get same bin size;
        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {
            this->objects[iObject].kdeShell = decomp;
        }

        this->logger->log->debug(
            "CObjectDecomposition::initEstimationShellCommon....initEstimationhistogram\n");
        //Now init all initEstimationHistogram of all shells and objects!

        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {

            //  std::cout << "**InitEstimationhistogram KDE for Object " << iterObject
            //      << "\n";
            std::map<int, CDensityEstimation> &kdeObject =
                this->objects[iterObject].kdeShell;
            //Now iter each shell of an object for init histogram
            for (unsigned int iterKdeShell = 0; iterKdeShell < kdeObject.size(); ++iterKdeShell)
            {
                //  std::cout << "***InitEstimationhistogrm for Object " << iterObject
                //      << " Shell " << iterKdeShell->first << "\n";
                //[0] since we only add one shell distribution!

                //  std::cout << "SIZE shell "
                //      << iterKdeShell->second.initEstimationHistogram().size();
                this->objects[iterObject].kdeEstimations.push_back(
                    kdeObject[iterKdeShell].initEstimation(true)[iterObject]);
            }

            this->objects[iterObject].combinedNormalizedKdeEstimations
            = this->combineShellEstimations(
                  this->objects[iterObject].kdeEstimations);
        }

        //Debug combined
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug(
                "CObjectDecomposition::initEstimationShellCommon::InitEstimationhistogram comb norm for Object %d --- combinedNormalizedKdeEstimationsSIZE %d\n",
                iterObject,
                this->objects[iterObject].combinedNormalizedKdeEstimations.size());

            for (unsigned int iiii = 0; iiii
                    < this->objects[iterObject].combinedNormalizedKdeEstimations.size(); ++iiii)
            {
                this->logger->log->debug(
                    "%lf ",
                    this->objects[iterObject].combinedNormalizedKdeEstimations[iiii]);
            }
            this->logger->log->debug("\n");
        }

        this->isGeoShellMode = true;
        this->isInit = true;
    }
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateCombinedGeoShellNormalEstimation(
    SObjectDescription &query)
{
    this->logger->log->debug(
        "*********CObjectDecomposition::estimateCombinedShellNormalEstimation...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    //GeoShell
    std::map<int, std::vector<double> > allCombHistosShell =
        this->estimateGeoShellHistogram(query);

    //Normal
    std::map<int, std::vector<double> > allCombHistosNormal =
        this->estimateNormalHistogram(query);
    //std::cout<<" estimateCombinedGeoShellNormalEstimation uncomment here!!!";

    // this->logger->log->error("CObjectDecomposition::estimateCombinedGeoShellNormalEstimation... Sth wrong...size=%d %d \n ",allCombHistosShell.size(),allCombHistosNormal.size());

    assert(allCombHistosNormal.size() == allCombHistosShell.size());
    if (allCombHistosNormal.size() == allCombHistosShell.size())
    {
        for (unsigned int i = 0; i < allCombHistosNormal.size(); ++i)
        {
            std::vector<std::vector<double> > combinedHist;
            combinedHist.push_back(allCombHistosShell[i]);
            combinedHist.push_back(allCombHistosNormal[i]);

            allCombNormHistos[i] = this->combineShellEstimations(combinedHist);

            ///std::fstream fout1;
            /// fout1.open(std::string(this->objects[i].filename
            ///         + "_queryGeoShellNormal_lastobject.kde").c_str(),
            ///         std::ios::out | std::ios::binary);

            for (unsigned int k = 0; k < allCombNormHistos[i].size(); ++k)
            {
                this->logger->log->debug("%lf ", allCombNormHistos[i][k]);
                ///     fout1 << allCombNormHistos[i][k] << " ";
            }
            this->logger->log->debug("\n");
            /// fout1.close();
        }

        //just for debug or eval purpose to compare distributions...
        //  this->compareCombinedShellNormal(allCombNormHistos);
    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::estimateCombinedGeoShellNormalShellEstimation... could not estimate (histolength!=) \n");
    }

    return allCombNormHistos;
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateCombinedGeoShellNodeToCenterEstimation(
    SObjectDescription &query)
{
    this->logger->log->debug("CObjectDecomposition::estimateCombinedGeoShellNodeToCenterEstimation...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    //GeoShell
    std::map<int, std::vector<double> > allCombHistosShell =
        this->estimateGeoShellHistogram(query);

    //Normal
    std::map<int, std::vector<double> > allCombHistosNodeToCenterDistance =  this->estimateNodeToCenterDistanceHistogram(query);
    //std::cout<<" estimateCombinedGeoShellNormalEstimation uncomment here!!!";

    // this->logger->log->error("CObjectDecomposition::estimateCombinedGeoShellNormalEstimation... Sth wrong...size=%d %d \n ",allCombHistosShell.size(),allCombHistosNormal.size());

    if (allCombHistosNodeToCenterDistance.size() == allCombHistosShell.size())
    {
        for (unsigned int i = 0; i < allCombHistosNodeToCenterDistance.size(); ++i)
        {
            std::vector<std::vector<double> > combinedHist;
            combinedHist.push_back(allCombHistosShell[i]);
            combinedHist.push_back(allCombHistosNodeToCenterDistance[i]);

            allCombNormHistos[i] = this->combineShellEstimations(combinedHist);

            std::fstream fout1;
            /// fout1.open(std::string(this->objects[i].filename
            ///         + "_queryGeoShellNormal_lastobject.kde").c_str(),
            ///         std::ios::out | std::ios::binary);

            for (unsigned int k = 0; k < allCombNormHistos[i].size(); ++k)
            {
                this->logger->log->debug("%lf ", allCombNormHistos[i][k]);
                ///     fout1 << allCombNormHistos[i][k] << " ";
            }
            this->logger->log->debug("\n");
            /// fout1.close();
        }

        //this->compareCombinedShellNormal(allCombNormHistos);
    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::estimateCombinedGeoShellNodeToCenterEstimation... could not estimate (histolength!=) \n");
        assert(allCombHistosNodeToCenterDistance.size() == allCombHistosShell.size());
    }

    return allCombNormHistos;
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateCombinedGeoShellNormalNormalShellEstimation(
    SObjectDescription &query)
{
    this->logger->log->debug(
        "*********CObjectDecomposition::estimateCombinedGeoShellNormalNormalShellEstimation...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    //GeoShell
    std::map<int, std::vector<double> > allCombHistosShell =
        this->estimateGeoShellHistogram(query);

    //Normal
    std::map<int, std::vector<double> > allCombHistosNormal =
        this->estimateNormalHistogram(query);
    //NormalShell
    std::map<int, std::vector<double> > allCombHistosNormalShell =
        this->estimateNormalShellHistogram(query);

    if (allCombHistosNormal.size() == allCombHistosShell.size()
            && allCombHistosShell.size() == allCombHistosNormalShell.size())
    {
        for (unsigned int i = 0; i < allCombHistosNormal.size(); ++i)
        {
            std::vector<std::vector<double> > combinedHist;
            combinedHist.push_back(allCombHistosShell[i]);
            combinedHist.push_back(allCombHistosNormal[i]);
            combinedHist.push_back(allCombHistosNormalShell[i]);

            allCombNormHistos[i] = this->combineShellEstimations(combinedHist);
            /// std::fstream fout1;
            /// fout1.open(std::string(this->objects[i].filename
            ///         + "_queryGeoShellNormalShell_lastobject.kde").c_str(),
            ///         std::ios::out | std::ios::binary);

            for (unsigned int k = 0; k < allCombNormHistos[i].size(); ++k)
            {
                this->logger->log->debug("%lf ", allCombNormHistos[i][k]);
                ///     fout1 << allCombNormHistos[i][k] << " ";
            }
            this->logger->log->debug("\n");
            /// fout1.close();
        }

        this->compareCombinedGeoShellNormalNormalShell(allCombNormHistos);
    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::estimateCombinedGeoShellNormalNormalShellEstimation... could not estimate (histolength!=)\n");
    }

    return allCombNormHistos;
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateNormalHistogram(
    SObjectDescription &query)
{

    this->logger->log->debug(
        "*********CObjectDecomposition::estimateNormalHistogram...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    if (this->isNormalMode)
    {
        std::map<int, std::vector<std::vector<double> > > estimate;

        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {
            this->logger->log->debug(
                "CObjectDecomposition::estimateNormalHistogram...Estimation Normal with Object %d \n",
                i);
            for (unsigned int j = 0; j < this->objects[i].kdeNormals.size(); ++j)
            {
                estimate[i].push_back(
                    this->objects[i].kdeNormals[j].estimateDensityDistribution(
                        query.pointNormals[j])[0]);
            }
        }

        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {

            this->logger->log->debug(
                "CObjectDecomposition::estimateNormalHistogram...Object %d\n",
                i);

            std::vector<std::vector<double> > est = estimate[i];

            std::vector<double> combinedEstimation = combineShellEstimations(
                        est);
            allCombNormHistos[i] = combinedEstimation;

            /// std::fstream fout1;
            /// fout1.open(std::string(this->objects[i].filename
            ///         + "_queryNormal_lastObject.kde").c_str(), std::ios::out
            ///         | std::ios::binary);
            this->logger->log->debug(
                "CObjectDecomposition::estimateNormalHistogram...Query combined Histogram (%d)\n",
                combinedEstimation.size());
            for (unsigned int k = 0; k < combinedEstimation.size(); ++k)
            {
                this->logger->log->debug("%lf ", combinedEstimation[k]);
                ///     fout1 << combinedEstimation[k] << "\n";
            }
            this->logger->log->debug("\n");
            /// fout1.close();
        }

        //this->compareNormals(allCombNormHistos);
    }
    else
    {
        this->logger->log->error("normal mode not init\n");
    }
    return allCombNormHistos;
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateGeoHistogram(
    SObjectDescription &query)
{
    std::map<int, std::vector<double> > estimate;
    this->logger->log->debug("CObjectDecomposition::estimateGeoHistogram...\n");
    if (this->isGeoMode)
    {
        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {
            this->logger->log->debug(
                "Obsolete dont use this estimation use Shell one!\n");
            estimate[i]
            = (this->objects[i].kdeShell[0].estimateDensityDistribution(
                   query.apspDistances)[0]);
        }
        this->compareShell(estimate);
    }
    else
    {
        this->logger->log->debug(
            "CObjectDecomposition::estimateGeoHistogram...Error Mode! Current Mode is shell mode so use estimateHistogramShell\n");
    }

    return estimate;
}

std::map<int, std::vector<double> > CObjectDecomposition::estimateNodeToCenterDistanceHistogram(
    SObjectDescription &query)
{
    std::map<int, std::vector<double> > estimate;
    this->logger->log->debug("CObjectDecomposition::estimateNodeToCenterDistanceHistogram...\n");
    if (this->isNodeToCenterMode)
    {
        for (unsigned int i = 0; i < this->objects.size(); ++i)
        {
            estimate[i]
            = (this->objects[i].kdeNodeToCenterDistance.estimateDensityDistribution(
                   query.nodeToCenterDistances)[0]);
        }
    }
    else
    {
        this->logger->log->debug(
            "CObjectDecomposition::estimateNodeToCenterDistanceHistogram...Error Mode!!!!\n");
    }

    return estimate;
}

//maybe estimate bin size with entire all object number
std::map<int, std::vector<double> > CObjectDecomposition::estimateGeoShellHistogram(
    SObjectDescription &query)
{
    this->logger->log->debug(
        "CObjectDecomposition::estimateGeoShellHistogram...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    if (this->isGeoShellMode)
    {

        //ModelObject 0->  Shell0->hist,  Shell1 -> hist ....
        std::map<int, std::vector<std::vector<double> > > estimatedHistos;
        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {
            this->logger->log->debug(
                "CObjectDecomposition::estimateGeoShellHistogram...Estimation GeoShell with Object = %d \n",
                iObject);
            query = this->decomposeShell(this->objects[iObject].shellConfig,
                                         query);

            int shellSize = this->objects[iObject].shellConfig.shellSize;
            std::map<int, std::vector<double> > nodeApspDistance =
                query.nodeApspDistance;

            for (int iterDecomp = 0; iterDecomp < shellSize; ++iterDecomp)
            {

                std::vector<int> nodesInShell =
                    query.decomposedNodes[iterDecomp];
                std::vector<double> nodesInShellWeights =
                    query.decomposedNodesWeights[iterDecomp];
                std::vector<double> allApspDistanceOfNodesInShell;
                for (unsigned int iterNodesInShell = 0; iterNodesInShell
                        < nodesInShell.size(); ++iterNodesInShell)
                {

                    std::vector<double> apspDistNode =
                        nodeApspDistance[nodesInShell[iterNodesInShell]];
                    for (unsigned int iterApspDistNode = 0; iterApspDistNode
                            < apspDistNode.size(); ++iterApspDistNode)
                    {

                        //without weighting
                        //allApspDistanceOfNodesInShell.push_back(apspDistNode[iterApspDistNode]);

                        //with weighting
                        allApspDistanceOfNodesInShell.push_back(
                            apspDistNode[iterApspDistNode]
                            * nodesInShellWeights[iterNodesInShell]);
                    }
                }

                //std::cout << "*****Shell" << iterDecomp << ":\n";
                //for each object do a vector of shell histos shellSize
                estimatedHistos[iObject].push_back(
                    this->objects[iObject].kdeShell[iterDecomp].estimateDensityDistribution(
                        allApspDistanceOfNodesInShell)[0]); //take first element since there is only a single one

            }
            //  std::cout
            //      << "*****Decomp and estimation histo creation done for object "
            //  << iObject << "\n";
        }

        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {

            this->logger->log->debug(
                "CObjectDecomposition::estimateGeoShellHistogram...Object %d \n",
                iObject);

            std::vector<std::vector<double> > estHistos =
                estimatedHistos[iObject];

            //print all shells of an object
            for (unsigned int j = 0; j < estHistos.size(); ++j)
            {
                this->logger->log->debug(
                    "CObjectDecomposition::estimateGeoShellHistogram...Shell %d :\n",
                    j);
                for (unsigned int k = 0; k < estHistos[j].size(); ++k)
                {
                    this->logger->log->debug("%lf ", estHistos[j][k]);
                }
                this->logger->log->debug("\n");
            }

            //  std::cout << "*****Estimated combined normalizatiedHisto: "
            //      << iterestHist->first << "\n";

            //do the shells in one vector
            std::vector<double> combNormHisto;
            combNormHisto = this->combineShellEstimations(estHistos);
            this->logger->log->debug(
                "CObjectDecomposition::estimateGeoShellHistogram...Query combined Histogram (%d)\n",
                combNormHisto.size());

            /// std::fstream fout1;
            /// fout1.open(std::string(this->objects[iObject].filename
            ///         + "_queryShell_lastobject.kde").c_str(), std::ios::out
            ///         | std::ios::binary);
            //      std::cout << "\n" << "Size combNormHisto" << combNormHisto.size()
            //          << "\n";
            for (unsigned int k = 0; k < combNormHisto.size(); ++k)
            {
                this->logger->log->debug("%lf ", combNormHisto[k]);
                ///     fout1 << combNormHisto[k] << "\n";
            }
            this->logger->log->debug("\n");
            /// fout1.close();

            allCombNormHistos[iObject] = combNormHisto;

        }

        this->logger->log->debug("Final Comparison with objects \n");
        //this->compareShell(allCombNormHistos);

    }
    else
    {
        this->logger->log->error(
            "Error Mode! Current Mode is shell mode so use estimateHistogram\n");
    }

    return allCombNormHistos;
}

//maybe estimate bin size with entire all object number
std::map<int, std::vector<double> > CObjectDecomposition::estimateNormalShellHistogram(
    SObjectDescription &query)
{
    this->logger->log->debug(
        "*********CObjectDecomposition::estimateHistogramNormalShell...\n");
    std::map<int, std::vector<double> > allCombNormHistos;

    if (this->isNormalShellMode)
    {

        //ModelObject 0->  Shell0->hist,  Shell1 -> hist ....
        std::map<int, std::vector<std::vector<double> > > estimatedNormalXHisto;
        std::map<int, std::vector<std::vector<double> > > estimatedNormalYHisto;
        std::map<int, std::vector<std::vector<double> > > estimatedNormalZHisto;
        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {
            this->logger->log->debug(
                "CObjectDecomposition::estimateHistogramNormalShell...Estimation Normal with Object =%d\n",
                iObject);
            query = this->decomposeNormalShell(
                        this->objects[iObject].normalShellConfig, query);

            int shellSize = this->objects[iObject].normalShellConfig.shellSize;

            //id, xyz, normals
            std::map<int, std::map<int, std::vector<double> > >
            nodePointNormals = query.nodePointNormals;

            for (int iterDecomp = 0; iterDecomp < shellSize; ++iterDecomp)
            {
                std::map<int, std::vector<double> > allXYZNormalNodesInShell;
                std::vector<int>
                nodesInShell =
                    this->objects[iObject].decomposedNormalShellNodes[iterDecomp];
                std::vector<double>
                nodesInShellWeights =
                    this->objects[iObject].decomposedNormalShellNodesWeights[iterDecomp];

                std::map<int, std::vector<double> >::iterator iterPointNormals;

                for (unsigned int iterNodesInShell = 0; iterNodesInShell
                        < nodesInShell.size(); ++iterNodesInShell)
                {

                    //xyz, normals (all normals close to node)
                    std::map<int, std::vector<double> > currNode =
                        nodePointNormals[nodesInShell[iterNodesInShell]];

                    ///X
                    std::vector<double> normalsAxis = currNode[0];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        allXYZNormalNodesInShell[0].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }

                    ///Y
                    normalsAxis = currNode[1];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        allXYZNormalNodesInShell[1].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }

                    ///X
                    normalsAxis = currNode[2];
                    for (unsigned int iterNormalAxis = 0; iterNormalAxis
                            < normalsAxis.size(); ++iterNormalAxis)
                    {
                        allXYZNormalNodesInShell[2].push_back(
                            normalsAxis[iterNormalAxis]
                            * nodesInShellWeights[iterNodesInShell]);
                    }
                }

                //std::cout << "*****Shell" << iterDecomp << ":\n";
                //for each object do a vector of shell histos shellSize
                //object, x shell 1, x shell2, x shell3
                estimatedNormalXHisto[iObject].push_back(
                    this->objects[iObject].kdeNormalXShell[iterDecomp].estimateDensityDistribution(
                        allXYZNormalNodesInShell[0])[0]);
                estimatedNormalYHisto[iObject].push_back(
                    this->objects[iObject].kdeNormalYShell[iterDecomp].estimateDensityDistribution(
                        allXYZNormalNodesInShell[1])[0]);
                estimatedNormalZHisto[iObject].push_back(
                    this->objects[iObject].kdeNormalZShell[iterDecomp].estimateDensityDistribution(
                        allXYZNormalNodesInShell[2])[0]);

            }
            //  std::cout
            //      << "*****Decomp and estimation histo creation done for object "
            //  << iObject << "\n";
        }

        for (unsigned int iObject = 0; iObject < this->objects.size(); ++iObject)
        {

            this->logger->log->debug(
                "CObjectDecomposition::estimateNormalShellHistogram...Object %d \n",
                iObject);

            std::vector<std::vector<double> > estXHistos =
                estimatedNormalXHisto[iObject];
            std::vector<std::vector<double> > estYHistos =
                estimatedNormalYHisto[iObject];
            std::vector<std::vector<double> > estZHistos =
                estimatedNormalZHisto[iObject];

            for (unsigned int j = 0; j < estXHistos.size(); ++j)
            {
                this->logger->log->debug(
                    "CObjectDecomposition::estimateNormalShellHistogram...X-Shell %d:\n",
                    j);
                for (unsigned int k = 0; k < estXHistos[j].size(); ++k)
                {
                    this->logger->log->debug("%lf ", estXHistos[j][k]);
                }
                this->logger->log->debug("\n");
            }

            for (unsigned int j = 0; j < estYHistos.size(); ++j)
            {
                this->logger->log->debug(
                    "CObjectDecomposition::estimateNormalShellHistogram...Y-Shell %d:\n",
                    j);
                for (unsigned int k = 0; k < estYHistos[j].size(); ++k)
                {
                    this->logger->log->debug("%lf ", estYHistos[j][k]);
                }
                this->logger->log->debug("\n");
            }

            for (unsigned int j = 0; j < estZHistos.size(); ++j)
            {
                this->logger->log->debug(
                    "CObjectDecomposition::estimateNormalShellHistogram...Z-Shell %d:\n",
                    j);
                for (unsigned int k = 0; k < estZHistos[j].size(); ++k)
                {
                    this->logger->log->debug("%lf ", estZHistos[j][k]);
                }
                this->logger->log->debug("\n");
            }

            //do the shells in one vector
            std::vector<double> combNormHistoX, combNormHistoY, combNormHistoZ;
            combNormHistoX = this->combineShellEstimations(estXHistos);
            combNormHistoY = this->combineShellEstimations(estYHistos);
            combNormHistoZ = this->combineShellEstimations(estZHistos);

            //x1,x2,x_n,y1,y2,yn,z1,....
            std::vector<double> combNormalXYZ;
            for (unsigned int iterComb = 0; iterComb < combNormHistoX.size(); ++iterComb)
            {
                combNormalXYZ.push_back(combNormHistoX[iterComb]);
            }

            for (unsigned int iterComb = 0; iterComb < combNormHistoY.size(); ++iterComb)
            {
                combNormalXYZ.push_back(combNormHistoY[iterComb]);
            }

            for (unsigned int iterComb = 0; iterComb < combNormHistoZ.size(); ++iterComb)
            {
                combNormalXYZ.push_back(combNormHistoZ[iterComb]);
            }

            /// std::fstream fout1;
            /// fout1.open(std::string(this->objects[iObject].filename
            ///         + "_queryNormalShell_lastobject.kde").c_str(),
            ///         std::ios::out | std::ios::binary);
            this->logger->log->debug(
                "CObjectDecomposition::estimateNormalShellHistogram...Combined final NormalXYZ histogram (Size %d)\n",
                combNormalXYZ.size());
            for (unsigned int k = 0; k < combNormalXYZ.size(); ++k)
            {
                this->logger->log->debug("%lf ", combNormalXYZ[k]);
                ///     fout1 << combNormalXYZ[k] << "\n";
            }
            this->logger->log->debug("\n");
            /// fout1.close();

            allCombNormHistos[iObject] = combNormalXYZ;

        }

        this->logger->log->debug(
            "CObjectDecomposition::estimateNormalShellHistogram...Final Comparison with objects \n");
        this->compareNormalShell(allCombNormHistos);

    }
    else
    {
        this->logger->log->error(
            "CObjectDecomposition::estimateNormalShellHistogram...Error Mode! Current Mode is shell mode so use estimateHistogram\n");
    }

    return allCombNormHistos;
}

/*
 float dist(feature_t *F1, feature_t *F2)
 {
 int dX = F1->X - F2->X, dY = F1->Y - F2->Y, dZ = F1->Z - F2->Z;
 return sqrt(dX*dX + dY*dY + dZ*dZ);
 }*/
float dist(feature_t *a, feature_t *b)
{

    std::vector<double> delta;
    double deltaSum = 0;

    assert(a->size() == b->size());
    delta.resize(a->size());
    for (unsigned int m = 0; m < a->size(); m++)
    {
        delta.at(m) = (a->at(m) - b->at(m));
    }

    for (unsigned int m = 0; m < a->size(); m++)
    {

        deltaSum += (pow(delta.at(m), 2));
    }

    return (float) sqrt(deltaSum);
}

void CObjectDecomposition::compareNormalShell(
    std::map<int, std::vector<double> > allCombNormHistos)
{

    this->logger->log->debug("compareNormalShell...\n");

    double euclDistSum = 0;
    double kldDistSum = 0;
    double bhatSum = 0;
    double cosineDistSum = 0;
    double jsDistSum = 0;
    double emDDistSum = 0;
    feature_t f1[1]; //Query
    feature_t f2[1];

    if (this->objects.size() > 0 && this->objects.size()
            == allCombNormHistos.size())
    {
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug("Compare with object %d: \n", iterObject);
            std::vector<double> combNormHisto = allCombNormHistos[iterObject];
            euclDistSum += this->getEuclideanDistance(
                               (this->objects[iterObject].combinedNormalShellEstimation),
                               (combNormHisto));

            bhatSum
            += this->getBhattacharyya(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalShellEstimation));
            kldDistSum
            += this->getKLD(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalShellEstimation));
            cosineDistSum += this->getCosineDistance(
                                 (this->objects[iterObject].combinedNormalShellEstimation),
                                 (combNormHisto));
            jsDistSum += this->getJensenShannonDivergence(this->normalizeHist(
                             this->objects[iterObject].combinedNormalShellEstimation),
                         this->normalizeHist(combNormHisto));

            f1[0] = this->normalizeHist(combNormHisto);
            f2[0] = this->normalizeHist(
                        this->objects[iterObject].combinedNormalShellEstimation);

            float w1[1] =
            { 0.5 }, w2[1] =
            { 0.5 }; // weight should be from the method 1/2^y...
            signature_t s1 =
            { 1, f1, w1 }, s2 =
            { 1, f2, w2 };
            emDDistSum += emd(&s1, &s2, dist, 0, 0);

        }

        this->logger->log->debug("Mean Eucl :\n");

        if (euclDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (euclDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("Mean Bhattacharyya :\n");
        if (bhatSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (bhatSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean KLD NORM!!! :\n");
        if (kldDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (kldDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean Cos :\n");

        if (cosineDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (cosineDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean JS  NORM!!! :\n");

        if (jsDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (jsDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nSingle Mean EMD  NORM!!! :\n");

        if (emDDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (emDDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

    }
    else
    {
        this->logger->log->warn(
            "decomp compare() Sth is wrong with the length or objects==0 \n");
    }
}

void CObjectDecomposition::compareShell(
    std::map<int, std::vector<double> > allCombNormHistos)
{

    this->logger->log->debug("compareShell...\n");

    double euclDistSum = 0;
    double kldDistSum = 0;
    double bhatSum = 0;
    double cosineDistSum = 0;
    double jsDistSum = 0;
    double emDDistSum = 0;
    feature_t f1[1]; //Query
    feature_t f2[1];

    if (this->objects.size() > 0 && this->objects.size()
            == allCombNormHistos.size())
    {
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug("Compare with object %d: \n", iterObject);
            std::vector<double> combNormHisto = allCombNormHistos[iterObject];

            // this->logger->log->error("CObjectDecomposition::compareShell... Sth wrong...size=%d %d \n ",this->objects[iterObject].combinedNormalizedKdeEstimations.size(),combNormHisto.size());
            euclDistSum
            += this->getEuclideanDistance(
                   (this->objects[iterObject].combinedNormalizedKdeEstimations),
                   (combNormHisto));

            bhatSum
            += this->getBhattacharyya(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeEstimations));
            kldDistSum
            += this->getKLD(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeEstimations));
            cosineDistSum += this->getCosineDistance(

                                 (this->objects[iterObject].combinedNormalizedKdeEstimations),
                                 (combNormHisto));
            jsDistSum
            += this->getJensenShannonDivergence(
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeEstimations),
                   this->normalizeHist(combNormHisto));

            f1[0] = this->normalizeHist(combNormHisto);
            f2[0] = this->normalizeHist(
                        this->objects[iterObject].combinedNormalizedKdeEstimations);

            float w1[1] =
            { 0.5 }, w2[1] =
            { 0.5 }; // weight should be from the method 1/2^y...
            signature_t s1 =
            { 1, f1, w1 }, s2 =
            { 1, f2, w2 };
            emDDistSum += emd(&s1, &s2, dist, 0, 0);

        }

        this->logger->log->debug("Mean Eucl :\n");

        if (euclDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (euclDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("Mean Bhattacharyya :\n");
        if (bhatSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (bhatSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean KLD NORM!!! :\n");
        if (kldDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (kldDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean Cos :\n");

        if (cosineDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (cosineDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean JS  NORM!!! :\n");

        if (jsDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (jsDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nSingle Mean EMD  NORM!!! :\n");

        if (emDDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (emDDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

    }
    else
    {
        this->logger->log->warn(
            "decomp compare() Sth is wrong with the length or objects==0 \n");
    }
}

//returns a vector which consists of the nodes ids which are 0 shell, 1 shell , 2 shell ....
SObjectDescription CObjectDecomposition::decomposeShell(
    SShellConfig shellConfig, SObjectDescription object)
{

    std::map<int, std::vector<double> >::iterator iterPos;

    //0 idx = shell0 -> nodes nearest to center, 1 idx= shell1 next to shell0,...
    std::vector<std::vector<int> > decomposedNodes;
    std::vector<std::vector<double> > decomposedNodesWeights;

    double widthShell = shellConfig.shellWidth;
    double minShell = shellConfig.minValue;
    double maxShell = shellConfig.minValue + widthShell;
    int sizeShell = shellConfig.shellSize;
    decomposedNodes.resize(sizeShell);
    decomposedNodesWeights.resize(sizeShell);

    for (unsigned int k = 0; k < (unsigned) sizeShell; k++)
    {

        double distance;
        for (iterPos = object.nodePosition.begin(); iterPos
                != object.nodePosition.end(); ++iterPos)
        {

            distance = this->getEuclideanDistance(object.centroid,
                                                  iterPos->second);
            //          std::cout << "Distance (min=" << minShell << "/max=" << maxShell
            //              << ")=" << distance << "\n";
            if (distance > minShell && distance <= maxShell)
            {
                decomposedNodes[k].push_back(iterPos->first);
                decomposedNodesWeights[k].push_back((1.0 / pow(2, ((double) k
                                                     - 1.0))));
            }
            //for(iterPos = object. )
            //      std::cout << "MIN bin " << minBin << "  ---- " << "MAX bin " << maxBin
            //          << " counts =" << histo[k] << "\n";


        }
        minShell = maxShell;
        maxShell = maxShell + widthShell;
        //      std::cout << k << " Shell owns" << decomposedNodes[k].size()
        //          << " nodes of total " << object.nodePosition.size() << "\n";
    }

    object.decomposedNodes = decomposedNodes;
    object.decomposedNodesWeights = decomposedNodesWeights;
    return object;
}

SObjectDescription CObjectDecomposition::decomposeNormalShell(
    SShellConfig shellConfig, SObjectDescription object)
{

    std::map<int, std::vector<double> >::iterator iterPos;

    //0 idx = shell0 -> nodes nearest to center, 1 idx= shell1 next to shell0,...
    std::vector<std::vector<int> > decomposedNodes;
    std::vector<std::vector<double> > decomposedNodesWeights;

    double widthShell = shellConfig.shellWidth;
    double minShell = shellConfig.minValue;
    double maxShell = shellConfig.minValue + widthShell;
    int sizeShell = shellConfig.shellSize;
    decomposedNodes.resize(sizeShell);
    decomposedNodesWeights.resize(sizeShell);

    for (unsigned int k = 0; k < (unsigned) sizeShell; k++)
    {

        double distance;
        for (iterPos = object.nodePosition.begin(); iterPos
                != object.nodePosition.end(); ++iterPos)
        {

            distance = this->getEuclideanDistance(object.centroid,
                                                  iterPos->second);
            //          std::cout << "Distance (min=" << minShell << "/max=" << maxShell
            //              << ")=" << distance << "\n";
            if (distance > minShell && distance <= maxShell)
            {
                decomposedNodes[k].push_back(iterPos->first);
                decomposedNodesWeights[k].push_back((1.0 / pow(2, ((double) k
                                                     - 1.0))));
            }
            //for(iterPos = object. )
            //      std::cout << "MIN bin " << minBin << "  ---- " << "MAX bin " << maxBin
            //          << " counts =" << histo[k] << "\n";


        }
        minShell = maxShell;
        maxShell = maxShell + widthShell;
        //      std::cout << k << " Shell owns" << decomposedNodes[k].size()
        //          << " nodes of total " << object.nodePosition.size() << "\n";
    }

    object.decomposedNormalShellNodes = decomposedNodes;
    object.decomposedNormalShellNodesWeights = decomposedNodesWeights;
    return object;
}

SShellConfig CObjectDecomposition::optimalShellSize(SObjectDescription object,
        int numShells)
{
    SShellConfig sC;
    sC.shellSize = numShells;

    std::vector<double> distancesToCentroid;
    std::map<int, std::vector<double> >::iterator posIter;
    for (posIter = object.nodePosition.begin(); posIter
            != object.nodePosition.end(); ++posIter)
    {
        distancesToCentroid.push_back(this->getEuclideanDistance(
                                          object.centroid, posIter->second));
    }

    sC.maxValue = *std::max_element(distancesToCentroid.begin(),
                                    distancesToCentroid.end());
    sC.minValue = 0; //*std::min_element(distancesToCentroid.begin(), distancesToCentroid.end());

    //xmin of nodes or centroid which means 0 ?!?!
    sC.shellWidth = (sC.maxValue - sC.minValue) / (double) sC.shellSize;

    this->logger->log->debug(" ShellmaxValue: %lf\n", sC.maxValue);
    this->logger->log->debug(" ShellminValue: %lf\n", sC.minValue);
    this->logger->log->debug(" ShellWidth: %lf\n", sC.shellWidth);

    return sC;
}

double CObjectDecomposition::getEuclideanDistance(std::vector<double> a,
        std::vector<double> b)
{

    std::vector<double> delta;
    double deltaSum = 0;

    assert(a.size() == b.size());
    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (unsigned int m = 0; m < a.size(); m++)
        {
            delta.at(m) = (a.at(m) - b.at(m));
        }

        for (unsigned int m = 0; m < a.size(); m++)
        {

            deltaSum += (pow(delta.at(m), 2));
        }

        return sqrt(deltaSum);
    }
    else
        return -1;
}

std::vector<double> CObjectDecomposition::combineShellEstimations(std::vector <
        std::vector<double> > shellEstimations)
{
    std::vector<double> combinedNormalizedEstimation;
    for (unsigned int iter = 0; iter < shellEstimations.size(); ++iter)
    {
        for (unsigned int iterHisto = 0; iterHisto
                < shellEstimations[iter].size(); ++iterHisto)
        {
            combinedNormalizedEstimation.push_back(
                shellEstimations[iter][iterHisto]);
        }
    }

    //normalize ...
    //combinedNormalizedEstimation = this->normalizeHist(
    //  combinedNormalizedEstimation);

    return combinedNormalizedEstimation;
}

std::vector<double> CObjectDecomposition::normalizeHist(
    std::vector<double> histo)
{
    //Normalize histogram to 1 !
    if (histo.size() > 0)
    {
        double sum = 0;
        for (unsigned k = 0; k < (unsigned) histo.size(); k++)
        {
            sum += histo[k];
        }

        if (sum > 0)
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = (double) histo[k] / (double) sum;
            }
        }
        else
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = 0;
            }
        }
    }
    else
    {
        histo.clear();
        histo.push_back(0);
    }

    return histo;
}

double CObjectDecomposition::getKLD(std::vector<double> a,
                                    std::vector<double> b)
{
    double kld = 0;

    for (unsigned int m = 0; m < a.size(); m++)
    {
        if (a[m] == 0)
        {
            a[m] = std::numeric_limits<float>::epsilon();
        }
        if (b[m] == 0)
        {
            b[m] = std::numeric_limits<float>::epsilon();
        }

        kld += a[m] * this->log_2((a[m] / b[m]));
    }

    return kld;
}

double CObjectDecomposition::getJensenShannonDivergence(std::vector<double> a,
        std::vector<double> b)
{

    std::vector<double> delta;

    assert(a.size() == b.size());
    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (unsigned int m = 0; m < a.size(); m++)
        {
            delta[m] = ((a[m] + b[m]) / 2.0f);
        }

        double firstTerm = this->diffEntropy(delta);
        double aVal, bVal;

        aVal = this->diffEntropy(a);

        bVal = this->diffEntropy(b);

        //std::cout << "aVal " << aVal << "          " << "bVal " << bVal << "\n";
        double secondTerm = (aVal + bVal) / 2.0f;

        return (firstTerm - secondTerm);
    }
    else
        return -1;
}

double CObjectDecomposition::diffEntropy(std::vector<double> a)
{
    double entropy = 0;
    for (unsigned int m = 0; m < a.size(); m++)
    {
        double res;
        res = a[m] * log_2(a[m]);
        //std::cout<<"entropy"<< a[m]<<" "<< entropy<<"\n";
        if (res != res) // check for nan
        {
            //std::cout<<" Y "<<std::endl;
            res = 0;
        }
        entropy += res;
    }

    return (entropy * (-1));
}

double CObjectDecomposition::getBhattacharyya(std::vector<double> a,
        std::vector<double> b)
{
    double deltaSum = 0;

    assert(a.size() == b.size());
    if (a.size() == b.size())
    {

        for (unsigned int m = 0; m < a.size(); m++)
        {
            deltaSum += sqrt(a[m] * b[m]);
        }

        return (-1 * log(deltaSum));
    }
    else
    {
        std::cout << "getBhattacharyya() : Sth is wrong a!=b vector size()\n";
        return -1;
    }
}

double CObjectDecomposition::getCosineDistance(std::vector<double> a,
        std::vector<double> b)
{

    double nominator = 0.0f;
    double aProduct = 0.0f;
    double bProduct = 0.0f;
    double denominator = 0.0f;

    assert(a.size() == b.size());
    if (a.size() == b.size())
    {
        for (unsigned int m = 0; m < a.size(); m++)
        {
            nominator += (a[m] * b[m]);
        }

        for (unsigned int m = 0; m < a.size(); m++)
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
        this->logger->log->warn(
            "Something is wrong with the vector length %d != %d\n",
            a.size(), b.size());
        return -1;
    }
}

double CObjectDecomposition::log_2(double n)
{
    // log(n)/log(2) is log2.
    return log((double) n) / log((double) 2);
}

void CObjectDecomposition::compareNormals(
    std::map<int, std::vector<double> > allCombNormHistos)
{
    this->logger->log->debug("compareNormals...\n");
    double euclDistSum = 0;
    double bhatSum = 0;
    double kldDistSum = 0;
    double cosineDistSum = 0;
    double jsDistSum = 0;
    double emDDistSum = 0;
    feature_t f1[1]; //Query
    feature_t f2[1];

    if (this->objects.size() > 0 && this->objects.size()
            == allCombNormHistos.size())
    {
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug("Compare with object %d:\n", iterObject);
            std::vector<double> combNormHisto = allCombNormHistos[iterObject];
            euclDistSum
            += this->getEuclideanDistance(
                   (this->objects[iterObject].combinedNormalizedKdeNormalEstimations),
                   (combNormHisto));

            bhatSum
            += this->getBhattacharyya(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeNormalEstimations));
            kldDistSum
            += this->getKLD(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeNormalEstimations));
            cosineDistSum += this->getCosineDistance(

                                 (this->objects[iterObject].combinedNormalizedKdeNormalEstimations),
                                 (combNormHisto));
            jsDistSum
            += this->getJensenShannonDivergence(
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeNormalEstimations),
                   this->normalizeHist(combNormHisto));

            f1[0] = this->normalizeHist(combNormHisto);
            f2[0]
            = this->normalizeHist(
                  this->objects[iterObject].combinedNormalizedKdeNormalEstimations);

            float w1[1] =
            { 0.5 }, w2[1] =
            { 0.5 }; // weight should be from the method 1/2^y...
            signature_t s1 =
            { 1, f1, w1 }, s2 =
            { 1, f2, w2 };
            emDDistSum += emd(&s1, &s2, dist, 0, 0);

        }

        this->logger->log->debug("Mean Eucl :\n");

        if (euclDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (euclDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("Mean Bhattacharyya :\n");
        if (bhatSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (bhatSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean KLD NORM!!! :\n");
        if (kldDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (kldDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean Cos :\n");

        if (cosineDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (cosineDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean JS  NORM!!! :\n");

        if (jsDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (jsDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nSingle Mean EMD  NORM!!! :\n");

        if (emDDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (emDDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

    }
    else
    {
        this->logger->log->warn(
            "decomp compare() Sth is wrong with the length or objects==0 \n");
    }
}

void CObjectDecomposition::compareCombinedShellNormal(std::map < int,
        std::vector<double> > allCombNormHistos)
{
    this->logger->log->debug("compareCombinedShellNormal...\n");
    double euclDistSum = 0;
    double kldDistSum = 0;
    double bhatSum = 0;
    double cosineDistSum = 0;
    double jsDistSum = 0;
    double emDDistSum = 0;
    feature_t f1[1]; //Query
    feature_t f2[1];

    if (this->objects.size() > 0 && this->objects.size()
            == allCombNormHistos.size())
    {
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug("Compare with object %d : \n", iterObject);
            std::vector<double> combNormHisto = allCombNormHistos[iterObject];
            euclDistSum
            += this->getEuclideanDistance(
                   (this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations),
                   (combNormHisto));
            kldDistSum
            += this->getKLD(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations));

            bhatSum
            += this->getBhattacharyya(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations));
            cosineDistSum
            += this->getCosineDistance(

                   (this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations),
                   (combNormHisto));
            jsDistSum
            += this->getJensenShannonDivergence(
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations),
                   this->normalizeHist(combNormHisto));

            f1[0] = this->normalizeHist(combNormHisto);
            f2[0]
            = this->normalizeHist(
                  this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations);

            float w1[1] =
            { 0.5 }, w2[1] =
            { 0.5 }; // weight should be from the method 1/2^y...
            signature_t s1 =
            { 1, f1, w1 }, s2 =
            { 1, f2, w2 };
            emDDistSum += emd(&s1, &s2, dist, 0, 0);

        }

        this->logger->log->debug("Mean Eucl :\n");

        if (euclDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (euclDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("Mean Bhattacharyya :\n");
        if (bhatSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (bhatSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean KLD NORM!!! :\n");
        if (kldDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (kldDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean Cos :\n");

        if (cosineDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (cosineDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean JS  NORM!!! :\n");

        if (jsDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (jsDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nSingle Mean EMD  NORM!!! :\n");

        if (emDDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (emDDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }
    }
    else
    {
        this->logger->log->warn(
            "decomp compare() Sth is wrong with the length or objects==0 \n");
    }
}

void CObjectDecomposition::compareCombinedGeoShellNormalNormalShell(std::map <
        int, std::vector<double> > allCombNormHistos)
{
    this->logger->log->debug(
        "CObjectDecomposition::compareCombinedGeoShellNormalNormalShell...\n");
    double euclDistSum = 0;
    double kldDistSum = 0;
    double bhatSum = 0;
    double cosineDistSum = 0;
    double jsDistSum = 0;
    double emDDistSum = 0;
    feature_t f1[1]; //Query
    feature_t f2[1];

    if (this->objects.size() > 0 && this->objects.size()
            == allCombNormHistos.size())
    {
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            this->logger->log->debug("Compare with object %d: \n", iterObject);
            std::vector<double> combNormHisto = allCombNormHistos[iterObject];
            euclDistSum
            += this->getEuclideanDistance(
                   (this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations),
                   (combNormHisto));
            kldDistSum
            += this->getKLD(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations));

            bhatSum
            += this->getBhattacharyya(
                   this->normalizeHist(combNormHisto),
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations));
            cosineDistSum
            += this->getCosineDistance(

                   (this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations),
                   (combNormHisto));
            jsDistSum
            += this->getJensenShannonDivergence(
                   this->normalizeHist(
                       this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations),
                   this->normalizeHist(combNormHisto));

            f1[0] = this->normalizeHist(combNormHisto);
            f2[0]
            = this->normalizeHist(
                  this->objects[iterObject].combinedNormalizedGeoShellNormalNormalShellEstimations);

            float w1[1] =
            { 0.5 }, w2[1] =
            { 0.5 }; // weight should be from the method 1/2^y...
            signature_t s1 =
            { 1, f1, w1 }, s2 =
            { 1, f2, w2 };
            emDDistSum += emd(&s1, &s2, dist, 0, 0);

        }

        this->logger->log->debug("Mean Eucl :\n");

        if (euclDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (euclDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("Mean Bhattacharyya :\n");
        if (bhatSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (bhatSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean KLD NORM!!! :\n");
        if (kldDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (kldDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean Cos :\n");

        if (cosineDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (cosineDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nMean JS  NORM!!! :\n");

        if (jsDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (jsDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        this->logger->log->debug("\nSingle Mean EMD  NORM!!! :\n");

        if (emDDistSum > 0)
        {
            this->logger->log->debug(" Label: %d => %lf \n",
                                     this->objects[0].label, (emDDistSum
                                             / (double) this->objects.size()));
        }
        else
        {
            this->logger->log->warn(" Label: %d => -1 \n",
                                    this->objects[0].label);
        }

        //this->compareEMDCommon(allCombNormHistos);
        //Do combined EMD
        ///query
        /*for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject) {
         std::vector<double> combNormHisto = allCombNormHistos[iterObject];
         feature_t queryFeat[1]; //Q
         queryFeat[0] = this->normalizeHist(combNormHisto);
         float weightsQuery[1] = { 0.5 };

         ///sources
         feature_t* sourcesFeat = NULL;
         float *weightsSources = NULL;
         sourcesFeat = new feature_t[this->objects.size()];
         weightsSources = new float[this->objects.size()];
         for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject) {
         sourcesFeat[iterObject]
         = this->normalizeHist(
         this->objects[iterObject].combinedNormalizedKdeEstimations);
         weightsSources[iterObject] = 0.5f;
         }

         signature_t querySig = { 1, queryFeat, weightsQuery };

         signature_t sourcesSig = { this->objects.size(), sourcesFeat,
         weightsSources };

         std::cout << " Augmented EMD:" << this->objects[0].label << " =>"
         << emd(&querySig, &sourcesSig, dist, 0, 0) << "\n";

         if (sourcesFeat != NULL) {
         delete[] sourcesFeat;
         }
         if (weightsSources != NULL) {
         delete[] weightsSources;
         }
         }*/
        /*std::cout << "EMD: " << std::endl;
         feature_t f1[4] = { { 100, 40, 22 }, { 211, 20, 2 }, { 32, 190, 150 }, { 2,
         100, 100 } }, f2[3] = { { 0, 0, 0 }, { 50, 100, 80 }, { 255, 255,
         255 } };
         float w1[5] = { 0.4, 0.3, 0.2, 0.1 }, w2[3] = { 0.5, 0.3, 0.2 };
         signature_t s1 = { 4, f1, w1 }, s2 = { 3, f2, w2 };
         float e;

         e = emd(&s1, &s2, dist, 0, 0);/

         printf("emd=%f\n", e);*/
    }
    else
    {
        this->logger->log->warn(
            "decomp compare() Sth is wrong with the length or objects==0 \n");
    }
}

void CObjectDecomposition::compareEMDCommon(
    std::map<int, std::vector<double> > allCombNormHistos)
{
    this->logger->log->debug("compareEMDCommon()...!!just combined ion use!!\n");
    for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
    {
        std::vector<double> combNormHisto = allCombNormHistos[iterObject];
        feature_t queryFeat[1]; //Q
        queryFeat[0] = this->normalizeHist(combNormHisto);
        float weightsQuery[1] =
        { 1.0f };

        ///sources
        feature_t* sourcesFeat = NULL;
        float *weightsSources = NULL;
        sourcesFeat = new feature_t[this->objects.size()];
        weightsSources = new float[this->objects.size()];
        for (unsigned int iterObject = 0; iterObject < this->objects.size(); ++iterObject)
        {
            sourcesFeat[iterObject]
            = this->normalizeHist(
                  this->objects[iterObject].combinedNormalizedKdeShellNormalEstimations);
            weightsSources[iterObject] = 1.0f;
        }

        signature_t querySig =
        { 1, queryFeat, weightsQuery };

        signature_t sourcesSig =
        { this->objects.size(), sourcesFeat, weightsSources };

        this->logger->log->debug(" Augmented EMD: %d => %lf \n",
                                 this->objects[0].label, emd(&querySig, &sourcesSig, dist, 0, 0));

        if (sourcesFeat != NULL)
        {
            delete[] sourcesFeat;
        }
        if (weightsSources != NULL)
        {
            delete[] weightsSources;
        }
    }

}

CObjectDecomposition::~CObjectDecomposition()
{
    // TODO Auto-generated destructor stub
}
