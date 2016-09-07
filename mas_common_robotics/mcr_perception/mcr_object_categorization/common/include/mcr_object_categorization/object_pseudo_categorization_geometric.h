/*
 *  CObjectPseudoCategorizationGeometric.h
 *
 *  Created on: 21.06.2011
 *      Author: Christian Mueller
 */

#ifndef COBJECTPSEUDOCATEGORIZATIONGEOMETRIC_H
#define COBJECTPSEUDOCATEGORIZATIONGEOMETRIC_H

#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <string>
#include <limits>

#include "toolbox_ros.h"
#include "logger.h"
#include "object_geometric_service.h"

#define OBJ_PSEUDO_CAT_GEO_ADDNOISE 0.1
#define OBJ_PSEUDO_CAT_GEO_NUM_ADDNOISE 3

class CObjectPseudoCategorizationGeometric
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    CLogger *logger;
protected:
    CToolBoxROS toolBox;
    CObjectGeometricService objectGeometricService;
    int geometricSrvEstimationType;
    bool geometricSrvAddNormalEstimation;

    //GeometricShapeReconstructionParameters
    int estimationType;
    float ng_consistDist;
    float ng_errorAdd;
    float ng_nodeAddInterval; // ng_errorAdd == -1
    float ng_maxEdgeAge;
    float ng_modelAccuracy;
    float ng_errorNewNodeDecrease;
    float ng_errorGeneralDecrease;
    float ng_nearestNeighborWeight; //move the nearest node to target
    float ng_nNeighborWeight; //move other neigbors to
    //////////////////////////////////////////

    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds;
    std::vector<int> objectLabels;
    std::vector<std::string> objectNames;
    bool isModelLoaded;
    bool doTrainEnNoisement;
    bool doTrainScalePyramid;
    int numCategories;
    std::string homePath;

    float addEnNoisement;
public:
    CObjectPseudoCategorizationGeometric();
    void inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, int label);
    void inputObjectPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames);
    void inputObjectPointClouds(std::string dataBasePath, int maxNumObjectPerClass, bool concat = false);

    void loadModel();
    void initGeometricShapeReconstructionParameters();

    void trainPseudoCategories();
    std::pair<int, double> queryPseudoCategories(pcl::PointCloud<pcl::PointXYZ> queryPointCloud, std::map<int, std::pair<int, double> > &getCurrentTreeReponses);
    std::vector<std::pair<int, double> >queryPseudoCategoriesEnNoised(pcl::PointCloud<pcl::PointXYZ> queryPointCloud, std::vector<std::map<int, std::pair<int, double> > > &getCurrentTreeReponses);

    int getNumCategories()
    {
        return numCategories;
    }
    void setHomePath(std::string homePath);

    void augmentObjectPointCloudsWithEnNoisement();
    void augmentObjectPointCloudsWithScalePyramid();
    void setTrainEnNoisement(bool doTrainnNoisement)
    {
        this->doTrainEnNoisement = doTrainEnNoisement;
    }
    void setTrainScalePyramid(bool doTrainScalePyramid)
    {
        this->doTrainScalePyramid = doTrainScalePyramid;
    }
};

#endif


