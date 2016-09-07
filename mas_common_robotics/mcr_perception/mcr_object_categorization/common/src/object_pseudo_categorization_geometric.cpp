/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <assert.h>

#include "object_pseudo_categorization_geometric.h"

CObjectPseudoCategorizationGeometric::CObjectPseudoCategorizationGeometric()
{
    this->logger = &CLogger::getInstance();
    this->isModelLoaded = false;
    this->homePath = HOME_PATH;
    this->addEnNoisement = OBJ_PSEUDO_CAT_GEO_ADDNOISE;
    this->doTrainEnNoisement = false;
    this->doTrainScalePyramid = true; //false;
}

void CObjectPseudoCategorizationGeometric::setHomePath(std::string homePath)
{
    this->homePath = homePath;
    this->objectGeometricService.setHomePath(this->homePath);
}

void CObjectPseudoCategorizationGeometric::inputObjectPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames)
{
    this->objectPointClouds = objectPointClouds;
    this->objectLabels = labels;
    this->objectNames = objectNames;
}

void CObjectPseudoCategorizationGeometric::inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, int label)
{
    std::cout << "CObjectCategorizationGeometric::inputObjectPointCloud... not implemented\n";
}

void CObjectPseudoCategorizationGeometric::loadModel()
{
    this->initGeometricShapeReconstructionParameters();
    this->objectGeometricService.setHomePath(this->homePath);
    this->objectGeometricService.loadCollection(); //Load all stuff Objectdecompositions etc (maybe required if further analysis is required)
    //this->objectGeometricService.loadRTPNNEnsemble();//Load just Forest, now only the trained model of the forest is loaded! so only query() is possible
    this->numCategories = this->objectGeometricService.getNumCategories();
    this->isModelLoaded = true;
}

void CObjectPseudoCategorizationGeometric::initGeometricShapeReconstructionParameters()
{

    //STD SETTINGS!!!
    /*  bool normalize = true;

     this->estimationType = 10;//10;//10; //10 //20
     //1 GeoNormal
     //10 GeoShell + Normal
     //15 GeoShell + NodeToCenterDistance
     //20 GeoShell + Normal + NormalShell
     //30 GeoNode


     //0.71 correct shell3
     //0.75 shell 10

     //0.2 0.904
     this->ng_modelAccuracy = -1;
     this->ng_consistDist = 0.15; //0.3 0.90
     this->ng_errorAdd = 5;

     this->ng_nodeAddInterval = -1;

     this->ng_maxEdgeAge = 200;

     this->ng_errorNewNodeDecrease = 0.5f;
     this->ng_errorGeneralDecrease = 0.995f;

     this->ng_nearestNeighborWeight = 0.2;
     this->ng_nNeighborWeight = 0.006;

     this->objectGeometricService = CObjectGeometricService(this->estimationType);
     this->objectGeometricService.setParametersShapeRecontructor(this->ng_consistDist, this->ng_errorAdd, this->ng_nodeAddInterval, this->ng_maxEdgeAge, this->ng_errorNewNodeDecrease,
     this->ng_errorGeneralDecrease, this->ng_nearestNeighborWeight, this->ng_nNeighborWeight, normalize, this->ng_modelAccuracy);
     this->objectGeometricService.setHomePath(this->homePath);*/

    bool normalize = true;

    this->estimationType = 10;//10;//10; //10 //20
    //1 GeoNormal
    //10 GeoShell + Normal
    //15 GeoShell + NodeToCenterDistance
    //20 GeoShell + Normal + NormalShell
    //30 GeoNode


    //0.71 correct shell3
    //0.75 shell 10

    //0.2 0.904
    this->ng_modelAccuracy = -1;
    this->ng_consistDist = 0.15; //0.3 0.90
    this->ng_errorAdd = 5;

    this->ng_nodeAddInterval = -1;

    this->ng_maxEdgeAge = 200;

    this->ng_errorNewNodeDecrease = 0.5f;
    this->ng_errorGeneralDecrease = 0.995f;

    this->ng_nearestNeighborWeight = 0.2;
    this->ng_nNeighborWeight = 0.006;

    this->objectGeometricService = CObjectGeometricService(this->estimationType);
    this->objectGeometricService.setParametersShapeRecontructor(this->ng_consistDist, this->ng_errorAdd, this->ng_nodeAddInterval, this->ng_maxEdgeAge, this->ng_errorNewNodeDecrease,
            this->ng_errorGeneralDecrease, this->ng_nearestNeighborWeight, this->ng_nNeighborWeight, normalize, this->ng_modelAccuracy);
    this->objectGeometricService.setHomePath(this->homePath);
}

void CObjectPseudoCategorizationGeometric::inputObjectPointClouds(std::string dataPath, int maxNumObjectPerClass, bool concat)
{
    if (concat == false)
    {
        this->objectPointClouds.clear();
        this->objectLabels.clear();
        this->objectNames.clear();
    }

    DIR *dp;
    struct dirent *dirp;
    pcl::PointCloud<pcl::PointXYZ> pointCloudPCD;

    //We start with first Catgory.... 0 = negative
    for (unsigned int iterCategory = 1; iterCategory < NUM_CLASSES; ++iterCategory)
    {
        int currNumObject = 1;
        std::string pointCloudPath = std::string(dataPath + CFileSettings::trainPath[iterCategory] + POINT_CLOUD_PATH);
        string fileName = string("object");

        //OpenDirectory
        std::cout << "Trying to open folder " << pointCloudPath << "... ";
        if ((dp = opendir(pointCloudPath.c_str())) == NULL)
        {
            //std::cout << "Error(" << errno << ") opening " << pointCloudPath << std::endl;
            //exit(0);
            int cont;
            std::cout << "Error(" << errno << ") opening " << pointCloudPath << std::endl;
            std::cout << "OR this category is not available in this source ?! Press any key to continue" << pointCloudPath << std::endl;
            std::cin >> cont;
            continue;

        }
        std::cout << "succeeded\n";

        while ((dirp = readdir(dp)) != NULL)
        {
            std::string file = string(dirp->d_name);
            if (file.compare(std::string(".")) != 0 && file.compare(std::string("..")) != 0)
            {
                if (currNumObject > maxNumObjectPerClass)
                    break;

                std::vector < std::string > fileTokens;
                CFileSettings::tokenize(file, fileTokens, std::string("_"));

                //Check whether it is a correct pcd File
                assert(boost::lexical_cast<unsigned int>(fileTokens[0]) == iterCategory);
                assert(fileTokens[3] == POINT_CLOUD_POSTIFX);

                if (pcl::io::loadPCDFile(std::string(pointCloudPath + file), pointCloudPCD) == -1)
                {
                    ROS_ERROR_STREAM("Was not able to open file \"" << file << "\".\n");
                    exit(0);
                }
                else //Successfully loaded pcd
                {
                    std::cout << "Point cloud file (" << file << ") successfully opened - point cloud size = (" << pointCloudPCD.size() << ")\n";

                    int objectLabel = boost::lexical_cast<unsigned int>(fileTokens[0]);
                    this->objectPointClouds.push_back(pointCloudPCD);
                    this->objectLabels.push_back(objectLabel);
                    this->objectNames.push_back(std::string(fileTokens[2] + "-" + fileTokens[0] + "-" + fileTokens[1]));
                }
                currNumObject++;
            }

        }
    }

    std::cout << "TotalNumber: " << objectPointClouds.size() << "\n";
}

void CObjectPseudoCategorizationGeometric::trainPseudoCategories()
{
    this->logger->log->info("CObjectPseudoCategorizationGeometric::trainPseudoCategories...start training");
    assert(this->objectPointClouds.size() > 0 && this->objectLabels.size() > 0);
    assert(this->objectPointClouds.size() == this->objectLabels.size());

    if (this->doTrainEnNoisement)
    {
        this->logger->log->info("CObjectPseudoCategorizationGeometric::trainPseudoCategories...do enNoisement");
        this->augmentObjectPointCloudsWithEnNoisement();
    }

    if (this->doTrainScalePyramid)
    {
        this->logger->log->info("CObjectPseudoCategorizationGeometric::trainPseudoCategories...do ScalePyramid");
        this->augmentObjectPointCloudsWithScalePyramid();
    }

    this->initGeometricShapeReconstructionParameters();

    for (unsigned int iObject = 0; iObject < this->objectPointClouds.size(); ++iObject)
    {
        this->objectGeometricService.inputObjectPointCloud(this->objectPointClouds[iObject], this->objectLabels[iObject], this->objectNames[iObject]);
        this->objectGeometricService.reconstructShape(true);
    }

    int initEst;
    std::cout.flush();
    std::cout << "DO initEstimation (0/1)?YES";
    std::cout.flush();
    //cin >> initEst;
    initEst = 1;
    //cin >> initEst;

    if (initEst == 1)
    {
        std::cout << "INIT ESTIMATATE" << std::endl;
        int isCommon = 0;
        std::cout << "isCommon 0/1?: NO";
        //cin >> isCommon;
        isCommon = 0;
        this->objectGeometricService.setIsCommonHistogrammCfg(isCommon);
        this->objectGeometricService.initEstimation();

        int saveEst = 1;
        //  std::cout << "SAVE?: NO ";
        //  cin >> saveEst;
        std::cout << "SAVING MODELS ...";
        if (saveEst == 1)
        {
            this->objectGeometricService.saveCollection();
        }
    }
    this->numCategories = this->objectGeometricService.getNumCategories();
    this->isModelLoaded = true;
}

std::pair<int, double> CObjectPseudoCategorizationGeometric::queryPseudoCategories(pcl::PointCloud<pcl::PointXYZ> queryPointCloud, std::map<int, std::pair<int, double> > &getCurrentTreeReponses)
{
    std::pair<int, double> res;

    if (this->isModelLoaded)
    {
        std::cout << "CObjectPseudoCategorizationGeometric::query...start..." << std::endl;
        clock_t begin_time = clock();
        res = this->objectGeometricService.query(queryPointCloud, getCurrentTreeReponses);
        std::cout << "CObjectPseudoCategorizationGeometric::query... done in " << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
    }
    else
    {
        assert(this->isModelLoaded == true);
    }
    return res;
}

std::vector<std::pair<int, double> > CObjectPseudoCategorizationGeometric::queryPseudoCategoriesEnNoised(pcl::PointCloud<pcl::PointXYZ> queryPointCloud,
        std::vector<std::map<int, std::pair<int, double> > > &getCurrentTreeReponsesEnNoised)
{
    std::vector < std::pair<int, double> > resEnNoised;

    if (this->isModelLoaded)
    {

        resEnNoised = this->objectGeometricService.queryEnNoised(queryPointCloud, getCurrentTreeReponsesEnNoised, 7);
    }
    else
    {
        assert(this->isModelLoaded == true);
    }

    return resEnNoised;

}

void CObjectPseudoCategorizationGeometric::augmentObjectPointCloudsWithEnNoisement()
{
    assert(this->objectPointClouds.size() > 0);
    unsigned int numObjectToAdd = OBJ_PSEUDO_CAT_GEO_NUM_ADDNOISE;

    unsigned int numObjectPointClouds = this->objectPointClouds.size();
    for (unsigned int iterPointCloud = 0; iterPointCloud < numObjectPointClouds; ++iterPointCloud)
    {
        for (unsigned int iterToAdd = 0; iterToAdd < numObjectToAdd; ++iterToAdd)
        {
            pcl::PointCloud<pcl::PointXYZ> newPointCloud = this->objectPointClouds[iterPointCloud];
            //Do EnNoisement....
            toolBox.enNoisePointCloud(newPointCloud, this->addEnNoisement);
            //Add to set....
            this->objectPointClouds.push_back(newPointCloud);
            this->objectLabels.push_back(this->objectLabels[iterPointCloud]);
            this->objectNames.push_back(std::string(this->objectNames[iterPointCloud] + "_EnNoised_" + boost::lexical_cast<std::string>(iterToAdd)));
        }
    }
}

void CObjectPseudoCategorizationGeometric::augmentObjectPointCloudsWithScalePyramid()
{
    assert(this->objectPointClouds.size() > 0);
    float minLeafSize = 0.01;
    float maxLeafSize = 0.5;
    int numStep = 3; // number of points clouds to add == number of subsampling steps;

    float step = ((maxLeafSize - minLeafSize) / double(numStep));

    unsigned int numObjectPointClouds = this->objectPointClouds.size();
    for (unsigned int iterPointCloud = 0; iterPointCloud < numObjectPointClouds; ++iterPointCloud)
    {
        std::cout << "Point cloud " << iterPointCloud << ". size in augment scale " << this->objectPointClouds[iterPointCloud].points.size() << "\n";
        for (float curLeafSize = minLeafSize; curLeafSize < maxLeafSize; curLeafSize = curLeafSize + step)
        {
            pcl::PointCloud<pcl::PointXYZ> newPointCloud = this->objectPointClouds[iterPointCloud];

            //Do subsampling....
            toolBox.subsampling(newPointCloud, curLeafSize);

            std::cout << "scal py pcl size kick out if less then 25 points " << newPointCloud.points.size() << std::endl;
            //Add to set....
            if (newPointCloud.points.size() > 80) // 100 89% (50 exp but think 10  , 91% 10ex )
            {
                this->objectPointClouds.push_back(newPointCloud);
                std::cout << "CObjectPseudoCategorizationGeometric::augmentSetWithScalePyramid.....add " << curLeafSize << " " << maxLeafSize << std::endl;
                this->objectLabels.push_back(this->objectLabels[iterPointCloud]);
                this->objectNames.push_back(std::string(this->objectNames[iterPointCloud] + "_Scaled_" + boost::lexical_cast<std::string>(curLeafSize)));
                std::cout << std::string(this->objectNames[iterPointCloud] + "_Scaled_" + boost::lexical_cast<std::string>(curLeafSize)) << std::endl;
            }
            else
            {
                std::cout << "kick out!!!!!!!!!!!!" << std::endl;
                break;
            }
        }
    }
}
