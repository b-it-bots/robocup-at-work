#include <cstdlib>
#include <list>
#include <fstream>
#include <climits>
#include <numeric>
#include <algorithm>
#include <assert.h>
#include <boost/lexical_cast.hpp>

#include "object_geometric_service.h"
#include "density_estimation.h"
#include "object_pose_normalization.hpp"
#include "prob_neural_network/hp_neural_network.h"
#include "prob_neural_network/rtp_neural_network.h"

#define PI 3.14159265

CObjectGeometricService::CObjectGeometricService(int estimationType, bool addNormalEstimation)
{
    this->numberCategories = 0;
    ng_consistDist = 3;//4;
    ng_errorAdd = 350.0; //400
    ng_nodeAddInterval = -1;
    ng_maxEdgeAge = 170; //50 also good if just trained once, 170 if trained twice
    ng_errorNewNodeDecrease = 0.5f; //0.5 //beteitige node dec (added node)
    ng_errorGeneralDecrease = 0.995f; //0.1  // all node dec
    ng_nearestNeighborWeight = 0.2; //0.5 //move the nearest node to target
    ng_nNeighborWeight = 0.006; //0.3//move other neigbors to target
    ng_normalize = false;
    ng_modelAccuracy = -1;
    this->neuralGasShapeReconstructor = new CNeuralGasClassifier(3, 1000, ng_nearestNeighborWeight, ng_nNeighborWeight, ng_errorAdd, ng_maxEdgeAge, ng_nodeAddInterval, ng_errorNewNodeDecrease,
            ng_errorGeneralDecrease, 1, 760, 1, 1, -1);
    this->filename = "temp";
    this->isCommon = false;

    this->estimationType = estimationType;
    this->addNormalEstimation = addNormalEstimation;
    this->logger = &CLogger::getInstance();
    this->isNodeHandle = false;

    this->homePath = HOME_PATH;

    this->enNoisement = -1;//0.5;

    this->meanModels.clear();
    numberQuery = 0;
}

void CObjectGeometricService::setHomePath(std::string homePath)
{
    this->homePath = homePath;
}

void CObjectGeometricService::setIsCommonHistogrammCfg(bool isCommon)
{
    this->isCommon = isCommon;
}

void CObjectGeometricService::inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, int label, std::string filename)
{

    this->logger->log->debug("CObjectGeometricService::inputObjectPointCloud...(%d,%d)\n", objectPointCloud.size(), label);
    this->objectPointCloud = objectPointCloud;
    this->label = label;
    this->filename = filename;

    if (this->ng_normalize)
    {
        CObjectPoseNormalization<pcl::PointXYZ> objectPoseNormalization;
        objectPoseNormalization.setPointCloudModel(objectPointCloud);

        clock_t begin_time = clock();
        this->objectPointCloud = objectPoseNormalization.normalizePose();
        std::cout << "CObjectGeometricService::inputObjectPointCloud...posenormal..." << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
        ///////////////7


        std::cout << " CObjectGeometricService::inputObjectPointCloud...do after objectPoseNormalization subsampling=?! normlize object";
        ////posenormalizatio here!
    }
    this->objectPointCloudNormals = this->computeNormals(this->objectPointCloud);

}

pcl::PointCloud<pcl::PointXYZ> CObjectGeometricService::normalizePointCould(pcl::PointCloud<pcl::PointXYZ> pointCloud)
{
    this->logger->log->debug("CObjectGeometricAnalyzer normalize pointCloud ...\n");

    if (pointCloud.points.size() > 0)
    {
        pcl::PointXYZ centroid;
        centroid = this->toolBox.pointCloudCentroid(pointCloud);

        //do translation that centroid is coord frame origin!
        std::vector<double> distances;
        double maxDistance = 0;
        double t = 0;

        pcl::PointXYZ normalizedCentroid; // :p
        normalizedCentroid.x = 0;
        normalizedCentroid.y = 0;
        normalizedCentroid.z = 0;

        //Translation to centroid
        for (unsigned int it = 0; it < pointCloud.points.size(); it++)
        {
            pointCloud.points[it].x = pointCloud.points[it].x - centroid.x;
            pointCloud.points[it].y = pointCloud.points[it].y - centroid.y;
            pointCloud.points[it].z = pointCloud.points[it].z - centroid.z;

            distances.push_back(this->toolBox.euclDistanceBtwPoints(pointCloud.points[it], normalizedCentroid)); // cent 0,0,0
        }

        //Normalization to unit sphere
        maxDistance = *std::max_element(distances.begin(), distances.end());
        t = ((double) 1 / maxDistance);
        for (unsigned int it = 0; it < pointCloud.points.size(); it++)
        {

            pointCloud.points[it].x = normalizedCentroid.x + (pointCloud.points[it].x - normalizedCentroid.x) * t;
            pointCloud.points[it].y = normalizedCentroid.y + (pointCloud.points[it].y - normalizedCentroid.y) * t;
            pointCloud.points[it].z = normalizedCentroid.z + (pointCloud.points[it].z - normalizedCentroid.z) * t;
        }
    }

    return pointCloud;
}

void CObjectGeometricService::setParametersShapeRecontructor(float ng_consistDist, float ng_errorAdd, float ng_nodeAddInterval, float ng_maxEdgeAge, float ng_errorNewNodeDecrease,
        float ng_errorGeneralDecrease, float ng_nearestNeighborWeight, float ng_nNeighborWeight, bool normalize, float modelAccuracy)
{

    this->ng_consistDist = ng_consistDist;
    this->ng_normalize = normalize;
    this->ng_modelAccuracy = modelAccuracy;

    this->neuralGasShapeReconstructor->reset();
    delete this->neuralGasShapeReconstructor;

    this->neuralGasShapeReconstructor = new CNeuralGasClassifier(3, 1000, ng_nearestNeighborWeight, ng_nNeighborWeight, ng_errorAdd, ng_maxEdgeAge, ng_nodeAddInterval, ng_errorNewNodeDecrease,
            ng_errorGeneralDecrease, 1, 760, 1, 1, -1);

}
void CObjectGeometricService::reconstructShape(bool addToCollection)
{
    clock_t begin_time = clock();
    this->logger->log->debug("CObjectGeometricService::reconstructShape...");
    std::vector<std::vector<double> > convCloud;
    double meanSquaredErrorToModel = 0;

    float addNoiseToModel = 0.1f; //0.1f 0.15 could work ok 0.2f ok but sparse ,0.5 destroyed

    for (unsigned int p = 0; p < this->objectPointCloud.size(); p++)
    {
        std::vector<double> convPoint;
        convPoint.push_back(objectPointCloud.points[p].x);
        convPoint.push_back(objectPointCloud.points[p].y);
        convPoint.push_back(objectPointCloud.points[p].z);
        convCloud.push_back(convPoint);
    }

    //if normlize the neural gas then do it at the end!!!!
    //std::cout<<"convCloud.size()  "<<convCloud.size()<<" \n";
    this->neuralGasShapeReconstructor->reset();
    this->neuralGasShapeReconstructor->train(convCloud, -1, 0, 500, this->ng_consistDist); //-1, 0
    this->neuralGasShapeReconstructor->train(convCloud, -1, 0, 500, this->ng_consistDist, this->ng_normalize); //this->ng_normalize
    ////////
    if (this->ng_modelAccuracy <= 0)
    {
        for (int i = 0; i < 3; i++) //10!!!!! 3 is fine
        {
            //const clock_t begin_time2 = clock();
            meanSquaredErrorToModel = this->neuralGasShapeReconstructor->train(convCloud, -1, 0, 500, this->ng_consistDist, this->ng_normalize, addNoiseToModel); //this->ng_normalize
            //  meanSquaredErrorToModel = this->neuralGasShapeReconstructor->train(convCloud, -1, 0, 500, -1, false, addNoiseToModel); //this->ng_normalize
            this->logger->log->debug("CObjectGeometricService::reconstructShape...MeanSquaredErrorToModel (noAccTest) %lf\n", meanSquaredErrorToModel);
            //std::cout<<"...."<<i<<". iter = "<<float(clock() - begin_time2) / CLOCKS_PER_SEC<<" "<<this->neuralGasShapeReconstructor->getNeuralGas()->getNumberNodes()<<"  with error = "<<meanSquaredErrorToModel<<"\n";
        }
    }
    else
    {
        for (int i = 0; i < 30; i++) //30!!!!!
        {
            meanSquaredErrorToModel = this->neuralGasShapeReconstructor->train(convCloud, -1, 0, 500, this->ng_consistDist, this->ng_normalize, addNoiseToModel); //this->ng_normalize
            this->logger->log->debug("CObjectGeometricService::reconstructShape...MeanSquaredErrorToModel %lf (NUM NEURONS = %d)\n", meanSquaredErrorToModel,
                                     this->neuralGasShapeReconstructor->getNeuralGas()->getNumberNodes());
            if (meanSquaredErrorToModel <= this->ng_modelAccuracy)
            {

                this->logger->log->debug("CObjectGeometricService::reconstructShape...MeanSquaredErrorToModel Accuracy met %lf... done!!\n", meanSquaredErrorToModel);
                break;
            }
        }
    }
    ////////////////
    //ShapeRefinement on Curvature difference
    std::vector<std::vector<double> > pointsToRefine = this->reconstructShapeRefinement(this->objectPointCloud, this->objectPointCloudNormals);
    if (pointsToRefine.size() > 0)
    {
        this->neuralGasShapeReconstructor->train(pointsToRefine, -1, 0, 500, this->ng_consistDist, this->ng_normalize);
    }
    else
    {
        std::cout << "CObjectGeometricService::reconstructShape....nothing to refine\n";
    }

    this->logger->log->debug("\nSaving...\n");

    //save to file
    //pcl::io::savePCDFileASCII(this->filename + ".pcd", objectPointCloud);
    //Save to file
    //this->neuralGasShapeReconstructor->saveNeuralGas(this->filename);

    //Save To File
    //this->johnsonApsp = this->neuralGasShapeReconstructor->computeJohnsonAPSP(this->filename);
    //DO not save to File
    this->johnsonApsp = this->neuralGasShapeReconstructor->computeJohnsonAPSP();




    //begin_time = clock();
    this->createDensityDistribution(addToCollection);
    //std::cout << "CObjectGeometricService::reconstructShape...DE "<< float(clock() - begin_time) / CLOCKS_PER_SEC<<std::end;

    //std::cout << "CObjectGeometricService::reconstructShape...(" << this->neuralGasShapeReconstructor->getNeuralGas()->getNumberNodes() << ")..." << float(clock() - begin_time) / CLOCKS_PER_SEC
    //          << std::endl;


}

void CObjectGeometricService::createDensityDistribution(bool addToCollection)
{

    if (this->neuralGasShapeReconstructor->getNeuralGas()->getNumberNodes() < 1)
    {
        this->logger->log->warn("CObjectGeometricService::createDensityDistribution...ShapeReconstructor empty ?!");
        return;
    }

    this->meanApspDistances.clear();
    this->apspDistances.clear();

    std::fstream fout;
    fout.open(std::string(this->filename + "_DD.ng").c_str(), std::ios::out | std::ios::binary);

    std::fstream fout2;
    fout2.open(std::string(this->filename + "_All_DD.ng").c_str(), std::ios::out | std::ios::binary);

    std::list<apsp_node*>::iterator i;
    std::list<apsp_node*>::iterator n;
    std::map<node*, double>::iterator d;
    std::map<node*, std::vector<double> >::iterator d2;

    list<apsp_node*> all_nodes = list<apsp_node*> (*(list<apsp_node*>*) this->johnsonApsp.get_nodes());

    int c = all_nodes.size();

    double sum = 0;

    //Addition maybe good for cat but not for recog?!
    /*double totalDistance = 0;
     for (i = all_nodes.begin(); i != all_nodes.end(); ++i) {
     for (n = all_nodes.begin(); n != all_nodes.end(); ++n) {
     totalDistance += (*n)->get_dist_from(*i);
     }
     }*/
    //

    for (i = all_nodes.begin(); i != all_nodes.end(); ++i)
    {
        //cout << (*i)->get_id() << " : ";
        sum = 0;
        std::vector<double> currAsap;
        for (n = all_nodes.begin(); n != all_nodes.end(); ++n)
        {
            //  double dist2 = (*n)->get_dist_from(*i);
            double dist = (*i)->get_dist_from(*n);
            //  std::cout<<"DIST- dist2 "<<dist2<< " "<< dist<<std::endl;
            if (dist >= APSP_INFINITY - std::numeric_limits<double>::epsilon())
            {
                ;//std::cout << "inf ";
            }
            else
            {
                sum += pow((dist), 2);
                //currAsap.push_back((pow((dist), 2))); //(pow((dist), 2));
                currAsap.push_back(dist); //(pow((dist), 2));
            }
        }
        //  std::cout << (sum / (double) c) << std::endl;
        this->meanApspDistances[*i] = (sum / (double) c);

        //ATTENTION!!! currAsap is not sorted regarding the distance in the APSP matrix! since it is a vec and not map!
        //But it can be used for distribution purposes
        //however this variable is just used in this class
        this->apspDistances[*i] = currAsap; //(sum / (double) c);
    }

    //Fill dataStructur for Object decomposition
    SObjectDescription oD;

    std::map<int, CNeuralGasNode*> nodes = this->neuralGasShapeReconstructor->getNeuralGas()->getNeuralGasNodes();
    oD.label = this->label;
    //DO NOT SAVE POINT CLOUD GAS AND JOHNSON ! if wanted uncomment it
    /////oD.objectPointCloud = objectPointCloud;
    //oD.neuralGas = *this->neuralGasShapeReconstructor->getNeuralGas();
    //oD.johnsonApsp = this->johnsonApsp;

    oD.centroid = this->neuralGasShapeReconstructor->getNeuralGas()->computeCentroid();
    oD.filename = this->filename;
    ////////////////////

    typedef std::map<int, CNeuralGasNode*>::iterator nodeIter;
    for (nodeIter it = nodes.begin(); it != nodes.end(); it++)
    {
        oD.nodePosition[it->first] = it->second->position;
    }

    //computes a vector of nodeToCenterDistances over all nodes to centroid
    if (this->estimationType == 15)
    {
        this->computeNodeToCenterDistances(oD);
    }

    this->logger->log->debug("CObjectGeometricService::createDensityDistribution...add !!normalized!! Normals to objectDescription\n");
    if (POINT_NORMAL_ESTIMATION_METHOD > 0)
    {
        for (unsigned int iterPointCloud = 0; iterPointCloud < this->objectPointCloudNormals.points.size(); ++iterPointCloud)
        {

            //Absolute value to do the normals invariant regaring its location on the surface position. fabs
            double normalX = fabs(this->objectPointCloudNormals.points[iterPointCloud].normal[0]);
            double normalY = fabs(this->objectPointCloudNormals.points[iterPointCloud].normal[1]);
            double normalZ = fabs(this->objectPointCloudNormals.points[iterPointCloud].normal[2]);

            if (POINT_NORMAL_ESTIMATION_METHOD == 1 || POINT_NORMAL_ESTIMATION_METHOD == 2)
            {
                oD.pointNormals[0].push_back(normalX);
                oD.pointNormals[1].push_back(normalY);
                oD.pointNormals[2].push_back(normalZ);
            }
            std::vector<double> normalPos;
            normalPos.resize(3);
            normalPos[0] = this->objectPointCloud.points[iterPointCloud].x;
            normalPos[1] = this->objectPointCloud.points[iterPointCloud].y;
            normalPos[2] = this->objectPointCloud.points[iterPointCloud].z;

            int nodeId = getNearestNode(oD.nodePosition, normalPos);
            //std::cout<<"Nearest NodeID Normal to "<< iterPointCloud <<" is "<< nodeId<<"\n";
            oD.nodePointNormals[nodeId][0].push_back(normalX);
            oD.nodePointNormals[nodeId][1].push_back(normalY);
            oD.nodePointNormals[nodeId][2].push_back(normalZ);
        }

        if (POINT_NORMAL_ESTIMATION_METHOD == 2)
        {
            this->computeAllPairNormalDifferences(oD);
        }
    }
    ///////////////////
    std::vector<double> vecMeanApspDistances;
    for (d = this->meanApspDistances.begin(); d != this->meanApspDistances.end(); ++d)
    {

        vecMeanApspDistances.push_back(d->second);
        oD.nodeMeanApspDistance[d->first->get_id()] = d->second;

        fout << d->first->get_id() << " " << d->second << std::endl;
    }
    oD.meanApspDistances = vecMeanApspDistances;

    //////////////
    std::vector<double> vecApspDistances;

    fout2 << this->apspDistances.size() << "\n";
    for (d2 = this->apspDistances.begin(); d2 != this->apspDistances.end(); ++d2)
    {

        std::vector<double> v = d2->second;
        for (unsigned int i = 0; i < v.size(); ++i)
        {
            vecApspDistances.push_back(v[i]);
        }
        oD.nodeApspDistance[d2->first->get_id()] = d2->second;

        fout2 << d2->first->get_id() << " ";
        for (unsigned int i = 0; i < v.size(); ++i)
        {
            fout2 << v[i] << " ";
        }
        fout2 << "\n";
    }
    oD.apspDistances = vecApspDistances;
    ///////////////
    this->objectDescription = oD;
    if (addToCollection)
    {
        this->objectDecompositon[oD.label].addObject(oD);
    }

    fout.close();
    fout2.close();

    //this->loadASPSMatrix(this->filename + "_All_DD.ng");
}

int CObjectGeometricService::getNearestNode(std::map<int, std::vector<double> > nodePosition, std::vector<double> position)
{
    std::map<int, std::vector<double> >::iterator iterNodePointNormal;

    int minId = -1;
    double minDistance = std::numeric_limits<double>::infinity();
    double distance = 0;
    for (iterNodePointNormal = nodePosition.begin(); iterNodePointNormal != nodePosition.end(); ++iterNodePointNormal)
    {
        distance = getEuclideanDistance(position, iterNodePointNormal->second);

        if (distance < minDistance)
        {
            minId = iterNodePointNormal->first;
            minDistance = distance;
        }
    }

    return minId;
}

void CObjectGeometricService::initEstimation(bool trainEnsemble)
{
    std::map<int, CObjectDecomposition>::iterator iDecomp;
    int numShell = 3;//10; ///std 3
    int numNormalShell = 3;

    //this->kdeDB.initEstimationHistogram();
    //this->kdeDB.initEstimationDensity();
    this->shuffleObjectDecomposition();
    for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
    {
        printf("CObjectGeometricService::initEstimation...Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());
        this->logger->log->info("CObjectGeometricService::initEstimation...Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());
        //object id, estimate
        std::map<int, std::vector<double> > estimate;
        if (this->estimationType == 1)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via Geo\n");
            iDecomp->second.initGeoEstimation();
        }
        else if (this->estimationType == 2)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via GeoShell\n");
            iDecomp->second.initGeoShellEstimation(numShell); //3 //3 top 310311  //5
        }
        else if (this->estimationType == 3)
        {
            this->logger->log->error("CObjectGeometricService::initEstimation... via GeoNode ... NOT IMPLEMENTED!!!\n");
            //  iDecomp->second.initEstimationNode();
        }
        else if (this->estimationType == 4)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via Normals\n");
            iDecomp->second.initNormalEstimation();
        }
        else if (this->estimationType == 5)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via NormalShell\n");
            iDecomp->second.initNormalShellEstimation(numShell);
        }
        else if (this->estimationType == 10)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via combined GeoShell + Normals\n");
            estimate = iDecomp->second.initCombinedGeoShellNormalEstimation(numShell, this->isCommon);
        }
        else if (this->estimationType == 15)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via combined GeoShell + Node to Center \n");
            estimate = iDecomp->second.initCombinedGeoShellNodeToCenterEstimation(numShell, this->isCommon);

        }
        else if (this->estimationType == 20)
        {
            this->logger->log->debug("CObjectGeometricService::initEstimation... via combined GeoShell + Normals+ NormalShell\n");
            estimate = iDecomp->second.initCombinedGeoShellNormalNormalShellEstimation(numShell, numNormalShell, this->isCommon);

        }
        else if (this->estimationType == 30)
        {
            this->logger->log->info("CObjectGeometricService::initEstimation... via combined GeoShell + Normals+ NormalShell\n");
            //estimate
            //      = iDecomp->second.initCombinedGeoShellNormalNormalShellEstimation(
            //              numShell, numNormalShell, this->isCommon);
            iDecomp->second.initNodeEstimation();

        }
        else
        {
            this->logger->log->error("CObjectGeometricService::initEstimation...Incorrect Estimation type has been chosen\n");
        }
    }

    //Additional step required for CHPNN, PNN and RPTEnsemble!
    /*Here we need to estimate the histogram for each object in each category to the configuration to all other objects*/
    if (this->estimationType != 30 && trainEnsemble)
    {
        for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
        {
            std::cout << "CObjectGeometricService::initEstimation...Estimate Related Model for Label " << CFileSettings::labels[iDecomp->first].c_str() << ":...\n";
            this->logger->log->info("CObjectGeometricService::initEstimation...Estimate Related Model for Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());

            std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > & objDesc = iDecomp->second.getObjects();

            for (unsigned int iterObjDesc = 0; iterObjDesc < objDesc.size(); ++iterObjDesc)
            {
                //label, object id, feat vector; regarding iDecomp lable and objectdesc object id
                std::map<int, std::map<int, std::vector<double> > > model;
                model = this->estimateModel(objDesc[iterObjDesc], false);
                objDesc[iterObjDesc].estimatedRelatedModels = model;
                //this->hProbNeuralNetwork.addHModel(iDecomp->first, iterObjDesc,model);
                this->rtpnnEnsemble.addHModel(iDecomp->first, iterObjDesc, model);
                //this->probNeuralNetwork.addModel(iDecomp->first,model[iDecomp->first]);
                //if(this->isCommon)
                //  break;
            }
        }
        //this->hProbNeuralNetwork.initHierarchy(HPNN_TRAINTEST_RATIO);
        this->rtpnnEnsemble.initEnsemble(HPNN_TRAINTEST_RATIO);
        //this->probNeuralNetwork.init();
        //this->probNeuralNetwork.verifyPNN(this->objectDecompositon);
    }

    this->numberCategories = this->rtpnnEnsemble.getNumCategories();
}

void CObjectGeometricService::initPnn(bool reset)
{
    std::map<int, CObjectDecomposition>::iterator iDecomp;

    if (reset)
    {
        this->probNeuralNetwork = CPNeuralNetwork();
    }

    for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
    {
        std::cout << "CObjectGeometricService::initEstimation PNN...Estimate Related Model for Label " << CFileSettings::labels[iDecomp->first].c_str() << ":...\n";
        this->logger->log->info("CObjectGeometricService::initEstimation PNN...Estimate Related Model for Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());

        std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > & objDesc = iDecomp->second.getObjects();

        for (unsigned int iterObjDesc = 0; iterObjDesc < objDesc.size(); ++iterObjDesc)
        {
            //label, object id, feat vector; regarding iDecomp lable and objectdesc object id
            std::map<int, std::map<int, std::vector<double> > > model;
            model = this->estimateModel(objDesc[iterObjDesc], false);
            objDesc[iterObjDesc].estimatedRelatedModels = model;
            //this->hProbNeuralNetwork.addHModel(iDecomp->first, iterObjDesc,model);
            //this->rtpnnEnsemble.addHModel(iDecomp->first, iterObjDesc, model);
            this->probNeuralNetwork.addModel(iDecomp->first, model[iDecomp->first]);
            //if(this->isCommon)
            //  break;
        }
    }

    this->probNeuralNetwork.init();
}

void CObjectGeometricService::initKNN()
{
    std::map<int, CObjectDecomposition>::iterator iDecomp;

    for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
    {
        std::cout << "CObjectGeometricService::initEstimation KNN...Estimate Related Model for Label " << CFileSettings::labels[iDecomp->first].c_str() << ":...\n";
        this->logger->log->info("CObjectGeometricService::initEstimation KNN...Estimate Related Model for Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());

        std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > & objDesc = iDecomp->second.getObjects();

        for (unsigned int iterObjDesc = 0; iterObjDesc < objDesc.size(); ++iterObjDesc)
        {
            //label, object id, feat vector; regarding iDecomp lable and objectdesc object id
            std::map<int, std::map<int, std::vector<double> > > model;
            model = this->estimateModel(objDesc[iterObjDesc], false);
            objDesc[iterObjDesc].estimatedRelatedModels = model;
            //this->hProbNeuralNetwork.addHModel(iDecomp->first, iterObjDesc,model);
            //this->rtpnnEnsemble.addHModel(iDecomp->first, iterObjDesc, model);
            this->knnClassifier.addModel(iDecomp->first, model[iDecomp->first]);
            //if(this->isCommon)
            //  break;
        }
    }

    this->knnClassifier.init();
}

void CObjectGeometricService::initNodeEstimation()
{
    std::map<int, CObjectDecomposition>::iterator iDecomp;

    for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
    {
        this->logger->log->info("CObjectGeometricService::initEstimation...Label %s:...\n", CFileSettings::labels[iDecomp->first].c_str());

        this->logger->log->debug("CObjectGeometricService::initEstimation... via GeoNode\n");

        iDecomp->second.initNodeEstimation();

    }

}

void CObjectGeometricService::extractStatisticalFeatures()
{
    SGeometricFeatures geometricFeatures;
    if (this->objectPointCloud.points.size() > 0)
    {
        struct SGeometricFeatures gF;
        gF.objectPointCloud = objectPointCloud;
        gF.numPoints = objectPointCloud.points.size();
        gF.numNodes = meanApspDistances.size();
        gF.meanApspDistances = meanApspDistances;
        gF.apspDistances = apspDistances;
        //gF.objectPointCloudCentroid = this->objectPointCloudCentroid;

        this->objectGeometricFeatures[this->label].push_back(gF);
    }
}

//isSave = save query histogram for later saveCollection
std::pair<int, double> CObjectGeometricService::query(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave, std::string name)
{


    clock_t begin_time = clock();
    this->inputObjectPointCloud(objectPointCloud, -1);
    std::cout << "CObjectGeometricService::query...poseNorm " << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;


    begin_time = clock();
    this->reconstructShape(false);
    std::cout << "CObjectGeometricService::query...reconstruct " << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

    //label, object id, feat vector;
    std::map<int, std::map<int, std::vector<double> > > queryModel;

    begin_time = clock();
    queryModel = this->estimateModel(this->objectDescription, doSave);
    std::cout << "CObjectGeometricService::query...Estimate Model..." << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;


    numberQuery++;
    //this->printModel(queryModel, 0, name);

    /// this->logger->log->debug(
    ///         "PROBAB NEURAL NETWORK QUERY: %s \n",
    ///         CFileSettings::labels[this->probNeuralNetwork.evaluate(queryModel).first].c_str());
    /*
     this->logger->log->debug(
     "PROBAB H NEURAL NETWORK QUERY: %s \n",
     CFileSettings::labels[this->hProbNeuralNetwork.hEvaluate(queryModel).first].c_str());*/

    begin_time = clock();
    std::pair<int, double> result = this->rtpnnEnsemble.fEvaluate(queryModel, this->currentTreeReponses);
    std::cout << "CObjectGeometricService::query...RTPNNEnsemble..." << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

    this->logger->log->info("RandTree PROBAB H Neural Network Ensemble QUERY: %s (%lf) \n", CFileSettings::labels[result.first].c_str(), result.second);
    printf("RandTree PROBAB H Neural Network Ensemble QUERY: %s (%lf) \n", CFileSettings::labels[result.first].c_str(), result.second);

    return result;
}

std::pair<int, double> CObjectGeometricService::queryPnn(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave, std::string name)
{
    this->inputObjectPointCloud(objectPointCloud, -1);
    this->reconstructShape(false);

    //label, object id, feat vector;
    std::map<int, std::map<int, std::vector<double> > > queryModel;
    queryModel = this->estimateModel(this->objectDescription, doSave);

    numberQuery++;
    //this->printModel(queryModel, 0, name);

    std::pair<int, double> result = this->probNeuralNetwork.evaluate(queryModel);
    std::cout << "PROBAB NEURAL NETWORK QUERY: " << CFileSettings::labels[result.first].c_str() << std::endl;
    this->logger->log->debug("PROBAB NEURAL NETWORK QUERY: %s \n", CFileSettings::labels[result.first].c_str());
    /*
     this->logger->log->debug(
     "PROBAB H NEURAL NETWORK QUERY: %s \n",
     CFileSettings::labels[this->hProbNeuralNetwork.hEvaluate(queryModel).first].c_str());*/
    return result;
}

void CObjectGeometricService::verifyPnn()
{
    this->probNeuralNetwork.verifyPNN();
}

void CObjectGeometricService::verifyKNN()
{
    this->knnClassifier.verifyKNN();
}

std::pair<int, double> CObjectGeometricService::queryKNN(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave, std::string name)
{
    this->inputObjectPointCloud(objectPointCloud, -1);
    this->reconstructShape(false);

    //label, object id, feat vector;
    std::map<int, std::map<int, std::vector<double> > > queryModel;
    queryModel = this->estimateModel(this->objectDescription, doSave);

    numberQuery++;
    //this->printModel(queryModel, 0, name);

    std::pair<int, double> result = this->knnClassifier.evaluate(queryModel);
    std::cout << "KNN QUERY: " << CFileSettings::labels[result.first].c_str() << std::endl;
    this->logger->log->debug("KNNK QUERY: %s \n", CFileSettings::labels[result.first].c_str());
    /*
     this->logger->log->debug(
     "PROBAB H NEURAL NETWORK QUERY: %s \n",
     CFileSettings::labels[this->hProbNeuralNetwork.hEvaluate(queryModel).first].c_str());*/
    return result;
}

std::pair<int, double> CObjectGeometricService::query(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, std::map<int, std::pair<int, double> > &getCurrentTreeReponses)
{
    std::pair<int, double> result = this->query(objectPointCloud);
    getCurrentTreeReponses = this->currentTreeReponses;
    return result;
}

std::vector<std::pair<int, double> > CObjectGeometricService::queryEnNoised(pcl::PointCloud<pcl::PointXYZ> objectPointCloud,
        std::vector<std::map<int, std::pair<int, double> > > &getCurrentTreeReponses, int numEnNoisements)
{
    this->inputObjectPointCloud(objectPointCloud, -1);
    this->reconstructShape(false);

    std::vector<std::pair<int, double> > enNoisedResults;
    for (unsigned int iterNoise = 0; iterNoise < numEnNoisements; ++iterNoise)
    {
        //label, object id, feat vector;
        std::map<int, std::map<int, std::vector<double> > > queryModel;
        queryModel = this->estimateModel(this->objectDescription, false);
        std::pair<int, double> result = this->rtpnnEnsemble.fEvaluate(queryModel, this->currentTreeReponses);

        enNoisedResults.push_back(result);
        getCurrentTreeReponses.push_back(this->currentTreeReponses);
    }
    /// this->logger->log->debug(
    ///         "PROBAB NEURAL NETWORK QUERY: %s \n",
    ///         CFileSettings::labels[this->probNeuralNetwork.evaluate(queryModel).first].c_str());
    /*
     this->logger->log->debug(
     "PROBAB H NEURAL NETWORK QUERY: %s \n",
     CFileSettings::labels[this->hProbNeuralNetwork.hEvaluate(queryModel).first].c_str());*/

    //this->logger->log->info("RandTree PROBAB H Neural Network Ensemble QUERY: %s \n", CFileSettings::labels[result.first].c_str());
    //printf("RandTree PROBAB H Neural Network Ensemble QUERY: %s \n", CFileSettings::labels[result.first].c_str());

    return enNoisedResults;
}

std::map<int, std::map<int, std::vector<double> > > CObjectGeometricService::estimateModel(SObjectDescription &objDesc, bool doSave)
{
    std::map<int, CObjectDecomposition>::iterator iDecomp;
    std::map<int, std::map<int, std::vector<double> > > queryModel;
    for (iDecomp = this->objectDecompositon.begin(); iDecomp != this->objectDecompositon.end(); ++iDecomp)
    {
        //std::cout << "CObjectGeometricService::estimateModel...Query against Label : " << iDecomp->first << "\n";
        this->logger->log->debug("CObjectGeometricService::estimateModel...Query against Label : %d \n", iDecomp->first);

        std::map<int, std::vector<double> > currQueryHisto;
        //Do estimation over the entire distances!
        if (this->estimationType == 1)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateGeoHistogram \n");
            currQueryHisto = (iDecomp->second.estimateGeoHistogram(objDesc));
        }
        else if (this->estimationType == 2)
        {
            //Do estimation over a set of shells
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateGeoShellHistogram \n");
            currQueryHisto = (iDecomp->second.estimateGeoShellHistogram(objDesc));
        }
        else if (this->estimationType == 3)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimation via apspNode not implements\n");
        }
        else if (this->estimationType == 4)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateNormalHistogram \n");
            currQueryHisto = (iDecomp->second.estimateNormalHistogram(objDesc));
        }
        else if (this->estimationType == 10)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateCombinedGeoShellNormalEstimation\n");
            currQueryHisto = (iDecomp->second.estimateCombinedGeoShellNormalEstimation(objDesc));
        }
        else if (this->estimationType == 15)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateCombinedGeoShellNodeToCenterEstimation\n");
            currQueryHisto = (iDecomp->second.estimateCombinedGeoShellNodeToCenterEstimation(objDesc));
        }

        else if (this->estimationType == 20)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateCombinedGeoShellNormalNormalShellEstimation\n");
            currQueryHisto = (iDecomp->second.estimateCombinedGeoShellNormalNormalShellEstimation(objDesc));
        }

        else if (this->estimationType == 30)
        {
            this->logger->log->debug("CObjectGeometricService::estimateModel...Estimate estimateCombinedGeoShellNormalNormalShellEstimation\n");
            currQueryHisto = (iDecomp->second.estimateNodeEstimation(objDesc));
        }

        if (doSave)
        {
            this->objectDecompositonQueries[iDecomp->first].push_back(currQueryHisto);
        }

        if (this->enNoisement > 0)
        {
            std::cout << "EnNoising....\n";
            queryModel[iDecomp->first] = enNoiseEstimation(currQueryHisto, this->enNoisement);
        }
        else
        {
            queryModel[iDecomp->first] = currQueryHisto;
        }
    }

    return queryModel;
}

std::map<int, std::vector<double> > CObjectGeometricService::enNoiseEstimation(std::map<int, std::vector<double> > estimation, double addNoise)
{
    std::map<int, std::vector<double> >::iterator iterEstimation;

    std::map<int, std::vector<double> > enNoisedEstimation;
    for (iterEstimation = estimation.begin(); iterEstimation != estimation.end(); ++iterEstimation)
    {
        std::vector<double> enNoisedVec;
        //CFileSettings::coutStdVector(iterEstimation->second,true);
        enNoisedVec.resize(iterEstimation->second.size());
        for (unsigned int i = 0; i < iterEstimation->second.size(); ++i)
        {
            //std::cout<<"actual "<< iterEstimation->second[i] <<" ennoised "<< toolBox.randNumber(-(iterEstimation->second[i] * addNoise), iterEstimation->second[i] * addNoise);
            enNoisedVec[i] = iterEstimation->second[i] + toolBox.randNumber(-(iterEstimation->second[i] * addNoise), iterEstimation->second[i] * addNoise);
        }
        //CFileSettings::coutStdVector(enNoisedVec,true);
        enNoisedEstimation[iterEstimation->first] = enNoisedVec;
    }

    return enNoisedEstimation;
}

int CObjectGeometricService::getNumNeuronsASPSMatrix(std::string filename)
{
    std::fstream in;

    int numLines = 0;
    in.open(std::string(filename).c_str(), std::ios::in | std::ios::binary);

    if (!in.is_open())
    {
        this->logger->log->debug("could not open %s \n", filename.c_str());
        return numLines;
    }

    std::string str;

    while (getline(in, str))
    {
        numLines++;
    }
    std::cout << filename << " Num Neurons = " << numLines << std::endl;

    return numLines;
}

std::map<int, std::vector<double> > CObjectGeometricService::loadASPSMatrix(std::string filename)
{
    std::fstream in;
    std::map<int, std::vector<double> > asps;

    in.open(std::string(filename).c_str(), std::ios::in | std::ios::binary);

    if (!in.is_open())
    {
        this->logger->log->debug("could not open %s \n", filename.c_str());
        return asps;
    }

    this->logger->log->debug("FileName  %s\n", filename.c_str());
    in.open(std::string(filename).c_str(), std::ios::in | std::ios::binary);

    if (!in.is_open())
    {
        this->logger->log->debug("could not open %s \n", filename.c_str());
        return asps;
    }

    int numNodes = 0;

    in >> numNodes;
    std::cout << "\n\n Numnodes" << numNodes << "\n\n";
    for (unsigned int iterNodes = 0; iterNodes < (unsigned int) numNodes; ++iterNodes)
    {
        int id;
        in >> id;
        std::cout << id << "\n";
        std::vector<double> dist;
        for (unsigned int iterNodesDist = 0; iterNodesDist < (unsigned int) numNodes; ++iterNodesDist)
        {
            double d;
            in >> d;
            //  std::cout<<(double)d<<" ";
            dist.push_back((double) d);
        }
        //  std::cout<<"\n";
        asps[id] = dist;
    }

    std::map<int, std::vector<double> >::iterator aspsIter;
    for (aspsIter = asps.begin(); aspsIter != asps.end(); ++aspsIter)
    {
        this->logger->log->debug("\n\n\n\n ID = %d \nDist: ", aspsIter->first);
        for (unsigned int iterNodesDist = 0; iterNodesDist < aspsIter->second.size(); ++iterNodesDist)
        {
            this->logger->log->debug(" %lf", aspsIter->second[iterNodesDist]);
        }
        this->logger->log->debug("\n");
    }
    in.close();

    return asps;
}

//ToDO 10?! as radius compute means distance between points!
pcl::PointCloud<pcl::Normal> CObjectGeometricService::computeNormals(pcl::PointCloud<pcl::PointXYZ> objectPointCloud)
{
    unsigned int kPoints = KPOINTS_NORMALS;
    pcl::PointCloud<pcl::Normal> cloud_normals;

    if (objectPointCloud.points.size() <= kPoints)
    {
        kPoints = objectPointCloud.points.size() - 1;
    }
    //Read tutorial on 3D normal estimation for object recognition 3D
    cloud_normals = toolBox.estimatingNormals(objectPointCloud, kPoints); //10 STD

    return cloud_normals;
}

void CObjectGeometricService::computeNodeToCenterDistances(SObjectDescription &oD)
{
    std::vector<double> nodeToCenterDistances;
    std::vector<double> &center = oD.centroid;
    std::map<int, std::vector<double> > &nodePosition = oD.nodePosition;
    std::map<int, std::vector<double> >::iterator iterNode;

    assert(nodePosition.size() > 0);

    for (iterNode = nodePosition.begin(); iterNode != nodePosition.end(); ++iterNode)
    {
        nodeToCenterDistances.push_back(getEuclideanDistance(center, iterNode->second));
    }

    oD.nodeToCenterDistances = nodeToCenterDistances;
}

void CObjectGeometricService::computeAllPairNormalDifferences(SObjectDescription &oD)
{
    int strategy = 3; //only 1 or 2 allowed

    std::vector<double> diffNormalX;
    std::vector<double> diffNormalY;
    std::vector<double> diffNormalZ;

    std::vector<double> &normalX = oD.pointNormals[0];
    std::vector<double> &normalY = oD.pointNormals[1];
    std::vector<double> &normalZ = oD.pointNormals[2];

    if (strategy == 1) // add all differences!
    {

        for (unsigned int i = 0; i < normalX.size(); ++i)
        {
            for (unsigned int j = 0; j < normalX.size(); ++j)
            {
                //  if(i!=j)
                //  {
                diffNormalX.push_back(fabs(normalX[i] - normalX[j]));
                //  }
            }
        }

        for (unsigned int i = 0; i < normalY.size(); ++i)
        {
            for (unsigned int j = 0; j < normalY.size(); ++j)
            {
                //if(i!=j)
                //  {
                diffNormalY.push_back(fabs(normalY[i] - normalY[j]));
                //  }
            }
        }

        for (unsigned int i = 0; i < normalZ.size(); ++i)
        {
            for (unsigned int j = 0; j < normalZ.size(); ++j)
            {
                //if(i!=j)
                //{
                diffNormalZ.push_back(fabs(normalZ[i] - normalZ[j]));
                //}
            }
        }

    }

    if (strategy == 2) // add only larges difference
    {

        double maxDiff = 0;
        for (unsigned int i = 0; i < normalX.size(); ++i)
        {
            for (unsigned int j = 0; j < normalX.size(); ++j)
            {
                double diff = fabs(normalX[i] - normalX[j]);
                if (diff >= maxDiff)
                {
                    maxDiff = diff;
                }
            }
            diffNormalX.push_back(maxDiff);
        }

        maxDiff = 0;
        for (unsigned int i = 0; i < normalY.size(); ++i)
        {
            for (unsigned int j = 0; j < normalY.size(); ++j)
            {
                double diff = fabs(normalY[i] - normalY[j]);
                if (diff >= maxDiff)
                {
                    maxDiff = diff;
                }
            }
            diffNormalY.push_back(maxDiff);
        }

        maxDiff = 0;
        for (unsigned int i = 0; i < normalZ.size(); ++i)
        {
            for (unsigned int j = 0; j < normalZ.size(); ++j)
            {
                double diff = fabs(normalZ[i] - normalZ[j]);
                if (diff >= maxDiff)
                {
                    maxDiff = diff;
                }
            }
            diffNormalZ.push_back(maxDiff);
        }

    }

    if (strategy == 3) // add only larges difference
    {
        double maxDiff = 0;
        int maxdiffIdx = 0;
        double maxDiffX = 0;
        double maxDiffY = 0;
        double maxDiffZ = 0;
        unsigned int numPoints = normalX.size();
        for (unsigned int i = 0; i < numPoints; ++i)
        {
            for (unsigned int j = 0; j < numPoints; ++j)
            {
                double diff = sqrt(pow((normalX[i] - normalX[j]), 2) + pow((normalY[i] - normalY[j]), 2) + pow((normalZ[i] - normalZ[j]), 2));
                if (diff >= maxDiff)
                {
                    maxDiff = diff;
                    maxDiffX = (pow((normalX[i] - normalX[j]), 2));
                    maxDiffY = (pow((normalY[i] - normalY[j]), 2));
                    maxDiffZ = (pow((normalZ[i] - normalZ[j]), 2));
                    maxdiffIdx = j;
                }
            }
            //diffNormalX.push_back(maxdiffIdx);
            //diffNormalY.push_back(maxdiffIdx);
            //diffNormalZ.push_back(maxdiffIdx);
            diffNormalX.push_back(maxDiffX);
            diffNormalY.push_back(maxDiffY);
            diffNormalZ.push_back(maxDiffZ);
            //std::cout<<"3 ";
        }
    }

    oD.pointNormals[0] = diffNormalX;
    oD.pointNormals[1] = diffNormalY;
    oD.pointNormals[2] = diffNormalZ;

}

void CObjectGeometricService::saveQueryCollection()
{

    if (this->objectDecompositonQueries.size() > 0)
    {

        CFileSettings::init();

        std::string fileName;
        std::map<int, std::vector<double> >::iterator iterObject;
        std::map<int, std::vector<std::map<int, std::vector<double> > > >::iterator iterLabel;

        //std::map<int, std::vector<std::map<int, std::vector<double> > > >

        std::vector<int> labels;
        std::vector<std::vector<double> > objHisto;
        unsigned int iObject = 0;
        int label = 0;//since query
        for (iterLabel = this->objectDecompositonQueries.begin(); iterLabel != this->objectDecompositonQueries.end(); ++iterLabel)
        {

            std::vector<std::map<int, std::vector<double> > > collection = iterLabel->second;
            for (unsigned int i = 0; i < collection.size(); ++i)
            {
                std::map<int, std::vector<double> > objectDecomp = collection[i];

                for (iterObject = objectDecomp.begin(); iterObject != objectDecomp.end(); ++iterObject)
                {
                    iObject++;
                    //objects from iterObject->first category label!
                    std::vector<double> object = iterObject->second;
                    std::string currentLabel;
                    std::string currentObject;
                    currentLabel = boost::lexical_cast<std::string>(static_cast<int>(label));
                    currentObject = boost::lexical_cast<std::string>(static_cast<int>(iObject));

                    if (this->estimationType == 4)
                    {
                        fileName = std::string(
                                       std::string(this->homePath) + CFileSettings::trainPath[label] + std::string(KDESHELLNORMALESTIMATION_PATH) + currentLabel + "_" + currentObject + "_" + "query_"
                                       + std::string(KDESHELLNORMALESTIMATION_POSTFIX));
                        CFileSettings::saveStdVector(fileName, object);
                    }
                    if (this->estimationType == 2)
                    {
                        fileName = std::string(
                                       std::string(this->homePath) + CFileSettings::trainPath[label] + std::string(KDESHELLESTIMATION_PATH) + currentLabel + "_" + currentObject + "_" + "query_" + "_"
                                       + std::string(KDESHELLESTIMATION_POSTFIX));
                        CFileSettings::saveStdVector(fileName, object);
                    }
                    if (this->addNormalEstimation && this->estimationType != 4)
                    {
                        fileName = std::string(
                                       std::string(this->homePath) + CFileSettings::trainPath[label] + std::string(KDENORMALESTIMATION_PATH) + currentLabel + "_" + currentObject + "_" + "query_" + "_"
                                       + std::string(KDENORMALESTIMATION_POSTFIX));
                        CFileSettings::saveStdVector(fileName, object);
                    }

                    //since it is a query!!
                    labels.push_back(label);
                    objHisto.push_back(object);

                    if (this->isCommon)
                    {
                        this->logger->log->debug("break saving for current objDecomp! since this->isCommon==true\n");
                        break; //since for all objects in the single KDE the bincfg is the same so the hiso of the query will be always the same
                    }
                }
            }
        }

        //Now do combined training set libsvm format
        if (this->estimationType == 4)
        {
            fileName = std::string(std::string(this->homePath) + TRAIN_PATH + KDESHELLNORMALESTIMATION_TRAIN_POSTFIX);
            CFileSettings::saveLibSvmItem(true, fileName, labels, objHisto);
        }
        if (this->addNormalEstimation && this->estimationType != 4)
        {
            fileName = std::string(std::string(this->homePath) + TRAIN_PATH + KDENORMALESTIMATION_TRAIN_POSTFIX);
            CFileSettings::saveLibSvmItem(true, fileName, labels, objHisto);
        }

        if (this->estimationType == 2)
        {
            fileName = std::string(std::string(this->homePath) + TRAIN_PATH + KDESHELLESTIMATION_TRAIN_POSTFIX);
            CFileSettings::saveLibSvmItem(true, fileName, labels, objHisto);
        }
    }
    //  computeSaveNormalizationDensityDistributions();
}

void CObjectGeometricService::saveCollection()
{

    //under construction....
    std::map<int, CObjectDecomposition>::iterator iterObject;
    std::map<int, CObjectDecomposition>::iterator iterObjectDecomp;
    std::map<int, CDensityEstimation>::iterator iterDe;
    CFileSettings::init();

    std::vector<std::vector<double> > featuresShellNormal;
    std::vector<std::vector<double> > featuresShell;
    std::vector<std::vector<double> > featuresNormal;
    std::string fileName;

    this->logger->log->info("CObjectGeometricService::saveCollection...Saving Collection parameters ...\n");
    for (iterObject = this->objectDecompositon.begin(); iterObject != this->objectDecompositon.end(); ++iterObject)
    {
        //objects from iterObject->first category label!
        std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > objects = iterObject->second.getObjects();

        for (unsigned int i = 0; i < objects.size(); ++i)
        {
            std::string currentLabel;
            std::string currentObject;
            currentLabel = boost::lexical_cast<std::string>(static_cast<int>(objects[i].label));
            currentObject = boost::lexical_cast<std::string>(static_cast<int>(i));

            //---------------SAVE POINT CLOUD
            /*  this->logger->log->info(
             "CObjectGeometricService::saveCollection...Saving...pointcloud\n");
             fileName = std::string(std::string(HOME_PATH)
             + CFileSettings::trainPath[objects[i].label] + std::string(
             POINT_CLOUD_PATH) + currentLabel + "_" + currentObject
             + "_" + objects[i].filename + "_" + std::string(
             POINT_CLOUD_POSTIFX));
             pcl::io::savePCDFileASCII(fileName, objects[i].objectPointCloud);

             //---------------SAVE APSP
             this->logger->log->info(
             "CObjectGeometricService::saveCollection...Saving...APSP\n");
             fileName = std::string(std::string(HOME_PATH)
             + CFileSettings::trainPath[objects[i].label] + std::string(
             NEURAL_GAS_PATH) + currentLabel + "_" + currentObject + "_"
             + objects[i].filename + "_" + std::string(
             NEURAL_GAS_ASAP_POSTFIX));
             //objects[i].johnsonApsp.print_matrix(fileName);
             objects[i].neuralGas.createJohnsonAllPairShortestPathGraph(fileName);

             //---------saving neural gas
             this->logger->log->info(
             "CObjectGeometricService::saveCollection...Saving...NeuralGas\n");
             fileName = std::string(std::string(HOME_PATH)
             + CFileSettings::trainPath[objects[i].label] + std::string(
             NEURAL_GAS_PATH) + currentLabel + "_" + currentObject + "_"
             + objects[i].filename + "_" + std::string(
             NEURAL_GAS_NEURALlGAS_POSTFIX));
             objects[i].neuralGas.saveNeuralGas(fileName);*/
        }
    }

    //Save Trees and Forrest for pre classification purposes
    fileName = std::string(std::string(this->homePath) + std::string(OBJECTDECOMPOSTION_PATH) + std::string(OBJECTDECOMPOSTION_FILENAME) + "_" + std::string(OBJECTDECOMPOSTION_POSTFIX));
    this->saveObjectDecompositon(fileName);

    fileName = std::string(std::string(this->homePath) + std::string(RTPNNENSEMBLE_PATH) + std::string(RTPNNENSEMBLE_FILENAME) + "_" + std::string(RTPNNENSEMBLE_POSTFIX));
    this->saveRTPNNEnsemble(fileName);

    fileName = std::string(std::string(this->homePath) + std::string(PNN_PATH) + std::string(PNN_FILENAME) + "_" + std::string(PNN_POSTFIX));
    this->savePnn(fileName);

    //------------------Now do combined training set libsvm format

    if (this->estimationType == 10)
    {
        ;//this->saveEstimations();
    }

    if (this->estimationType == 30)
    {
        this->logger->log->info("CObjectGeometricService::saveCollection...Saving Node based estimation ...\n");
        std::map<int, std::vector<double> >::iterator iterNodes;
        for (iterObject = this->objectDecompositon.begin(); iterObject != this->objectDecompositon.end(); ++iterObject)
        {
            std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > objects = iterObject->second.getObjects();

            for (unsigned int i = 0; i < objects.size(); ++i)
            {

                std::string currentLabel;
                std::string currentObject;
                currentLabel = boost::lexical_cast<std::string>(static_cast<int>(objects[i].label));
                currentObject = boost::lexical_cast<std::string>(static_cast<int>(i));

                std::vector<std::vector<double> > allNodesConcat;
                std::vector<int> labels;
                for (iterNodes = objects[i].nodeKdeEstimations.begin(); iterNodes != objects[i].nodeKdeEstimations.end(); ++iterNodes)
                {
                    allNodesConcat.push_back(iterNodes->second);
                    labels.push_back(objects[i].label);
                }

                fileName = std::string(std::string(this->homePath) + KDEGEONODEESTIMATION_PATH + currentLabel + "_" + currentObject + "_" + KDEGEONODEESTIMATION_TRAIN_UNSUP_POSTFIX);
                CFileSettings::saveUnsupervisedItem(false, fileName, allNodesConcat);

                //Add related Models for current object
                for (iterObjectDecomp = this->objectDecompositon.begin(); iterObjectDecomp != this->objectDecompositon.end(); ++iterObjectDecomp)
                {
                    //if(iterObjectDecomp->first!=iterObject->first)
                    {
                        std::string filenameRM;
                        std::map<int, std::vector<double> > relatedModel;
                        std::string currentRelatedModelLabel = boost::lexical_cast<std::string>(static_cast<int>(iterObjectDecomp->first));
                        relatedModel = iterObjectDecomp->second.estimateNodeEstimation(objects[i]);

                        filenameRM = std::string(
                                         std::string(this->homePath) + KDEGEONODEESTIMATION_PATH + currentLabel + "_" + currentObject + "_" + currentRelatedModelLabel + "_"
                                         + KDEGEONODEESTIMATION_TRAIN_UNSUP_POSTFIX);
                        CFileSettings::saveUnsupervisedItem(false, filenameRM, relatedModel);
                    }
                }

                //  CFileSettings::saveLibSvmItem(false, fileName, labels,
                //      allNodesConcat);
            }
        }
    }
    /*fileName = std::string(std::string(HOME_PATH) + TRAIN_PATH
     + KDESHELLNORMALESTIMATION_TRAIN_POSTFIX);
     CFileSettings::saveLibSvmItem(false, fileName, labels, featuresShellNormal);

     fileName = std::string(std::string(HOME_PATH) + TRAIN_PATH
     + KDENORMALESTIMATION_TRAIN_POSTFIX);
     CFileSettings::saveLibSvmItem(false, fileName, labels, featuresNormal);

     fileName = std::string(std::string(HOME_PATH) + TRAIN_PATH
     + KDESHELLESTIMATION_TRAIN_POSTFIX);
     CFileSettings::saveLibSvmItem(false, fileName, labels, featuresShell);*/
    //-------------------------------------------------------------
    //  computeSaveNormalizationDensityDistributions();
}

void CObjectGeometricService::saveEstimations()
{
    std::map<int, CObjectDecomposition>::iterator iterObject;
    std::map<int, CDensityEstimation>::iterator iterDe;
    CFileSettings::init();
    std::vector<int> labels;
    std::vector<std::vector<double> > featuresShellNormal;
    std::vector<std::vector<double> > featuresShell;
    std::vector<std::vector<double> > featuresNormal;
    std::string fileName;

    for (iterObject = this->objectDecompositon.begin(); iterObject != this->objectDecompositon.end(); ++iterObject)
    {
        //objects from iterObject->first category label!
        std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > objects = iterObject->second.getObjects();

        for (unsigned int i = 0; i < objects.size(); ++i)
        {
            std::string currentLabel;
            std::string currentObject;
            currentLabel = boost::lexical_cast<std::string>(static_cast<int>(objects[i].label));
            currentObject = boost::lexical_cast<std::string>(static_cast<int>(i));

            //-------------------------------------------SAVE ESTIMATIONS !!!
            fileName = std::string(
                           std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(KDESHELLNORMALESTIMATION_PATH) + currentLabel + "_" + currentObject + "_"
                           + objects[i].filename + "_" + std::string(KDESHELLNORMALESTIMATION_POSTFIX));
            CFileSettings::saveStdVector(fileName, objects[i].combinedNormalizedKdeShellNormalEstimations);

            fileName = std::string(
                           std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(KDESHELLESTIMATION_PATH) + currentLabel + "_" + currentObject + "_" + objects[i].filename
                           + "_" + std::string(KDESHELLESTIMATION_POSTFIX));
            CFileSettings::saveStdVector(fileName, objects[i].combinedNormalizedKdeEstimations);

            fileName = std::string(
                           std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(KDENORMALESTIMATION_PATH) + currentLabel + "_" + currentObject + "_" + objects[i].filename
                           + "_" + std::string(KDENORMALESTIMATION_POSTFIX));
            CFileSettings::saveStdVector(fileName, objects[i].combinedNormalizedKdeNormalEstimations);

            labels.push_back(objects[i].label);
            featuresShellNormal.push_back(objects[i].combinedNormalizedKdeShellNormalEstimations);
            featuresShell.push_back(objects[i].combinedNormalizedKdeEstimations);
            featuresNormal.push_back(objects[i].combinedNormalizedKdeNormalEstimations);

            //--------------------------------------------------------------
            //---------------------------------------SAVE SHELL Cfg
            fileName = std::string(
                           std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(SHELL_CONFIG_PATH) + currentLabel + "_" + currentObject + "_" + objects[i].filename + "_"
                           + std::string(SHELL_CONFIG_POSTFIX));

            std::vector<double> shellConfigVec;
            SShellConfig currentShellConfig = objects[i].shellConfig;
            shellConfigVec.push_back(currentShellConfig.shellSize);
            shellConfigVec.push_back(currentShellConfig.shellWidth);
            shellConfigVec.push_back(currentShellConfig.minValue);
            shellConfigVec.push_back(currentShellConfig.maxValue);
            CFileSettings::saveStdVector(fileName, shellConfigVec);
            //-------------------------------------------------------------------
            //------------------------------------SAVE SHELL BinCFG
            std::map<int, CDensityEstimation> kdeShell = objects[i].kdeShell;
            for (iterDe = kdeShell.begin(); iterDe != kdeShell.end(); ++iterDe)
            {
                std::string currentShell = boost::lexical_cast<std::string>(static_cast<int>(iterDe->first));
                fileName = std::string(
                               std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(BIN_CONFIG_SHELL_PATH) + currentLabel + "_" + currentObject + "_" + currentShell + "_"
                               + objects[i].filename + "_" + std::string(BIN_CONFIG_SHELL_POSTFIX));
                std::vector<double> data;

                std::vector<SDensityDistribution> dDist = iterDe->second.getDensityDistributions();
                //Content of Shell cfg !
                data.push_back(dDist.size());
                for (unsigned int iterCfg = 0; iterCfg < dDist.size(); ++iterCfg)
                {
                    data.push_back(dDist[iterCfg].bc.binSize);
                    data.push_back(dDist[iterCfg].bc.binWidth);
                    data.push_back(dDist[iterCfg].bc.minValue);
                    data.push_back(dDist[iterCfg].bc.maxValue);
                    data.push_back(dDist[iterCfg].bc.cost);
                    data.push_back(dDist[iterCfg].bc.mean);
                    data.push_back(dDist[iterCfg].bc.var);
                }
                CFileSettings::saveStdVector(fileName, data);
            }
            //----------------------------------------------------------------------
            //-----------------------------SAVE NORMALE BINCFG
            std::map<int, CDensityEstimation> kdeNormal = objects[i].kdeNormals;
            for (iterDe = kdeNormal.begin(); iterDe != kdeNormal.end(); ++iterDe)
            {
                std::string currentAxis = boost::lexical_cast<std::string>(static_cast<int>(iterDe->first));
                fileName = std::string(
                               std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(BIN_CONFIG_NORMAL_PATH) + currentLabel + "_" + currentObject + "_" + currentAxis + "_"
                               + objects[i].filename + "_" + std::string(BIN_CONFIG_NORMAL_POSTFIX));
                std::vector<double> data;

                std::vector<SDensityDistribution> dDist = iterDe->second.getDensityDistributions();
                //Content of Shell cfg !
                data.push_back(dDist.size());
                for (unsigned int iterCfg = 0; iterCfg < dDist.size(); ++iterCfg)
                {
                    data.push_back(dDist[iterCfg].bc.binSize);
                    data.push_back(dDist[iterCfg].bc.binWidth);
                    data.push_back(dDist[iterCfg].bc.minValue);
                    data.push_back(dDist[iterCfg].bc.maxValue);
                    data.push_back(dDist[iterCfg].bc.cost);
                    data.push_back(dDist[iterCfg].bc.mean);
                    data.push_back(dDist[iterCfg].bc.var);
                }
                CFileSettings::saveStdVector(fileName, data);
            }
            //------------------------------------------------------------
            //SAVE RELATED MODELS
            std::map<int, std::map<int, std::vector<double> > >::iterator iterEstModelLabels;
            std::map<int, std::vector<double> >::iterator iterEstModelObject;
            std::map<int, std::map<int, std::vector<double> > > relatedModels = objects[i].estimatedRelatedModels;
            for (iterEstModelLabels = relatedModels.begin(); iterEstModelLabels != relatedModels.end(); ++iterEstModelLabels)
            {

                std::string currentLabelModel = boost::lexical_cast<std::string>(static_cast<int>(iterEstModelLabels->first));
                std::map<int, std::vector<double> > relatedModelObjects = iterEstModelLabels->second;
                for (iterEstModelObject = relatedModelObjects.begin(); iterEstModelObject != relatedModelObjects.end(); ++iterEstModelObject)
                {

                    std::string currentObjectModel = boost::lexical_cast<std::string>(static_cast<int>(iterEstModelObject->first));

                    fileName = std::string(
                                   std::string(this->homePath) + CFileSettings::trainPath[objects[i].label] + std::string(RELATED_MODEL_ESTIMATIONS_PATH) + currentLabel + "_" + currentObject + "_"
                                   + currentLabelModel + "_" + currentObjectModel + "_" + objects[i].filename + "_" + std::string(RELATED_MODEL_ESTIMATIONS_POSTFIX));

                    CFileSettings::saveStdVector(fileName, iterEstModelObject->second);

                }

            }
        }

        fileName = std::string(
                       std::string(this->homePath) + CFileSettings::trainPath[iterObject->first] + std::string(KDESHELLNORMALESTIMATION_PATH) + CFileSettings::labels[iterObject->first] + std::string(
                           KDESHELLNORMALESTIMATION_POSTFIX));
        CFileSettings::saveUnsupervisedItem(false, fileName, featuresShellNormal);
        //(bool isAppend, std::string fileName,std::vector<std::vector<double> > features)
    }
}

void CObjectGeometricService::loadCollection()
{

    std::map<int, SObjectDescription>::iterator iterObjDesc;
    using boost::lexical_cast;

    CFileSettings::init();

    this->objectDecompositon.clear();

    //std::string dirName;

    //std::string currentlabel;
    //DIR *dp = NULL;
    //struct dirent *dirp=NULL;

    this->logger->log->info("CObjectGeometricService::loadCollection()...Loading Collection parameters ...\n");

    std::string fileName = std::string(std::string(this->homePath) + std::string(OBJECTDECOMPOSTION_PATH) + std::string(OBJECTDECOMPOSTION_FILENAME) + "_" + std::string(OBJECTDECOMPOSTION_POSTFIX));
    this->loadObjectDecompositon(fileName);

    //fileName = std::string(std::string(HOME_PATH) + std::string(RTPNNENSEMBLE_PATH) + std::string(RTPNNENSEMBLE_FILENAME) + "_" + std::string(RTPNNENSEMBLE_POSTFIX));
    //this->loadRTPNNEnsemble(fileName);
    this->loadRTPNNEnsemble();

    //fileName = std::string(std::string(this->homePath) + std::string(PNN_PATH) + std::string(PNN_FILENAME) + "_" + std::string(PNN_POSTFIX));
    //this->loadPnn(fileName);

    //fileName = std::string(std::string(this->homePath) + std::string(KNN_PATH) + std::string(KNN_FILENAME) + "_" + std::string(KNN_POSTFIX));
    //this->loadKNN(fileName);
    //this->rtpnnEnsemble.selfVerificationRTPNNModel();
    //this->rtpnnEnsemble.getNumTrees();
    //this->hProbNeuralNetwork.initHierarchy(HPNN_TRAINTEST_RATIO);
    ////////////this->rtpnnEnsemble.initEnsemble(HPNN_TRAINTEST_RATIO);
    //this->probNeuralNetwork.init();
    //this->probNeuralNetwork.verifyPNN();

    //free(dp);
    //free(dirp);
}

void CObjectGeometricService::loadRTPNNEnsemble()
{
    this->logger->log->info("CObjectGeometricService::loadRTPNNEnsemble()...\n");
    std::string fileName = std::string(std::string(this->homePath) + std::string(RTPNNENSEMBLE_PATH) + std::string(RTPNNENSEMBLE_FILENAME) + "_" + std::string(RTPNNENSEMBLE_POSTFIX));
    this->loadRTPNNEnsemble(fileName);
    this->logger->log->info("CObjectGeometricService::loadRTPNNEnsemble()...done\n");
}

double CObjectGeometricService::getEuclideanDistance(std::vector<double> a, std::vector<double> b)
{

    std::vector<double> delta;
    double deltaSum = 0;

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
    {
        std::cout << "\n" << a.size() << "!=" << b.size() << "\n";
        assert(a.size() == b.size());
        return -1;
    }
}

void CObjectGeometricService::shuffleObjectDecomposition()
{
    this->logger->log->warn("CObjectGeometricService::shuffleObjectDecomposition...not implemented");

    /*  std::map<int, CObjectDecomposition>::iterator iDecomp;
     std::map<int, CObjectDecomposition> shuffledObjectDecomposition;

     for (iDecomp = this->objectDecompositon.begin(); iDecomp
     != this->objectDecompositon.end(); ++iDecomp)
     {
     ;
     }*/
}

std::vector<std::vector<double> > CObjectGeometricService::reconstructShapeRefinement(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointCloud<pcl::Normal> &pointNormal)
{
    std::vector<std::vector<double> > convCloud;

    pcl::PointCloud<pcl::PointNormal> pointCloudNormal;

    pcl::concatenateFields(pointCloud, pointNormal, pointCloudNormal);

    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> >();

    tree->setInputCloud(pointCloudNormal.makeShared());

    //std::cout<<"---> "<<pointCloudNormal.points[0].normal[0]<<"   "<<pointCloudNormal.points.size()<<"\n";
    //if you change this you must change accordingly toolbox.markClusteredPointCloud if correct visualization is required!
    unsigned int k = SHAPE_REFINEMENT_KNN; //30
    if (pointCloudNormal.size() > k)
    {
        for (unsigned int iterPoint = 0; iterPoint < pointCloudNormal.points.size(); iterPoint++)
        {

            pcl::PointNormal point;
            point = pointCloudNormal.points.at(iterPoint);

            //int counterK = 0;
            std::vector<int> k_indicies;
            std::vector<float> k_distances;
            k_distances.resize(k);
            k_indicies.resize(k);
            tree->nearestKSearch(point, k, k_indicies, k_distances);

            std::vector<double> neighborCurvaturesDiff;
            for (unsigned int iK = 0; iK < k; ++iK)
            {
                neighborCurvaturesDiff.push_back(point.curvature - pointCloudNormal.points[k_indicies[iK]].curvature);
            }
            //std::cout << point << std::endl;
            double meanCurDiff = toolBox.mean(neighborCurvaturesDiff);
            double stdDevCurDiff = sqrt(toolBox.variance(neighborCurvaturesDiff, meanCurDiff));
            //std::cout << " |->" << stdDevCurDiff << std::endl;

            //if (abs(pointRGB.curvature) < 1.0)
            if (stdDevCurDiff > SHAPE_REFINEMENT_RATIO) //0.4
            {
                std::vector<double> convPoint;
                std::cout << " x " << point.x << " y " << point.y << " z " << point.z << std::endl;
                convPoint.push_back(point.x);
                convPoint.push_back(point.y);
                convPoint.push_back(point.z);
                convCloud.push_back(convPoint);
            }
        }

    }
    else
    {
        for (unsigned int p = 0; p < this->objectPointCloud.points.size(); p++)
        {
            std::vector<double> convPoint;
            convPoint.push_back(objectPointCloud.points[p].x);
            convPoint.push_back(objectPointCloud.points[p].y);
            convPoint.push_back(objectPointCloud.points[p].z);
            convCloud.push_back(convPoint);
        }
    }

    return convCloud;
}

void CObjectGeometricService::saveRTPNNEnsemble(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::saveRTPNNEnsemble...%s\n", filename.c_str());
        //  std::ofstream ofs("filename");
        std::ofstream ofs(filename.c_str());
        const CRTPNeuralNetworkEnsemble t = this->rtpnnEnsemble;

        boost::archive::text_oarchive oa(ofs);
        //boost::archive::binary_oarchive oa(ofs);
        // write class instance to archive
        oa << t;
        // archive and stream closed when destructors are called

        //this->rtpnnEnsemble.save(filename);

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::saveRTPNNEnsemble...%s", ae.what());
    }
}

void CObjectGeometricService::loadRTPNNEnsemble(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::loadRTPNNEnsemble...%s\n", filename.c_str());
        std::ifstream ifs(filename.c_str()); //("filename");

        CRTPNeuralNetworkEnsemble t;
        //boost::archive::text_iarchive ia(ifs);
        boost::archive::binary_iarchive ia(ifs);
        // write class instance to archive
        ia >> t;

        //################################ TEMP_FRED ########################################
        //std::ofstream ofs(std::string(filename + "_binary").c_str());
        //boost::archive::binary_oarchive oa(ofs);
        //oa << t;

        this->rtpnnEnsemble = t;
        // archive and stream closed when destructors are called

        //  this->rtpnnEnsemble.load(filename);
        this->numberCategories = this->rtpnnEnsemble.getNumCategories();
    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::loadRTPNNEnsemble...%s", ae.what());
    }
}

void CObjectGeometricService::saveObjectDecompositon(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::saveObjectDecompositon...%s\n", filename.c_str());
        std::ofstream ofs(filename.c_str());
        const std::map<int, CObjectDecomposition> t = this->objectDecompositon;

        boost::archive::text_oarchive oa(ofs);
        //boost::archive::binary_oarchive oa(ofs);
        // write class instance to archive
        oa << t;

        this->logger->log->warn("CObjectGeometricService::saveObjectDecompositon...NeuralGasCfg and ROS-PointCloud will not be saved!\n");

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::saveObjectDecompositon...%s", ae.what());
    }

}

void CObjectGeometricService::loadObjectDecompositon(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::loadObjectDecompositon...%s\n", filename.c_str());
        std::ifstream ifs(filename.c_str()); //("filename");

        std::map<int, CObjectDecomposition> t;
        //boost::archive::text_iarchive ia(ifs);
        boost::archive::binary_iarchive ia(ifs);
        // write class instance to archive
        ia >> t;

        //################################ TEMP_FRED ########################################
        //std::ofstream ofs(std::string(filename + "_binary").c_str());
        //boost::archive::binary_oarchive oa(ofs);
        //oa << t;

        this->objectDecompositon = t;

        this->logger->log->warn("CObjectGeometricService::loaded ObjectDecompositon...NeuralGasCfg and ROS-PointCloud will not be loaded!\n");
    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::loadObjectDecompositon...%s", ae.what());
    }
}

void CObjectGeometricService::saveKNN(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::saveKNN...%s\n", filename.c_str());
        std::ofstream ofs(filename.c_str());
        const CKNNClassifier t = this->knnClassifier;

        boost::archive::text_oarchive oa(ofs);
        //boost::archive::binary_oarchive oa(ofs);
        // write class instance to archive
        oa << t;

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::saveKNN...%s", ae.what());
    }
}

void CObjectGeometricService::savePnn(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::savePnn...%s\n", filename.c_str());
        std::ofstream ofs(filename.c_str());
        const CPNeuralNetwork t = this->probNeuralNetwork;

        boost::archive::text_oarchive oa(ofs);
        //boost::archive::binary_oarchive oa(ofs);
        // write class instance to archive
        oa << t;

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::savePnn...%s", ae.what());
    }
}

void CObjectGeometricService::loadKNN(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::loadKNN...%s\n", filename.c_str());
        std::ifstream ifs(filename.c_str()); //("filename");

        CKNNClassifier t;
        boost::archive::text_iarchive ia(ifs);
        //boost::archive::binary_iarchive ia(ifs);
        // write class instance to archive
        ia >> t;

        this->knnClassifier = t;
    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::loadKNN...%s", ae.what());
    }

}

void CObjectGeometricService::loadPnn(std::string filename)
{
    try
    {
        this->logger->log->info("CObjectGeometricService::loadPnn...%s\n", filename.c_str());
        std::ifstream ifs(filename.c_str()); //("filename");

        CPNeuralNetwork t;
        boost::archive::text_iarchive ia(ifs);
        //boost::archive::binary_iarchive ia(ifs);
        // write class instance to archive
        ia >> t;

        this->probNeuralNetwork = t;
    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::loadPnn...%s", ae.what());
    }

}

void CObjectGeometricService::computeMeanModels(std::string category)
{
    std::cout << "CObjectGeometricService::computeMeanModels..." << std::endl;
    if (this->meanModels.empty())
    {
        return;
    }

    std::map<int, std::map<int, std::vector<double> > >::iterator iterModel;
    std::map<int, std::vector<double> >::iterator iterExample;

    for (iterModel = meanModels.begin(); iterModel != meanModels.end(); ++iterModel)
    {
        std::string strLabelModel = boost::lexical_cast<std::string>(static_cast<int>(iterModel->first));

        std::map<int, std::vector<double> > models = iterModel->second;
        for (iterExample = models.begin(); iterExample != models.end(); ++iterExample)
        {
            std::fstream fout1, fout2;
            std::string strExample = boost::lexical_cast<std::string>(static_cast<int>(iterExample->first));
            fout1.open(std::string(category + "_MeanModel_" + strLabelModel + "_" + strExample).c_str(), std::ios::out | std::ios::binary);
            fout2.open(std::string(category + "_stdVarModel_" + strLabelModel + "_" + strExample).c_str(), std::ios::out | std::ios::binary);

            for (unsigned int k = 0; k < iterExample->second.size(); ++k)
            {
                this->meanModels[iterModel->first][iterExample->first][k] /= (float) this->numberQuery;
                fout1 << this->meanModels[iterModel->first][iterExample->first][k] << " ";
                fout2 << sqrt(this->toolBox.variance(this->varModels[iterModel->first][iterExample->first][k], this->meanModels[iterModel->first][iterExample->first][k])) << " ";
            }
            fout1.close();
            fout2.close();
        }
    }

    std::cout << "CObjectGeometricService::computeMeanModels...Reset" << std::endl;
    std::cout << "CObjectGeometricService::computeVarModels...Reset" << std::endl;
    this->meanModels.clear();
    this->varModels.clear();
    this->numberQuery = 0;
}

void CObjectGeometricService::printModel(std::map<int, std::map<int, std::vector<double> > > model, int label, std::string filename)
{
    std::map<int, std::map<int, std::vector<double> > >::iterator iterModel;
    std::map<int, std::vector<double> >::iterator iterExample;

    std::string strLabel = boost::lexical_cast<std::string>(static_cast<int>(label));
    for (iterModel = model.begin(); iterModel != model.end(); ++iterModel)
    {
        std::string strLabelModel = boost::lexical_cast<std::string>(static_cast<int>(iterModel->first));

        std::map<int, std::vector<double> > models = iterModel->second;
        for (iterExample = models.begin(); iterExample != models.end(); ++iterExample)
        {
            std::fstream fout1;

            std::string strExample = boost::lexical_cast<std::string>(static_cast<int>(iterExample->first));
            fout1.open(std::string(filename + "_" + strLabel + "_against_" + strLabelModel + "_" + strExample).c_str(), std::ios::out | std::ios::binary);

            for (unsigned int k = 0; k < iterExample->second.size(); ++k)
            {
                if (this->meanModels[iterModel->first][iterExample->first].empty())
                {
                    this->meanModels[iterModel->first][iterExample->first].resize(iterExample->second.size());
                }
                if (this->varModels[iterModel->first][iterExample->first].empty())
                {
                    this->varModels[iterModel->first][iterExample->first].resize(iterExample->second.size());
                }

                this->meanModels[iterModel->first][iterExample->first][k] += iterExample->second[k];
                this->varModels[iterModel->first][iterExample->first][k].push_back(iterExample->second[k]);
                fout1 << iterExample->second[k] << " ";
            }

            fout1.close();
        }
    }

}
