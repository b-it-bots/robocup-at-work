/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef CObjectGeometricService_H
#define CObjectGeometricService_H

#include <iostream>
#include <string>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include "toolbox_ros.h"
#include "kde.h"
#include "object_decomposition.h"
#include "geometric_features.h"
#include "file_settings.h"
#include "logger.h"
#include "johnson_apsp/apsp_graph.hpp"
#include "prob_neural_network/p_neural_network.h"
#include "prob_neural_network/hp_neural_network.h"
#include "prob_neural_network/rtp_neural_network.h"
#include "prob_neural_network/rtp_neural_network_ensemble.h"
#include "prob_neural_network/knn_classifier.h"
#include "neural_gas/neural_gas_node.h"
#include "neural_gas/neural_gas.h"
#include "neural_gas/neural_gas_classifier.h"

#define numShells 5

//if you change this you must change accordingly toolbox.markClusteredPointCloud if correct visualization is required!
#define SHAPE_REFINEMENT_KNN 25

#define SHAPE_REFINEMENT_RATIO 0.4

#define HPNN_TRAINTEST_RATIO 0.5

#define KPOINTS_NORMALS 10

#define POINT_NORMAL_ESTIMATION_METHOD 1

class CObjectGeometricService
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    friend class boost::serialization::access;
    CLogger *logger;
    CToolBoxROS toolBox;
    float ng_consistDist;
    float ng_errorAdd;
    float ng_nodeAddInterval; // ng_errorAdd == -1
    float ng_maxEdgeAge;
    float ng_modelAccuracy;

    float ng_errorNewNodeDecrease;
    float ng_errorGeneralDecrease;

    float ng_nearestNeighborWeight; //move the nearest node to target
    float ng_nNeighborWeight; //move other neigbors to target
    bool ng_normalize;

    int estimationType;
    bool addNormalEstimation;
    bool isNodeHandle;

    CNeuralGasClassifier *neuralGasShapeReconstructor;
    std::string filename; // to save current object details

    //label -> objectDecompostion
    std::map<int, CObjectDecomposition> objectDecompositon;

    CPNeuralNetwork probNeuralNetwork;
    CHPNeuralNetwork hProbNeuralNetwork;
    CRTPNeuralNetworkEnsemble rtpnnEnsemble;
    CKNNClassifier knnClassifier;

    //label -> vector of belonging objects
    std::map<int, std::vector<SGeometricFeatures> > objectGeometricFeatures;

    //bincfg
    bool isCommon;
    ros::NodeHandle *nodeHandle;

    std::string homePath;


    int numberQuery; //number of query which have been applied yet.
    //labelBincfg, examples values, summed
    std::map<int, std::map<int, std::vector<double> > > meanModels; //Mean density estimation for each class : accumulated by printModels and computed by computed computeMeanModels().
    std::map<int, std::map<int, std::vector<std::vector<double> > > > varModels;
protected:
    /////////////////////////////////////////////////////////////
    //All properties from the current input ObjectPointsCloud,
    SObjectDescription objectDescription;
    pcl::PointCloud<pcl::PointXYZ> objectPointCloud; //current pcl
    pcl::PointCloud<pcl::Normal> objectPointCloudNormals; //current pcl
    apsp_graph johnsonApsp;
    std::map<node*, std::vector<double> > apspDistances; // allpair distance to each node
    std::map<node*, double> meanApspDistances;
    int label;
    std::map<int, std::pair<int, double> > currentTreeReponses; // current Tree responses from query(Random Forrest)
    ///////////////////////////////////////////////////////////////7

    //An individial histogram of the collection objects from the query is done.
    //label, some query, result histo according the i's object, histo
    std::map<int, std::vector<std::map<int, std::vector<double> > > > objectDecompositonQueries;

    std::map<int, std::map<int, std::vector<double> > > estimateModel(SObjectDescription &objDesc, bool doSave);
    void extractStatisticalFeatures();

    int getNearestNode(std::map<int, std::vector<double> > nodePosition, std::vector<double> position);
    double getEuclideanDistance(std::vector<double> a, std::vector<double> b);
    //  void computeSaveNormalizationDensityDistributions();
    void shuffleObjectDecomposition();

    void computeNodeToCenterDistances(SObjectDescription &oD);
    void computeAllPairNormalDifferences(SObjectDescription &oD);

    std::map<int, std::vector<double> > enNoiseEstimation(std::map<int, std::vector<double> > estimation, double addNoise);
    void printModel(std::map<int, std::map<int, std::vector<double> > > model, int label, std::string filename);

    //It is set after init!!!
    int numberCategories;

    float enNoisement;
public:
    //CObjectGeometricService();
    CObjectGeometricService(int estimationType = 1, bool addNormalEstimation = true);

    void setNodeHandle(ros::NodeHandle& nodeHandle)
    {
        isNodeHandle = true;
        this->nodeHandle = &nodeHandle;
        std::cout << "SETNODE____1\n";
    }

    void inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> pointCloud, int label = -1, std::string filename = "noName");
    void setParametersShapeRecontructor(float ng_consistDist, float ng_errorAdd, float ng_nodeAddInterval, float ng_maxEdgeAge, float ng_errorNewNodeDecrease, float ng_errorGeneralDecrease,
                                        float ng_nearestNeighborWeight, float ng_nNeighborWeight, bool normalize, float modelAccuracy = -1);
    void reconstructShape(bool addToCollection = false);

    std::pair<int, double> query(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave = false, std::string name = "query");
    std::pair<int, double> query(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, std::map<int, std::pair<int, double> > &getCurrentTreeReponses);
    std::pair<int, double> queryPnn(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave = false, std::string name = "query");
    std::pair<int, double> queryKNN(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, bool doSave = false, std::string name = "query");

    std::vector< std::pair<int, double> > queryEnNoised(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, std::vector <std::map<int, std::pair<int, double> > > &getCurrentTreeReponses, int numEnNoisements);

    void createDensityDistribution(bool addToCollection = false);

    void initPnn(bool reset = false);
    void initKNN();
    void initEstimation(bool trainEnsemble = true);
    void initNodeEstimation();
    std::map<int, std::vector<double> > loadASPSMatrix(std::string filename);
    pcl::PointCloud<pcl::PointXYZ> normalizePointCould(pcl::PointCloud<pcl::PointXYZ> objectPointCloud);

    pcl::PointCloud<pcl::Normal> computeNormals(pcl::PointCloud<pcl::PointXYZ> objectPointCloud);

    void setIsCommonHistogrammCfg(bool isCommon);
    void saveCollection();
    void loadCollection();
    void saveQueryCollection();
    void saveEstimations();

    void saveObjectDecompositon(std::string filename);
    void saveRTPNNEnsemble(std::string filename);
    void savePnn(std::string filename);
    void saveKNN(std::string filename);
    void loadObjectDecompositon(std::string filename);
    void loadRTPNNEnsemble(std::string filename);
    void loadRTPNNEnsemble();
    void loadPnn(std::string filename);
    void loadKNN(std::string filename);

    std::vector<std::vector<double> > reconstructShapeRefinement(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointCloud<pcl::Normal> &pointNormal);

    void setHomePath(std::string homePath);


    int getNumCategories()
    {
        return this->numberCategories;
    }
    int getNumNeuronsASPSMatrix(std::string filename);
    std::map<int, CObjectDecomposition> getObjectDecompositon()
    {
        return objectDecompositon;
    }
    SObjectDescription getLastObjectDecomposition()
    {
        return this->objectDescription;
    }
    void clearObjectDecomposition()
    {
        objectDecompositon.clear();
    }

    void computeMeanModels(std::string category);

    void verifyPnn();
    void verifyKNN();
};

BOOST_CLASS_VERSION(CObjectGeometricService, 1)
#endif
