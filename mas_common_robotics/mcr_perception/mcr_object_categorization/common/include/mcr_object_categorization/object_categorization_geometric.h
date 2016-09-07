/*
 *  CObjectPseudoCategorizationGeometric.h
 *
 *  Created on: 21.06.2011
 *      Author: Christian Mueller
 */

#ifndef COBJECTCATEGORIZATIONGEOMETRIC_H
#define COBJECTCATEGORIZATIONGEOMETRIC_H

#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <vector>
#include <string>
#include <limits>
#include <map>
#include <algorithm>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "toolbox_ros.h"

#include "logger.h"
#include "object_point_cloud_data.h"
#include "object_pseudo_categorization_geometric.h"
#include "sparse_auto_encoder/stacked_auto_encoder.h"
#include "svm/svm_classifier.h"
#include "svm/svm_cascade.h"
#include "svm/svm_one_class_classifier.h"
#include "svm/online_lib_linear/svm_linear_classifier.h"

//min confidence which leads still to a classification ... if below then no classifcation == query == 0 ;
#define MIN_CONFIDENCE 0.70

//Noise to add for EnNoisement (% of value for each dimension XYZ)
#define OBJ_CAT_GEO_ADDNOISE 0.1

class CObjectCategorizationGeometric
{
    //ADAPT 1
    //BootStrapping 2
#define SVM_TYPE 1

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    CLogger *logger;
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & trainingData;
        ar & testingData;
        ar & testingData2;
        //ar & autoEncoders;
        //ar & svms;
        //ar & isPseudoCatLoaded;
        //ar & objectPseudoCategorizationGeometric;

        //ar & doTrainAutoEncoder;
        //ar & doTrainSvm;
        //ar & doQueryAutoEncoder;
        //ar & doQuerySvm;
    }

protected:
    CToolBoxROS toolBox;
    CObjectPseudoCategorizationGeometric objectPseudoCategorizationGeometric;

    SObjectPointCloudData trainingData;
    SObjectPointCloudData testingData;
    SObjectPointCloudData testingData2;

    //int numCategories;
    bool isPseudoCatLoaded;

    //AutoEncoders
    std::map<int, CStackedAutoEncoder> autoEncoders;
    std::map<int, double> aEConfidences;

    //SVM
    std::map<int, CSvmClassifier> svms;

    std::map<int, CSvmOneClassClassifier> svmOnes;
    std::map<int, double> svmOnesConfidences;

    //SVM Cascade
    std::map<int, CSvmCascade> svmCascade;

    CSvmLinearClassifier svmLinear;

    //print
    void print(std::map<int, std::vector<std::vector<double> > > &featureVector);


    void verifySvm(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);

    void verifySvmLinear(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);

    void loadSVMModel();
    void saveSVMModel();

    void loadSVMCascade();
    void loadSVMOneClassModel();

    void loadSVMLinearModel();
    void saveSVMLinearModel();

    void saveSVMCascade();
    void loadAutoEncoderModel();
    void saveAutoEncoderModel();

    void augmentSetWithScalePyramid(SObjectPointCloudData &set);
    void augmentSetWithEnNoisement(SObjectPointCloudData &set);

    std::map<int, std::vector<std::vector<double> > > createOneVsAllsetMap(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors);

    bool doTrainAutoEncoder;
    bool doTrainSvm;
    bool doTrainSvmLinear;

    bool doQueryAutoEncoder;
    bool doQuerySvm;
    bool doQuerySvmLinear;

    std::string homePath;

    float minConfidence;
    float addEnNoisement;
    bool doTrainScalePyramid;

    std::string tag;
    bool isTag;

public:
    CObjectCategorizationGeometric();

    void inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, int label, int type);
    void inputObjectPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames, int type);
    void inputObjectPointClouds(std::string dataBasePath, int maxNumObjectPerClass, int type, bool concat);
    void inputObjectPointClouds(std::string dataPath, int maxNumObjectPerClass, int type, bool concat, int label);

    void inputObjectPointCloudsPseudoCategories(std::string dataPathPseudoCategory, int maxNumObjectPerClass, bool concat = false);
    void trainPseudoCategories();
    void trainPseudoCategories(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames);

    void loadPseudoCategories();
    void loadModel();

    void train();

    std::vector<std::vector<double> > createBootstrap(std::vector<std::vector<double> > &featureVectors, unsigned int numberVec);
    std::vector<std::vector<double> > createOneVsAllset(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors, double ratio = -1.0);
    std::vector<std::vector<double> > createOneClassSet(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors);
    std::pair<std::vector<std::vector<double> >, std::vector<std::vector<double> > > splitPosNegSet(std::vector<std::vector<double> > set);
    std::vector<std::vector<double> > removeRandExamples(std::vector<std::vector<double> > &set, unsigned int numtoExtractExamples);

    std::vector<double> extractFeatureVectorRT(std::map<int, std::pair<int, double> > &treeReponses);
    std::pair<int, double> query(pcl::PointCloud<pcl::PointXYZ> queryPointCloud);
    std::pair<int, double> queryAE(std::vector<double> featureVector);

    //one vs all; rbf svm
    std::pair<int, double> query(std::vector<double> featureVector);

    //multiclass; linear svm
    std::pair<int, double> query2(std::vector<double> featureVector);
    std::pair<int, double> queryEnNoised(pcl::PointCloud<pcl::PointXYZ> queryPointCloud);

    void verifyAutoEncoders();
    void verifySvmOneClass();

    void trainAutoEncoders(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors);
    void trainSvms(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void trainSvmLinears(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void trainSvmsAdaptive(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void trainSvmsAdaptiveParallel(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
                                   std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void trainSvmsBoostrapping(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void loadTraintrain();
    void trainNeuralGas(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);
    void trainSvmOneClass(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors, std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors);

    void setDoTrainAutoEncoder(bool train)
    {
        this->doTrainAutoEncoder = train;
    }
    void setDoTrainSvm(bool train)
    {
        this->doTrainSvm = train;
    }

    void setDoTrainSvmLinear(bool train)
    {
        this->doTrainSvmLinear = train;
    }

    void setDoQuerySvm(bool query)
    {
        this->doQuerySvm = query;
    }

    void setDoQuerySvmLinear(bool query)
    {
        this->doQuerySvmLinear = query;
    }

    void setDoQueryAutoEncoder(bool query)
    {
        this->doQueryAutoEncoder = query;
    }

    void setHomePath(std::string homePath);

    SObjectPointCloudData getTrainingData()
    {
        return this->trainingData;
    }
    SObjectPointCloudData getTestingData()
    {
        return this->trainingData;
    }

    void setTag(std::string tag)
    {
        this->tag = tag;
        isTag = true;
    }
    //pointcloud
    double evaluateQuery1();
    void evaluateQuery2();
    double evaluateQuery22(SObjectPointCloudData dataSet);
    double evaluateQuery22(int set = 2);
    void evaluateSvm1();
    bool evaluateSvm2();
    void evaluateSvm3();
    bool evaluateSvm4();
    bool querySvm(pcl::PointCloud<pcl::PointXYZ> queryPointCloud, unsigned int category);
    std::pair<int, double> queryPseudo(pcl::PointCloud<pcl::PointXYZ> queryPointCloud);
    std::pair<int, std::vector<double> > queryPseudo2(pcl::PointCloud<pcl::PointXYZ> queryPointCloud);
    void updateModelSvmLinear(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors);


};

BOOST_CLASS_VERSION(CObjectCategorizationGeometric, 1)

#endif

