/*
 * CObjectPseudoCategorizationGeometric.cpp
 *
 *  Created on: Jun 21, 2011
 *      Author: Christian Mueller
 */

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <assert.h>
#include <fstream>
#include <omp.h>

#include <boost/random.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "object_categorization_geometric.h"
#include "file_settings.h"
#include "sparse_auto_encoder/stacked_auto_encoder.h"
#include "svm/svm_classifier.h"
#include "svm/b_svm_classifier.h"
#include "svm/svm_cascade.h"
#include "svm/svm_one_class_classifier.h"
#include "neural_gas/neural_gas_classifier.h"

CObjectCategorizationGeometric::CObjectCategorizationGeometric()
{
    CFileSettings::init();
    this->logger = &CLogger::getInstance();
    this->isPseudoCatLoaded = false;
    this->doTrainAutoEncoder = false;
    this->doTrainSvm = false;
    this->doTrainSvmLinear = false;

    this->doQueryAutoEncoder = false;
    this->doQuerySvm = false;
    this->doQuerySvmLinear = false;
    this->homePath = std::string(HOME_PATH);
    this->minConfidence = MIN_CONFIDENCE;
    this->doTrainScalePyramid = true;
    this->addEnNoisement = OBJ_CAT_GEO_ADDNOISE;

    this->tag = "no Tag";
    isTag = false;
}

void CObjectCategorizationGeometric::setHomePath(std::string homePath)
{
    this->homePath = homePath;
    this->objectPseudoCategorizationGeometric.setHomePath(this->homePath);
}

void CObjectCategorizationGeometric::loadModel()
{
    this->logger->log->info("CObjectCategorizationGeometric::loadModel...");

    this->logger->log->info("CObjectCategorizationGeometric::loadModel...loadPseudoCategories");
    this->loadPseudoCategories();

    if (this->doQuerySvm)
    {
        this->logger->log->info("CObjectCategorizationGeometric::loadModel... loadSVMModel");
        this->loadSVMModel();

        //this->logger->log->info("CObjectCategorizationGeometric::loadModel... loadSVMCascadeModel");
        //this->loadSVMCascade();

        /*
        this->logger->log->info("CObjectCategorizationGeometric::loadModel... loadSVMOneClass");
        this->loadSVMOneClassModel();

        this->logger->log->info("CObjectCategorizationGeometric::loadModel... loadSVMOneClass Confidences are fixed!!");
        //extract by calling svmOneVerify after trainig svmoneclass
        //classification errors
        svmOnesConfidences[1] = 0.111111;
        svmOnesConfidences[2] = 0.022222;
        svmOnesConfidences[3] = 0.066666;
        svmOnesConfidences[4] = 0.033333;
        svmOnesConfidences[5] = 0.022222;
        svmOnesConfidences[6] = 0.0;
        svmOnesConfidences[7] = 0.0444444;
        */
    }
    if (this->doQuerySvmLinear)
    {
        this->loadSVMLinearModel();
    }
    if (this->doQueryAutoEncoder)
    {
        this->logger->log->info("CObjectCategorizationGeometric::loadModel...loadAutoEncoderModel");
        this->loadAutoEncoderModel();
    }
    this->logger->log->info("CObjectCategorizationGeometric::loadModel...done");
}

void CObjectCategorizationGeometric::loadSVMModel()
{
    this->logger->log->info("CObjectCategorizationGeometric::loadSVMModel...\n");

    if (SVM_TYPE == 1)
    {
        for (unsigned int iterCategories = 1; iterCategories < NUM_CLASSES; ++iterCategories)
        {
            std::string modelFileName = std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterCategories]) + "_"
                                        + std::string(SVM_MODEL_POSTFIX);
            if (boost::filesystem::exists(modelFileName))
            {
                this->svms[iterCategories].loadSvmModel(modelFileName);
            }
            else
            {
                this->logger->log->info("CObjectCategorizationGeometric::loadSVMModel()....could not open File: %s\n", modelFileName.c_str());
            }
        }
    }
    this->logger->log->info("CObjectCategorizationGeometric::loadSVMModel()....done\n");
}

void CObjectCategorizationGeometric::loadSVMOneClassModel()
{
    this->logger->log->info("CObjectCategorizationGeometric::loadSVMOneClass...\n");

    //if (SVM_TYPE == 1)
    {
        for (unsigned int iterCategories = 1; iterCategories < NUM_CLASSES; ++iterCategories)
        {
            std::string modelFileName = std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_ONECLASS_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterCategories]) + "_"
                                        + std::string(SVM_MODEL_POSTFIX);
            if (boost::filesystem::exists(modelFileName))
            {
                this->svmOnes[iterCategories].loadSvmModel(modelFileName);
            }
            else
            {
                this->logger->log->info("CObjectCategorizationGeometric::loadSVMOneClass()....could not open File: %s\n", modelFileName.c_str());
            }
        }
    }
    this->logger->log->info("CObjectCategorizationGeometric::loadSVMOneClass()....done\n");
}

void CObjectCategorizationGeometric::saveSVMModel()
{
    this->logger->log->info("CObjectCategorizationGeometric::saveSVMMode...");
    assert(this->svms.size() > 0);

    std::map<int, CSvmClassifier>::iterator iterSVM;

    for (iterSVM = this->svms.begin(); iterSVM != this->svms.end(); ++iterSVM)
    {
        iterSVM->second.saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterSVM->first]) + "_" + std::string(SVM_MODEL_POSTFIX));
    }
    this->logger->log->info("CObjectCategorizationGeometric::saveSVMMode...done");
}

void CObjectCategorizationGeometric::saveSVMCascade()
{
    this->logger->log->info("CObjectCategorizationGeometric::saveSVMCascade...");
    assert(this->svmCascade.size() > 0);

    std::map<int, CSvmCascade>::iterator iterSVM;

    for (iterSVM = this->svmCascade.begin(); iterSVM != this->svmCascade.end(); ++iterSVM)
    {
        iterSVM->second.saveCascadeModel();
        std::string filename = std::string(
                                   std::string(this->homePath) + std::string(SVM_CASCADE_PATH) + std::string(SVM_MODEL_CASCADE_FILENAME) + "_" + boost::lexical_cast<std::string>(iterSVM->first) + "_" + std::string(
                                       CFileSettings::labels[iterSVM->first]) + "_" + std::string(SVM_MODEL_POSTFIX));

        try
        {
            this->logger->log->info("CObjectCategorizationGeometric::saveSVMCascade...%s\n", filename.c_str());
            std::ofstream ofs(filename.c_str());
            const CSvmCascade t = iterSVM->second;

            boost::archive::text_oarchive oa(ofs);
            //boost::archive::binary_oarchive oa(ofs);
            // write class instance to archive
            oa << t;

        }
        catch (boost::archive::archive_exception ae)
        {
            printf("CObjectCategorizationGeometric::saveSVMCascade..%s", ae.what());
        }

    }
    this->logger->log->info("CObjectCategorizationGeometric::saveSVMCascade...done");
}

void CObjectCategorizationGeometric::loadSVMCascade()
{
    this->logger->log->info("CObjectCategorizationGeometric::loadSVMCascade...");
    //assert(this->svmCascade.size()>0);

    std::map<int, CSvmCascade>::iterator iterSVM;

    DIR *dp;
    struct dirent *dirp;
    std::string filePath = std::string(std::string(this->homePath) + std::string(SVM_CASCADE_PATH));

    std::cout << "Trying to open folder " << filePath << "... ";
    if ((dp = opendir(filePath.c_str())) == NULL)
    {
        std::cout << "Error(" << errno << ") opening " << filePath << std::endl;
        exit(0);
    }
    std::cout << "succeeded\n";

    while ((dirp = readdir(dp)) != NULL)
    {
        std::string file = string(dirp->d_name);
        if (file.compare(std::string(".")) != 0 && file.compare(std::string("..")) != 0)
        {

            std::vector<std::string> fileTokens;
            CFileSettings::tokenize(file, fileTokens, std::string("_"));

            assert(boost::lexical_cast<unsigned int>(fileTokens[1]) < NUM_CLASSES);
            int label = boost::lexical_cast<unsigned int>(fileTokens[1]);

            try
            {
                this->logger->log->info("CObjectCategorizationGeometric::loadSVMCascade...%s", file.c_str());
                std::ifstream ifs(std::string(filePath + file).c_str()); //("filename");

                CSvmCascade t;
                boost::archive::text_iarchive ia(ifs);
                //boost::archive::binary_iarchive ia(ifs);
                // write class instance to archive
                ia >> t;

                this->svmCascade[label] = t;
                this->svmCascade[label].setHomePath(this->homePath);
                //this->logger->log->info("CObjectCategorizationGeometric::loadSVMCascade...Model");
                this->svmCascade[label].loadCascadeModel();
            }
            catch (boost::archive::archive_exception ae)
            {
                printf("CObjectCategorizationGeometric::loadSVMCascade..%s", ae.what());
            }

        }

    }

    this->logger->log->info("CObjectCategorizationGeometric::loadSVMCascade...done");
}

void CObjectCategorizationGeometric::saveSVMLinearModel()
{
    this->svmLinear.saveModel2(std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_LINEAR_MODEL_FILENAME));
}

void CObjectCategorizationGeometric::loadSVMLinearModel()
{
    this->svmLinear.loadOfflineData(std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_LINEAR_MODEL_FILENAME));
    std::cout << "CObjectCategorizationGeometric::loadSVMLinearModel...loaded" << std::endl;
}

void CObjectCategorizationGeometric::loadAutoEncoderModel()
{
    this->logger->log->info("CObjectCategorizationGeometric::loadAutoEncoderModel...");
    try
    {
        std::string fileName = std::string((this->homePath) + std::string(AUTOENCODER_MODEL_PATH) + std::string(AUTOENCODER_MODEL_FILENAME) + "_" + std::string(AUTOENCODER_MODEL_POSTFIX));
        this->logger->log->info("CObjectCategorizationGeometric::loadAutoEncoderModel...%s\n", fileName.c_str());
        std::ifstream ifs(fileName.c_str());

        std::map<int, CStackedAutoEncoder> t;
        boost::archive::text_iarchive ia(ifs);
        ia >> t;
        this->autoEncoders = t;

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectGeometricService::loadAutoEncoderModel...%s", ae.what());
    }

    //errors
    //aEConfidences[1] = 0.163492;
    //aEConfidences[2] = 0.17619;
    //aEConfidences[3] = 0.103175;
    //aEConfidences[4] = 0.192063;
    //aEConfidences[5] = 0.0380952;
    //aEConfidences[6] = 0.193651;
    //aEConfidences[7] = 0.133333;

    //bayes P(isCup|detectedCup) =...
    aEConfidences[1] = 0.485437;
    aEConfidences[2] = 0.810811;
    aEConfidences[3] = 0.953846;
    aEConfidences[4] = 0.743802;
    aEConfidences[5] = 0.916667;
    aEConfidences[6] = 0.532787;
    aEConfidences[7] = 0.833333;
    this->logger->log->info("CObjectCategorizationGeometric::loadAutoEncoderModel... done");
}

void CObjectCategorizationGeometric::saveAutoEncoderModel()
{
    this->logger->log->info("CObjectCategorizationGeometric ::saveAutoEncoderModel...");
    try
    {
        std::string fileName = std::string((this->homePath) + std::string(AUTOENCODER_MODEL_PATH) + std::string(AUTOENCODER_MODEL_FILENAME) + "_" + std::string(AUTOENCODER_MODEL_POSTFIX));
        this->logger->log->info("CObjectCategorizationGeometric::saveAutoEncoderModel...%s\n", fileName.c_str());
        std::ofstream ofs(fileName.c_str());
        const std::map<int, CStackedAutoEncoder> t = this->autoEncoders;
        boost::archive::text_oarchive oa(ofs);
        oa << t;

        this->logger->log->warn("CObjectCategorizationGeometric::saveAutoEncoderModel...Done!\n");

    }
    catch (boost::archive::archive_exception ae)
    {
        this->logger->log->error("CObjectCategorizationGeometric::saveAutoEncoderModel...%s", ae.what());
    }
    this->logger->log->info("CObjectCategorizationGeometric ::saveAutoEncoderModel...done");
}

void CObjectCategorizationGeometric::inputObjectPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames, int type)
{
    switch (type)
    {
    case 1:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...trainingExamples added");
        this->trainingData.objectPointClouds = objectPointClouds;
        this->trainingData.objectLabels = labels;
        this->trainingData.objectNames = objectNames;
        break;
    case 2:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...testing Examples added");
        this->testingData.objectPointClouds = objectPointClouds;
        this->testingData.objectLabels = labels;
        this->testingData.objectNames = objectNames;
        break;
    default:
        assert(type == 1 || type == 2);
    }
}

void CObjectCategorizationGeometric::inputObjectPointCloud(pcl::PointCloud<pcl::PointXYZ> objectPointCloud, int label, int type)
{
    std::cout << "CObjectCategorizationGeometric::inputObjectPointCloud... not implemented\n";
}

void CObjectCategorizationGeometric::inputObjectPointClouds(std::string dataPath, int maxNumObjectPerClass, int type, bool concat, int label)
{

    switch (type)
    {
    case 1:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...adding training examples from %s", dataPath.c_str());
        if (!concat)
        {
            this->trainingData.objectPointClouds.clear();
        }
        break;
    case 2:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...adding testing examples from %s", dataPath.c_str());
        if (!concat)
        {
            this->testingData.objectPointClouds.clear();
        }
        break;
    default:
        assert(type == 1 || type == 2);
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
            std::cout << "Error(" << errno << ") opening " << pointCloudPath << std::endl;
            exit(0);
        }
        std::cout << "succeeded\n";

        while ((dirp = readdir(dp)) != NULL)
        {
            std::string file = string(dirp->d_name);
            if (file.compare(std::string(".")) != 0 && file.compare(std::string("..")) != 0)
            {
                if (currNumObject > maxNumObjectPerClass)
                    break;

                std::vector<std::string> fileTokens;
                CFileSettings::tokenize(file, fileTokens, std::string("_"));

                ////Check whether it is a correct pcd File
                //assert(boost::lexical_cast<unsigned int>(fileTokens[0]) == iterCategory);
                //assert(fileTokens[fileTokens.size()-1] == POINT_CLOUD_POSTIFX);

                if (pcl::io::loadPCDFile(std::string(pointCloudPath + file), pointCloudPCD) == -1)
                {
                    this->logger->log->error("Was not able to open file  %s .\n", file.c_str());
                    exit(0);
                }
                else //Successfully loaded pcd
                {
                    std::cout << "Point cloud file (" << file << ") successfully opened - point cloud size = (" << pointCloudPCD.size() << ")\n";
                    ;
                    std::string objectLabel = boost::lexical_cast<std::string>(label);
                    switch (type)
                    {
                    case 1:
                        this->trainingData.objectPointClouds.push_back(pointCloudPCD);
                        this->trainingData.objectLabels.push_back(label);
                        this->trainingData.objectNames.push_back(std::string(file + "-" + objectLabel));

                        this->trainingData.numExamples[label]++;
                        break;
                    case 2:
                        this->testingData.objectPointClouds.push_back(pointCloudPCD);
                        this->testingData.objectLabels.push_back(label);
                        this->testingData.objectNames.push_back(std::string(file + "-" + objectLabel));

                        this->testingData.numExamples[label]++;
                        break;

                    default:
                        assert(type == 1 || type == 2);
                    }

                }
                currNumObject++;
            }
        }
    }
    this->logger->log->debug("CObjectCategorizationGeometric::inputObjectPointClouds...done!");

    cout << "this->trainingData.objectPointClouds,size = " << this->trainingData.objectPointClouds.size() << "\n";
    cout << "this->testingData.objectPointClouds,size = " << this->testingData.objectPointClouds.size() << "\n";
}

void CObjectCategorizationGeometric::inputObjectPointClouds(std::string dataPath, int maxNumObjectPerClass, int type, bool concat)
{

    switch (type)
    {
    case 1:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...adding training examples from %s", dataPath.c_str());
        if (!concat)
        {
            this->trainingData.objectPointClouds.clear();
        }
        break;
    case 2:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...adding testing examples from %s", dataPath.c_str());
        if (!concat)
        {
            this->testingData.objectPointClouds.clear();
        }
        break;
    case 3:
        this->logger->log->info("CObjectCategorizationGeometric::inputObjectPointClouds...adding testing examples from %s", dataPath.c_str());
        if (!concat)
        {
            this->testingData2.objectPointClouds.clear();
        }
        break;
    default:
        assert(type == 1 || type == 2 || type == 3);
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

                std::vector<std::string> fileTokens;
                CFileSettings::tokenize(file, fileTokens, std::string("_"));

                //Check whether it is a correct pcd File
                assert(boost::lexical_cast<unsigned int>(fileTokens[0]) == iterCategory);
                assert(fileTokens[3] == POINT_CLOUD_POSTIFX);

                if (pcl::io::loadPCDFile(std::string(pointCloudPath + file), pointCloudPCD) == -1)
                {
                    this->logger->log->error("Was not able to open file  %s .\n", file.c_str());
                    exit(0);
                }
                else //Successfully loaded pcd
                {
                    std::cout << "Point cloud file (" << file << ") successfully opened - point cloud size = (" << pointCloudPCD.size() << ")\n";
                    int objectLabel = boost::lexical_cast<unsigned int>(fileTokens[0]);
                    switch (type)
                    {
                    case 1:
                        this->trainingData.objectPointClouds.push_back(pointCloudPCD);
                        this->trainingData.objectLabels.push_back(objectLabel);
                        this->trainingData.objectNames.push_back(std::string(fileTokens[2] + "-" + fileTokens[0] + "-" + fileTokens[1]));

                        this->trainingData.numExamples[objectLabel]++;
                        break;
                    case 2:
                        this->testingData.objectPointClouds.push_back(pointCloudPCD);
                        this->testingData.objectLabels.push_back(objectLabel);
                        this->testingData.objectNames.push_back(std::string(fileTokens[2] + "-" + fileTokens[0] + "-" + fileTokens[1]));

                        this->testingData.numExamples[objectLabel]++;
                        break;
                    case 3:
                        this->testingData2.objectPointClouds.push_back(pointCloudPCD);
                        this->testingData2.objectLabels.push_back(objectLabel);
                        this->testingData2.objectNames.push_back(std::string(fileTokens[2] + "-" + fileTokens[0] + "-" + fileTokens[1]));

                        this->testingData2.numExamples[objectLabel]++;
                        break;

                    default:
                        assert(type == 1 || type == 2 || type == 3);
                    }

                }
                currNumObject++;
            }
        }
    }
    this->logger->log->debug("CObjectCategorizationGeometric::inputObjectPointClouds...done!");

    cout << "this->trainingData.objectPointClouds,size = " << this->trainingData.objectPointClouds.size() << "\n";
    cout << "this->testingData.objectPointClouds,size = " << this->testingData.objectPointClouds.size() << "\n";
    cout << "this->testingData2.objectPointClouds,size = " << this->testingData2.objectPointClouds.size() << "\n";
}

void CObjectCategorizationGeometric::inputObjectPointCloudsPseudoCategories(std::string dataPathPseudoCategory, int maxNumObjectPerClass, bool concat)
{
    this->objectPseudoCategorizationGeometric.inputObjectPointClouds(dataPathPseudoCategory, maxNumObjectPerClass, concat);
}

void CObjectCategorizationGeometric::trainPseudoCategories()
{
    this->objectPseudoCategorizationGeometric.trainPseudoCategories();
    this->isPseudoCatLoaded = true;
}

void CObjectCategorizationGeometric::trainPseudoCategories(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds, std::vector<int> labels, std::vector<std::string> objectNames)
{
    this->objectPseudoCategorizationGeometric.inputObjectPointClouds(objectPointClouds, labels, objectNames);
    this->objectPseudoCategorizationGeometric.trainPseudoCategories();
    this->isPseudoCatLoaded = true;
}

void CObjectCategorizationGeometric::loadPseudoCategories()
{
    this->objectPseudoCategorizationGeometric.setHomePath(this->homePath);
    this->objectPseudoCategorizationGeometric.loadModel();
    this->isPseudoCatLoaded = true;
}

void CObjectCategorizationGeometric::train()
{

    this->logger->log->debug("CObjectCategorizationGeometric::train...");
    // This are the real object categories!!!! train set!!!
    assert(this->trainingData.objectPointClouds.size() > 0 && this->trainingData.objectLabels.size() > 0);
    assert(this->trainingData.objectPointClouds.size() == this->trainingData.objectLabels.size());
    assert(this->trainingData.objectPointClouds.size() == this->trainingData.objectNames.size());
    assert(this->doTrainAutoEncoder || this->doTrainSvm || this->doTrainSvmLinear);

    //number of categories > 0
    assert(this->trainingData.numExamples.size() > 0);

    if (this->doTrainScalePyramid)
    {
        this->augmentSetWithScalePyramid(this->trainingData);
    }

    //Here the pseudo train set should be train for objectPseudoCategorizationGeometric
    if (this->isPseudoCatLoaded)
    {
        this->trainingData.extractedFeatureVectors.clear();
        this->trainingData.extractedFeatureVectors.clear();

        //Create PseudoCat responses for each training example
        for (unsigned int i = 0; i < (this->trainingData.objectPointClouds.size()); ++i)
        {
            std::map<int, std::pair<int, double> > getCurrentTreeReponses;
            this->objectPseudoCategorizationGeometric.queryPseudoCategories(this->trainingData.objectPointClouds[i], getCurrentTreeReponses);
            this->trainingData.extractedFeatureVectors[this->trainingData.objectLabels[i]].push_back(this->extractFeatureVectorRT(getCurrentTreeReponses));
        }
        this->print(this->trainingData.extractedFeatureVectors);

        //Create PseudoCat responses for each testing example
        for (unsigned int i = 0; i < (this->testingData.objectPointClouds.size()); ++i)
        {
            std::map<int, std::pair<int, double> > getCurrentTreeReponses;
            this->objectPseudoCategorizationGeometric.queryPseudoCategories(this->testingData.objectPointClouds[i], getCurrentTreeReponses);
            this->testingData.extractedFeatureVectors[this->testingData.objectLabels[i]].push_back(this->extractFeatureVectorRT(getCurrentTreeReponses));
        }
        this->print(this->testingData.extractedFeatureVectors);

        //Create PseudoCat responses for each testing example2
        for (unsigned int i = 0; i < (this->testingData2.objectPointClouds.size()); ++i)
        {
            std::map<int, std::pair<int, double> > getCurrentTreeReponses;
            this->objectPseudoCategorizationGeometric.queryPseudoCategories(this->testingData2.objectPointClouds[i], getCurrentTreeReponses);
            this->testingData2.extractedFeatureVectors[this->testingData2.objectLabels[i]].push_back(this->extractFeatureVectorRT(getCurrentTreeReponses));
        }
        this->print(this->testingData2.extractedFeatureVectors);

        //Now train AutoEncoders with tree reponses for each object
        if (this->doTrainAutoEncoder)
        {
            this->trainAutoEncoders(this->trainingData.extractedFeatureVectors);
        }

        if (this->doTrainSvm)
        {
            this->trainSvms(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
            //this->trainSvmsAdaptive(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
            //this->trainNeuralGas(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
            //this->trainSvmsBoostrapping(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
            //this->trainSvmsAdaptiveParallel(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);

        }

        if (this->doTrainSvmLinear)
        {
            this->trainSvmLinears(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
        }
    }
    else
    {
        assert(this->isPseudoCatLoaded);
    }
}

void CObjectCategorizationGeometric::trainAutoEncoders(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        this->logger->log->info("CObjectCategorizationGeometric::trainAutoEncoders...");
        std::cout << "Train AutoEncoder for  label: " << iterCategories->first << "\n";

        this->autoEncoders[iterCategories->first] = CStackedAutoEncoder(4); //2 :::1 single AE with single hiddenLayer
        this->autoEncoders[iterCategories->first].input(iterCategories->second);
        this->autoEncoders[iterCategories->first].trainOpt(0.001f, 10); //1000
    }
    this->saveAutoEncoderModel();
    this->verifyAutoEncoders();
}

std::pair<int, double> CObjectCategorizationGeometric::queryAE(std::vector<double> featureVector)
{
    assert(this->autoEncoders.size() > 0);

    int minMSELabel = 0;
    double minMSEError = 1;

    std::map<int, CStackedAutoEncoder>::iterator iterAE;

    for (iterAE = this->autoEncoders.begin(); iterAE != this->autoEncoders.end(); ++iterAE)
    {
        //this->logger->log->info("Verify AutoEncoder for label: %s....", CFileSettings::labels[iterAE->first].c_str());
        double getMSEError;
        iterAE->second.encodeDecode(featureVector, getMSEError);
        if (getMSEError < minMSEError)
        {
            minMSEError = getMSEError;
            minMSELabel = iterAE->first;
        }
    }

    std::pair<int, double> result;
    result.first = minMSELabel;
    result.second = minMSEError;

    return result;
}

void CObjectCategorizationGeometric::verifySvmOneClass()
{
    this->logger->log->debug("CObjectCategorizationGeometric::verifySvmOneClass...");
    assert(this->svmOnes.size() > 0);

    SObjectPointCloudData dataSet = this->testingData;

    assert(dataSet.extractedFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterdata;
    std::map<int, CStackedAutoEncoder>::iterator iterAE;
    std::map<int, int>::iterator iterRes;

    std::map<int, double> pBA;
    std::map<int, double> pA;
    std::map<int, double> pB;

    int glTotalCorrect = 0;
    int glTotalCounts = 0;

    int errorTotal = 0;
    int errorCat = 0;
    for (iterdata = dataSet.extractedFeatureVectors.begin(); iterdata != (dataSet.extractedFeatureVectors.end()); ++iterdata)
    {
        this->logger->log->info("LABEL : %s -> SIZE= %d", CFileSettings::labels[iterdata->first].c_str(), iterdata->second.size());
        std::vector<std::vector<double> > featureVectors = iterdata->second;
        int totalCorrect = 0;
        int totalCount = 0;
        int errorCount = 0;

        std::cout << " Label....." << iterdata->first << ":" << std::endl;
        for (unsigned int iterFV = 0; iterFV < featureVectors.size(); ++iterFV)
        {
            double **svmOneProbability = (double**) malloc(sizeof(double*));

            //  std::map<int, CSvmOneClassClassifier>::iterator iterSVMOne;

            //  for(iterSVMOne =  this->svmOnes.bein(); iterSVMOne != this->svmOnes.end(); ++iterSVMOne)
            //{
            char rslSvmOneClass = this->svmOnes[iterdata->first].predict(featureVectors[iterFV], svmOneProbability, false);
            std::cout << " Example: " << iterFV << ". ==" << rslSvmOneClass << std::endl;

            if (rslSvmOneClass != '?') // && iterSVMOne->first==iterdata->first)
            {

                ++totalCorrect;
                ++glTotalCorrect;
            }
            else //if (rslSvmOneClass != '?')
            {
                errorCount++;
                errorTotal++;
            }
            //}
            ++glTotalCounts;
            ++totalCount;
        }

        //      this->svmOnes[iterdata->first].setConfidence((double) errorCount / totalCount);
        this->logger->log->info(" Error of label %d ::: %lf", iterdata->first, (double) errorCount / totalCount); //this->svmOnes[iterdata->first].getConfidence());
    }

    this->logger->log->info("SVM one class error: %lf ", (double) errorTotal / glTotalCounts);

}

void CObjectCategorizationGeometric::verifyAutoEncoders()
{
    this->logger->log->debug("CObjectCategorizationGeometric::verifyAutoEncoders...");
    assert(this->autoEncoders.size() > 0);

    SObjectPointCloudData dataSet = this->testingData;

    assert(dataSet.extractedFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterdata;
    std::map<int, CStackedAutoEncoder>::iterator iterAE;
    std::map<int, int>::iterator iterRes;

    std::map<int, double> pBA;
    std::map<int, double> pA;
    std::map<int, double> pB;

    int glTotalCorrect = 0;
    int glTotalCounts = 0;

    for (iterdata = dataSet.extractedFeatureVectors.begin(); iterdata != (dataSet.extractedFeatureVectors.end()); ++iterdata)
    {
        std::map<int, int> errorCat;
        this->logger->log->info("LABEL : %s -> SIZE= %d", CFileSettings::labels[iterdata->first].c_str(), iterdata->second.size());
        std::vector<std::vector<double> > featureVectors = iterdata->second;
        int totalCorrect = 0;
        int totalCount = 0;

        for (unsigned int iterFV = 0; iterFV < featureVectors.size(); ++iterFV)
        {

            int minMSELabel = 0;
            double minMSEError = 1;

            std::pair<int, double> result = this->queryAE(featureVectors[iterFV]);
            minMSELabel = result.first;
            minMSEError = result.second;

            if (minMSELabel == iterdata->first)
            {
                ++totalCorrect;
                ++glTotalCorrect;
            }
            else
            {
                errorCat[minMSELabel]++;
            }
            ++glTotalCounts;
            ++totalCount;
        }

        this->logger->log->info("CObjectCategorizationGeometric::verifyAutoEncoders...TOTAL CORRECT Label:: %s ::> %lf(%d)", CFileSettings::labels[iterdata->first].c_str(),
                                ((double) totalCorrect / (double) featureVectors.size()), totalCorrect);

        pBA[iterdata->first] = (double) totalCorrect / (double) featureVectors.size();
        pA[iterdata->first] = (double) featureVectors.size();
        pB[iterdata->first] += totalCorrect;

        for (iterRes = errorCat.begin(); iterRes != errorCat.end(); ++iterRes)
        {
            this->logger->log->info("CObjectCategorizationGeometric::verifyAutoEncoders..Label:: %s misclassified as %s :: %lf(%d)", CFileSettings::labels[iterdata->first].c_str(),
                                    CFileSettings::labels[iterRes->first].c_str(), (double) iterRes->second / (double) featureVectors.size(), iterRes->second);
            pB[iterRes->first] += iterRes->second;
            this->logger->log->info("PB  %d for %d ", iterRes->second, iterRes->first);
        }
    }

    std::map<int, double>::iterator iterClass;
    for (iterClass = pA.begin(); iterClass != pA.end(); ++iterClass)
    {
        int label = iterClass->first;

        std::cout << " FALSE CLASS: " << (((double) pB[label] / (double) glTotalCounts)) << std::endl; //"----" <<pB[label]<<" ---- " <<glTotalCounts<<  std::endl;

        double bayes = ((double) pBA[label] * ((double) pA[label] / (double) glTotalCounts)) / (double)(pB[label] / (double) glTotalCounts);
        this->logger->log->info("CObjectCategorizationGeometric::verifyAutoEncoders..Bayes for Class %s :: %lf", CFileSettings::labels[label].c_str(), bayes);
    }

    this->logger->log->info("CObjectCategorizationGeometric::verifyAutoEncoders..Total CORR :: %lf", (double) glTotalCorrect / glTotalCounts);
}
void CObjectCategorizationGeometric::trainSvms(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        std::vector<std::vector<double> > trainingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTrainingFeatureVectors, 0.5); //0.75
        std::vector<std::vector<double> > testingFeatureVector;

        if (extractedTestingFeatureVectors.size() > 0)
        {
            testingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTestingFeatureVectors);
        }
        else
        {
            testingFeatureVector = trainingFeatureVector;
        }

        this->svms[iterCategories->first] = CSvmClassifier();//CSvm(ocSettings);
        this->svms[iterCategories->first].setGridSearchIterations(10); //5 //10
        double svmTeResult = this->svms[iterCategories->first].parameterGridSearch(trainingFeatureVector, testingFeatureVector, -1, true); //, false);//false
        this->svms[iterCategories->first].saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterCategories->first]) + "_" + std::string(
                SVM_MODEL_POSTFIX));

        results[iterCategories->first] = svmTeResult;
        std::cout << "\nLABEL :::" << iterCategories->first << "  svmTeResult" << svmTeResult << "\n";

    }

    this->verifySvm(extractedTrainingFeatureVectors, extractedTestingFeatureVectors);

    for (iterRes = results.begin(); iterRes != results.end(); ++iterRes)
    {
        std::cout << "\nLABEL :::" << iterRes->first << "  svmTeResult" << iterRes->second << "\n";
    }
}

void CObjectCategorizationGeometric::trainSvmLinears(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;

    std::map<int, std::vector<std::vector<double> > > trSet = extractedTrainingFeatureVectors;
    ////////7
    //std::cout<<"CObjectCategorizationGeometric::trainSvmLinears....Selected Train examples:"<<boost::lexical_cast<int>(tag)<<std::endl;
    //trSet = this->removeRandExamples(extractedTrainingFeatureVectors,boost::lexical_cast<int>(tag));
    ////////////////

    std::string modelFileName = std::string(std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_LINEAR_MODEL_FILENAME) + "_" + std::string(SVM_MODEL_POSTFIX));
    this->svmLinear.trainModel(trSet, (char*) modelFileName.c_str());
    this->svmLinear.crossValidation(trSet, 10);

    std::string fileModelName = std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_LINEAR_MODEL_FILENAME);
    this->svmLinear.saveModel2(fileModelName);

    //LEARN TEST AS WELL
    //for (iterCategories = extractedTestingFeatureVectors.begin(); iterCategories != extractedTestingFeatureVectors.end(); ++iterCategories)
    //{
    //this->svmLinear.updateModel(fileModelName, iterCategories->second, iterCategories->first, fileModelName);
    //}
    //this->svmLinear.crossValidation(extractedTrainingFeatureVectors, 10);
    //

    this->verifySvmLinear(trSet, extractedTestingFeatureVectors);
}

void CObjectCategorizationGeometric::updateModelSvmLinear(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    std::string fileModelName = std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_LINEAR_MODEL_FILENAME);
    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        //this->svmLinear.setCrossValidationNFoldCMD(10);
        this->svmLinear.updateModel(fileModelName, iterCategories->second, iterCategories->first, fileModelName);
    }

}

void CObjectCategorizationGeometric::trainSvmsAdaptiveParallel(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double> bestRatios;
    std::map<int, double> resultsCV;
    std::map<int, double>::iterator iterRes;

    std::vector<int> labels;
    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        labels.push_back(iterCategories->first);
    }

    omp_set_num_threads(labels.size());
    #pragma omp parallel for
    for (unsigned int iLabel = 0; iLabel < labels.size(); ++iLabel)
    {
        std::cout << " CObjectCategorizationGeometric::trainSvmsAdaptiveParallel started for label: " << labels[iLabel] << std::endl;
        float step = 0.1;
        float bestTest = 1.0f;
        float bestRatio = 0.5f;
        double svmTeResult;
        std::vector<std::vector<double> > bestTrainingFeatureVector;
        std::vector<std::vector<double> > bestTestingFeatureVector;

        for (float ratio = 0.2; ratio <= 1.0; ratio = ratio + step)
        {
            //Training set
            std::vector<std::vector<double> > trainingFeatureVector = this->createOneVsAllset(labels[iLabel], extractedTrainingFeatureVectors, ratio); //0.75
            //std::pair < std::vector<std::vector<double> >, std::vector<std::vector<double> > > splitted = this->splitPosNegSet(trainingFeatureVector);

            //testing set
            std::vector<std::vector<double> > testingFeatureVector;
            if (extractedTestingFeatureVectors.size() > 0)
            {
                testingFeatureVector = this->createOneVsAllset(labels[iLabel], extractedTestingFeatureVectors);
            }
            else
            {
                testingFeatureVector = trainingFeatureVector;
            }
            //------------------------------------

            this->svms[labels[iLabel]] = CSvmClassifier();//CSvm(ocSettings);
            this->svms[labels[iLabel]].setGridSearchIterations(3); //3 //5 //10
            svmTeResult = this->svms[labels[iLabel]].parameterGridSearch(trainingFeatureVector, testingFeatureVector, -1, true); //, false);//false

            if (svmTeResult < bestTest)
            {
                bestTest = svmTeResult;
                bestRatio = ratio;
                bestRatios[labels[iLabel]] = ratio;
                bestTrainingFeatureVector = trainingFeatureVector;
                bestTestingFeatureVector = testingFeatureVector;
            }
        }

        this->svms[labels[iLabel]] = CSvmClassifier();//CSvm(ocSettings);
        this->svms[labels[iLabel]].setGridSearchIterations(10); //10 //5 //10
        svmTeResult = this->svms[labels[iLabel]].parameterGridSearch(bestTrainingFeatureVector, bestTestingFeatureVector, -1, true); //, false);//false
        this->svms[labels[iLabel]].saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[labels[iLabel]]) + "_" + this->tag + "_" + std::string(
                SVM_MODEL_POSTFIX));

        results[labels[iLabel]] = svmTeResult;
        resultsCV[labels[iLabel]] = this->svms[labels[iLabel]].crossValidation(10);
        std::cout << "\nLABEL :::" << labels[iLabel] << "  svmTeResult" << svmTeResult << resultsCV[labels[iLabel]] << "best Ration " << bestRatio << std::endl;

    }

    this->verifySvm(extractedTrainingFeatureVectors, extractedTestingFeatureVectors);

    std::fstream fout;
    fout.open(std::string(this->homePath + std::string(SVM_PATH) + std::string("svmResults" + this->tag + ".txt")).c_str(), std::ios::out | std::ios::binary);
    fout << this->tag << std::endl;

    for (iterRes = results.begin(); iterRes != results.end(); ++iterRes)
    {
        std::cout << "\nLABEL :::" << iterRes->first << "  svmTeResult" << iterRes->second << "  svmCvResult " << resultsCV[iterRes->first] << "best ration " << bestRatios[iterRes->first]
                  << std::endl;
        fout << iterRes->first << "\t" << iterRes->second << "\t" << resultsCV[iterRes->first] << "\t" << bestRatios[iterRes->first] << std::endl;
    }

    fout << "TL\t" << this->evaluateQuery22(this->trainingData) << "\t" << this->evaluateQuery22(this->testingData) << "\t" << this->evaluateQuery22(this->testingData2) << std::endl;

    fout.close();
}

void CObjectCategorizationGeometric::trainSvmsAdaptive(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {

        float step = 0.1;
        float bestTest = 1.0f;
        float bestRatio = 0.5f;
        double svmTeResult;
        std::vector<std::vector<double> > bestTrainingFeatureVector;
        std::vector<std::vector<double> > bestTestingFeatureVector;

        for (float ratio = 0.2; ratio <= 1.0; ratio = ratio + step)
        {
            //Training set
            std::vector<std::vector<double> > trainingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTrainingFeatureVectors, ratio); //0.75
            //std::pair < std::vector<std::vector<double> >, std::vector<std::vector<double> > > splitted = this->splitPosNegSet(trainingFeatureVector);

            //testing set
            std::vector<std::vector<double> > testingFeatureVector;
            if (extractedTestingFeatureVectors.size() > 0)
            {
                testingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTestingFeatureVectors);
            }
            else
            {
                testingFeatureVector = trainingFeatureVector;
            }
            //------------------------------------

            this->svms[iterCategories->first] = CSvmClassifier();//CSvm(ocSettings);
            this->svms[iterCategories->first].setGridSearchIterations(3); //3 //5 //10
            svmTeResult = this->svms[iterCategories->first].parameterGridSearch(trainingFeatureVector, testingFeatureVector, -1, true); //, false);//false

            if (svmTeResult < bestTest)
            {
                bestTest = svmTeResult;
                bestRatio = ratio;
                bestTrainingFeatureVector = trainingFeatureVector;
                bestTestingFeatureVector = testingFeatureVector;
            }
        }

        this->svms[iterCategories->first] = CSvmClassifier();//CSvm(ocSettings);
        this->svms[iterCategories->first].setGridSearchIterations(10); //10 //5 //10
        svmTeResult = this->svms[iterCategories->first].parameterGridSearch(bestTrainingFeatureVector, bestTestingFeatureVector, -1, true); //, false);//false
        this->svms[iterCategories->first].saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterCategories->first]) + "_" + std::string(
                SVM_MODEL_POSTFIX));

        results[iterCategories->first] = svmTeResult;
        std::cout << "\nLABEL :::" << iterCategories->first << "  svmTeResult" << svmTeResult << "\n";

    }

    this->verifySvm(extractedTrainingFeatureVectors, extractedTestingFeatureVectors);

    for (iterRes = results.begin(); iterRes != results.end(); ++iterRes)
    {
        std::cout << "\nLABEL :::" << iterRes->first << "  svmTeResult" << iterRes->second << "\n";
    }
}

void CObjectCategorizationGeometric::trainNeuralGas(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    std::cout << "CObjectCategorizationGeometric::trainNeuralGas..." << std::endl;
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;
    std::vector<int> combinedTrainingSetLabels;

    std::vector<vector<double> > combinedTrainingSet;
    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVector = 0; iterVector < iterCategories->second.size(); ++iterVector)
        {
            combinedTrainingSet.push_back(iterCategories->second[iterVector]);
            combinedTrainingSetLabels.push_back(iterCategories->first);

            std::cout << "added--->" << iterCategories->first << " ";
        }
    }
    assert(combinedTrainingSetLabels.size() == combinedTrainingSet.size());

    std::cout << "CObjectCategorizationGeometric::trainNeuralGas...start train" << std::endl;
    CNeuralGasClassifier *gngClassifierPlease = new CNeuralGasClassifier(combinedTrainingSet[0].size(), 1000, 0.3f, 0.15f, 1.0f, 5, 5, 0.7f, 0.3f, 1, 760, 1, 1, 0.00001f);
    //do{
    gngClassifierPlease->train(combinedTrainingSet, combinedTrainingSetLabels, -1, true, 300, -1);
    //}while(1);
    std::cout << "CObjectCategorizationGeometric::trainNeuralGas...start train done" << std::endl;
}

void CObjectCategorizationGeometric::loadTraintrain()
{
    assert(this->trainingData.extractedFeatureVectors.size() > 0);
    assert(this->testingData.extractedFeatureVectors.size() > 0);

    std::cout << "this->trainingData.extractedFeatureVectors.size() == " << this->trainingData.extractedFeatureVectors.size() << " this->trainingData.extractedFeatureVectors[1].size() == "
              << this->trainingData.extractedFeatureVectors[1].size() << std::endl;
    std::cout << "this->testingData.extractedFeatureVectors.size() == " << this->testingData.extractedFeatureVectors.size() << " this->testingData.extractedFeatureVectors[1].size() == "
              << this->testingData.extractedFeatureVectors[1].size() << std::endl;
    //this->loadSVMCascade();
    //this->trainSvmsBoostrapping(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
    //this->trainSvmsAdaptive(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
    //this->trainSvmsAdaptiveParallel(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
    //this->trainSvmOneClass(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
    //this->trainAutoEncoders(this->trainingData.extractedFeatureVectors);
    this->trainSvmLinears(this->trainingData.extractedFeatureVectors, this->testingData.extractedFeatureVectors);
}

void CObjectCategorizationGeometric::trainSvmOneClass(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{

    this->logger->log->info("CObjectCategorizationGeometric::trainSvmOneClass...");
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories, iterCategories2;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        this->logger->log->info("CObjectCategorizationGeometric::trainSvmOneClass...train %s", CFileSettings::labels[iterCategories->first].c_str());
        std::vector<std::vector<double> > trainingFeatureVector = this->createOneClassSet(iterCategories->first, extractedTrainingFeatureVectors); //0.75
        std::vector<std::vector<double> > testingFeatureVector;
        std::vector<std::vector<double> > testingFeatureVectorOneClass;

        if (extractedTestingFeatureVectors.size() > 0)
        {
            testingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTestingFeatureVectors);
        }
        else
        {
            testingFeatureVector = trainingFeatureVector;
        }

        /////////////////
        testingFeatureVectorOneClass = this->createOneClassSet(iterCategories->first, extractedTestingFeatureVectors); //0.75

        this->svmOnes[iterCategories->first] = CSvmOneClassClassifier();//CSvm(ocSettings);
        double svmTeResult; // = this->svmOnes[iterCategories->first].trainSvm(trainingFeatureVector, testingFeatureVector); //, false);//false
        svmTeResult = this->svmOnes[iterCategories->first].parameterGridSearch2(trainingFeatureVector, testingFeatureVector, -1, true); //, false);//false;
        this->svmOnes[iterCategories->first].saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_ONECLASS_MODEL_FILENAME) + "_" + std::string(CFileSettings::labels[iterCategories->first]) + "_" + std::string(
                SVM_MODEL_POSTFIX));

        results[iterCategories->first] = svmTeResult;
        std::cout << "\nLABEL :::" << iterCategories->first << "  svmTeResult" << svmTeResult << "\n";

    }

    //this->verifySvm(extractedTrainingFeatureVectors, extractedTestingFeatureVectors);

    for (iterRes = results.begin(); iterRes != results.end(); ++iterRes)
    {
        std::cout << "\nLABEL :::" << iterRes->first << "  svmTeResult" << iterRes->second << "\n";
    }

    this->verifySvmOneClass();

}

void CObjectCategorizationGeometric::trainSvmsBoostrapping(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    //  COCSettings ocSettings;
    //  ocSettings.readCurrentEvalNumber(false);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;
    std::map<int, double> results;
    std::map<int, double>::iterator iterRes;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        //Training set
        std::map<int, std::vector<std::vector<double> > > trainingFeatureVector = this->createOneVsAllsetMap(iterCategories->first, extractedTrainingFeatureVectors);

        //testing set
        std::map<int, std::vector<std::vector<double> > > testingFeatureVector;
        if (extractedTestingFeatureVectors.size() > 0)
        {
            testingFeatureVector = this->createOneVsAllsetMap(iterCategories->first, extractedTestingFeatureVectors);
        }
        else
        {
            testingFeatureVector = trainingFeatureVector;
        }
        std::cout << "CObjectCategorizationGeometric::trainSvmsBoostrapping...Train label " << iterCategories->first << std::endl;
        logger->log->info("CObjectCategorizationGeometric::trainSvmsBoostrapping...Train label %d\n", iterCategories->first);
        std::map<int, std::vector<int> > missClassified;
        this->svmCascade[iterCategories->first] = CSvmCascade();//CSvm(ocSettings);
        this->svmCascade[iterCategories->first].setHomePath(this->homePath);
        this->svmCascade[iterCategories->first].setNumIterations(10);
        this->svmCascade[iterCategories->first].setId(CFileSettings::labels[iterCategories->first]);
        this->svmCascade[iterCategories->first].setTrainSet(trainingFeatureVector);
        this->svmCascade[iterCategories->first].setTestSet(testingFeatureVector);
        //this->svmCascade[iterCategories->first].setNumStages(30);
        this->svmCascade[iterCategories->first].train1();
        //exit(1);
    }
    this->saveSVMCascade();
}

std::vector<std::vector<double> > CObjectCategorizationGeometric::removeRandExamples(std::vector<std::vector<double> > &set, unsigned int numtoExtractExamples)
{
    assert(set.size() > 0);
    assert(set.size() > numtoExtractExamples);

    std::vector<std::vector<double> > extractedSet;
    boost::mt19937 rng;
    boost::uniform_int<> exampleSetRange(0, set.size() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSetRange);

    for (unsigned int iterNegExamples = 0; iterNegExamples < (unsigned int) numtoExtractExamples; ++iterNegExamples)
    {
        int randIdx = getRandom();
        assert(randIdx >= 0 && randIdx < (int)set.size());
        extractedSet.push_back(set[randIdx]);
        set.erase(set.begin() + randIdx);
    }

    return extractedSet;
}

void CObjectCategorizationGeometric::verifySvm(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        std::cout << std::endl;
        std::vector<std::vector<double> > trainingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTrainingFeatureVectors);
        std::vector<std::vector<double> > testingFeatureVector;

        if (extractedTestingFeatureVectors.size() > 0)
        {
            testingFeatureVector = this->createOneVsAllset(iterCategories->first, extractedTestingFeatureVectors);
        }
        else
        {
            testingFeatureVector = trainingFeatureVector;
        }

        int error = 0;
        int correct = 0;
        int errorCat = 0;
        int correctCat = 0;
        int countCat = 0;
        std::cout << "Verify Training Label " << CFileSettings::labels[iterCategories->first] << ":\n";
        for (unsigned int i = 0; i < trainingFeatureVector.size(); ++i)
        {
            double **svmProbability = (double**) malloc(sizeof(double*));
            *svmProbability = NULL;
            char rslSvm = this->svms[iterCategories->first].predict(trainingFeatureVector[i], svmProbability, true);
            rslSvm == trainingFeatureVector[i][0] ? correct++ : error++;

            if (trainingFeatureVector[i][0] == 'A')
            {
                rslSvm == trainingFeatureVector[i][0] ? correctCat++ : errorCat++;
                ++countCat;
            }

            free(*svmProbability);
            free(svmProbability);
        }
        std::cout << "**Verification TrainSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correct / (double) trainingFeatureVector.size()) << " Error::"
                  << ((double) error / (double) trainingFeatureVector.size()) << std::endl;

        std::cout << "***Verification TrainCatSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correctCat / (double) countCat) << " Error::" << ((double) errorCat
                  / (double) countCat) << " " << "(" << countCat << "|" << error << ")" << std::endl;

        error = 0;
        correct = 0;
        errorCat = 0;
        correctCat = 0;
        countCat = 0;
        std::cout << "Verify Testing Label " << CFileSettings::labels[iterCategories->first] << ":\n";
        for (unsigned int i = 0; i < testingFeatureVector.size(); ++i)
        {
            double **svmProbability = (double**) malloc(sizeof(double*));
            *svmProbability = NULL;
            char rslSvm = this->svms[iterCategories->first].predict(testingFeatureVector[i], svmProbability, true);
            rslSvm == testingFeatureVector[i][0] ? correct++ : error++;

            if (testingFeatureVector[i][0] == 'A')
            {
                rslSvm == testingFeatureVector[i][0] ? correctCat++ : errorCat++;
                ++countCat;
            }

            free(*svmProbability);
            free(svmProbability);
        }
        std::cout << "**Verification TestSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correct / (double) testingFeatureVector.size()) << " Error::"
                  << ((double) error / (double) testingFeatureVector.size()) << std::endl;
        std::cout << "***Verification TestCatSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correctCat / (double) countCat) << " Error::" << ((double) errorCat
                  / (double) countCat) << " " << "(" << countCat << "|" << error << ")" << std::endl;
    }
}

void CObjectCategorizationGeometric::verifySvmLinear(std::map<int, std::vector<std::vector<double> > > &extractedTrainingFeatureVectors,
        std::map<int, std::vector<std::vector<double> > > &extractedTestingFeatureVectors)
{
    assert(extractedTrainingFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    std::vector<std::vector<double> > trainingFeatureVectorAll;
    std::vector<double> trainingFeatureVectorAllLabel;

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            trainingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            trainingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    for (iterCategories = extractedTestingFeatureVectors.begin(); iterCategories != extractedTestingFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    std::cout << "Train Total set :" << trainingFeatureVectorAll.size() << std::endl;
    std::cout << "Test Total set :" << testingFeatureVectorAll.size() << std::endl;

    for (iterCategories = extractedTrainingFeatureVectors.begin(); iterCategories != extractedTrainingFeatureVectors.end(); ++iterCategories)
    {
        std::vector<std::vector<double> > trainingFeatureVector;
        std::vector<std::vector<double> > testingFeatureVector;

        int error = 0;
        int correct = 0;
        int errorCat = 0;
        int correctCat = 0;
        int countCat = 0;
        //ALL
        std::cout << "Verify Training Label " << CFileSettings::labels[iterCategories->first] << ":\n";

        for (unsigned int i = 0; i < trainingFeatureVectorAll.size(); ++i)
        {
            double *dec = (double *) malloc((NUM_CLASSES) * sizeof(double));
            int rslSvm = this->svmLinear.predict(trainingFeatureVectorAll[i], dec);
            rslSvm == trainingFeatureVectorAllLabel[i] ? correct++ : error++;

            free(dec);
            //  if(rslSvm != trainingFeatureVectorAllLabel[i])
            //      std::cout<<"error "<<rslSvm<<"!="<<trainingFeatureVectorAllLabel[i]<<std::endl;

        }
        std::cout << "**Verification TrainSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correct / (double) trainingFeatureVectorAll.size()) << " Error::"
                  << ((double) error / (double) trainingFeatureVectorAll.size()) << std::endl;
        ///////////label wise
        trainingFeatureVector = extractedTrainingFeatureVectors[iterCategories->first];
        for (unsigned int i = 0; i < trainingFeatureVector.size(); ++i)
        {
            double *dec = (double *) malloc((NUM_CLASSES) * sizeof(double));
            int rslSvm = this->svmLinear.predict(trainingFeatureVector[i], dec);
            rslSvm == iterCategories->first ? correctCat++ : errorCat++;
            ++countCat;
            free(dec);
        }
        std::cout << "***Verification TrainCatSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correctCat / (double) countCat) << " Error::" << ((double) errorCat
                  / (double) countCat) << " " << "(" << errorCat << "|" << countCat << ")" << std::endl;

        error = 0;
        correct = 0;
        errorCat = 0;
        correctCat = 0;
        countCat = 0;
        //ALL
        std::cout << "Verify Testing Label " << CFileSettings::labels[iterCategories->first] << ":\n";

        for (unsigned int i = 0; i < testingFeatureVectorAll.size(); ++i)
        {
            double *dec = (double *) malloc((NUM_CLASSES) * sizeof(double));
            int rslSvm = this->svmLinear.predict(testingFeatureVectorAll[i], dec);
            rslSvm == testingFeatureVectorAllLabel[i] ? correct++ : error++;
        }
        std::cout << "**Verification TestSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correct / (double) testingFeatureVectorAll.size()) << " Error::"
                  << ((double) error / (double) testingFeatureVectorAll.size()) << std::endl;
        ///////////label wise
        testingFeatureVector = extractedTestingFeatureVectors[iterCategories->first];
        for (unsigned int i = 0; i < testingFeatureVector.size(); ++i)
        {
            double *dec = (double *) malloc((NUM_CLASSES) * sizeof(double));
            int rslSvm = this->svmLinear.predict(testingFeatureVector[i], dec);
            rslSvm == iterCategories->first ? correctCat++ : errorCat++;
            ++countCat;
            free(dec);
        }
        std::cout << "***Verification TestCatSVM " << CFileSettings::labels[iterCategories->first] << " Correct::" << ((double) correctCat / (double) countCat) << " Error::" << ((double) errorCat
                  / (double) countCat) << " " << "(" << errorCat << "|" << countCat << ")" << std::endl;

        std::cout << std::endl;
    }
}

std::pair<int, double> CObjectCategorizationGeometric::queryPseudo(pcl::PointCloud<pcl::PointXYZ> queryPointCloud)
{
    assert(this->isPseudoCatLoaded);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    std::map<int, std::pair<int, double> > getCurrentTreeReponses;
    res = this->objectPseudoCategorizationGeometric.queryPseudoCategories(queryPointCloud, getCurrentTreeReponses);

    return res;
}

std::pair<int, std::vector<double> > CObjectCategorizationGeometric::queryPseudo2(pcl::PointCloud<pcl::PointXYZ> queryPointCloud)
{
    assert(this->isPseudoCatLoaded);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;
    std::pair<int, std::vector<double> > resVector;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    std::map<int, std::pair<int, double> > getCurrentTreeReponses;
    res = this->objectPseudoCategorizationGeometric.queryPseudoCategories(queryPointCloud, getCurrentTreeReponses);

    resVector.first = res.first;
    resVector.second = this->extractFeatureVectorRT(getCurrentTreeReponses);

    return resVector;
}

std::vector<std::vector<double> > CObjectCategorizationGeometric::createOneVsAllset(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors, double ratio)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories2;
    std::vector<std::vector<double> > featureVector;
    std::vector<std::vector<double> > negativeExampleFeatureVector;

    for (iterCategories2 = extractedFeatureVectors.begin(); iterCategories2 != extractedFeatureVectors.end(); ++iterCategories2)
    {
        std::vector<std::vector<double> > t = iterCategories2->second;
        for (unsigned int iterVec = 0; iterVec < t.size(); ++iterVec)
        {
            if (label == iterCategories2->first)
            {
                t[iterVec].insert(t[iterVec].begin(), 'A');
                featureVector.push_back(t[iterVec]);
            }
            else
            {
                t[iterVec].insert(t[iterVec].begin(), 'B');
                if (ratio < 0)
                {
                    featureVector.push_back(t[iterVec]);
                }
                else
                {
                    negativeExampleFeatureVector.push_back(t[iterVec]);
                }
            }
        }
    }

    //Now check how many negative examples should be added to positve ones
    if (ratio > 0)
    {
        //minimum ratio since true example have a proportion to the entire set so lesser the this prop is impossible since ratio is the ratio of postive examples
        assert(ratio >= ((float)featureVector.size() / ((float)featureVector.size() + (float)negativeExampleFeatureVector.size())));
        assert(featureVector.size() > 0);
        assert(negativeExampleFeatureVector.size() > 0);

        int numNegativeExamples = (int)((double) featureVector.size() / ratio) - (double) featureVector.size();

        assert(numNegativeExamples > 0);

        boost::mt19937 rng;
        boost::uniform_int<> exampleSet(0, negativeExampleFeatureVector.size() - 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);

        std::cout << "Pos: " << featureVector.size() << " Neg" << numNegativeExamples << "\n";
        for (unsigned int iterNegExamples = 0; iterNegExamples < (unsigned int) numNegativeExamples; ++iterNegExamples)
        {
            int randIdx = getRandom();
            assert(randIdx >= 0 && randIdx < (int)negativeExampleFeatureVector.size());
            featureVector.push_back(negativeExampleFeatureVector[randIdx]);
        }
    }
    return featureVector;
}

std::vector<std::vector<double> > CObjectCategorizationGeometric::createOneClassSet(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories2;
    std::vector<std::vector<double> > featureVector;
    std::vector<std::vector<double> > negativeExampleFeatureVector;

    for (iterCategories2 = extractedFeatureVectors.begin(); iterCategories2 != extractedFeatureVectors.end(); ++iterCategories2)
    {
        std::vector<std::vector<double> > t = iterCategories2->second;
        for (unsigned int iterVec = 0; iterVec < t.size(); ++iterVec)
        {
            if (label == iterCategories2->first)
            {
                t[iterVec].insert(t[iterVec].begin(), 'A');
                featureVector.push_back(t[iterVec]);
            }
        }
    }

    return featureVector;
}

std::vector<std::vector<double> > CObjectCategorizationGeometric::createBootstrap(std::vector<std::vector<double> > &featureVectors, unsigned int numberVec)
{
    logger->log->info("CObjectCategorizationGeometric::createBootstrap...");
    boost::mt19937 rng;
    assert(featureVectors.size() > 0);
    assert(featureVectors.size() > numberVec);

    std::vector<std::vector<double> > bootstrapped;

    boost::uniform_int<> exampleSet(0, featureVectors.size() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);

    logger->log->debug("CBSvmClassifier::createBootstrap...select %d", numberVec);
    std::vector<int> randNumDrawn;
    for (unsigned int iterVec = 0; iterVec < (unsigned int) numberVec; ++iterVec)
    {
        int randIdx = getRandom();
        assert(randIdx >= 0 && randIdx < (int)featureVectors.size());
        logger->log->debug("CBSvmClassifier::createBootstrap...selected %d", randIdx);

        if (std::find(randNumDrawn.begin(), randNumDrawn.end(), randIdx) != randNumDrawn.end())
        {
            logger->log->debug("CBSvmClassifier::createBootstrap...already selected => %d ... no reject it!", randIdx);
            iterVec--;
            continue;
        }
        randNumDrawn.push_back(randIdx);
        bootstrapped.push_back(featureVectors[randIdx]);
    }

    assert(bootstrapped.size() == numberVec);
    return bootstrapped;
}

std::map<int, std::vector<std::vector<double> > > CObjectCategorizationGeometric::createOneVsAllsetMap(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors)
{
#define POSITIVE_LABEL 1
#define NEGATIVE_LABEL 0

    assert(extractedFeatureVectors.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories2;
    std::map<int, std::vector<std::vector<double> > > sortedFeatureVector;

    for (iterCategories2 = extractedFeatureVectors.begin(); iterCategories2 != extractedFeatureVectors.end(); ++iterCategories2)
    {

        if (label == iterCategories2->first)
        {
            std::vector<std::vector<double> > t = iterCategories2->second;
            for (unsigned int iterVec = 0; iterVec < t.size(); ++iterVec)
            {
                sortedFeatureVector[POSITIVE_LABEL].push_back(t[iterVec]);
            }

        }
        else
        {
            std::vector<std::vector<double> > t = iterCategories2->second;
            for (unsigned int iterVec = 0; iterVec < t.size(); ++iterVec)
            {
                sortedFeatureVector[NEGATIVE_LABEL].push_back(t[iterVec]);
            }
        }

    }
    return sortedFeatureVector;
}

std::pair<std::vector<std::vector<double> >, std::vector<std::vector<double> > > CObjectCategorizationGeometric::splitPosNegSet(std::vector<std::vector<double> > set)
{
    std::pair<std::vector<std::vector<double> >, std::vector<std::vector<double> > > splittedSet;

    for (unsigned int iterVec = 0; iterVec < set.size(); ++iterVec)
    {
        if (set[iterVec][0] == 'A')
        {
            splittedSet.first.push_back(set[iterVec]);
        }

        if (set[iterVec][0] == 'B')
        {
            splittedSet.second.push_back(set[iterVec]);
        }
    }

    return splittedSet;
}

void CObjectCategorizationGeometric::print(std::map<int, std::vector<std::vector<double> > > &featureVector)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterFv;

    std::cout << "Num Label:" << featureVector.size() << "\n";
    for (iterFv = featureVector.begin(); iterFv != featureVector.end(); ++iterFv)
    {
        std::cout << "Label " << iterFv->first << ":" << std::endl;
        for (unsigned int j = 0; j < iterFv->second.size(); ++j)
        {
            CFileSettings::coutStdVector(iterFv->second[j], true);
        }
    }
}

std::vector<double> CObjectCategorizationGeometric::extractFeatureVectorRT(std::map<int, std::pair<int, double> > &treeReponses)
{
    assert(treeReponses.size() > 0);
    assert(this->objectPseudoCategorizationGeometric.getNumCategories() > 0);

    std::map<int, std::pair<int, double> >::iterator iterTreeResponses;

    // num of trees * num of labesl
    //int numPseudoCat =  this->objectPseudoCategorizationGeometric.getNumCategories();
    int featureVectorLength = treeReponses.size() * NUM_CLASSES;
    //int offSetLabel = 1; // since category labels start with 1;

    //initFeatureVector
    std::vector<double> extractedFeatureVector(featureVectorLength, 0.0);

    //Keep in mind that the result of label 2 is not supposed to be copied to index 1!! it depends on the number of categories!!
    //To get a consistent featureVector we need to know the max value of othe max label = > NUM_CLASSES
    int tree = 0;
    for (iterTreeResponses = treeReponses.begin(); iterTreeResponses != treeReponses.end(); ++iterTreeResponses)
    {
        //std::cout<<iterTreeResponses->second.first<<"->"<< iterTreeResponses->second.second<<"\n";
        extractedFeatureVector[(tree * NUM_CLASSES) + iterTreeResponses->second.first] = iterTreeResponses->second.second;
        ++tree;
    }

    //  CFileSettings::coutStdVector(extractedFeatureVector,true);

    return extractedFeatureVector;
}

std::pair<int, double> CObjectCategorizationGeometric::query(pcl::PointCloud<pcl::PointXYZ> queryPointCloud)
{
    clock_t begin_time = clock();
    assert(this->svms.size() > 0 || this->autoEncoders.size() > 0);
    assert(this->isPseudoCatLoaded);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    if (queryPointCloud.points.size() < 80)
    {
        std::cout << "CObjectCategorizationGeometric::query... min Point cloud size not reached!" << std::endl;
        return mostConfidentRes;
    }

    std::map<int, std::pair<int, double> > getCurrentTreeReponses;

    std::pair<int, double> rslRF = this->objectPseudoCategorizationGeometric.queryPseudoCategories(queryPointCloud, getCurrentTreeReponses);
    std::vector<double> featureVector = this->extractFeatureVectorRT(getCurrentTreeReponses);

    clock_t begin_time2 = clock();
    res = this->query(featureVector);
    std::cout << "CObjectCategorizationGeometric::query...Highlevel done in " << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
    std::cout << "CObjectCategorizationGeometric::query complete...done in " << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
    return res;
}

//one vs all query
std::pair<int, double> CObjectCategorizationGeometric::query(std::vector<double> featureVector)
{

    assert(this->svms.size() > 0 || this->autoEncoders.size() > 0);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    for (unsigned int iterCategories = 1; iterCategories < NUM_CLASSES; ++iterCategories)
    {
        if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(iterCategories) != this->svms.end())
        {
            double **svmProbability = (double**) malloc(sizeof(double*));
            double **svmOneProbability = (double**) malloc(sizeof(double*));
            *svmProbability = NULL;
            *svmOneProbability = NULL;
            double confidence = 0, confidenceCascade = 0, confidenceOneClass = 0;
            char rslSvm;

            //LIBSVM
            rslSvm = this->svms[iterCategories].predict(featureVector, svmProbability, false);
            confidence = svmProbability[0][rslSvm - 'A'];
            std::cout << "SVM " << CFileSettings::labels[iterCategories] << " Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << confidence << ")" << std::endl;

            ///LINEAR SVM
            //  int svmLinearLabel;
            //  double *dec = (double *) malloc((this->svmLinear.getMaxLabel()) * sizeof(double));
            //  svmLinearLabel = this->svmLinear.predict(featureVector,dec);
            //  std::cout << "SVM Linear " << " Response::\t" <<  CFileSettings::labels[svmLinearLabel] <<"("<<dec[svmLinearLabel-1] <<")"<<std::endl;
            //  free(dec);

            //SVM ONE CLASS
            //char rslSvmOneClass = this->svmOnes[iterCategories].predict(featureVector, svmOneProbability, false);
            //confidenceOneClass = svmOneProbability[0][1];
            //std::cout << "SVM One Class" << rslSvmOneClass << std::endl;

            if (rslSvm == 'A')
            {
                if (confidence >= mostConfidentRes.second) // && svmProbability[0][rslSvm - 'A'] > this->minConfidence)
                {
                    mostConfidentRes.first = iterCategories;
                    mostConfidentRes.second = confidence;

                    /*
                    //check one class svm
                    if (rslSvmOneClass == '?')
                    {
                        std::cout << "SVMOne reduce..to " << mostConfidentRes.second * (double) (1.0 - svmOnesConfidences[iterCategories]) << std::endl;
                        mostConfidentRes.second = mostConfidentRes.second * (double) (1.0 - svmOnesConfidences[iterCategories]);
                    }
                    else
                        std::cout << "SVMOne OK" << std::endl;*/

                    //check auto encoder
                    if (this->doQueryAutoEncoder)
                    {

                        std::pair<int, double> aeResult = this->queryAE(featureVector);

                        if (aeResult.first != mostConfidentRes.first)
                            //if(aeResult.first == mostConfidentRes.first)
                            //{
                            //  std::cout << "AutoEncoder imporve..by " <<mostConfidentRes.second * aEConfidences[aeResult.first]<<std::endl;
                            //  mostConfidentRes.second += mostConfidentRes.second * aEConfidences[aeResult.first];
                            //}
                            //else
                        {
                            std::cout << "AutoEncoder reduce..to " << mostConfidentRes.second * (1.0 - aEConfidences[aeResult.first]) << std::endl;
                            mostConfidentRes.second = mostConfidentRes.second * (1.0 - aEConfidences[aeResult.first]);
                        }
                        else
                        {
                            std::cout << "AutoEncoder OK" << std::endl;
                        }
                    }

                    if (mostConfidentRes.second > 1)
                    {
                        mostConfidentRes.second = 1;
                    }
                }
            }
            free(*svmProbability);
            free(svmProbability);
            //free(*svmOneProbability);
            //free(svmOneProbability);
        }
        res = mostConfidentRes;
    }
    return res;
}

//combined with one vs all
std::pair<int, double> CObjectCategorizationGeometric::query2(std::vector<double> featureVector)
{

    assert(this->svms.size() > 0 || this->autoEncoders.size() > 0);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    ///LINEAR SVM
    int svmLinearLabel;
    double *dec = (double *) malloc((this->svmLinear.getMaxLabel()) * sizeof(double));
    svmLinearLabel = this->svmLinear.predict(featureVector, dec);
    std::cout << "SVM Linear " << " Response::\t" << CFileSettings::labels[svmLinearLabel] << "(" << dec[svmLinearLabel - 1] << ")" << std::endl;

    //  mostConfidentRes.first = svmLinearLabel;
    //  mostConfidentRes.second = 100.0f;

    res.first = svmLinearLabel;
    res.second = dec[svmLinearLabel - 1];
    free(dec);

    std::cout << "SVMLEARNER LABEL " << svmLinearLabel << std::endl;

    return res;
}

std::pair<int, double> CObjectCategorizationGeometric::queryEnNoised(pcl::PointCloud<pcl::PointXYZ> queryPointCloud)
{
    assert(this->svms.size() > 0 || this->autoEncoders.size() > 0);
    assert(this->isPseudoCatLoaded);
    std::pair<int, double> res;
    std::pair<int, double> mostConfidentRes;

    mostConfidentRes.first = 0;
    mostConfidentRes.second = 0.0f;

    std::vector<std::map<int, std::pair<int, double> > > getCurrentTreeReponses;
    std::vector<std::pair<int, double> > rslRF = this->objectPseudoCategorizationGeometric.queryPseudoCategoriesEnNoised(queryPointCloud, getCurrentTreeReponses);

    std::vector<std::vector<double> > featureVector;
    for (unsigned int ik = 0; ik < getCurrentTreeReponses.size(); ++ik)
    {
        featureVector.push_back(this->extractFeatureVectorRT(getCurrentTreeReponses[ik]));
    }

    for (unsigned int iterCategories = 1; iterCategories < NUM_CLASSES; ++iterCategories)
    {
        if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(iterCategories) != this->svms.end())
        {
            for (unsigned int ik = 0; ik < featureVector.size(); ++ik)
            {
                double **svmProbability = (double**) malloc(sizeof(double*));
                *svmProbability = NULL;
                char rslSvm = this->svms[iterCategories].predict(featureVector[ik], svmProbability, false);
                std::cout << "SVM " << CFileSettings::labels[iterCategories] << "FV " << ik << ". Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << svmProbability[0][rslSvm - 'A'] << ")"
                          << std::endl;
                //int)rslSvm <<"\n";
                if (rslSvm == 'A')
                {
                    if (svmProbability[0][rslSvm - 'A'] > mostConfidentRes.second) // && svmProbability[0][rslSvm - 'A'] > this->minConfidence)
                    {
                        //if(iterCategories == rslRF.first &&  rslRF.second > 100) // reliable but falseNegatives!!!
                        //if((iterCategories == rslRF.first) || (iterCategories != rslRF.first &&  rslRF.second < 100))
                        {
                            mostConfidentRes.first = iterCategories;
                            mostConfidentRes.second = svmProbability[0][rslSvm - 'A'];
                        }
                    }
                }
                free(*svmProbability);
                free(svmProbability);
            }
        }

        res = mostConfidentRes;
        std::cout << std::endl;
    }
    return res;
}

void CObjectCategorizationGeometric::augmentSetWithScalePyramid(SObjectPointCloudData &set)
{
    assert(set.objectPointClouds.size() > 0);
    float minLeafSize = 0.01;
    float maxLeafSize = 0.25; //0.5;
    int numStep = 2; // number of points clouds to add == number of subsampling steps;

    float step = ((maxLeafSize - minLeafSize) / double(numStep));

    unsigned int numObjectPointClouds = set.objectPointClouds.size();
    for (unsigned int iterPointCloud = 0; iterPointCloud < numObjectPointClouds; ++iterPointCloud)
    {
        for (float curLeafSize = minLeafSize; curLeafSize < maxLeafSize; curLeafSize = curLeafSize + step)
        {
            pcl::PointCloud<pcl::PointXYZ> newPointCloud = set.objectPointClouds[iterPointCloud];

            //Do subsampling....
            toolBox.subsampling(newPointCloud, curLeafSize);

            if (newPointCloud.points.size() > 80)
            {
                //Add to set....
                set.objectPointClouds.push_back(newPointCloud);
                std::cout << "CObjectCategorizationGeometric::augmentSetWithScalePyramid.....add " << curLeafSize << " " << maxLeafSize << std::endl;
                set.objectLabels.push_back(set.objectLabels[iterPointCloud]);
                set.objectNames.push_back(std::string(set.objectNames[iterPointCloud] + "_Scaled_" + boost::lexical_cast<std::string>(curLeafSize)));
                set.numExamples[set.objectLabels[iterPointCloud]]++;
                std::cout << std::string(set.objectNames[iterPointCloud] + "_Scaled_" + boost::lexical_cast<std::string>(curLeafSize)) << std::endl;
            }
            else
            {
                std::cout << "CObjectCategorizationGeometric::augmentSetWithScalePyramid...kick out\n";
            }
        }
    }
}

void CObjectCategorizationGeometric::augmentSetWithEnNoisement(SObjectPointCloudData &set)
{
    assert(set.objectPointClouds.size() > 0);
    unsigned int numObjectToAdd = 2; //3;

    unsigned int numObjectPointClouds = set.objectPointClouds.size();
    for (unsigned int iterPointCloud = 0; iterPointCloud < numObjectPointClouds; ++iterPointCloud)
    {
        for (unsigned int iterToAdd = 0; iterToAdd < numObjectToAdd; ++iterToAdd)
        {
            pcl::PointCloud<pcl::PointXYZ> newPointCloud = set.objectPointClouds[iterPointCloud];
            //Do EnNoisement....
            toolBox.enNoisePointCloud(newPointCloud, this->addEnNoisement);
            //Add to set....
            set.objectPointClouds.push_back(newPointCloud);
            set.objectLabels.push_back(set.objectLabels[iterPointCloud]);
            set.objectNames.push_back(std::string(set.objectNames[iterPointCloud] + "_EnNoised_" + boost::lexical_cast<std::string>(iterToAdd)));
            set.numExamples[set.objectLabels[iterPointCloud]]++;
        }
    }
}

//with pcd file as input
double CObjectCategorizationGeometric::evaluateQuery1() //(SObjectPointCloudData dataSet)
{
    SObjectPointCloudData dataSet = testingData;
    assert(dataSet.objectPointClouds.size() > 0);

    int totalCorrect = 0;
    int totalError = 0;
    if (dataSet.objectPointClouds.size())
    {
        std::cout << "Testing eval....\n";
        //label, count
        std::map<int, int> error;
        std::map<int, int> correct;
        //label, num example
        std::map<int, int> numExamples;

        //label, missclassified as label, counts
        std::map<int, std::map<int, int> > missclassified;

        //  int totalCorrect = 0;
        //  int totalError = 0;

        for (unsigned int i = 0; i < NUM_CLASSES; ++i)
        {
            error[i] = 0;
            correct[i] = 0;
            numExamples[i] = 0;
            for (unsigned int j = 0; j < NUM_CLASSES; ++j)
            {
                missclassified[i][j] = 0;
            }
        }

        for (unsigned int i = 0; i < dataSet.objectPointClouds.size(); ++i)
        {
            std::pair<int, double> result;

            std::cout << "Testing ... " << dataSet.objectNames[i] << "(" << i << ")\n";
            numExamples[dataSet.objectLabels[i]]++;

            result = this->query(dataSet.objectPointClouds[i]);

            std::cout << "Predicted Label = " << result.first << std::endl;

            if (result.first != dataSet.objectLabels[i])
            {
                error[dataSet.objectLabels[i]]++;
                missclassified[dataSet.objectLabels[i]][result.first]++;
                totalError++;
            }
            else
            {
                correct[dataSet.objectLabels[i]]++;
                totalCorrect++;
            }
        }

        std::cout << "....................................." << std::endl;
        for (unsigned int i = 0; i < NUM_CLASSES; ++i)
        {
            assert(correct[i] + error[i] == numExamples[i]);

            std::cout << "class " << i << ":" << std::endl;
            std::cout << "  total num examples  =" << dataSet.objectPointClouds.size() << std::endl;
            std::cout << "  num Examples from class = " << numExamples[i] << std::endl;
            std::cout << "  num Error from class = " << error[i] << std::endl;
            std::cout << "  error against total = " << (double) error[i] / (double) dataSet.objectPointClouds.size() << std::endl;

            for (unsigned int j = 0; j < NUM_CLASSES; ++j)
            {
                std::cout << "		MissClassifications(" << i << ") to Class " << j << " are " << missclassified[i][j] << "(" << (double) missclassified[i][j] / (double) numExamples[i] << " ->numEx || "
                          << (double) missclassified[i][j] / (double) dataSet.objectPointClouds.size() << " ->total)" << std::endl;
            }
            std::cout << "  num Correct from class = " << correct[i] << "(" << (double) correct[i] / (double) numExamples[i] << " ->numEx || " << (double) correct[i]
                      / (double) dataSet.objectPointClouds.size() << " ->total)" << std::endl;
            std::cout << "  num error from class = " << error[i] << "(" << (double) error[i] / (double) numExamples[i] << " ->numEx || " << (double) error[i]
                      / (double) dataSet.objectPointClouds.size() << " ->total)" << std::endl;
        }

        assert(totalError + totalCorrect == dataSet.objectPointClouds.size());
        std::cout << "Total Correct = " << (double) totalCorrect / (double) dataSet.objectPointClouds.size() << std::endl;
        std::cout << "Total Error   = " << (double) totalError / (double) dataSet.objectPointClouds.size() << std::endl;
    }

    return (double) totalError / (double) dataSet.objectPointClouds.size();
}

void CObjectCategorizationGeometric::evaluateSvm1()
{
    assert(this->trainingData.objectPointClouds.size() > 0 || this->testingData.objectPointClouds.size() > 0);

    if (this->testingData.objectPointClouds.size())
    {
        std::cout << "Testing eval....\n";
        //label, count
        std::map<int, int> error;
        std::map<int, int> correct;
        //label, num example
        std::map<int, int> numExamples;

        //label, missclassified as label, counts
        std::map<int, std::map<int, int> > missclassified;

        int totalCorrect = 0;
        int totalError = 0;
        int total = 0;

        for (unsigned int i = 0; i < NUM_CLASSES; ++i)
        {
            error[i] = 0;
            correct[i] = 0;
            numExamples[i] = 0;
            for (unsigned int j = 0; j < NUM_CLASSES; ++j)
            {
                missclassified[i][j] = 0;
            }
        }

        for (unsigned int iterCat = 0; iterCat < NUM_CLASSES; ++iterCat)
        {
            for (unsigned int i = 0; i < this->testingData.objectPointClouds.size(); ++i)
            {
                bool result;

                std::cout << "Testing ... " << this->testingData.objectNames[i] << "(" << i << ")\n";

                result = this->querySvm(this->testingData.objectPointClouds[i], iterCat);

                std::cout << "Predicted Label = " << this->testingData.objectLabels[i] << "=" << result << std::endl;

                if (!result && iterCat == this->testingData.objectLabels[i])
                {
                    error[iterCat]++;
                    totalError++;
                }
                else if (result && iterCat != this->testingData.objectLabels[i])
                {
                    error[iterCat]++;
                    totalError++;
                }
                else
                {
                    correct[iterCat]++;
                    totalCorrect++;
                }

                if (iterCat == this->testingData.objectLabels[i])
                {
                    numExamples[i]++;
                }
            }

        }

        std::cout << "....................................." << std::endl;
        for (unsigned int i = 0; i < NUM_CLASSES; ++i)
        {
            std::cout << "class " << i << ":" << std::endl;
            std::cout << "  total num examples  =" << this->testingData.objectPointClouds.size() << std::endl;
            std::cout << "  num Examples from class " << numExamples[i] << std::endl;
            std::cout << "  num Error of class = " << error[i] << std::endl;
            std::cout << "  num correct of class = " << correct[i] << std::endl;
            std::cout << "  error against total = " << (double) error[i] / (double) this->testingData.objectPointClouds.size() << std::endl;
            std::cout << "  correct against total = " << (double) correct[i] / (double) this->testingData.objectPointClouds.size() << std::endl;


        }

        std::cout << "Total Correct = " << (double) totalCorrect / (double) total << std::endl;
        std::cout << "Total Error   = " << (double) totalError / (double) total << std::endl;
    }
}

//with pcd
bool CObjectCategorizationGeometric::querySvm(pcl::PointCloud<pcl::PointXYZ> queryPointCloud, unsigned int category)
{
    assert(this->svms.size() > 0 || this->autoEncoders.size() > 0);
    assert(this->isPseudoCatLoaded);

    bool res = false;
    std::map<int, std::pair<int, double> > getCurrentTreeReponses;
    std::pair<int, double> rslRF = this->objectPseudoCategorizationGeometric.queryPseudoCategories(queryPointCloud, getCurrentTreeReponses);
    std::vector<double> featureVector = this->extractFeatureVectorRT(getCurrentTreeReponses);

    if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(category) != this->svms.end())
    {
        double **svmProbability = (double**) malloc(sizeof(double*));
        *svmProbability = NULL;
        char rslSvm = this->svms[category].predict(featureVector, svmProbability, false);
        std::cout << "SVM " << CFileSettings::labels[category] << " Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << svmProbability[0][rslSvm - 'A'] << ")" << std::endl;
        //int)rslSvm <<"\n";
        if (rslSvm == 'A')
        {

            res = true;

        }
        free(*svmProbability);
        free(svmProbability);
    }

    return res;
}

//only with extracted Feature vector as input
bool CObjectCategorizationGeometric::evaluateSvm2()
{
    assert(this->svms.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    assert(this->testingData.extractedFeatureVectors.size() > 0);
    assert(this->trainingData.extractedFeatureVectors.size() > 0);

    std::vector<std::vector<double> > trainingFeatureVectorAll;
    std::vector<double> trainingFeatureVectorAllLabel;

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = this->trainingData.extractedFeatureVectors.begin(); iterCategories != this->trainingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            trainingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            trainingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    for (iterCategories = this->testingData.extractedFeatureVectors.begin(); iterCategories != this->testingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    std::vector<int> errors;
    errors.resize(this->svms.size() + 1);
    for (unsigned int iterVec = 0; iterVec < testingFeatureVectorAll.size(); ++iterVec)
    {

        for (unsigned int iterCat = 1; iterCat < this->testingData.objectLabels.size(); ++iterCat)
        {
            if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(iterCat) != this->svms.end())
            {

                double **svmProbability = (double**) malloc(sizeof(double*));
                *svmProbability = NULL;
                char rslSvm = this->svms[iterCat].predict(testingFeatureVectorAll[iterVec], svmProbability, false);
                std::cout << "SVM " << CFileSettings::labels[iterCat] << " Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << svmProbability[0][rslSvm - 'A'] << ")" << std::endl;
                //int)rslSvm <<"\n";
                if (rslSvm != 'A' && iterCat == testingFeatureVectorAllLabel[iterVec])
                {
                    errors[iterCat]++;
                }
                free(*svmProbability);
                free(svmProbability);
            }
        }
    }

    for (unsigned int iterCat = 1; iterCat < errors.size(); ++iterCat)
    {
        std::cout << "Catergory " << iterCat << " Error" << (double) errors[iterCat] / testingFeatureVectorAll.size() << std::endl;
    }

    return true;
}

//only with extracted Feature vector as input
void CObjectCategorizationGeometric::evaluateSvm3()
{
    assert(this->svms.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    assert(this->testingData.extractedFeatureVectors.size() > 0);
    assert(this->trainingData.extractedFeatureVectors.size() > 0);

    std::vector<std::vector<double> > trainingFeatureVectorAll;
    std::vector<double> trainingFeatureVectorAllLabel;

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = this->trainingData.extractedFeatureVectors.begin(); iterCategories != this->trainingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            trainingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            trainingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    for (iterCategories = this->testingData.extractedFeatureVectors.begin(); iterCategories != this->testingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    std::vector<int> errors;
    errors.resize(this->svms.size() + 1);
    for (unsigned int iterVec = 0; iterVec < testingFeatureVectorAll.size(); ++iterVec)
    {

        for (unsigned int iterCat = 1; iterCat < this->testingData.objectLabels.size(); ++iterCat)
        {
            if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(iterCat) != this->svms.end())
            {

                double **svmProbability = (double**) malloc(sizeof(double*));
                *svmProbability = NULL;
                char rslSvm = this->svms[iterCat].predict(testingFeatureVectorAll[iterVec], svmProbability, false);
                //std::cout << "SVM " << CFileSettings::labels[iterCat] << " Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << svmProbability[0][rslSvm - 'A'] << ")" << std::endl;
                //int)rslSvm <<"\n";
                if (rslSvm != 'A' && iterCat == testingFeatureVectorAllLabel[iterVec])
                {
                    errors[iterCat]++;
                }
                free(*svmProbability);
                free(svmProbability);
            }
        }
    }

    for (unsigned int iterCat = 1; iterCat < errors.size(); ++iterCat)
    {
        std::cout << "Catergory Test " << iterCat << " Error" << (double) errors[iterCat] / testingFeatureVectorAll.size() << std::endl;
    }

    std::vector<int> errorsT;
    errorsT.resize(this->svms.size() + 1);
    for (unsigned int iterVec = 0; iterVec < trainingFeatureVectorAll.size(); ++iterVec)
    {

        for (unsigned int iterCat = 1; iterCat < this->trainingData.objectLabels.size(); ++iterCat)
        {
            if (this->svms.size() > 0 && this->doQuerySvm && this->svms.find(iterCat) != this->svms.end())
            {

                double **svmProbability = (double**) malloc(sizeof(double*));
                *svmProbability = NULL;
                char rslSvm = this->svms[iterCat].predict(trainingFeatureVectorAll[iterVec], svmProbability, false);
                //std::cout << "SVM " << CFileSettings::labels[iterCat] << " Response::\t" << (rslSvm == 'A' ? "1" : "0") << "\t(" << svmProbability[0][rslSvm - 'A'] << ")" << std::endl;
                //int)rslSvm <<"\n";
                if (rslSvm != 'A' && iterCat == trainingFeatureVectorAllLabel[iterVec])
                {
                    errorsT[iterCat]++;
                }
                free(*svmProbability);
                free(svmProbability);
            }
        }
    }

    for (unsigned int iterCat = 1; iterCat < errors.size(); ++iterCat)
    {
        std::cout << "Catergory Train " << iterCat << " Error" << (double) errors[iterCat] / trainingFeatureVectorAll.size() << std::endl;
    }
}

void CObjectCategorizationGeometric::evaluateQuery2()
{
    assert(this->svms.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    assert(this->testingData.extractedFeatureVectors.size() > 0);
    assert(this->trainingData.extractedFeatureVectors.size() > 0);

    std::vector<std::vector<double> > trainingFeatureVectorAll;
    std::vector<double> trainingFeatureVectorAllLabel;

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = this->trainingData.extractedFeatureVectors.begin(); iterCategories != this->trainingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            trainingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            trainingFeatureVectorAllLabel.push_back(iterCategories->first);

            //std::cout<<iterCategories->first<<" ";
        }
    }

    for (iterCategories = this->testingData.extractedFeatureVectors.begin(); iterCategories != this->testingData.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    int errors = 0;

    std::vector<int> errorsCat;
    errorsCat.resize(this->testingData.extractedFeatureVectors.size() + 1);
    std::vector<int> numCat;
    numCat.resize(this->testingData.extractedFeatureVectors.size() + 1);

    for (unsigned int iterVec = 0; iterVec < testingFeatureVectorAll.size(); ++iterVec)
    {
        std::pair<int, double> res = this->query(testingFeatureVectorAll[iterVec]);

        if (res.first != testingFeatureVectorAllLabel[iterVec])
        {
            errors++;
            errorsCat[testingFeatureVectorAllLabel[iterVec]]++;
        }
        numCat[testingFeatureVectorAllLabel[iterVec]]++;
    }

    std::cout << "Catergory Test total Error" << (double) errors / testingFeatureVectorAll.size() << std::endl;
    for (unsigned int it = 0; it < numCat.size(); ++it)
    {
        std::cout << "Catergory Test Cat " << it << " Error" << (double) errorsCat[it] / numCat[it] << "(" << errorsCat[it] << "|" << numCat[it] << ") " << std::endl;
    }

    errorsCat.clear();
    numCat.clear();
    errorsCat.resize(this->trainingData.extractedFeatureVectors.size() + 1);
    numCat.resize(this->trainingData.extractedFeatureVectors.size() + 1);

    errors = 0;
    //errors.resize(testingFeatureVectorAllLabel.size());
    for (unsigned int iterVec = 0; iterVec < trainingFeatureVectorAll.size(); ++iterVec)
    {
        std::pair<int, double> res = this->query(trainingFeatureVectorAll[iterVec]);

        if (res.first != trainingFeatureVectorAllLabel[iterVec])
        {
            errors++;
            errorsCat[trainingFeatureVectorAllLabel[iterVec]]++;
        }
        numCat[trainingFeatureVectorAllLabel[iterVec]]++;
    }
    std::cout << "Catergory Train total Error" << (double) errors / trainingFeatureVectorAll.size() << std::endl;
    for (unsigned int it = 0; it < numCat.size(); ++it)
    {
        std::cout << "Catergory Train Cat " << it << " Error" << (double) errorsCat[it] / numCat[it] << "(" << errorsCat[it] << "|" << numCat[it] << ") " << std::endl;
    }

    CFileSettings::coutStdVector(trainingFeatureVectorAll[0], true);
    CFileSettings::coutStdVector(testingFeatureVectorAll[0], true);
}

double CObjectCategorizationGeometric::evaluateQuery22(SObjectPointCloudData dataSet)
{

    assert(this->svms.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    assert(dataSet.extractedFeatureVectors.size() > 0);

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = dataSet.extractedFeatureVectors.begin(); iterCategories != dataSet.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    int errors = 0;

    std::vector<int> errorsCat;
    errorsCat.resize(dataSet.extractedFeatureVectors.size() + 1);
    std::vector<int> numCat;
    numCat.resize(dataSet.extractedFeatureVectors.size() + 1);

    for (unsigned int iterVec = 0; iterVec < testingFeatureVectorAll.size(); ++iterVec)
    {
        std::pair<int, double> res = this->query(testingFeatureVectorAll[iterVec]);

        if (res.first != testingFeatureVectorAllLabel[iterVec])
        {
            errors++;
            errorsCat[testingFeatureVectorAllLabel[iterVec]]++;
        }
        numCat[testingFeatureVectorAllLabel[iterVec]]++;
    }

    double totErr = (double) errors / testingFeatureVectorAll.size();
    std::cout << "Catergory Test total Error" << totErr << std::endl;
    for (unsigned int it = 0; it < numCat.size(); ++it)
    {
        std::cout << "Catergory Test Cat " << it << " Error" << (double) errorsCat[it] / numCat[it] << "(" << errorsCat[it] << "|" << numCat[it] << ") " << std::endl;
    }
    return totErr;
}

double CObjectCategorizationGeometric::evaluateQuery22(int set)
{
    SObjectPointCloudData dataSet;
    if (set == 1)
        SObjectPointCloudData dataSet = this->trainingData;
    if (set == 2)
        dataSet = this->testingData;
    if (set == 3)
        dataSet = this->testingData2;

    assert(this->svms.size() > 0);

    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories;

    assert(dataSet.extractedFeatureVectors.size() > 0);

    std::vector<std::vector<double> > testingFeatureVectorAll;
    std::vector<double> testingFeatureVectorAllLabel;

    for (iterCategories = dataSet.extractedFeatureVectors.begin(); iterCategories != dataSet.extractedFeatureVectors.end(); ++iterCategories)
    {
        for (unsigned int iterVec = 0; iterVec < iterCategories->second.size(); ++iterVec)
        {
            testingFeatureVectorAll.push_back(iterCategories->second[iterVec]);
            testingFeatureVectorAllLabel.push_back(iterCategories->first);
        }
    }

    int errors = 0;

    std::vector<int> errorsCat;
    errorsCat.resize(dataSet.extractedFeatureVectors.size() + 1);
    std::vector<int> numCat;
    numCat.resize(dataSet.extractedFeatureVectors.size() + 1);

    for (unsigned int iterVec = 0; iterVec < testingFeatureVectorAll.size(); ++iterVec)
    {
        std::pair<int, double> res = this->query(testingFeatureVectorAll[iterVec]);

        if (res.first != testingFeatureVectorAllLabel[iterVec])
        {
            errors++;
            errorsCat[testingFeatureVectorAllLabel[iterVec]]++;
        }
        numCat[testingFeatureVectorAllLabel[iterVec]]++;
    }

    double totErr = (double) errors / testingFeatureVectorAll.size();
    std::cout << "Catergory Test total Error" << totErr << std::endl;
    for (unsigned int it = 0; it < numCat.size(); ++it)
    {
        std::cout << "Catergory Test Cat " << it << " Error" << (double) errorsCat[it] / numCat[it] << "(" << errorsCat[it] << "|" << numCat[it] << ") " << std::endl;
    }
    return totErr;
}

