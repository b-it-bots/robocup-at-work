/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/


#ifndef __CSVMClassifier_H__
#define __CSVMClassifier_H__

#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>

#include <toolbox_ros.h>

#include <libsvm/svm.h>


#define GRID_SEARCH_ITERATIONS 10 //12 //12 
//12
#define MALLOC(type,n) (type *)malloc((n)*sizeof(type))

struct SSetting
{
    SSetting() : c(-1), gamma(-1), nu(-1), gammaExponent(-1), cExponent(-1) , trainError(-1), testError(-1), cvError(-1), trainExampleSize(-1) {};

    double c;
    double gamma;
    double nu;

    double gammaExponent; // 2^gammaExponent = gamma
    double cExponent; // 2^cExponent = c

    double trainError;
    double testError;
    double cvError;
    int trainExampleSize;
};

class CSvmClassifier
{
protected:
    CToolBoxROS toolBox;
    struct svm_parameter strSvmParameter;
    struct svm_problem strSvmProblem;
    struct svm_model *svmModel;
    std::vector< std::vector<double> > trainingSet;
    SSetting currentSetupResults;
    int gridSearchIterations;
    std::string id;
    std::string logFile;
    bool isOneClass;

public:
    CSvmClassifier();
    CSvmClassifier(std::string id, std::string logFile);
    int convertTrainingDataAdaBoostToSVM(std::string inFilename, std::string outFilename);
    void setOneClass(bool isOneClass);
    void setSvmParameter();
    void setTrainingSet(std::vector< std::vector<double> > param_trainingSet);
    int readSvmProblem();
    int readSvmProblem(unsigned int numOfExamples);
    double trainSvm(std::vector< std::vector<double> > param_traininSet);

    char predict(std::vector<double> testExample, double **prob_estimates, bool isLabeled);

    double parameterGridSearch(std::vector< std::vector<double> > trainSet, std::vector< std::vector<double> > testSet, int numOfTrainExamples, bool prob);
    double crossValidation(int iNrfold);
    double crossValidation2(int numFolds);
    double evaluate(std::vector< std::vector<double> > testSet);
    SSetting findBestModel(std::vector<SSetting> results, int option);
    int trainingSetCrossValidation(std::vector< std::vector<double> > trainSet, std::vector< std::vector<double> > testSet);
    double evalOptimalModelTraining(std::vector< std::vector<double> > trainSet, std::vector< std::vector<double> > testSet, int numOfTrainExamples, bool prob);

    void loadSvmModel(std::string file);
    void saveSvmModel(std::string file);

    bool isProbability();
    void setGridSearchIterations(int gridSearchIterations);
};


#endif
