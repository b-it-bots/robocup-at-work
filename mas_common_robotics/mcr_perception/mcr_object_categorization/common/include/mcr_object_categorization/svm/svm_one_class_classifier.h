/*
 * CSvmOneClassClassifier.h
 *
 *  Created on: Aug 11, 2011
 *      Author: Christian Mueller
 */

#ifndef CSVMONECLASSCLASSIFIER_H_
#define CSVMONECLASSCLASSIFIER_H_

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include "logger.h"
#include "svm/svm_classifier.h"

class CSvmOneClassClassifier : public CSvmClassifier
{
private:
    CLogger *logger;
    int positiveLabel;
    double confidence;
public:
    CSvmOneClassClassifier();
    CSvmOneClassClassifier(std::string id, std::string logFile);

    void setConfidence(double confidence)
    {
        this->confidence = confidence;
    };
    double getConfidence(double confidence)
    {
        return this->confidence;
    };
    void setSvmParameter();
    double trainSvm(std::vector< std::vector<double> > param_traininSet, std::vector< std::vector<double> > param_testSet);
    double parameterGridSearch(std::vector< std::vector<double> > trainSet, std::vector< std::vector<double> > testSet, int numOfTrainExamples, bool prob);
    double parameterGridSearch2(std::vector< std::vector<double> > trainSet, std::vector< std::vector<double> > testSet, int numOfTrainExamples, bool prob);
    double evaluateOneClass(std::vector< std::vector<double> > testSet, bool doDebug = false);
    ~CSvmOneClassClassifier();
};

#endif /* CSVMONECLASSCLASSIFIER_H_ */
