/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/


#ifndef __CBSVMClassifier_H__
#define __CBSVMClassifier_H__

#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>

#include <boost/random.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/lexical_cast.hpp>

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

#include "logger.h"
#include "svm/svm_classifier.h"

class CBSvmClassifier
{

#define POSITIVE 1
#define NEGATIVE 0
#define NUM_CLASSIFIER_TRAILS 50
#define NUM_CLASSIFIERS 20

private:

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & svmModels;
        ar & svmError;
        ar & svmAccuracy;
        ar & numClassifier;
        ar & homePath;
    }



    CLogger *logger;
    //id, svm
    std::map<int, CSvmClassifier> svms;

    //label,acc
    std::map<int, std::map<int, double> >svmAccuracy;

    //id, error of Testset
    std::map<int, double> svmError;
    unsigned int numClassifier;

    //entireSet
    //label(1/0),featurevector
    std::map<int, std::vector<std::vector<double> > > trainSet;
    std::map<int, std::vector<std::vector<double> > > testSet;
    std::map<int, std::string> svmModels; //filenames w/o homepath!!!

    std::vector<std::vector<double> > createBootstrap(std::vector<std::vector<double> > &featureVectors, unsigned int numberVec);
    std::vector<std::vector<double> > createOneVsAllset(int label, std::map<int, std::vector<std::vector<double> > > &featureVectors);
    int signFnct(double value);

    std::string homePath;
    boost::mt19937 rng;
    std::string id;
public:
    CBSvmClassifier();

    void setId(std::string id);
    void setHomePath(std::string homePath);
    void setTrainSet(std::map<int, std::vector<std::vector<double> > > trainSet);
    void setTestSet(std::map<int, std::vector<std::vector<double> > > testSet);

    void loadModel();
    void saveModel();

    void setNumClassifier(unsigned int numClassifier);

    double train(std::map<int, std::vector<int> > &missClassified);
    char predict(std::vector<double> query, double &confidence, bool isLabeled = false);
    double verifyModel(std::map<int, std::vector<int> > &missClassified);
    std::map<int, double> verifyWeakLearnerModel(CSvmClassifier &weakLearner);
    unsigned int getNumClassifers()
    {
        return this->numClassifier;
    }
};

BOOST_CLASS_VERSION(CBSvmClassifier, 1)
#endif
