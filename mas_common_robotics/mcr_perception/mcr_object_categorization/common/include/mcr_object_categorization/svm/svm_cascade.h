/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/


#ifndef CSVMCASCADE_H_
#define CSVMCASCADE_H_

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

#include "svm/b_svm_classifier.h"

class CSvmCascade
{
#define POSITIVE 1
#define NEGATIVE 0
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & cascade;
        ar & numStages;
        ar & numIterations;
        ar & id;
        ar & homePath;
        ar & trainingType;
        ar & cascadeStrongClassifierErr;
    }

    CLogger *logger;
    std::map<int, std::vector<std::vector<double> > > trainSet;
    std::map<int, std::vector<std::vector<double> > > validationSet; //which is part of the trainSet
    std::map<int, std::vector<std::vector<double> > > testSet;

    //cascade, iteration, subClassifiers
    std::map<int, CBSvmClassifier> cascade;
    std::map<int, double> cascadeStrongClassifierErr;
    std::map<int, int, std::string> subClassifersModels; //filenames

    unsigned int numStages;
    unsigned int numIterations;
    std::string id;
    std::string homePath;
    boost::mt19937 rng;

    int trainingType;

    std::map<int, std::vector<std::vector<double> > > createValidationSet(double ratio);
public:
    CSvmCascade();

    void setHomePath(std::string homePath);
    void setTrainSet(std::map<int, std::vector<std::vector<double> > > fv);
    void setTestSet(std::map<int, std::vector<std::vector<double> > > fv);
    void setNumStages(unsigned int numStages);
    void setNumIterations(unsigned int numIterations);
    void setId(std::string id)
    {
        this->id = id;
    }

    void train0();
    void train1();
    void train2();
    char predict(std::vector<double> query, double &confidence, bool isLabels);
    double verifyModel(std::map<int, std::vector<std::vector<double> > > set, std::map<int, std::vector<int> > &missClassified);

    std::vector<std::vector<double> > createSet(int label, std::vector<std::vector<double> > set, std::vector<int> idx, int minSize);
    std::vector<std::vector<double> > createSet2(int label, std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, std::vector<int> idx, int minSize);
    std::vector<std::vector<double> > createSet3(int label, std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, std::vector<int> idx, int minSize);
    void saveCascadeModel();
    void loadCascadeModel();
    ~CSvmCascade();
};

#endif /* CSVMCASCADE_H_ */
