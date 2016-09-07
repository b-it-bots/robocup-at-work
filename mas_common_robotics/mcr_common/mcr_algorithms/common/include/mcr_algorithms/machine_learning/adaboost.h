/*
 * adaboost.h
 *
 *  Created on: Jul 22, 2011
 *      Author: Frederik Hegger
 */

#ifndef ADABOOST_H_
#define ADABOOST_H_

#include <opencv2/core/core_c.h>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <cstdio>
#include <math.h>

#define FIRST_LABEL     '1'

using namespace std;

typedef std::map<char, double> LabelMap;

class AdaBoost
{
public:
    AdaBoost(const unsigned int no_of_classes, const unsigned int no_of_features);

    void setNumberOfFeature(const double no_of_feature)
    {
        this->number_of_features_ = no_of_feature;
    };

    int train(const char* samples_filename, const char* model_filename, const double ratio, double &train_error, double &test_error);
    int loadModel(const char* model_filename);
    int test(const char* sample_filename, const char* model_filename, double &test_error);
    LabelMap classify(CvMat* data);
    LabelMap classify(const vector<double> feature_vector);

private:
    int read_num_class_data(const char* filename, int var_count, CvMat** data, CvMat** responses);

    CvBoost classifier_;
    CvBoostParams boost_parameters_;
    unsigned int number_of_features_;
    int number_of_classes_;
    bool is_modelfile_loaded_;
};

#endif /* ADABOOST_H_ */
