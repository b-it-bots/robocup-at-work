/*
 * random_trees.h
 *
 *  Created on: Apr 19, 2011
 *      Author: Frederik Hegger
 */

#ifndef RANDOM_TREES_H_
#define RANDOM_TREES_H_

#include <opencv2/core/core_c.h>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <cstdio>
#include <math.h>

using namespace std;

typedef std::map<char, double> LabelMap;

class RandomTrees
{
public:
    RandomTrees(const unsigned int no_of_features);

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

    CvRTrees forest_;
    CvRTParams tree_parameters_;
    unsigned int number_of_features_;
    bool is_modelfile_loaded_;
};

#endif /* RANDOM_TREES_H_ */
