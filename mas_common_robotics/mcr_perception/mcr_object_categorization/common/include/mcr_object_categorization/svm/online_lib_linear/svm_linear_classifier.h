/*
 * CSVMLinearClassifier.cpp
 *
 *  Created on: Oct 26, 2011
 *      Author: mca
 *      Based on Kevin Lai, Liefeng Bo, Xiaofeng Ren, and Dieter Fox. A Scalable Tree-based Approach for Joint Object and Pose Recognition. In the Twenty-Fifth Conference on Artificial Intelligence (AAAI), August 2011.
 */

#ifndef CSVMLINEARCLASS_H
#define CSVMLINEARCLASS_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>
#include <map>
#include <vector>
#include "linear.h"

struct example
{
    double *x;
    int y;
};

class CSvmLinearClassifier
{
protected:
    struct feature_node *x_space;
    long int x_space_len;

    struct parameter param;
    struct problem prob;
    struct model* model_;
    char *model_path;
    std::map<int, std::vector< std::vector<double> > > trainingFeatureVectors;

    int flag_cross_validation;
    int nr_fold;
    double bias;
    char *line;
    int max_line_len;
    int cv;


    //Online learning
    struct example *examples;
    double *feature_space;


    void print_null(const char *s);
    void exit_with_help();

    char* readline(FILE *input);
    void exit_input_error(int line_num);
    void parse_command_line(int argc, char **argv, char *input_file_name, char *model_file_name);
    void read_problem(const char *filename);
    void read_problem(std::map<int, std::vector< std::vector<double> > > trainingFeatureVectors);
    struct example * readNewExamples(const char *examples_path, int &numNewExamples);
    struct example * readNewExamples(std::vector< std::vector<double> > trainingFeatureVectors, int label, int &numNewExamples);



    void onlineTrainAll(bool saveModel);
public:
    CSvmLinearClassifier();

    int getMaxLabel();
    void setCrossValidationNFoldCMD(int nFold = 10);
    void crossValidation(std::map<int, std::vector<std::vector<double> > > trainingFeatureVectors, int nFold = 10);
    void do_cross_validation();
    void setParameter();
    int trainModel(std::map<int, std::vector<std::vector<double> > > trainingFeatureVectors, char* modelName);
    int trainModel(int argc, char **argv);


    void loadModel(std::string modelFile);
    void clean();

    void saveTrainingData(std::string filename);

    void updateModel(std::string oldModel, std::vector<std::vector<double> > trainingFeatureVectors, int label, std::string updatedModel);

    /*online part*/
    /* Load existing data and the model trained on it */
    void loadOfflineData(const char *prob_path, const char *model_path);
    void loadOfflineData(std::string path);

    /* Get the number of classes in the current model */
    int getNumClasses();

    /* test example using current model */
    int predict(std::vector<double> featureVector, double *dec_values);
    int testExample(std::vector<double> featureVector, double *dec_values);
    int testExample(const double *feature, int feature_len, double *dec_values);

    void saveModel(std::string fileName);
    void saveProblem(std::string filename);
    void saveModel2(std::string filename);

    /* Read new examples */
//  int AddNewExamples(const char *example_path, double *feature, int example);

    /* Add example to the dataset and optionally update the model */
    void onlineAddExample(const double *feature, int feature_len, int label, bool updateModel);

    /* Update the model and optionally save it to file */
//  void onlineTrainAll(bool saveModel);*/


};

#endif

