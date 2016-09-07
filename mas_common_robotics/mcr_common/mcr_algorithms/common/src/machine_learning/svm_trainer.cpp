#include <mcr_algorithms/machine_learning/svm_trainer.h>
#include <map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/archive/text_oarchive.hpp>

SVMTrainer::SVMTrainer(int num_features)
    : model_(0), num_features_(num_features), means_(num_features, 0.0), std_deviations_(num_features, 0.0)
{
}

SVMTrainer::~SVMTrainer()
{
}

bool SVMTrainer::addTrainingSample(const std::vector<double> &features, const std::string &label)
{
    if (features.size() != num_features_)
    {
        return false;
    }

    training_sample s(label, features);
    training_samples_.push_back(s);
    labels_.push_back(label);

    return true;
}

bool SVMTrainer::trainAndSaveModel(const bfs::path &model_filepath, const bfs::path &config_filepath)
{
    if (training_samples_.empty())
    {
        return false;
    }

    normalizeTrainingSamples(training_samples_);

    // get unique labels
    std::vector<std::string>::iterator iter = unique(labels_.begin(), labels_.end());
    labels_.resize(distance(labels_.begin(), iter));

    // labels are sorted so that the label_index corresponding to the string label remains
    // consistent regardless of the order in which training samples are added
    sort(labels_.begin(), labels_.end());

    std::map<std::string, int> label_encoder;
    std::vector<std::string>::iterator lab_it;
    int label_index = 0;

    for (lab_it = labels_.begin(); lab_it != labels_.end(); lab_it++)
    {
        label_encoder.insert(std::pair<std::string, int>(*lab_it, label_index));
        label_index++;
    }

    svm_problem problem;

    // see docs/machine_learning/libsvm_README for structure of svm_problem
    problem.l = training_samples_.size();

    // feature_vector_ptr holds pointers to the vector of svm_nodes in feature_vector_vector
    // std::vectors are used to avoid use of new and delete
    std::vector<std::vector<svm_node> > feature_vector_vector(training_samples_.size());
    std::vector<svm_node *> feature_vector_ptr(training_samples_.size());

    std::vector<double> labels_vector(training_samples_.size());

    std::vector<training_sample>::iterator it;

    int index = 0;

    for (it = training_samples_.begin(); it != training_samples_.end(); it++)
    {

        std::vector<svm_node> feature_vector(num_features_ + 1);
        feature_vector[num_features_].index = -1;

        for (int i = 0; i < num_features_; i++)
        {
            feature_vector[i].index = i + 1;
            feature_vector[i].value = it->second.at(i);
        }

        feature_vector_vector[index] = feature_vector;
        feature_vector_ptr[index] = &feature_vector_vector[index][0];
        labels_vector[index] = (label_encoder[it->first] + 1);
        index++;
    }

    // pointer to vector of svm_node pointers
    problem.x = &feature_vector_ptr[0];
    problem.y = &labels_vector[0];

    // see docs/machine_learning/libsvm_README for structure of svm_parameter
    svm_parameter param;

    // Default parameters used as in https://github.com/cjlin1/libsvm/blob/master/svm-train.c
    param.svm_type = C_SVC; // multi-class classifier
    param.kernel_type = RBF; // radial basis function
    param.degree = 3; // degree of the radial basis function
    param.gamma = 1.0 / num_features_;
    param.coef0 = 0;
    param.nu = 0.5;
    param.cache_size = 100;
    param.C = 1;
    param.eps = 1e-3;
    param.p = 0.1;
    param.shrinking = 1;
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;

    model_ = svm_train(&problem, &param);

    saveModel(model_filepath, config_filepath);

    svm_destroy_param(&param);
    svm_free_and_destroy_model(&model_);

    return true;
}

bool SVMTrainer::saveModel(const bfs::path &model_filepath, const bfs::path &config_filepath) const
{
    svm_save_model(model_filepath.string().c_str(), model_);

    std::ofstream config_file;
    config_file.open(config_filepath.string().c_str());

    if (config_file.is_open())
    {
        boost::archive::text_oarchive archive(config_file);
        archive & num_features_;
        archive & labels_;
        archive & means_;
        archive & std_deviations_;

        config_file.close();

        return true;
    }

    config_file.close();

    return false;
}

void SVMTrainer::normalizeTrainingSamples(std::vector<training_sample> &samples)
{
    std::vector<training_sample>::iterator it;

    // sum up features
    for (it = samples.begin(); it != samples.end(); it++)
    {
        std::vector<double> features = it->second;

        for (int j = 0; j < features.size(); j++)
        {
            means_[j] = means_[j] + features[j];
        }
    }

    // get their means
    for (int i = 0; i < means_.size(); i++)
    {
        means_[i] = means_[i] / samples.size();
    }

    // squared sum of difference from means
    for (it = samples.begin(); it != samples.end(); it++)
    {
        std::vector<double> features = it->second;

        for (int j = 0; j < features.size(); j++)
        {
            std_deviations_[j] = std_deviations_[j] + std::pow((features[j] - means_[j]), 2);
        }
    }

    // calculate standard deviations
    for (int i = 0; i < std_deviations_.size(); i++)
    {
        std_deviations_[i] = std::sqrt(std_deviations_[i] / samples.size());
    }

    // normalize samples
    for (it = samples.begin(); it != samples.end(); it++)
    {
        for (int j = 0; j < it->second.size(); j++)
        {
            it->second[j] = (it->second[j] - means_[j]) / std_deviations_[j];
        }
    }
}
