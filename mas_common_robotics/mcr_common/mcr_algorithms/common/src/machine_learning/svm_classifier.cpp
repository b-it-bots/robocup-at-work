#include <mcr_algorithms/machine_learning/svm_classifier.h>
#include <map>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/archive/text_iarchive.hpp>


SVMClassifier::SVMClassifier()
    : model_(0), num_features_(0)
{
}

SVMClassifier::~SVMClassifier()
{
    if (model_ != 0)
    {
        svm_free_and_destroy_model(&model_);
    }
}

bool SVMClassifier::loadModel(const bfs::path &model_filepath, const bfs::path &config_filepath)
{
    model_ = svm_load_model(model_filepath.string().c_str());

    std::ifstream config_file;
    config_file.open(config_filepath.string().c_str());

    if (config_file.is_open())
    {
        boost::archive::text_iarchive archive(config_file);
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

int SVMClassifier::classify(std::vector<double> features, std::string &label) const
{
    if (features.size() != num_features_)
    {
        label = "";
        return 1;
    }

    normalizeFeatures(features);

    // see docs/machine_learning/libsvm_README for structure of svm_node
    std::vector<svm_node> feature_vector(num_features_ + 1);
    feature_vector[num_features_].index = -1;

    for (int i = 0; i < num_features_; i++)
    {
        feature_vector[i].index = i + 1;
        feature_vector[i].value = features.at(i);
    }

    int label_index = svm_predict(model_, &feature_vector[0]);

    if (labels_.empty() || label_index > labels_.size())
    {
        label = "";
        return 2;
    }

    label = labels_.at(label_index - 1);

    return 0;
}

int SVMClassifier::classifyWithProbability(std::vector<double> features, std::string &label, double &probability) const
{
    if (features.size() != num_features_)
    {
        label = "";
        probability = 0.0;
        return 1;
    }

    normalizeFeatures(features);

    // see docs/machine_learning/libsvm_README for structure of svm_node
    std::vector<svm_node> feature_vector(num_features_ + 1);
    feature_vector[num_features_].index = -1;

    for (int i = 0; i < num_features_; i++)
    {
        feature_vector[i].index = i + 1;
        feature_vector[i].value = features.at(i);
    }

    std::vector<double> probabilities(num_features_);

    int label_index = 0;

    if (svm_check_probability_model(model_))
    {
        label_index = svm_predict_probability(model_, &feature_vector[0], &probabilities[0]);
        probability = probabilities.at(label_index - 1);
    }
    else
    {
        label_index = svm_predict(model_, &feature_vector[0]);
        probability = 1.0;
    }

    if (labels_.empty() || label_index > labels_.size())
    {
        label = "";
        probability = 0.0;
        return 2;
    }

    label = labels_.at(label_index - 1);

    return 0;
}

void SVMClassifier::normalizeFeatures(std::vector<double> &features) const
{
    for (int i = 0; i < features.size(); i++)
    {
        features[i] = (features[i] - means_[i]) / std_deviations_[i];
    }
}
