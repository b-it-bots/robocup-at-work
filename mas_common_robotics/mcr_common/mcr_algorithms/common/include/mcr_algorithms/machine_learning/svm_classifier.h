#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include <libsvm/svm.h>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

/**
 * This class is a wrapper for multi-class classification using libsvm.
 * The class is used to classify a sample (defined by a set of features) into a class. The
 * SVM classfier should have been created using the SVMTrainer class and saved.
 *
 * The SVM classifier (created by SVMTrainer) needs to be loaded first (both the model and configuration files).
 * Samples can then be classified using the classify method.
 *
 * Features are specified as a vector of doubles and the returned label is a string.
 * The features are normalized based on the means and standard deviations of the training samples used during the
 * training phase.
 */
class SVMClassifier
{
public:
    /**
     * Default constructor
     */
    SVMClassifier();

    /**
     * Destructor
     */
    virtual ~SVMClassifier();

    /**
     * Loads SVM model and config files from the given paths.
     * The SVM model is the serialized libsvm model.
     * The config file contains the number of features labels, means and standard deviations of the features.
     * Both files are stored in locations specified by the users of this and the SVMTrainer class.
     *
     * @param model_filepath
     * Full path of SVM model file.
     *
     * @param config_filename
     * Full path of configuration file containing labels, means and standard deviations.
     *
     *
     * @return true if files are loaded successfully, false otherwise.
     */
    bool loadModel(const bfs::path &model_filepath, const bfs::path &config_filepath);

    /**
     * Classifies and returns label corresponding to the class.
     *
     * @param features
     * Set of features that are to be classified.
     *
     * @param label
     * The label for the class returned by the classifier. It is modified in place and either
     * set to the correct label (if return is 0) or an empty string (if return is 1 or 2)
     *
     * @return 0 for success.
     *         1 if number of features specified is wrong.
     *         2 if classifier returns a class that doesn't correspond to a label.
     */
    int classify(std::vector<double> features, std::string &label) const;

    /**
     * Classifies and returns label and probability corresponding to the class.
     *
     * @param features
     * Set of features that are to be classified.
     *
     * @param label
     * The label for the class returned by the classifier. It is modified in place and either
     * set to the correct label (if return is 0) or an empty string (if return is 1 or 2)
     *
     * @param probability
     * The probability that the features correspond to the class returned. It is modified in place
     * and set to the probability if a label is returned, 1.0 if the SVM model does not have enough
     * information to return probability information and 0.0 if return of the function is 1 or 2
     *
     * @return 0 for success.
     *         1 if number of features specified is wrong.
     *         2 if classifier returns a class that doesn't correspond to a label.
     */
    int classifyWithProbability(std::vector<double> features, std::string &label, double &probability) const;


private:
    /**
     * Copy constructor.
     */
    SVMClassifier(const SVMClassifier &other);

    /**
     * Copy assignment operator.
     */
    SVMClassifier &operator=(SVMClassifier other);

    /**
     * Normalizes the set of features with the calculated or loaded.
     * means and standard deviations.
     * The set of features is modified in place and set to the normalized features
     *
     * @param features
     * Features that should be normalized.
     */
    void normalizeFeatures(std::vector<double> &features) const;


private:
    /**
     * Number of features that define each sample.
     */
    int num_features_;

    /**
     * SVM model object.
     */
    svm_model *model_;

    /**
     * Unique and sorted set of labels for the classes.
     */
    std::vector<std::string> labels_;

    /**
     * Set of means for the features.
     */
    std::vector<double> means_;

    /**
     * Set of standard deviations for the features.
     */
    std::vector<double> std_deviations_;

};
#endif
