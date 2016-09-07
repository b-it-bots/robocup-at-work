#ifndef SVM_TRAINER_H
#define SVM_TRAINER_H

#include <libsvm/svm.h>
#include <string>
#include <vector>
#include <utility>
#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

/**
 * This class is a wrapper for multi-class classification using libsvm.
 * The class is used to train the SVM classifier using a number of training samples.
 *
 * The class needs to be initialized with the number of features available for each training sample.
 * Training samples (a label and a set of corresponding features) can be added one by one.
 * Finally, the classifier can be trained and the model and configuration can be saved to file.
 *
 * Features are specified as a vector of doubles and the label is specified as a string.
 * A feature can be any numeric representation of a distinguishing feature of the items being classified
 * (for example, the length of an object)
 * Before the training phase the training samples are normalized to have a mean of 0 and a standard deviation of 1.
 *
 * The saved configuration file contains the number of features, sorted labels, the means and standard deviations
 * which can later be loaded by the SVMClassifier for classification.
 */

class SVMTrainer
{
private:
    /**
     * A training sample is defined as a pair of label (string) and a set of features (vector).
     * Only used internally by the class.
     */
    typedef std::pair<std::string, std::vector<double> > training_sample;


public:
    /**
     * Constructor
     * Initializes trainer with number of features.
     *
     * @param num_features
     * The number of features that will describe each training sample.
     */
    SVMTrainer(int num_features);

    /**
     * Destructor
     */
    virtual ~SVMTrainer();

    /**
     * Add training sample to set for training.
     *
     * @param features
     * Features of the training sample as a set of doubles.
     * All samples must have the same number of features.
     *
     * @param label
     * The label of the training sample.
     *
     * @return true if added successfully, false otherwise.
     */
    bool addTrainingSample(const std::vector<double> &features, const std::string &label);

    /**
     * Train the classifier with the training samples added so far.
     *
     * @return true if training was successful, false otherwise.
     */
    bool trainAndSaveModel(const bfs::path &model_filepath, const bfs::path &config_filepath);


private:
    /**
     * Copy constructor.
     */
    SVMTrainer(const SVMTrainer &other);

    /**
     * Copy assignment operator.
     */
    SVMTrainer &operator=(SVMTrainer other);

    /**
     * Normalizes the training samples after calculating the means and
     * standard deviations of the features.
     *
     * @param samples
     * Set of training samples to be normalized.
     * The samples are modified in place.
     */
    void normalizeTrainingSamples(std::vector<SVMTrainer::training_sample> &samples);

    /**
     * Save the SVM model to the given file
     *
     * @param model_filename
     * File to save the SVM model to
     *
     * @return true if files are saved,
     *         false otherwise.
     */
    bool saveModel(const bfs::path &model_filepath, const bfs::path &config_filepath) const;


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
     * Set of training samples defined as a string, vector<double> pair.
     */
    std::vector<SVMTrainer::training_sample> training_samples_;

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
