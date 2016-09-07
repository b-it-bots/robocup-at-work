/*
 * adaboost.cpp
 *
 *  Created on: Jul 22, 2011
 *      Author: Frederik Hegger
 */

#include "mcr_algorithms/machine_learning/adaboost.h"

AdaBoost::AdaBoost(const unsigned int no_of_classes, const unsigned int no_of_features)
{
    this->is_modelfile_loaded_ = false;
    this->number_of_features_ = no_of_features;
    this->number_of_classes_ = no_of_classes;

    this->boost_parameters_ = CvBoostParams(CvBoost::REAL, 100, 0.95, 5, false, 0);
}

int AdaBoost::train(const char* samples_filename, const char* model_filename, const double ratio, double &train_error, double &test_error)
{
    CvMat* data = 0;
    CvMat* responses = 0;
    CvMat* var_type = 0;
    CvMat* temp_sample = 0;
    CvMat* weak_responses = 0;

    int ok = read_num_class_data(samples_filename, this->number_of_features_, &data, &responses);
    int nsamples_all = 0, ntrain_samples = 0;
    int var_count = 0;
    int i = 0, j = 0, k = 0;
    double train_hr = 0, test_hr = 0;
    CvBoost boost;


    if (!ok)
    {
        cout << "Could not read the sample in" << samples_filename << endl;;
        return -1;
    }

    cout << "The sample file " << samples_filename << " is loaded." << endl;
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all * ratio);
    var_count = data->cols;

    // create classifier by using <data> and <responses>
    cout << "Training the classifier ..." << endl;

    // create classifiers
    CvMat* new_data = cvCreateMat(ntrain_samples * this->number_of_classes_, var_count + 1 , CV_32F); //+1
    CvMat* new_responses = cvCreateMat(ntrain_samples * this->number_of_classes_, 1, CV_32S);

    // unroll the database type mask
    printf("Unrolling the samples ...\n");

    for (i = 0; i < ntrain_samples; i++)
    {
        float* data_row = (float*)(data->data.ptr + data->step * i);

        for (j = 0; j < this->number_of_classes_; j++)
        {
            float* new_data_row = (float*)(new_data->data.ptr + new_data->step * (i * this->number_of_classes_ + j));

            for (k = 0; k < var_count; k++)
                new_data_row[k] = data_row[k];

            new_data_row[var_count] = (float)j;
            new_responses->data.i[i * this->number_of_classes_ + j] = responses->data.fl[i] == j + FIRST_LABEL;
        }
    }

    // create type mask
    var_type = cvCreateMat(var_count + 2, 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));

    // the last indicator variable, as well
    // as the new (binary) response are categorical
    cvSetReal1D(var_type, var_count, CV_VAR_CATEGORICAL);  //CV_VAR_CATEGORICAL CV_VAR_NUMERICAL
    cvSetReal1D(var_type, var_count + 1, CV_VAR_CATEGORICAL); //CV_VAR_CATEGORICAL

    // train classifier
    //printf( "training the classifier (may take a few minutes)...");
    boost.train(new_data, CV_ROW_SAMPLE, new_responses, 0, 0, var_type, 0, this->boost_parameters_);

    cvReleaseMat(&new_data);
    cvReleaseMat(&new_responses);
    //printf("\n");

    temp_sample = cvCreateMat(1, var_count + 1, CV_32F);
    weak_responses = cvCreateMat(1, boost.get_weak_predictors()->total, CV_32F);

    // compute prediction error on train and test data
    for (i = 0; i < nsamples_all; i++)
    {
        int best_class = 0;
        double max_sum = -DBL_MAX;
        double r;
        CvMat sample;
        cvGetRow(data, &sample, i);

        for (k = 0; k < var_count; k++)
            temp_sample->data.fl[k] = sample.data.fl[k];

        for (j = 0; j < this->number_of_classes_; j++)
        {
            temp_sample->data.fl[var_count] = (float)j;

            boost.predict(temp_sample, 0, weak_responses);
            double sum = cvSum(weak_responses).val[0];

            if (max_sum < sum)
            {
                max_sum = sum;
                best_class = j + FIRST_LABEL;
            }
        }

        r = fabs(best_class - responses->data.fl[i]) < FLT_EPSILON ? 1 : 0;

        if (i < ntrain_samples)
            train_hr += r;
        else
            test_hr += r;
    }

    train_hr /= (double)ntrain_samples;
    test_hr /= ((double)nsamples_all - (double)ntrain_samples);

    cout << "Recognition rate: train = " << train_hr * 100 << ", test = " << test_hr * 100 << endl;

    // fill result-parameters
    train_error = 1 - train_hr;
    test_error = 1 - test_hr;

    // Save classifier to file if needed
    if (model_filename)
        boost.save(model_filename);

    boost.clear();
    cvReleaseMat(&temp_sample);
    cvReleaseMat(&weak_responses);
    cvReleaseMat(&var_type);
    cvReleaseMat(&data);
    cvReleaseMat(&responses);

    return 0;
}

int AdaBoost::loadModel(const char* model_filename)
{
    // load classifier from the specified file
    this->classifier_.load(model_filename);
    if (this->classifier_.get_weak_predictors())
    {
        printf("Could not read the model from file %s\n", model_filename);
        this->is_modelfile_loaded_ = false;
        return -1;
    }
    printf("The model %s is loaded.\n", model_filename);
    this->is_modelfile_loaded_ = true;

    return 0;
}

int AdaBoost::test(const char* sample_filename, const char* model_filename, double &test_error)
{
    CvMat* data = 0;
    CvMat* responses = 0;
    CvMat* var_type = 0;
    CvMat* temp_sample = 0;
    CvMat* weak_responses = 0;

    int ok = 0;
    int nsamples_all = 0;
    int var_count;
    int i, j, k;
    double test_hr = 0;
    CvBoost boost;

    ok = read_num_class_data(sample_filename, this->number_of_features_, &data, &responses);

    if (!ok)
    {
        printf("Could not read the test-file %s\n", sample_filename);
        return -1;
    }

    printf("The test-file %s is loaded.\n", sample_filename);

    nsamples_all = data->rows;
    var_count = data->cols;

    cout << "no. of test samples: " << nsamples_all << std::endl;
    cout << "no. of features: " <<  var_count << std::endl;
    cout << "no. of classifiers: " <<  this->number_of_classes_ << std::endl;

    // load classifier from the specified file
    boost.load(model_filename);

    if (!boost.get_weak_predictors())
    {
        printf("Could not read the classifier %s\n", model_filename);
        return -1;
    }

    //printf( "The classifier %s is loaded.\n", filename_to_load );

    temp_sample = cvCreateMat(1, var_count + 1, CV_32F);
    weak_responses = cvCreateMat(1, boost.get_weak_predictors()->total, CV_32F);

    // compute prediction error on test data
    for (i = 0; i < nsamples_all; i++)
    {
        int best_class = 0;
        double max_sum = -DBL_MAX;
        double r;
        CvMat sample;
        cvGetRow(data, &sample, i);

        for (k = 0; k < var_count; k++)
            temp_sample->data.fl[k] = sample.data.fl[k];

        for (j = 0; j < this->number_of_classes_; j++)
        {
            temp_sample->data.fl[var_count] = (float)j;

            boost.predict(temp_sample, 0, weak_responses);
            double sum = cvSum(weak_responses).val[0];

            if (max_sum < sum)
            {
                max_sum = sum;
                best_class = j + FIRST_LABEL;
            }
        }

        r = fabs(best_class - responses->data.fl[i]) < FLT_EPSILON ? 1 : 0;

        test_hr += r;
    }

    test_hr /= (double) nsamples_all;

    test_error = 1 - test_hr;

    boost.clear();
    cvReleaseMat(&temp_sample);
    cvReleaseMat(&weak_responses);
    cvReleaseMat(&var_type);
    cvReleaseMat(&data);
    cvReleaseMat(&responses);

    return 0;
}

LabelMap AdaBoost::classify(const vector<double> feature_vector)
{
    //copy feature vector to opencv matrix
    CvMat* data = cvCreateMat(1, this->number_of_features_, CV_32F);
    for (unsigned int i = 0; i < this->number_of_features_; ++i)
        cvmSet(data, 0, i, feature_vector[i]);

    LabelMap classification_result = this->classify(data);

    cvReleaseMat(&data);

    return classification_result;
}

LabelMap AdaBoost::classify(CvMat* data)
{
    if (!is_modelfile_loaded_)
    {
        printf("no model file is loaded");
        exit(0);
    }

    LabelMap classification_result;
    LabelMap::iterator iter;

    CvMat* responses = 0;
    CvMat* var_type = 0;
    CvMat* temp_sample = 0;
    CvMat* weak_responses = 0;

    int var_count = 0;
    int j = 0, k = 0;

    var_count = data->cols;

    temp_sample = cvCreateMat(1, var_count + 1, CV_32F);
    weak_responses = cvCreateMat(1, this->classifier_.get_weak_predictors()->total, CV_32F);

    double max_sum = -DBL_MAX;
    CvMat sample;
    cvGetRow(data, &sample, 0);

    for (k = 0; k < var_count; k++)
        temp_sample->data.fl[k] = (float)sample.data.db[k];

    for (j = 0; j < this->number_of_classes_; j++)
    {
        temp_sample->data.fl[var_count] = (float)j;

        this->classifier_.predict(temp_sample, 0, weak_responses);

        double sum = cvSum(weak_responses).val[0];

        classification_result[((char)(j + FIRST_LABEL))] = sum;

        if (max_sum < sum)
            max_sum = sum;
    }

    cvReleaseMat(&temp_sample);
    cvReleaseMat(&weak_responses);
    cvReleaseMat(&var_type);
    cvReleaseMat(&data);
    cvReleaseMat(&responses);

    return classification_result;
}

int AdaBoost::read_num_class_data(const char* filename, int var_count, CvMat** data, CvMat** responses)
{
    const int M = 1024;
    FILE* f = fopen(filename, "rt");
    CvMemStorage* storage;
    CvSeq* seq;
    char buf[M + 2];
    float* el_ptr;
    CvSeqReader reader;
    int i = 0, j = 0;

    if (!f)
        return 0;

    el_ptr = new float[var_count + 1];
    storage = cvCreateMemStorage();
    seq = cvCreateSeq(0, sizeof(*seq), (var_count + 1) * sizeof(float), storage);

    for (;;)
    {
        char* ptr;

        if (!fgets(buf, M, f) || !strchr(buf, ','))
            break;

        el_ptr[0] = buf[0];
        ptr = buf + 2;

        for (i = 1; i <= var_count; i++)
        {
            int n = 0;
            sscanf(ptr, "%f%n", el_ptr + i, &n);
            ptr += n + 1;
        }

        if (i <= var_count)
            break;

        cvSeqPush(seq, el_ptr);
    }
    fclose(f);

    *data = cvCreateMat(seq->total, var_count, CV_32F);
    *responses = cvCreateMat(seq->total, 1, CV_32F);

    cvStartReadSeq(seq, &reader);

    for (i = 0; i < seq->total; i++)
    {
        const float* sdata = (float*) reader.ptr + 1;
        float* ddata = data[0]->data.fl + var_count * i;
        float* dr = responses[0]->data.fl + i;

        for (j = 0; j < var_count; j++)
            ddata[j] = sdata[j];

        *dr = sdata[-1];
        CV_NEXT_SEQ_ELEM(seq->elem_size, reader);
    }

    cvReleaseMemStorage(&storage);
    delete el_ptr;
    return 1;
}
