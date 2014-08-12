/*
 * random_trees.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: Frederik Hegger
 */

#include "mcr_algorithms/machine_learning/random_trees.h"

RandomTrees::RandomTrees(const unsigned int no_of_features)
{
	this->is_modelfile_loaded_ = false;
	this->number_of_features_ = no_of_features;

	this->tree_parameters_ = CvRTParams(10, 10, 0, false, 10, 0, true, ((int)sqrt(no_of_features)), 100, 0.01f, CV_TERMCRIT_ITER);


	this->tree_parameters_.max_depth = INT_MAX;								// max levels in a tree
	this->tree_parameters_.min_sample_count = 10;							// dont split a node if lesser than this number
	this->tree_parameters_.regression_accuracy = 0;
	this->tree_parameters_.use_surrogates = false;
	this->tree_parameters_.max_categories = 10;
	this->tree_parameters_.priors = 0;
	this->tree_parameters_.calc_var_importance = true;						//true for better evaluation
	this->tree_parameters_.nactive_vars = ((int)sqrt(no_of_features));		//sqrt(number of features)
	//this->tree_parameters_.max_num_of_trees_in_the_forest;
	//this->tree_parameters_.forest_accuracy;
	//this->tree_parameters_.term_crit = CV_TERMCRIT_ITER;


	/*
    int _max_depth,
    int _min_sample_count,
    float _regression_accuracy,
    bool _use_surrogates,
    int _max_categories,
    const float* _priors,							NO PRIORS
    bool _calc_var_importance,
    int _nactive_vars,
    int max_num_of_trees_in_the_forest,
    float forest_accuracy,
    int termcrit_type
    */

}

int RandomTrees::train(const char* samples_filename, const char* model_filename, const double ratio, double &train_error, double &test_error)
{
	CvMat* data = 0;
	CvMat* responses = 0;
	CvMat* var_type = 0;
	CvMat* sample_idx = 0;

	this->tree_parameters_.nactive_vars = (int)sqrt(this->number_of_features_);

	int ok = read_num_class_data( samples_filename, this->number_of_features_, &data, &responses );
	int nsamples_all = 0, ntrain_samples = 0;
	int i = 0;
	double train_hr = 0, test_hr = 0;
	CvRTrees forest;
	CvMat* var_importance = 0;

	if( !ok )
	{
		cout << "Could not read the sample in" << samples_filename << endl;;
		return -1;
	}

	cout << "The sample file " << samples_filename << " is loaded." << endl;
	nsamples_all = data->rows;
	ntrain_samples = (int)(nsamples_all * ratio);


	// create classifier by using <data> and <responses>
	cout << "Training the classifier ..." << endl;

	// 1. create type mask
	var_type = cvCreateMat( data->cols + 1, 1, CV_8U );
	cvSet( var_type, cvScalarAll(CV_VAR_ORDERED) );
	cvSetReal1D( var_type, data->cols, CV_VAR_CATEGORICAL );

	// 2. create sample_idx
	sample_idx = cvCreateMat( 1, nsamples_all, CV_8UC1 );
	{
		CvMat mat;
		cvGetCols( sample_idx, &mat, 0, ntrain_samples );
		cvSet( &mat, cvRealScalar(1) );

		cvGetCols( sample_idx, &mat, ntrain_samples, nsamples_all );
		cvSetZero( &mat );
	}

	// 3. train classifier
	forest.train( data, CV_ROW_SAMPLE, responses, 0, sample_idx, var_type, 0, this->tree_parameters_);
	cout << endl;


	// compute prediction error on train and test data
	for( i = 0; i < nsamples_all; i++ )
	{
		double r;
		CvMat sample;
		cvGetRow( data, &sample, i );

		r = forest.predict( &sample );
		r = fabs((double)r - responses->data.fl[i]) <= FLT_EPSILON ? 1 : 0;

		if( i < ntrain_samples )
			train_hr += r;
		else
			test_hr += r;
	}

	test_hr /= (double)(nsamples_all-ntrain_samples);
	train_hr /= (double)ntrain_samples;

	train_error = 1 - train_hr;
	test_error = 1 - test_hr;

	cout << "Recognition rate: train = " << train_hr*100 << ", test = " << test_hr*100 << endl;
	cout << "Number of trees: " << forest.get_tree_count() << endl;

	// Print variable importance
	var_importance = (CvMat*)forest.get_var_importance();
	if( var_importance )
	{
		double rt_imp_sum = cvSum( var_importance ).val[0];
		printf("var#\timportance (in %%):\n");
		for( i = 0; i < var_importance->cols; i++ )
			printf( "%-2d\t%-4.1f\n", i,100.f*var_importance->data.fl[i]/rt_imp_sum);
	}

	// Save Random Trees classifier to file if needed
	if( model_filename )
		forest.save( model_filename );

	//cvReleaseMat( &var_importance );		//causes a segmentation fault
	cvReleaseMat( &sample_idx );
	cvReleaseMat( &var_type );
	cvReleaseMat( &data );
	cvReleaseMat( &responses );

	return 0;
}

int RandomTrees::loadModel(const char* model_filename)
{
	// load classifier from the specified file
	this->forest_.load( model_filename );
	if( this->forest_.get_tree_count() == 0 )
	{
		this->is_modelfile_loaded_ = false;
		return -1;
	}
	this->is_modelfile_loaded_ = true;

	return 0;
}

int RandomTrees::test(const char* sample_filename, const char* model_filename, double &test_error)
{
	CvMat* data = 0;
	CvMat* responses = 0;

	int ok = read_num_class_data( sample_filename, this->number_of_features_, &data, &responses );
	int nsamples_all = 0;
	int i = 0;

	if( !ok )
	{
		printf( "Could not read the sample file %s\n", sample_filename );
		return -1;
	}

	printf( "The sample file %s is loaded.\n", sample_filename );
	nsamples_all = data->rows;

	// compute prediction error on train and test data
	for( i = 0; i < nsamples_all; i++ )
	{
		CvMat sample;
		cvGetRow( data, &sample, i );

		this->loadModel(model_filename);
		this->classify(&sample);
	}

	cvReleaseMat( &data );
	cvReleaseMat( &responses );

	return 0;
}

LabelMap RandomTrees::classify(const vector<double> feature_vector)
{
	//copy feature vector to opencv matrix
	CvMat* data = cvCreateMat( 1, this->number_of_features_, CV_32F );
	for(unsigned int i=0; i < this->number_of_features_; ++i)
		cvmSet(data, 0, i, feature_vector[i]);

	LabelMap classification_result = this->classify(data);

	cvReleaseMat( &data );

	return classification_result;
}

LabelMap RandomTrees::classify(CvMat* data)
{
	if( !is_modelfile_loaded_ )
	{
		printf("no model file is loaded");
		exit(0);
	}

	LabelMap classification_result;
	LabelMap::iterator iter;
	CvDTreeNode *tree_node;
	for(int i=0; i < this->forest_.get_tree_count(); ++i)
	{
		tree_node = this->forest_.get_tree(i)->predict(data);

		iter = classification_result.find(((char)tree_node->value));
		if (iter == classification_result.end() )
			classification_result[((char)tree_node->value)] = 1.0;
		else
			iter->second++;
	}

	LabelMap::const_iterator map_end = classification_result.end();
	for (LabelMap::iterator it = classification_result.begin(); it != map_end; ++it)
		it->second /= this->forest_.get_tree_count();

	return classification_result;
}

int RandomTrees::read_num_class_data(const char* filename, int var_count, CvMat** data, CvMat** responses)
{
    const int M = 1024;
    FILE* f = fopen( filename, "rt" );
    CvMemStorage* storage;
    CvSeq* seq;
    char buf[M+2];
    float* el_ptr;
    CvSeqReader reader;
    int i, j;

    if( !f )
        return 0;

    el_ptr = new float[var_count+1];
    storage = cvCreateMemStorage();
    seq = cvCreateSeq( 0, sizeof(*seq), (var_count+1)*sizeof(float), storage );

    for(;;)
    {
        char* ptr;
        if( !fgets( buf, M, f ) || !strchr( buf, ',' ) )
            break;
        el_ptr[0] = buf[0];
        ptr = buf+2;
        for( i = 1; i <= var_count; i++ )
        {
            int n = 0;
            sscanf( ptr, "%f%n", el_ptr + i, &n );
            ptr += n + 1;
        }
        if( i <= var_count )
            break;
        cvSeqPush( seq, el_ptr );
    }
    fclose(f);

    *data = cvCreateMat( seq->total, var_count, CV_32F );
    *responses = cvCreateMat( seq->total, 1, CV_32F );

    cvStartReadSeq( seq, &reader );

    for( i = 0; i < seq->total; i++ )
    {
        const float* sdata = (float*)reader.ptr + 1;
        float* ddata = data[0]->data.fl + var_count*i;
        float* dr = responses[0]->data.fl + i;

        for( j = 0; j < var_count; j++ )
            ddata[j] = sdata[j];
        *dr = sdata[-1];
        CV_NEXT_SEQ_ELEM( seq->elem_size, reader );
    }

    cvReleaseMemStorage( &storage );
    delete el_ptr;
    return 1;
}
