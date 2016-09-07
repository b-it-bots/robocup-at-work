/*
 * CSVMLinearClassifier.cpp
 *
 *  Created on: Oct 26, 2011
 *      Author: mca
 */

#include <fstream>
#include <cassert>

#include "svm/online_lib_linear/svm_linear_classifier.h"

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define INF HUGE_VAL

CSvmLinearClassifier::CSvmLinearClassifier()
{
    model_ = NULL;
    line = NULL;
    nr_fold = -1;
    bias = -1;
}

void CSvmLinearClassifier::setCrossValidationNFoldCMD(int nFold)
{
    std::cout << "CSvmLinearClassifier::setCrossValidationNFold(char *featureVectorsPath, int nFold=10) ... NOT IMPLEMENTED!!!!!!";
    this->flag_cross_validation = 1;
    this->nr_fold = nFold;
}

void CSvmLinearClassifier::crossValidation(std::map<int, std::vector<std::vector<double> > > trainingFeatureVectors, int nFold)
{
    std::cout << "CSvmLinearClassifier::setCrossValidationNFold..." << std::endl;
    this->flag_cross_validation = 1;
    this->nr_fold = nFold;
    this->trainModel(trainingFeatureVectors, NULL);
}

void CSvmLinearClassifier::clean()
{
    if (this->model_ != NULL)
    {
        std::cout << "CSvmLinearClassifier::clean ... " << std::endl;
        free_and_destroy_model(&model_);

        destroy_param(&param);
        free(prob.y);
        free(prob.x);
        free(x_space);
        free(line);
        trainingFeatureVectors.clear();
    }
}

void CSvmLinearClassifier::print_null(const char *s)
{
    std::cout << "quit" << std::endl;
}

void CSvmLinearClassifier::exit_with_help()
{
    printf("Usage: train [options] training_set_file [model_file]\n"
           "options:\n"
           "-s type : set type of solver (default 1)\n"
           "   0 -- L2-regularized logistic regression (primal)\n"
           "   1 -- L2-regularized L2-loss support vector classification (dual)\n"
           "   2 -- L2-regularized L2-loss support vector classification (primal)\n"
           "   3 -- L2-regularized L1-loss support vector classification (dual)\n"
           "   4 -- multi-class support vector classification by Crammer and Singer\n"
           "   5 -- L1-regularized L2-loss support vector classification\n"
           "   6 -- L1-regularized logistic regression\n"
           "   7 -- L2-regularized logistic regression (dual)\n"
           "-c cost : set the parameter C (default 1)\n"
           "-e epsilon : set tolerance of termination criterion\n"
           "   -s 0 and 2\n"
           "       |f'(w)|_2 <= eps*min(pos,neg)/l*|f'(w0)|_2,\n"
           "       where f is the primal function and pos/neg are # of\n"
           "       positive/negative data (default 0.01)\n"
           "   -s 1, 3, 4 and 7\n"
           "       Dual maximal violation <= eps; similar to libsvm (default 0.1)\n"
           "   -s 5 and 6\n"
           "       |f'(w)|_inf <= eps*min(pos,neg)/l*|f'(w0)|_inf,\n"
           "       where f is the primal function (default 0.01)\n"
           "-B bias : if bias >= 0, instance x becomes [x; bias]; if < 0, no bias term added (default -1)\n"
           "-wi weight: weights adjust the parameter C of different classes (see README for details)\n"
           "-v n: n-fold cross validation mode\n"
           "-q : quiet mode (no outputs)\n");
    exit(1);
}

void CSvmLinearClassifier::exit_input_error(int line_num)
{
    fprintf(stderr, "Wrong input format at line %d\n", line_num);
    exit(1);
}

char* CSvmLinearClassifier::readline(FILE *input)
{
    int len;

    if (fgets(line, max_line_len, input) == NULL)
        return NULL;

    while (strrchr(line, '\n') == NULL)
    {
        max_line_len *= 2;
        line = (char *) realloc(line, max_line_len);
        len = (int) strlen(line);
        if (fgets(line + len, max_line_len - len, input) == NULL)
            break;
    }
    return line;
}

void CSvmLinearClassifier::setParameter()
{

    //L2R_LR, L2R_L2LOSS_SVC_DUAL, L2R_L2LOSS_SVC, L2R_L1LOSS_SVC_DUAL, MCSVM_CS, L1R_L2LOSS_SVC, L1R_LR, L2R_LR_DUAL }; /* solver_type */


    param.solver_type = L2R_L2LOSS_SVC_DUAL; // L2R_L1LOSS_SVC_DUAL;//L2R_L2LOSS_SVC_DUAL;

    std::cout << "CSvmLinearClassifier::setParameter..." << param.solver_type << std::endl;
    param.C = 1; //100 //1
    param.eps = INF; // see setting below
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;
    flag_cross_validation = 0;
    bias = -1;

    //param.solver_type = 2;


    if (nr_fold != -1)
    {
        std::cout << "CSvmLinearClassifier::setParameter... " << nr_fold << std::endl;
        this->flag_cross_validation = 1;
        //nr_fold = cv;
        if (nr_fold < 2)
        {
            fprintf(stderr, "n-fold cross validation: n must >= 2\n");
            exit_with_help();
        }
    }

    if (param.eps == INF)
    {
        if (param.solver_type == L2R_LR || param.solver_type == L2R_L2LOSS_SVC)
            param.eps = 0.01;
        else if (param.solver_type == L2R_L2LOSS_SVC_DUAL || param.solver_type == L2R_L1LOSS_SVC_DUAL || param.solver_type == MCSVM_CS || param.solver_type == L2R_LR_DUAL)
            param.eps = 0.1;
        else if (param.solver_type == L1R_L2LOSS_SVC || param.solver_type == L1R_LR)
            param.eps = 0.01;
    }
}

int CSvmLinearClassifier::trainModel(std::map<int, std::vector<std::vector<double> > > trainingFeatureVectors, char* modelName)
{
    std::cout << "CSvmLinearClassifier::trainModel...(probability are forced)" << std::endl;
    //int check_probability_model(const struct model *model_) in linear.cpp

    this->trainingFeatureVectors = trainingFeatureVectors;

    std::string model_file_name;
    if (modelName == NULL)
    {
        model_file_name = std::string("svmModel.model");
    }
    else
    {
        model_file_name = std::string(modelName);
    }
    const char *error_msg;
    clock_t start, end;
//  std::cout<<"flag_cross_validation1 "<<flag_cross_validation<< " " << nr_fold<<std::endl;
    this->setParameter();
//  std::cout<<"flag_cross_validation2 "<<flag_cross_validation<< " " << nr_fold<<std::endl;
    start = clock();
    this->read_problem(trainingFeatureVectors);
    end = clock();
    printf("\nLoad data %lfs", (double)(end - start) / (double) CLOCKS_PER_SEC);
    error_msg = check_parameter(&prob, &param);

    if (error_msg)
    {
        fprintf(stderr, "Error: %s\n", error_msg);
        exit(1);
    }

    if (flag_cross_validation)
    {
        std::cout << "CSvmLinearClassifier::trainModel...cross_validation..." << std::endl;
        do_cross_validation();
    }
    else
    {
        std::cout << "CSvmLinearClassifier::trainModel...train..." << std::endl;
        this->model_ = train(&prob, &param);
        if (save_model(model_file_name.c_str(), model_))
        {
            fprintf(stderr, "CSvmLinearClassifier::trainModel...can't save model to file %s\n", model_file_name.c_str());
            exit(1);
        }
        //free_and_destroy_model(&model_);
    }
    /*destroy_param(&param);
     free(prob.y);
     free(prob.x);
     free(x_space);
     free(line);*/

    return 0;
}
int CSvmLinearClassifier::trainModel(int argc, char **argv)
{
    std::cout << "CSvmLinearClassifier::trainModel..." << std::endl;
    char input_file_name[1024];
    char model_file_name[1024];
    const char *error_msg;
    clock_t start, end;

    parse_command_line(argc, argv, input_file_name, model_file_name);
    start = clock();
    read_problem(input_file_name);
    end = clock();
    printf("Load data %lfs", (double)(end - start) / (double) CLOCKS_PER_SEC);
    error_msg = check_parameter(&prob, &param);

    if (error_msg)
    {
        fprintf(stderr, "Error: %s\n", error_msg);
        exit(1);
    }

    if (flag_cross_validation)
    {
        std::cout << "CSvmLinearClassifier::trainModel...cross_validation..." << std::endl;
        do_cross_validation();
    }
    else
    {
        std::cout << "CSvmLinearClassifier::trainModel...train..." << std::endl;
        model_ = train(&prob, &param);
        if (save_model(model_file_name, model_))
        {
            fprintf(stderr, "CSvmLinearClassifier::trainModel...can't save model to file %s\n", model_file_name);
            exit(1);
        }
        //free_and_destroy_model(&model_);
    }
    /*destroy_param(&param);
     free(prob.y);
     free(prob.x);
     free(x_space);
     free(line);*/

    return 0;
}

void CSvmLinearClassifier::do_cross_validation()
{
    std::cout << "CSvmLinearClassifier::do_cross_validation()...Nfold:" << "nr_fold" << std::endl;
    int i;
    int total_correct = 0;
    int *target = Malloc(int, prob.l);

    cross_validation(&prob, &param, nr_fold, target);

    for (i = 0; i < prob.l; i++)
        if (target[i] == prob.y[i])
            ++total_correct;
    printf("Cross Validation Accuracy = %g%%\n", 100.0 * total_correct / prob.l);

    free(target);
}

void CSvmLinearClassifier::parse_command_line(int argc, char **argv, char *input_file_name, char *model_file_name)
{
    int i;
    void (*print_func)(const char*) = NULL; // default printing to stdout

    // default values
    param.solver_type = L2R_L2LOSS_SVC_DUAL;
    param.C = 1;
    param.eps = INF; // see setting below
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;
    flag_cross_validation = 0;
    bias = -1;

    if (nr_fold != -1)
    {
        flag_cross_validation = 1;
        //nr_fold = cv;
        if (nr_fold < 2)
        {
            fprintf(stderr, "n-fold cross validation: n must >= 2\n");
            exit_with_help();
        }
    }

    // parse options
    for (i = 1; i < argc; i++)
    {
        if (argv[i][0] != '-')
            break;
        if (++i >= argc)
            exit_with_help();
        switch (argv[i - 1][1])
        {
        case 's':
            param.solver_type = atoi(argv[i]);
            break;

        case 'c':
            param.C = atof(argv[i]);
            break;

        case 'e':
            param.eps = atof(argv[i]);
            break;

        case 'B':
            bias = atof(argv[i]);
            break;

        case 'w':
            ++param.nr_weight;
            param.weight_label = (int *) realloc(param.weight_label, sizeof(int) * param.nr_weight);
            param.weight = (double *) realloc(param.weight, sizeof(double) * param.nr_weight);
            param.weight_label[param.nr_weight - 1] = atoi(&argv[i - 1][2]);
            param.weight[param.nr_weight - 1] = atof(argv[i]);
            break;

        case 'v':
            flag_cross_validation = 1;
            nr_fold = atoi(argv[i]);
            if (nr_fold < 2)
            {
                fprintf(stderr, "n-fold cross validation: n must >= 2\n");
                exit_with_help();
            }
            break;

        case 'q':
            //print_func = &print_null;
            i--;
            break;

        default:
            fprintf(stderr, "unknown option: -%c\n", argv[i - 1][1]);
            exit_with_help();
            break;
        }
    }

    set_print_string_function(print_func);

    // determine filenames
    if (i >= argc)
        exit_with_help();

    strcpy(input_file_name, argv[i]);

    if (i < argc - 1)
        strcpy(model_file_name, argv[i + 1]);
    else
    {
        char *p = strrchr(argv[i], '/');
        if (p == NULL)
            p = argv[i];
        else
            ++p;
        sprintf(model_file_name, "%s.model", p);
    }

    if (param.eps == INF)
    {
        if (param.solver_type == L2R_LR || param.solver_type == L2R_L2LOSS_SVC)
            param.eps = 0.01;
        else if (param.solver_type == L2R_L2LOSS_SVC_DUAL || param.solver_type == L2R_L1LOSS_SVC_DUAL || param.solver_type == MCSVM_CS || param.solver_type == L2R_LR_DUAL)
            param.eps = 0.1;
        else if (param.solver_type == L1R_L2LOSS_SVC || param.solver_type == L1R_LR)
            param.eps = 0.01;
    }

}

// read in a problem (in libsvm format)
void CSvmLinearClassifier::read_problem(const char *filename)
{
    std::cout << "CSvmLinearClassifier::read_problem(const char *filename)...." << std::endl;
    int max_index, inst_max_index, i;
    long int elements, j;
    FILE *fp = fopen(filename, "r");
    char *endptr;
    char *idx, *val, *label;

    if (fp == NULL)
    {
        fprintf(stderr, "can't open input file %s\n", filename);
        exit(1);
    }

    prob.l = 0;
    elements = 0;
    max_line_len = 1024;
    line = Malloc(char, max_line_len);
    while (readline(fp) != NULL)
    {
        char *p = strtok(line, " \t"); // label

        // features
        while (1)
        {
            p = strtok(NULL, " \t");
            if (p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
                break;
            elements++;
        }
        elements++; // for bias term
        prob.l++;
    }
    rewind(fp);

    prob.bias = bias;

    prob.y = Malloc(int, prob.l);
    prob.x = Malloc(struct feature_node *, prob.l);
    x_space = Malloc(struct feature_node, elements + prob.l);
    x_space_len = elements + prob.l;

    max_index = 0;
    j = 0;
    for (i = 0; i < prob.l; i++)
    {
        inst_max_index = 0; // strtol gives 0 if wrong format
        readline(fp);
        prob.x[i] = &x_space[j];
        label = strtok(line, " \t");
        prob.y[i] = (int) strtol(label, &endptr, 10);
        if (endptr == label)
            exit_input_error(i + 1);

        while (1)
        {
            idx = strtok(NULL, ":");
            val = strtok(NULL, " \t");

            if (val == NULL)
                break;

            errno = 0;
            x_space[j].index = (int) strtol(idx, &endptr, 10);
            if (endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
                exit_input_error(i + 1);
            else
                inst_max_index = x_space[j].index;

            errno = 0;
            x_space[j].value = strtod(val, &endptr);
            if (endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
                exit_input_error(i + 1);

            ++j;
        }

        if (inst_max_index > max_index)
            max_index = inst_max_index;

        if (prob.bias >= 0)
            x_space[j++].value = prob.bias;

        x_space[j++].index = -1;
    }

    //std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAACC "<<max_index<<std::endl;

    if (prob.bias >= 0)
    {
        prob.n = max_index + 1;
        for (i = 1; i < prob.l; i++)
            (prob.x[i] - 2)->index = prob.n;
        x_space[j - 2].index = prob.n;
    }
    else
        prob.n = max_index;

    fclose(fp);

    //std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA "<<prob.n<<std::endl;
}

void CSvmLinearClassifier::read_problem(std::map<int, std::vector<std::vector<double> > > trainingFeatureVectors)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterFeatVec;

    int max_index, inst_max_index, i;
    long int elements, j;
    //char *endptr;
    //char *idx, *val, *label;

    if (trainingFeatureVectors.size() == 0)
    {
        fprintf(stderr, "CSvmLinearClassifier::read_problem... empty");
        exit(1);
    }

    prob.l = 0;
    elements = 0;

    for (iterFeatVec = trainingFeatureVectors.begin(); iterFeatVec != trainingFeatureVectors.end(); ++iterFeatVec)
    {
        std::vector<std::vector<double> > lFeatVec = iterFeatVec->second;
        for (unsigned int iterVec = 0; iterVec < lFeatVec.size(); ++iterVec)
        {
            elements += lFeatVec[iterVec].size();
            prob.l++;
        }
    }

    prob.bias = bias;
    prob.y = Malloc(int, prob.l);
    prob.x = Malloc(struct feature_node *, prob.l);
    x_space = Malloc(struct feature_node, elements + prob.l);
    x_space_len = elements + prob.l;

    max_index = 0;
    j = 0;
    for (i = 0, j = 0, iterFeatVec = trainingFeatureVectors.begin(); iterFeatVec != trainingFeatureVectors.end(); ++iterFeatVec)
    {
        //  assert( iterFeatVec->second.size()>0);
        //  std::cout<<iterFeatVec->second.size()<<" ---- "<< iterFeatVec->second[0].size()<<std::endl;
        //  std::vector<std::vector<double> > lFeatVec = iterFeatVec->second;
        for (unsigned int iterVec = 0; iterVec < iterFeatVec->second.size(); ++iterVec, ++i)
        {
            inst_max_index = 0;
            prob.y[i] = iterFeatVec->first; //label
            prob.x[i] = &x_space[j];

            //  std::cout<<"*************** : "<<prob.y[i];
            for (int k = 1; k <= (iterFeatVec->second[iterVec].size()); k++, j++)
            {
                errno = 0;
                x_space[j].index = k;
                inst_max_index = x_space[j].index;

                errno = 0;
                x_space[j].value = (double) iterFeatVec->second[iterVec][k - 1];

                //  std::cout<< " "<<x_space[j].index <<":"<<x_space[j].value;
            }
            //std::cout<<std::endl;
            x_space[j].index = -1;
            x_space[j].value = 0;

            if (inst_max_index > max_index)
                max_index = inst_max_index;

            if (prob.bias >= 0)
                x_space[j++].value = prob.bias;

            x_space[j++].index = -1;
        }
    }

    if (prob.bias >= 0)
    {
        prob.n = max_index + 1;
        for (i = 1; i < prob.l; i++)
            (prob.x[i] - 2)->index = prob.n;
        x_space[j - 2].index = prob.n;
    }
    else
        prob.n = max_index;

    //std::cout<<"BBBBBBBBBBBBBBBBB "<<prob.n<<std::endl;
}

void CSvmLinearClassifier::loadModel(std::string modelFile)
{
    std::cout << " CSvmLinearClassifier::loadModel...." << std::endl;
    free_and_destroy_model(&model_);
    this->model_ = NULL;
    this->model_ = load_model(modelFile.c_str());

    if (this->model_ == NULL)
    {
        std::cout << " CSvmLinearClassifier::loadModel....could not load: " << modelFile << std::endl;
    }
}

void CSvmLinearClassifier::loadOfflineData(std::string path)
{
    std::cout << " CSvmLinearClassifier::loadOfflineData " << path << std::endl;
    std::string filenameTr = std::string(path + ".tr");
    std::string filenameMo = std::string(path + ".model");
    this->loadOfflineData(filenameTr.c_str(), filenameMo.c_str());
    this->trainingFeatureVectors.clear(); //since old


    std::cout << " CSvmLinearClassifier::loadOfflineData...done " << path << " example size: " << prob.l << " model: num classes " << model_->nr_class << std::endl;
}

void CSvmLinearClassifier::loadOfflineData(const char *prob_path, const char *model_path_)
{
    model_path = (char*) model_path_;
    read_problem(prob_path);
    model_ = load_model(model_path);
    param = model_->param;
    param.C = 100;
    param.eps = 0.1;
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;

    const char *error_msg = check_parameter(&prob, &param);
    if (error_msg)
    {
        fprintf(stderr, "Error: %s\n", error_msg);
        exit(1);
    }
}

void CSvmLinearClassifier::saveTrainingData(std::string filename)
{
    assert(this->trainingFeatureVectors.size() > 0);

    std::fstream fout;
    fout.open(std::string(filename).c_str(), std::ios::out | std::ios::binary | std::ios::app);

    std::map<int, std::vector<std::vector<double> > >::iterator iterFeatVec;

    for (iterFeatVec = trainingFeatureVectors.begin(); iterFeatVec != trainingFeatureVectors.end(); ++iterFeatVec)
    {
        std::vector<std::vector<double> > lFeatVec = iterFeatVec->second;
        for (unsigned int iterVec = 0; iterVec < lFeatVec.size(); ++iterVec)
        {
            fout << iterFeatVec->first << " ";
            std::cout << iterFeatVec->first << " ";
            for (unsigned int j = 0; j < lFeatVec[iterVec].size(); ++j)
            {
                int no = j + 1;
                fout << no << ":" << lFeatVec[iterVec][j] << " ";
                std::cout << no << ":" << lFeatVec[iterVec][j] << " ";
            }
            fout << std::endl;
            std::cout << std::endl;

        }
    }
    fout.close();
    //} else {
    //  std::cout << "Could not save libsvm file!!!" << fileName << "\n";
    //}
}

int CSvmLinearClassifier::getNumClasses()
{
    //std::cout << "CSvmLinearClassifier::getNumClasses()..." << model_->nr_class << std::endl;
    return model_->nr_class;
}

int CSvmLinearClassifier::predict(std::vector<double> featureVector, double *dec_values)
{
    return testExample(featureVector, dec_values);
}

int CSvmLinearClassifier::testExample(std::vector<double> featureVector, double *dec_values)
{

    //std::cout << "CSvmLinearClassifier::testExample..." << std::endl;
    double *feature = Malloc(double, featureVector.size());

    if (this->model_ != NULL)
    {

        for (unsigned int i = 0; i < featureVector.size(); ++i)
        {
            feature[i] = featureVector[i];
        }
        return testExample(feature, featureVector.size(), dec_values);
    }
    else
    {
        std::cout << "CSvmLinearClassifier::testExample...no model loaded" << std::endl;
    }

    return -1;
}

int CSvmLinearClassifier::testExample(const double *feature, int feature_len, double *dec_values)
{
    // build feature_node
    std::vector<struct feature_node> v;
    for (int i = 0; i < feature_len; ++i)
    {
        if (feature[i] != 0.0)
        {
            struct feature_node node;
            node.index = i + 1;
            node.value = feature[i];
            v.push_back(node);
        }
    }
    struct feature_node node;
    node.index = -1;
    v.push_back(node);

    struct feature_node *x = new struct feature_node[v.size()];
    copy(v.begin(), v.end(), x);

    int label = predict_values(this->model_, x, dec_values);
//  std::cout << "Label: " << label << std::endl;

    //  for (int i = 0; i < getNumClasses(); ++i)
    //  {
    //      std::cout << dec_values[i] << " " << model_->label[i] << std::endl;
    //  }

    //double *values = Malloc(double,getNumClasses());
    //BUG ERROR ?!
    double *values = Malloc(double, getMaxLabel());
    for (int i = 0; i < getMaxLabel(); ++i)
    {
        values[i] = 0;
    }

    for (int i = 0; i < getNumClasses(); ++i)
        values[model_->label[i] - 1] = dec_values[i];
    for (int i = 0; i < getMaxLabel(); ++i)
        dec_values[i] = values[i];

//  for (int i = 0; i < getMaxLabel(); ++i)
//  {
//      std::cout << dec_values[i] << " -> Label" << 1+i << std::endl;
//  }

    free(values);
    delete[] x;
    return label;
}

int CSvmLinearClassifier::getMaxLabel()
{
    int maxLabel = -1;
    for (int i = 0; i < getNumClasses(); ++i)
    {
        if (model_->label[i] > maxLabel)
            maxLabel = model_->label[i];
    }
    return maxLabel;
}

void CSvmLinearClassifier::onlineAddExample(const double *feature, int feature_len, int label, bool updateModel)
{
    // Update problem
    int old_l = prob.l;
    long int old_x_space_len = x_space_len;

    ++prob.l;
    int max_index = prob.n;
    for (int i = 0; i < feature_len; ++i)
    {
        if (feature[i] != 0.0)
        {
            if (i + 1 > max_index)
                max_index = i + 1;
            ++x_space_len;
        }
    }
    x_space_len += 2;

    prob.y = (int *) realloc(prob.y, prob.l * sizeof(int));
    prob.x = (struct feature_node **) realloc(prob.x, prob.l * sizeof(struct feature_node *));
    x_space = (struct feature_node *) realloc(x_space, x_space_len * sizeof(struct feature_node));

    prob.y[prob.l - 1] = label;

    int endIndex = old_x_space_len;
    if (prob.bias < 0)
    {
        endIndex -= old_l;
    }
    long int cur = 0;
    prob.x[cur] = x_space;
    long int last = 0;
    for (long int i = 0; i < endIndex; ++i)
    {
        if (x_space[i].index == -1)
        {
            ++cur;
            prob.x[cur] = &(x_space[i + 1]);
            last = i + 1;
        }
    }
    cur = last;
    for (int i = 0; i < feature_len; ++i)
    {
        if (feature[i] != 0.0)
        {
            x_space[cur].index = i + 1;
            x_space[cur].value = feature[i];
            ++cur;
        }
    }
    if (prob.bias >= 0)
    {
        x_space[cur].value = prob.bias;
        ++cur;
    }
    x_space[cur].index = -1;
    ++cur;
    if (prob.bias >= 0)
    {
        prob.n = max_index + 1;
        for (int i = 1; i < prob.l; i++)
            (prob.x[i] - 2)->index = prob.n;
        x_space[cur - 2].index = prob.n;
    }
    else
        prob.n = max_index;

    if (updateModel)
    {
        // Evaluate classifiers on new data to decide which classifiers need to be updated.
        int nClass = getNumClasses();
        bool *update = Malloc(bool, nClass);
        for (int j = 0; j < nClass; ++j)
            update[j] = false;
        double *dec_values = Malloc(double, nClass);
        for (int i = old_l; i < prob.l; ++i)
        {
            predict_values(model_, prob.x[i], dec_values);
            for (int j = 0; j < nClass; ++j)
            {
                int y = -1;
                if (prob.y[i] == model_->label[j])
                    y = +1;
                if (y * dec_values[j] <= 1)
                    update[j] = true;
            }
        }
        free(dec_values);

        onlinetrain(&prob, &param, model_, update);
        free(update);
    }
}

struct example * CSvmLinearClassifier::readNewExamples(std::vector<std::vector<double> > trainingFeatureVectors, int label, int &numNewExamples)
{

    numNewExamples = trainingFeatureVectors.size();

    examples = Malloc(struct example, numNewExamples);
    feature_space = Malloc(double, prob.n*numNewExamples);

    for (int i = 0; i < prob.n * numNewExamples; ++i)
        feature_space[i] = 0.0;

    for (int i = 0; i < numNewExamples; i++)
    {
        examples[i].y = label;

        examples[i].x = &feature_space[i * prob.n];

        std::cout << std::endl << prob.n << " " << trainingFeatureVectors[i].size() << std::endl;

        std::cout << "Example " << i << " ";
        for (unsigned int iterFeatVec = 0; iterFeatVec < trainingFeatureVectors[i].size(); ++iterFeatVec)
        {
            std::cout << trainingFeatureVectors[i][iterFeatVec] << " ";
            examples[i].x[iterFeatVec] = trainingFeatureVectors[i][iterFeatVec];
        }
        std::cout << std::endl;
    }
    return examples;
}

struct example * CSvmLinearClassifier::readNewExamples(const char *examples_path, int &numNewExamples)
{
    FILE *fp = fopen(examples_path, "r");
    char *endptr;
    char *idx, *val, *label;

    if (fp == NULL)
    {
        fprintf(stderr, "can't open input file %s\n", examples_path);
        exit(1);
    }

    max_line_len = 1024;
    line = Malloc(char, max_line_len);

    numNewExamples = 0;
    while (readline(fp) != NULL)
    {
        char *p = strtok(line, " \t"); // label

        // features
        while (1)
        {
            p = strtok(NULL, " \t");
            if (p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
                break;
        }
        numNewExamples++;
    }
    rewind(fp);

    examples = Malloc(struct example, numNewExamples);
    feature_space = Malloc(double, prob.n*numNewExamples);
    for (int i = 0; i < prob.n * numNewExamples; ++i)
        feature_space[i] = 0.0;

    for (int i = 0; i < numNewExamples; i++)
    {
        readline(fp);
        label = strtok(line, " \t");
        examples[i].y = (int) strtol(label, &endptr, 10);

        examples[i].x = &feature_space[i * prob.n];
        while (1)
        {
            idx = strtok(NULL, ":");
            val = strtok(NULL, " \t");
            if (val == NULL)
                break;

            int index = (int) strtol(idx, &endptr, 10);
            examples[i].x[index - 1] = strtod(val, &endptr); //examples[i].x[index - 1] = strtod(val, &endptr);
        }
    }
    free(line);
    return examples;
}

void CSvmLinearClassifier::updateModel(std::string oldModel, std::vector<std::vector<double> > trainingFeatureVectors, int label, std::string updatedModel)
{
    //loadOfflineData(argv[1], argv[2]); // Load previously learned model
    loadOfflineData(oldModel);
    int nClass = this->getNumClasses();

    printf("Read old model: Number of classes = %d, Feature dimensionality=%d, Number of examples=%d\n", nClass, prob.n, prob.l);
    int numNewExamples = 0;
    struct example *examples = readNewExamples(trainingFeatureVectors, label, numNewExamples);

    printf("Read %d new examples\n", numNewExamples);

    double *dec_values;
    int nCorrectBefore = 0;
    for (int i = 0; i < numNewExamples; ++i)
    {
        std::cout << "Test Example No. " << i << std::endl;
        dec_values = Malloc(double, this->getMaxLabel());
        int pred = testExample(examples[i].x, prob.n, dec_values);
        if (pred == examples[i].y)
            ++nCorrectBefore;
        free(dec_values);

        //  std::cout << "OnlineAddExample Example No. " << i << "("<<prob.n<<")"<<std::endl;
        //  for(unsigned int iterElement = 0; iterElement  < prob.n ; ++iterElement)
        //          {
        //              std::cout<<examples[i].x[iterElement]<< " ";
        //          }
        //  std::cout<<std::endl;

        //onlineAddExample(examples[i].x, prob.n, examples[i].y, true); // Add example without updating classifiers
        //onlineAddExample(examples[i].x, prob.n, examples[i].y, false); // Add example without updating classifiers
        onlineAddExample(examples[i].x, prob.n, examples[i].y, (i + 1) % 10 == 0); // Update classifiers every 10 examples
    }
    onlineTrainAll(false); // Do a final model update and optionally save it to file (change false to true).

    nClass = getNumClasses();

    printf("New model: Number of classes = %d, Feature dimensionality=%d, Number of examples=%d\n", nClass, prob.n, prob.l);

    dec_values = Malloc(double, this->getMaxLabel());
    int nCorrectAfter = 0;
    for (int i = 0; i < numNewExamples; ++i)
    {
        int pred = testExample(examples[i].x, prob.n, dec_values);
        if (pred == examples[i].y)
            ++nCorrectAfter;
    }

    this->saveModel2(updatedModel);

    //this->do_cross_validation();

    printf("Accuracy on new data: Before=%f After=%f\n", double(nCorrectBefore) / numNewExamples, double(nCorrectAfter) / numNewExamples);
}

void CSvmLinearClassifier::onlineTrainAll(bool saveModel)
{
    int nClass = getNumClasses();
    bool *update = Malloc(bool, nClass);
    for (int j = 0; j < nClass; ++j)
        update[j] = true;

    onlinetrain(&prob, &param, model_, update);
    free(update);

    if (saveModel)
    {
        if (save_model(model_path, model_))
        {
            fprintf(stderr, "can't save model to file %s\n", model_path);
            exit(1);
        }
    }
}

void CSvmLinearClassifier::saveModel2(std::string filename)
{
    std::string filenameTr = std::string(filename + ".tr");
    this->saveProblem(filenameTr);

    std::string filenameMo = std::string(filename + ".model");
    save_model(filenameMo.c_str(), model_);
}

//save vector + model
void CSvmLinearClassifier::saveModel(std::string filename)
{
    std::string filenameTr = std::string(filename + ".tr");
    this->saveTrainingData(filenameTr);

    std::string filenameMo = std::string(filename + ".model");
    save_model(filenameMo.c_str(), model_);
}

void CSvmLinearClassifier::saveProblem(std::string filename)
{
    std::cout << "CSvmLinearClassifier::saveModel...." << std::endl;
    std::fstream fout;
    fout.open(std::string(filename).c_str(), std::ios::out | std::ios::binary);

    for (int iterFeatVecs = 0; iterFeatVecs  < prob.l ; ++iterFeatVecs)
    {
        int label = (int)prob.y[iterFeatVecs];
        fout << label << " ";
        //std::cout << (int)label <<" ";
        for (int iterElement = 0; iterElement  < prob.n ; ++iterElement)
        {

            if ((int)prob.x[iterFeatVecs][iterElement].index != -1)
            {
                fout << (int)prob.x[iterFeatVecs][iterElement].index << ":" << (float)prob.x[iterFeatVecs][iterElement].value << " ";
                //  std::cout << (int)prob.x[iterFeatVecs][iterElement].index <<":"<<(float)prob.x[iterFeatVecs][iterElement].value<<" ";
            }
            else
                break;
        }
        fout << std::endl;
        //std::cout<<std::endl;
    }
}
