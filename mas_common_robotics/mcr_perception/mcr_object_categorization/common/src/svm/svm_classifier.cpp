/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/


#include <boost/lexical_cast.hpp>

#include "svm/svm_classifier.h"
#include <libsvm/svm.h>

using namespace std;
CSvmClassifier::CSvmClassifier()
{
    this->id = "no_id";
    this->isOneClass = false;
    strSvmProblem.l = 0;
    this->setSvmParameter();
    this->gridSearchIterations = GRID_SEARCH_ITERATIONS;
}

CSvmClassifier::CSvmClassifier(std::string id, std::string logFile)
{
    this->id = id;
    this->isOneClass = false;
    this->logFile = logFile;
    strSvmProblem.l = 0;
    this->setSvmParameter();
    this->gridSearchIterations = GRID_SEARCH_ITERATIONS;
}

void CSvmClassifier::setOneClass(bool isOneClass)
{
    this->isOneClass = isOneClass;
}

int CSvmClassifier::convertTrainingDataAdaBoostToSVM(std::string inFilename, std::string outFilename)
{

    char acBuffer[1000];
    char* temp_line = NULL;
    std::fstream inputFile;
    std::ofstream outputFile;
    std::vector < std::string > vecTokens;
    int offset = 'A' - 1;

    //open files
    inputFile.open(inFilename.c_str(), std::ios::in);
    outputFile.open(outFilename.c_str(), std::ios::out | std::ios::trunc);

    //check if they are really opened
    if (!outputFile.is_open() || !inputFile.is_open())
        return -1;

    // run through input file
    while (!inputFile.eof())
    {
        // get next line
        inputFile.getline(acBuffer, sizeof(acBuffer));
        temp_line = (char*) malloc(sizeof(char*) * strlen(acBuffer));
        strcpy(temp_line, acBuffer);
        vecTokens.clear();
        const std::string strString = temp_line;

        this->toolBox.stringTokenize(strString, vecTokens, ",");

        if (vecTokens.size() > 0)
        {
            outputFile << ((char) vecTokens.at(0).c_str()[0] - offset) << " ";
            for (unsigned int i = 1; i < vecTokens.size(); ++i)
            {
                outputFile << i << ":" << vecTokens.at(i) << " ";
            }
            outputFile << endl;
        }
    }

    //close files
    inputFile.close();
    outputFile.close();

    return 0;
}

void CSvmClassifier::setSvmParameter()
{
    if (this->isOneClass)
    {
        this->strSvmParameter.svm_type = ONE_CLASS;
    }
    else
    {
        this->strSvmParameter.svm_type = C_SVC;
    }
    this->strSvmParameter.kernel_type = RBF;//POLY;//LINEAR_KERNEL;//RBF;
    this->strSvmParameter.degree = 3.0;//3;
    this->strSvmParameter.gamma = (1.0 / 31.0);//(1/13);    // 1/num_features
    this->strSvmParameter.coef0 = 0;
    this->strSvmParameter.nu = 0.5;//0.5;
    this->strSvmParameter.cache_size = 100;
    this->strSvmParameter.C = 1;//1
    this->strSvmParameter.eps = 1e-3;
    this->strSvmParameter.p = 0.1;
    this->strSvmParameter.shrinking = 1;
    this->strSvmParameter.probability = 0;
    this->strSvmParameter.nr_weight = 0;
    this->strSvmParameter.weight_label = NULL;
    this->strSvmParameter.weight = NULL;
}

double CSvmClassifier::trainSvm(std::vector<vector<double> > param_traininSet)
{
    const char *error_msg;
    this->setTrainingSet(param_traininSet);
    this->readSvmProblem();
    error_msg = svm_check_parameter(&strSvmProblem, &strSvmParameter);
    if (error_msg)
    {
        fprintf(stderr, "SVM parameter check error: %s\n", error_msg);
        return 0;
    }

    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);

    //if(saveModel)
    //  this->saveSvmModel();
    //  dCrossValError = this->crossValidationSamples(strSvmProblem, strSvmParameters, 5);

    return 0;

}

void CSvmClassifier::setTrainingSet(std::vector<vector<double> > param_trainingSet)
{
    trainingSet = param_trainingSet;
}

int CSvmClassifier::readSvmProblem()
{
    int offset = 'A' - 1;
    if ((trainingSet.size() < 1))
    {
        cout << "mismatch size!!!" << endl;
        return -1;
    }

    //clean
    if (strSvmProblem.l > 0)
    {
        strSvmProblem.l = 0;
        free(strSvmProblem.y);
        free(strSvmProblem.x);
    }

    struct svm_node *x_values;
    int numFeatures = (trainingSet.at(0).size() - 1); //-1 since label is in Feat Vec

    strSvmProblem.l = trainingSet.size();
    strSvmProblem.y = new double[trainingSet.size()];
    strSvmProblem.x = new struct svm_node*[trainingSet.size()];
    x_values = new struct svm_node[(numFeatures + 1) * trainingSet.size()];

    for (unsigned int i = 0, j = 0; i < trainingSet.size(); i++, j++)
    {
        //
        strSvmProblem.y[i] = (trainingSet.at(i).at(0) - offset);
        //
        strSvmProblem.x[i] = &x_values[j];
        for (int k = 1; k <= (numFeatures); k++, j++)
        {
            x_values[j].index = k;
            x_values[j].value = (double) trainingSet.at(i).at(k);
        }
        x_values[j].index = -1;
        x_values[j].value = 0;
        //  for(int o=0;o<=j;o++)
        //      std::cout<<x_values[o].index <<" "<<x_values[o].value<<endl;
    }
    return 1;
}

int CSvmClassifier::readSvmProblem(unsigned int numOfExamples)
{
    int offset = 'A' - 1;
    if ((trainingSet.size() < 1) || (numOfExamples < 1) || (numOfExamples > trainingSet.size()))
    {
        cout << "mismatch size!!!" << endl;
        return -1;
    }

    //clean
    if (strSvmProblem.l > 0)
    {
        strSvmProblem.l = 0;
        free(strSvmProblem.y);
        free(strSvmProblem.x);
    }
    struct svm_node *x_values;
    int numFeatures = (trainingSet.at(0).size() - 1); //-1 since label is in Feat Vec

    strSvmProblem.l = numOfExamples;
    strSvmProblem.y = new double[numOfExamples];
    strSvmProblem.x = new struct svm_node*[numOfExamples];
    x_values = new struct svm_node[(numFeatures + 1) * numOfExamples];

    for (int i = 0, j = 0; (i < numOfExamples) && (i < trainingSet.size()); i++, j++)
    {
        //
        strSvmProblem.y[i] = (trainingSet.at(i).at(0) - offset);
        //
        strSvmProblem.x[i] = &x_values[j];
        for (int k = 1; k <= (numFeatures); k++, j++)
        {
            x_values[j].index = k;
            x_values[j].value = (double) trainingSet.at(i).at(k);
        }
        x_values[j].index = -1;
        x_values[j].value = 0;
        //  for(int o=0;o<=j;o++)
        //      std::cout<<x_values[o].index <<" "<<x_values[o].value<<endl;
    }

    return 1;
}

char CSvmClassifier::predict(std::vector<double> testExample, double **prob_estimates, bool isLabeled)
{
    int resultOffset = 'A' - 1; //lib svm : a=1 , b=2 , c=3, .... resulting lables
    int offset = 0; // offset if label in featur vector

    if (isLabeled)
    {
        offset = 1;
    }
    double predictedLabel = 'x';

    struct svm_problem problemToPredict;
    struct svm_node *x_values;

    int numFeatures = (testExample.size() - offset); //-1 since label is in Feat Vec

    problemToPredict.l = 1;
    problemToPredict.y = new double[1];
    problemToPredict.x = new struct svm_node*[1];
    x_values = new struct svm_node[numFeatures + 1]; //+1 since closing svm-1, but also -1 because of the label in the test example

    int j = 0;
    for (int k = 1; k <= numFeatures; k++, j++)
    {
        x_values[j].index = k;
        x_values[j].value = (double) testExample.at(k - (1 - offset)); //depends on labeled or not!!
    }
    x_values[j].index = -1;
    x_values[j].value = 0;

    //  for(int o=0;o<=j;o++)
    //          std::cout<<x_values[o].index <<" "<<x_values[o].value<<endl;


    //cout<<this->svmModel->nr_class;
    if (svm_check_probability_model(this->svmModel))
    {
        *prob_estimates = (double*) malloc(sizeof(double) * this->svmModel->nr_class);
        predictedLabel = svm_predict_probability(this->svmModel, x_values, *prob_estimates);
        //  std::cout << " svm_predict_probability : " << predictedLabel << "\n";
        //      cout<<(double)prob_estimates[0][0]<<"    "<<(double)prob_estimates[0][1]<<" "<<(double)prob_estimates[0][2]<<endl;
        //      free(prob_estimates);
    }
    else
    {
        predictedLabel = svm_predict(this->svmModel, x_values);
        prob_estimates = NULL;
    }
    free(problemToPredict.y);
    free(problemToPredict.x);
    free(x_values);
    return (char)(predictedLabel + resultOffset);
}

double CSvmClassifier::crossValidation(int iNrfold)
{
    int i;
    int total_correct = 0;
    double total_error = 0;
    double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
    double *target = MALLOC(double, strSvmProblem.l);
    double dCrossValError = -1;

    svm_cross_validation(&strSvmProblem, &strSvmParameter, iNrfold, target);

    if (strSvmParameter.svm_type == EPSILON_SVR || strSvmParameter.svm_type == NU_SVR)
    {
        for (i = 0; i < strSvmProblem.l; i++)
        {
            double y = strSvmProblem.y[i];
            double v = target[i];
            total_error += (v - y) * (v - y);
            sumv += v;
            sumy += y;
            sumvv += v * v;
            sumyy += y * y;
            sumvy += v * y;
        }

        dCrossValError = total_error / (double) strSvmProblem.l;

        printf("Cross Validation Mean squared error = %g\n", dCrossValError);
        printf("Cross Validation Squared correlation coefficient = %g\n",
               ((strSvmProblem.l * sumvy - sumv * sumy) * (strSvmProblem.l * sumvy - sumv * sumy)) / ((strSvmProblem.l * sumvv - sumv * sumv) * (strSvmProblem.l * sumyy - sumy * sumy)));
    }
    else
    {
        for (i = 0; i < strSvmProblem.l; i++)
            if (target[i] == strSvmProblem.y[i])
                ++total_correct;

        dCrossValError = 1 - ((double) total_correct / (double) strSvmProblem.l);
        printf("Cross Validation Accuarcy = %lf%%\n", 100 * (1 - dCrossValError));
    }

    free(target);

    return dCrossValError;
}

double CSvmClassifier::crossValidation2(int numFolds)
{

    double *target = new double[strSvmProblem.l];
    int total_correct = 0;
    double retval = 0.0;

    svm_cross_validation(&strSvmProblem, &strSvmParameter, numFolds, target);

    for (int m = 0; m < strSvmProblem.l; m++)
    {
        if (target[m] == strSvmProblem.y[m])
            ++total_correct;
        cout << "Cross Validation Accuracy = " << 100.0 * total_correct / strSvmProblem.l << endl;
        retval = 100.0 * total_correct / strSvmProblem.l;
    }

    free(target);
    return retval;
}

//NOT IN USE!!!
int CSvmClassifier::trainingSetCrossValidation(std::vector<vector<double> > trainSet, std::vector<vector<double> > testSet)
{
    double trError = 1;
    double teError = 1;
    double cvError = 1;
    int start = 0;
    vector<SSetting> results;
    SSetting bestModel;

    const char *error_msg;
    this->setTrainingSet(trainSet);
    error_msg = svm_check_parameter(&strSvmProblem, &strSvmParameter);
    if (error_msg)
    {
        fprintf(stderr, "SVM parameter check error: %s\n", error_msg);
    }

    for (int l = start; l < this->trainingSet.size(); l++)// this->trainingSet.size();l++)
    {
        SSetting result;
        this->readSvmProblem(l);
        this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
        cvError = this->crossValidation(5);
        trError = evaluate(trainSet);
        teError = evaluate(testSet);

        result.c = strSvmParameter.C;
        result.gamma = strSvmParameter.gamma;
        result.trainError = trError;
        result.testError = teError;
        result.cvError = cvError;
        result.trainExampleSize = this->strSvmProblem.l;
        results.push_back(result);
    }

    bestModel = findBestModel(results, 1);
    this->setSvmParameter();
    strSvmParameter.C = bestModel.c;
    strSvmParameter.gamma = bestModel.gamma;
    this->readSvmProblem(bestModel.trainExampleSize);
    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
    cvError = this->crossValidation(5);
    trError = evaluate(trainSet);
    teError = evaluate(testSet);

    for (int i = 0; i < results.size(); i++)
    {
        cout << results.at(i).trainExampleSize << "\t" << results.at(i).gamma << "\t" << results.at(i).c << "\t" << results.at(i).cvError << "\t" << results.at(i).trainError << "\t"
             << results.at(i).testError << endl;
    }
    cout << "BEST Model found: " << bestModel.trainExampleSize << "\t" << bestModel.gamma << "\t" << bestModel.c << "\t" << cvError << "\t" << trError << "\t" << teError << endl;

    //  if(saveModel)
    //      this->saveSvmModel();
    return bestModel.trainExampleSize;
}
double CSvmClassifier::evalOptimalModelTraining(std::vector<vector<double> > trainSet, std::vector<vector<double> > testSet, int numOfTrainExamples, bool prob)
{
    int currentNumberExamples = 10;
    std::vector < vector<double> > currentTrainingSet;
    int steps = 25;
    std::vector<svm_model> svmModels;
    std::vector<SSetting> results;

    //  fstream out(string(ocSettings.path +
    //      ocSettings.folderTraining +
    //      "svnNumExamplesResults" +
    //      ocSettings.currentEvalNumber +
    //      ocSettings.dotCfg).c_str(), ios::out | ios::binary);

    for (; currentNumberExamples < trainSet.size(); currentNumberExamples += steps)
    {
        std::string file;
        currentTrainingSet.clear();
        for (int i = 0; i < currentNumberExamples; i++)
        {
            currentTrainingSet.push_back(trainSet.at(i));
        }
        //out<<currentNumberExamples<< " ";
        //Smart Grid Search()
        this->parameterGridSearch(currentTrainingSet, testSet, -1, false);
        svmModels.push_back(*this->svmModel);

        file = boost::lexical_cast<std::string>(static_cast<int>(currentNumberExamples));

        string modelname = string(string("model") + file);
        svm_save_model(modelname.c_str(), svmModel);

        //  out<<this->currentSetupResults.c<<" "<<this->currentSetupResults.gamma<< "  "<<currentSetupResults.cvError<<" "<<currentSetupResults.testError<<endl;;
    }
    //  out.close();

    return 0;
}

//Smart grid search
double CSvmClassifier::parameterGridSearch(std::vector<vector<double> > trainSet, std::vector<vector<double> > testSet, int numOfTrainExamples, bool prob)
{
    // read model must be execute beforehand
    // this is suppose to be for RBF!!!
    vector<SSetting> results;
    //double steps=1;//0.25//0.5
    double trError = 1;
    double teError = 1;
    double cvError = 1;
    SSetting bestModel;
    int gridSearchIter = this->gridSearchIterations;//; 12;//12;//12;//20;//15gut gut //gut 10;

    const char *error_msg;
    this->setTrainingSet(trainSet);

    if (numOfTrainExamples < 1)
        this->readSvmProblem();
    else
        this->readSvmProblem(numOfTrainExamples);

    error_msg = svm_check_parameter(&strSvmProblem, &strSvmParameter);
    if (error_msg)
    {
        fprintf(stderr, "SVM parameter check error: %s\n", error_msg);
        return -1;
    }

    //If probability estimates ?
    if (prob)
        this->strSvmParameter.probability = 1;
    else
        this->strSvmParameter.probability = 0;

    double gammaMin = -15;
    double gammaMax = 15;
    double cMin = -15;
    double cMax = 15;
    double gammaOpt;
    double cOpt;
    int iter = 0;
    double steps = 10; //3
    double overallIter = 0;

    if (strSvmParameter.kernel_type == RBF)
    {
        do
        {
            results.clear();
            cout << " SVM CV (" << iter << "): gammaMin-Expo=" << gammaMin << " gammaMax-Expo=" << gammaMax << endl;
            cout << " SVM CV (" << iter << "): cMin-Expo=" << cMin << " cMax-Expo=" << cMax << endl;
            for (double gamma = gammaMin; gamma <= (gammaMax + numeric_limits<double>::epsilon()); gamma += steps)
            {
                strSvmParameter.gamma = pow(2, gamma);
                for (double c = cMin; c <= (cMax + numeric_limits<double>::epsilon()); c += steps)
                {
                    overallIter++;
                    SSetting result;
                    strSvmParameter.C = pow(2, c);
                    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
                    trError = evaluate(trainSet);
                    teError = evaluate(testSet);
                    cvError = this->crossValidation(10);//5
                    //out<<strSvmParameter.gamma<<"\t"<<strSvmParameter.C<<"\t"<<cvError<<"\t"<<trError<<"\t"<<teError<<endl;
                    cout << " SVM CV(" << iter << "):  gamma-Expo=" << gamma << " C-Expo=" << c << " CV-Err=" << cvError << " Tr-Err=" << trError << " Te-Err=" << teError << endl;

                    result.c = strSvmParameter.C;
                    result.cExponent = c;

                    result.gamma = strSvmParameter.gamma;
                    result.gammaExponent = gamma;

                    result.trainError = trError;
                    result.testError = teError;
                    result.cvError = cvError;

                    results.push_back(result);
                }
            }
            bestModel = findBestModel(results, 1);

            //+-step size since max dist between previous min/max
            gammaMin = (bestModel.gammaExponent - ((double) steps)); //(bestModel.gamma-((bestModel.gamma*0.1)/iter));
            gammaMax = (bestModel.gammaExponent + ((double) steps));

            cMin = (bestModel.cExponent - ((double) steps));//(bestModel.c-((bestModel.c*0.1)/iter));
            cMax = (bestModel.cExponent + ((double) steps));//(bestModel.c+((bestModel.c*0.1)/iter));

            steps = steps * 0.5;

            iter++;
        }
        while (iter < gridSearchIter);
    }

    bestModel = findBestModel(results, 1);

    this->setTrainingSet(trainSet);
    this->setSvmParameter();

    if (prob)
        this->strSvmParameter.probability = 1;
    else
        this->strSvmParameter.probability = 0;

    if (numOfTrainExamples < 1)
        this->readSvmProblem();
    else
        this->readSvmProblem(numOfTrainExamples);
    strSvmParameter.C = bestModel.c;
    strSvmParameter.gamma = bestModel.gamma;
    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
    bestModel.trainError = evaluate(trainSet);
    bestModel.testError = evaluate(testSet);
    bestModel.cvError = this->crossValidation(10);

    cout << "BEST Model found: " << bestModel.gamma << "\t" << bestModel.c << "\t" << bestModel.cvError << "\t" << bestModel.trainError << "\t" << bestModel.testError << endl;
    //out<<endl<<endl<<bestModel.gamma<<"\t"<<bestModel.c<<"\t"<<bestModel.cvError<<"\t"<<bestModel.trainError<<"\t"<<bestModel.testError<<endl;
    //out<<overallIter;
    //out.close();

    this->currentSetupResults = bestModel;

    return this->currentSetupResults.testError;
}

//must be labeld
double CSvmClassifier::evaluate(std::vector<vector<double> > testSet)
{
    vector<double> errorResults;
    char testExampleLabel = 'x';

    for (int iRuns = 0; iRuns < testSet.size(); iRuns++)
    {
        double **svmOvO_prob_estimates = (double**) malloc(sizeof(double*));
        *svmOvO_prob_estimates = NULL;

        testExampleLabel = (char) testSet.at(iRuns).at(0);
        //testSet.at(iRuns).at(0)=0;//delete label
        errorResults.push_back((char) this->predict(testSet.at(iRuns), svmOvO_prob_estimates, true) == testExampleLabel ? 0 : 1);
        //  std::cout<<testExampleLabel<<"..."<<this->predict(testSet.at(iRuns), svmOvO_prob_estimates, true)<<std::endl;

        free(*svmOvO_prob_estimates);
        free(svmOvO_prob_estimates);
    }

    return toolBox.meanError(errorResults);

}

SSetting CSvmClassifier::findBestModel(vector<SSetting> results, int option)
{
    double minTestError = 1;
    double minCvError = 1;
    vector<int> idxMinTestError;
    int idxMinCvTeError = 0;

    //test -> cv
    if (option == 1)//last best result
    {
        for (int i = 0; i < results.size(); i++)
        {
            if (results.at(i).testError <= (minTestError))
            {
                if (results.at(i).testError < (minTestError))
                    minCvError = 1;

                minTestError = results.at(i).testError;

                if (results.at(i).cvError <= (minCvError)) //<=
                {
                    minCvError = results.at(i).cvError;
                    idxMinCvTeError = i;
                }
            }
        }
    }

    //test -> cv
    if (option == 2) //first best result
    {
        for (int i = 0; i < results.size(); i++)
        {
            if (results.at(i).testError <= (minTestError))
            {

                if (results.at(i).testError < (minTestError))
                    minCvError = 1;

                minTestError = results.at(i).testError;

                if (results.at(i).cvError < (minCvError)) //<=
                {
                    minCvError = results.at(i).cvError;
                    idxMinCvTeError = i;
                }
            }
        }
    }

    //cv -> test NOT TESTED
    if (option == 3) //first best result
    {
        for (int i = 0; i < results.size(); i++)
        {
            if (results.at(i).cvError <= (minCvError + numeric_limits<double>::epsilon()))
            {

                if (results.at(i).cvError < (minCvError + numeric_limits<double>::epsilon()))
                    minTestError = 1;

                minCvError = results.at(i).cvError;

                if (results.at(i).testError < (minTestError + numeric_limits<double>::epsilon())) //<=
                {
                    minTestError = results.at(i).testError;
                    idxMinCvTeError = i;
                }
            }
        }
    }

    cout << "BEST Model found at: " << idxMinCvTeError << endl;
    return results.at(idxMinCvTeError);
}

void CSvmClassifier::saveSvmModel(string filename)
{
    svm_save_model((char*) filename.c_str(), this->svmModel);
    cout << "SVM Model saved " << filename << "\n";
}

void CSvmClassifier::loadSvmModel(string filename)
{
    this->svmModel = svm_load_model((char*) filename.c_str());
    cout << "SVM Model loaded " << filename << endl;
}

bool CSvmClassifier::isProbability()
{
    if (this->svmModel->probA != NULL || this->svmModel->probB != NULL)
        return true;
    else
        return false;
}

void CSvmClassifier::setGridSearchIterations(int gridSearchIterations)
{
    this->gridSearchIterations = gridSearchIterations;
}
