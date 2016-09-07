/*
 * CSvmOneClassClassifier.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: mca
 */


#include <limits>
#include <assert.h>

#include "svm/svm_one_class_classifier.h"

CSvmOneClassClassifier::CSvmOneClassClassifier()
{
    logger = &CLogger::getInstance();
    this->id = "no_id";
    //this->isOneClass = false;
    strSvmProblem.l = 0;
    this->setSvmParameter();
    this->gridSearchIterations = GRID_SEARCH_ITERATIONS;
    this->isOneClass = true;
}

CSvmOneClassClassifier::CSvmOneClassClassifier(std::string id, std::string logFile)
{
    logger = &CLogger::getInstance();
    this->id = id;
    //this->isOneClass = false;
    this->logFile = logFile;
    strSvmProblem.l = 0;
    this->setSvmParameter();
    this->gridSearchIterations = GRID_SEARCH_ITERATIONS;
    this->isOneClass = true;
}

void CSvmOneClassClassifier::setSvmParameter()
{
    //if (this->isOneClass)
    //{
    std::cout << "It is one Class" << std::endl;
    this->strSvmParameter.svm_type = ONE_CLASS;
    //}
    //  else
    //  {
    //      std::cout<<"C_SVC"<<std::endl;
    //      this->strSvmParameter.svm_type = C_SVC;
    //  }
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

double CSvmOneClassClassifier::trainSvm(std::vector<std::vector<double> > param_traininSet, std::vector<std::vector<double> > param_testSet)
{
    const char *error_msg;
    assert(param_traininSet.size() > 0);
    this->positiveLabel = param_traininSet[0][0];

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

    return this->evaluateOneClass(param_testSet);
}

//Smart grid search
double CSvmOneClassClassifier::parameterGridSearch(std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, int numOfTrainExamples, bool prob)
{
    // read model must be execute beforehand
    // this is suppose to be for RBF!!!
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...");
    std::vector<SSetting> results;
    assert(trainSet.size() > 0);
    this->positiveLabel = trainSet[0][0];

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

    double nuMin = 0.1;
    double nuMax = 1.0;
    double cMin = -15;
    double cMax = 15;
    double nuOpt;
    //double cOpt;
    int iter = 0;
    double steps = 0.1; //3
    double stepsC = 10; //3
    double overallIter = 0;

    //gamma, nu

    if (strSvmParameter.kernel_type == RBF)
    {
        do
        {
            results.clear();
            std::cout << " SVM CV (" << iter << "): nuMin=" << nuMin << " nuMax=" << nuMax << std::endl;
            std::cout << " SVM CV (" << iter << "): cMin-Expo=" << cMin << " cMax-Expo=" << cMax << std::endl;
            for (double nu = nuMin; nu <= (nuMax + std::numeric_limits<double>::epsilon()); nu += steps)
            {
                strSvmParameter.nu = nu; //pow(2, gamma);
                for (double c = cMin; c <= (cMax + std::numeric_limits<double>::epsilon()); c += stepsC)
                {
                    overallIter++;
                    SSetting result;
                    strSvmParameter.C = pow(2, c);
                    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
                    trError = evaluateOneClass(trainSet);
                    teError = evaluateOneClass(testSet);
                    cvError = this->crossValidation(10);//5
                    //out<<strSvmParameter.gamma<<"\t"<<strSvmParameter.C<<"\t"<<cvError<<"\t"<<trError<<"\t"<<teError<<endl;
                    std::cout << " SVM CV(" << iter << "):  nu=" << nu << "c=" << c << " CV-Err=" << cvError << " Tr-Err=" << trError << " Te-Err=" << teError << std::endl;

                    result.c = strSvmParameter.C;
                    result.cExponent = c;
                    result.nu = strSvmParameter.nu;

                    result.trainError = trError;
                    result.testError = teError;
                    result.cvError = cvError;

                    results.push_back(result);
                }
            }
            bestModel = findBestModel(results, 1);

            //+-step size since max dist between previous min/max
            nuMin = (bestModel.nu - ((double) steps)); //(bestModel.gamma-((bestModel.gamma*0.1)/iter));
            if (nuMin <= 0)
            {
                nuMin = std::numeric_limits<double>::epsilon();
            }
            nuMax = (bestModel.nu + ((double) steps));

            cMin = (bestModel.cExponent - ((double) stepsC));//(bestModel.c-((bestModel.c*0.1)/iter));
            cMax = (bestModel.cExponent + ((double) stepsC));//(bestModel.c+((bestModel.c*0.1)/iter));

            stepsC = stepsC * 0.5;
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
    strSvmParameter.nu = bestModel.nu;
    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...Eval TrainSet");
    bestModel.trainError = evaluateOneClass(trainSet, true);
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...Eval TestSet");
    bestModel.testError = evaluateOneClass(testSet, true);
    bestModel.cvError = this->crossValidation(10);

    std::cout << "BEST Model found: " << bestModel.nu << "\t" << bestModel.cvError << "\t" << bestModel.trainError << "\t" << bestModel.testError << std::endl;
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...BEST Model found:  nu = %lf \t  c=%lf(%lf) \t  cv=%lf \t  tr=%lf \t  te=%lf ", bestModel.nu, bestModel.c, bestModel.cExponent, bestModel.cvError,
                      bestModel.trainError, bestModel.testError);
    //out<<endl<<endl<<bestModel.gamma<<"\t"<<bestModel.c<<"\t"<<bestModel.cvError<<"\t"<<bestModel.trainError<<"\t"<<bestModel.testError<<endl;
    //out<<overallIter;
    //out.close();

    this->currentSetupResults = bestModel;

    return this->currentSetupResults.testError;
}


//Smart grid search Crossvalidation!!
double CSvmOneClassClassifier::parameterGridSearch2(std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, int numOfTrainExamples, bool prob)
{
    // read model must be execute beforehand
    // this is suppose to be for RBF!!!
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...");
    std::vector<SSetting> results;
    assert(trainSet.size() > 0);
    this->positiveLabel = trainSet[0][0];

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

    double nuMin = 0.1;
    double nuMax = 1.0;
    double cMin = -15;
    double cMax = 15;
    double nuOpt;
    //double cOpt;
    int iter = 0;
    double steps = 0.1; //3
    double stepsC = 10; //3
    double overallIter = 0;

    //gamma, nu

    if (strSvmParameter.kernel_type == RBF)
    {
        do
        {
            results.clear();
            std::cout << " SVM CV (" << iter << "): nuMin=" << nuMin << " nuMax=" << nuMax << std::endl;
            std::cout << " SVM CV (" << iter << "): cMin-Expo=" << cMin << " cMax-Expo=" << cMax << std::endl;
            for (double nu = nuMin; nu <= (nuMax + std::numeric_limits<double>::epsilon()); nu += steps)
            {
                strSvmParameter.nu = nu; //pow(2, gamma);
                for (double c = cMin; c <= (cMax + std::numeric_limits<double>::epsilon()); c += stepsC)
                {
                    overallIter++;
                    SSetting result;
                    strSvmParameter.C = pow(2, c);
                    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
                    trError = evaluateOneClass(trainSet);
                    teError = evaluateOneClass(testSet);
                    cvError = this->crossValidation(10);//5
                    //out<<strSvmParameter.gamma<<"\t"<<strSvmParameter.C<<"\t"<<cvError<<"\t"<<trError<<"\t"<<teError<<endl;
                    std::cout << " SVM CV(" << iter << "):  nu=" << nu << "c=" << c << " CV-Err=" << cvError << " Tr-Err=" << trError << " Te-Err=" << teError << std::endl;

                    result.c = strSvmParameter.C;
                    result.cExponent = c;
                    result.nu = strSvmParameter.nu;

                    result.trainError = trError;
                    result.testError = teError;
                    result.cvError = cvError;

                    results.push_back(result);
                }
            }
            bestModel = findBestModel(results, 3);

            //+-step size since max dist between previous min/max
            nuMin = (bestModel.nu - ((double) steps)); //(bestModel.gamma-((bestModel.gamma*0.1)/iter));
            if (nuMin <= 0)
            {
                nuMin = std::numeric_limits<double>::epsilon();
            }
            nuMax = (bestModel.nu + ((double) steps));

            cMin = (bestModel.cExponent - ((double) stepsC));//(bestModel.c-((bestModel.c*0.1)/iter));
            cMax = (bestModel.cExponent + ((double) stepsC));//(bestModel.c+((bestModel.c*0.1)/iter));

            stepsC = stepsC * 0.5;
            steps = steps * 0.5;

            iter++;
        }
        while (iter < gridSearchIter);
    }

    bestModel = findBestModel(results, 3);

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
    strSvmParameter.nu = bestModel.nu;
    this->svmModel = svm_train(&strSvmProblem, &strSvmParameter);
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...Eval TrainSet");
    bestModel.trainError = evaluateOneClass(trainSet, true);
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...Eval TestSet");
    bestModel.testError = evaluateOneClass(testSet, true);
    bestModel.cvError = this->crossValidation(10);

    std::cout << "BEST Model found: " << bestModel.nu << "\t" << bestModel.cvError << "\t" << bestModel.trainError << "\t" << bestModel.testError << std::endl;
    logger->log->info("CSvmOneClassClassifier::parameterGridSearch...BEST Model found:  nu = %lf \t  c=%lf(%lf) \t  cv=%lf \t  tr=%lf \t  te=%lf ", bestModel.nu, bestModel.c, bestModel.cExponent, bestModel.cvError,
                      bestModel.trainError, bestModel.testError);
    //out<<endl<<endl<<bestModel.gamma<<"\t"<<bestModel.c<<"\t"<<bestModel.cvError<<"\t"<<bestModel.trainError<<"\t"<<bestModel.testError<<endl;
    //out<<overallIter;
    //out.close();

    this->currentSetupResults = bestModel;

    return this->currentSetupResults.testError;
}


double CSvmOneClassClassifier::evaluateOneClass(std::vector<std::vector<double> > testSet, bool doDebug)
{
    std::vector<double> errorResults;
    char testExampleLabel = 'x';

    int positiveCounts = 0;
    int positiveTotalCounts = 0;
    int negativeCounts = 0;
    int negativeTotalCounts = 0;

    for (int iRuns = 0; iRuns < testSet.size(); iRuns++)
    {
        double **svmOvO_prob_estimates = (double**) malloc(sizeof(double*));
        *svmOvO_prob_estimates = NULL;

        testExampleLabel = (char) testSet.at(iRuns).at(0);
        //testSet.at(iRuns).at(0)=0;//delete label

        if (testExampleLabel == this->positiveLabel)
        {

            errorResults.push_back((char) this->predict(testSet.at(iRuns), svmOvO_prob_estimates, true) == testExampleLabel ? 0 : 1);

            //std::cout<<" "<<svmOvO_prob_estimates[0][]<<std::endl;
            if (errorResults.back() == 1)
            {
                positiveCounts++;
            }
            positiveTotalCounts++;
        }
        else
        {

            errorResults.push_back((char) this->predict(testSet.at(iRuns), svmOvO_prob_estimates, true) == '?' ? 0 : 1);
            if (errorResults.back() == 1)
            {
                negativeCounts++;
            }
            negativeTotalCounts++;
        }

        free(*svmOvO_prob_estimates);
        free(svmOvO_prob_estimates);
    }

    if (doDebug)
    {
        this->logger->log->info("CSvmOneClassClassifier::evaluateOneClass...PositiveError=%lf (%d/%d)", (double) positiveCounts / (double) positiveTotalCounts, positiveCounts, positiveTotalCounts);
        this->logger->log->info("CSvmOneClassClassifier::evaluateOneClass...NegativeError=%lf (%d/%d)", (double) negativeCounts / (double) negativeTotalCounts, negativeCounts, negativeTotalCounts);
    }

    return toolBox.meanError(errorResults);
}

CSvmOneClassClassifier::~CSvmOneClassClassifier()
{
    // TODO Auto-generated destructor stub
}
