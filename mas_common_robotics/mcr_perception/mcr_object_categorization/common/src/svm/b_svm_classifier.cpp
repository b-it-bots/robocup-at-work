/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/




#include <assert.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "file_settings.h"
#include "svm/b_svm_classifier.h"

CBSvmClassifier::CBSvmClassifier()
{
    CFileSettings::init();
    this->logger = &CLogger::getInstance();
    this->numClassifier = NUM_CLASSIFIERS;
    this->id = "no-name";
}

void CBSvmClassifier::setHomePath(std::string homePath)
{
    this->homePath = homePath;
}

void CBSvmClassifier::loadModel()
{
    std::map<int, std::string>::iterator iterModel;
    this->logger->log->info("CBSvmClassifier::loadSVMModel()...");
    for (iterModel = svmModels.begin(); iterModel != svmModels.end(); ++iterModel)
    {
        std::string modelFileName = std::string(std::string(this->homePath) + std::string(iterModel->second));

        if (boost::filesystem::exists(modelFileName))
        {
            this->svms[iterModel->first].loadSvmModel(modelFileName);
            this->logger->log->info("CBSvmClassifier::loadSVMModel()....loaded : %s\n", modelFileName.c_str());
        }
        else
        {
            this->logger->log->info("CBSvmClassifier::loadSVMModel()....could not open File: %s\n", modelFileName.c_str());
        }
    }
}

void CBSvmClassifier::saveModel()
{
    assert(this->svms.size() > 0);
    std::map<int, CSvmClassifier>::iterator iterSvm;

    for (iterSvm = this->svms.begin(); iterSvm != this->svms.end(); ++iterSvm)
    {
        std::string svmId = boost::lexical_cast<std::string>(static_cast<unsigned int>(iterSvm->first));
        iterSvm->second.saveSvmModel(
            std::string(this->homePath) + std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_WeakLearner" + svmId + "_" + this->id + "_" + std::string(SVM_MODEL_POSTFIX));
        this->svmModels[iterSvm->first] = std::string(std::string(SVM_PATH) + std::string(SVM_MODEL_FILENAME) + "_WeakLearner" + svmId + "_" + this->id + "_" + std::string(SVM_MODEL_POSTFIX));
    }
}

void CBSvmClassifier::setId(std::string id)
{
    this->id = id;
}

void CBSvmClassifier::setTrainSet(std::map<int, std::vector<std::vector<double> > > trainSet)
{
    assert(trainSet.size() > 0);
    this->trainSet = trainSet;
}

void CBSvmClassifier::setTestSet(std::map<int, std::vector<std::vector<double> > > testSet)
{
    assert(testSet.size() > 0);
    this->testSet = testSet;
}

void CBSvmClassifier::setNumClassifier(unsigned int numClassifier)
{
    assert(numClassifier > 0);
    this->numClassifier = numClassifier;
}

double CBSvmClassifier::train(std::map<int, std::vector<int> > &missClassified)
{
    assert(trainSet.size() > 0);
    assert(testSet.size() > 0);

    this->numClassifier = this->trainSet[NEGATIVE].size() / this->trainSet[POSITIVE].size();
    if (this->numClassifier < 1)
    {
        std::cout << "Min Num Classifier reached!! == 1 " << this->numClassifier << std::endl;
        std::cout << "TrainSET size pos  " << this->trainSet[POSITIVE].size() << std::endl;
        std::cout << "TrainSET size neg  " << this->trainSet[NEGATIVE].size() << std::endl;
        this->numClassifier = 1;
    }
    logger->log->info("CBSvmClassifier::train()...Num Classifiers are %d (%d,%d)", numClassifier, this->trainSet[POSITIVE].size(), this->trainSet[NEGATIVE].size());

    std::vector<std::vector<double> > extractedFeatureVectorsTest = this->createOneVsAllset(POSITIVE, testSet);

    for (unsigned int iterSvm = 0; iterSvm < this->numClassifier; ++iterSvm)
    {
        logger->log->info("CBSvmClassifier::train()...SVM No. %d/%d...........", iterSvm, this->numClassifier);
        std::vector<std::vector<double> > negBootstrapped = this->createBootstrap(this->trainSet[NEGATIVE], this->trainSet[POSITIVE].size());

        std::map<int, std::vector<std::vector<double> > > bootstrappedFeatureVectors;
        bootstrappedFeatureVectors[POSITIVE] = this->trainSet[POSITIVE];
        bootstrappedFeatureVectors[NEGATIVE] = negBootstrapped;
        std::vector<std::vector<double> > extractedFeatureVectorsTrain = createOneVsAllset(POSITIVE, bootstrappedFeatureVectors);

        svms[iterSvm] = CSvmClassifier();//CSvm(ocSettings);
        this->svms[iterSvm].setGridSearchIterations(1); //10
        this->svmError[iterSvm] = this->svms[iterSvm].parameterGridSearch(extractedFeatureVectorsTrain, extractedFeatureVectorsTest, -1, true); //, false);//false
        this->svmAccuracy[iterSvm] = this->verifyWeakLearnerModel(svms[iterSvm]);

        std::string svmId = boost::lexical_cast<std::string>(static_cast<unsigned int>(iterSvm));
        logger->log->info("CBSvmClassifier::train()...TestError of %d. WeakLearner is %lf", iterSvm, this->svmError[iterSvm]);
        logger->log->info("CBSvmClassifier::train()...AccuracyError of %d. WeakLearner is %d->%lf :: %d->%lf\n", iterSvm, POSITIVE, this->svmAccuracy[iterSvm][POSITIVE], NEGATIVE,
                          this->svmAccuracy[iterSvm][NEGATIVE]);
    }

    double strongLearnerTrainError = this->verifyModel(missClassified);
    logger->log->info("CBSvmClassifier::train()...Trainerror of StrongLearner is %lf", strongLearnerTrainError);

    //this->saveModel();
    return strongLearnerTrainError;
}

char CBSvmClassifier::predict(std::vector<double> query, double &confidence, bool isLabeled)
{
    assert(query.size() > 0);
    assert(this->svms.size());
    assert(this->svmError.size());
    assert(this->svmAccuracy.size());

    std::map<int, CSvmClassifier>::iterator iterSvm;
    std::map<int, double> responses;
    std::map<int, unsigned int> numResponses;

    //double responseSum = 0.0f;
    //if (isLabeled)
    //{
    //  std::cout << "CBSvmClassifier::predict... Label is " << (int) (query[0] - 'A') << std::endl;
    //}

    for (iterSvm = this->svms.begin(); iterSvm != this->svms.end(); ++iterSvm)
    {
        double **svmProbability = (double**) malloc(sizeof(double*));
        *svmProbability = NULL;

        int response = (int) iterSvm->second.predict(query, svmProbability, isLabeled) - 'A';

        if (response == POSITIVE)
        {
            responses[response] += this->svmAccuracy[iterSvm->first][response];// * svmProbability[0][response - 'A'];
            numResponses[response]++;
        }
        else if (response == NEGATIVE)
        {
            responses[response] += this->svmAccuracy[iterSvm->first][response];// * svmProbability[0][response - 'A'];
            numResponses[response]++;
        }
        else
        {
            assert(response != POSITIVE && response != NEGATIVE);
        }

        std::cout << "CBSvmClassifier::predict... SVM " << iterSvm->first << " reponse is " << response << std::endl;

        //responseSum += (((double)response) - ((   (double) this->svms.size() - 1) / 2.0));
        //std::cout << "CBSvmClassifier::predict... current SUM RESPONSE with adding" << iterSvm->first << " is " << responseSum << std::endl;

        free(*svmProbability);
        free(svmProbability);
    }

    if (responses[POSITIVE] > responses[NEGATIVE])
    {
        confidence = (double)responses[POSITIVE] / (double)numResponses[POSITIVE];
        //std::cout<<"Condf"<<confidence<<" "<<responses[POSITIVE]<<" "<<numResponses[POSITIVE]<<std::endl;
        return (POSITIVE + 'A');
    }
    else
    {
        confidence = (double)responses[NEGATIVE] / (double)numResponses[NEGATIVE];
        //std::cout<<"Condf"<<confidence<<" "<<responses[NEGATIVE]<<" "<<numResponses[NEGATIVE]<<std::endl;
        return (NEGATIVE + 'A');
    }
}

int CBSvmClassifier::signFnct(double value)
{
    if (value < 0)
        return -1;
    else if (value == 0)
        return 0;
    else if (value > 0)
        return 1;

    return 0;
}

std::map<int, double> CBSvmClassifier::verifyWeakLearnerModel(CSvmClassifier &weakLearner)
{
    assert(testSet.size() > 0);

    //label
    std::map<int, double> accuracy;
    std::map<unsigned int, unsigned int> totalNumTests;
    std::vector<std::vector<double> > extractedFeatureVectorsTest = this->createOneVsAllset(POSITIVE, testSet);

    accuracy[POSITIVE] = 0;
    accuracy[NEGATIVE] = 0;
    for (unsigned int iterTest = 0; iterTest < extractedFeatureVectorsTest.size(); ++iterTest)
    {

        double **svmProbability = (double**) malloc(sizeof(double*));
        *svmProbability = NULL;

        char response = weakLearner.predict(extractedFeatureVectorsTest[iterTest], svmProbability, true);

        if ((char) extractedFeatureVectorsTest[iterTest][0] == (POSITIVE + 'A'))
        {
            totalNumTests[POSITIVE]++;
            if ((char) extractedFeatureVectorsTest[iterTest][0] == (response))
            {
                if (response == (POSITIVE + 'A'))
                {
                    accuracy[POSITIVE]++;
                }

            }
        }
        if ((char) extractedFeatureVectorsTest[iterTest][0] == (NEGATIVE + 'A'))
        {
            totalNumTests[NEGATIVE]++;
            if ((char) extractedFeatureVectorsTest[iterTest][0] == (response))
            {
                if (response == (NEGATIVE + 'A'))
                {
                    accuracy[NEGATIVE]++;
                }

            }
        }
        free(*svmProbability);
        free(svmProbability);
    }

    assert((totalNumTests[POSITIVE] + totalNumTests[NEGATIVE]) == extractedFeatureVectorsTest.size());

    accuracy[POSITIVE] /= totalNumTests[POSITIVE];
    accuracy[NEGATIVE] /= totalNumTests[NEGATIVE];

    return accuracy;
}

double CBSvmClassifier::verifyModel(std::map<int, std::vector<int> > &missClassified)
{
    logger->log->info("CBSvmClassifier::verifyModel....");
    assert(this->trainSet.size() > 0);

    //label, count
    std::map<int, int> errorCount;
    std::map<int, int> totalCount;
    std::map<int, int>::iterator errorCountIter;

    std::map<int, std::vector<std::vector<double> > >::iterator iterTest;

    errorCount[(int)(POSITIVE + 'A')] = 0;
    errorCount[(int)(NEGATIVE + 'A')] = 0;
    totalCount[(int)(POSITIVE + 'A')] = 0;
    totalCount[(int)(NEGATIVE + 'A')] = 0;
    int totalErrorCount = 0;
    int totalNumTest = 0;

    for (iterTest = testSet.begin(); iterTest != testSet.end(); ++iterTest)
    {
        std::vector<std::vector<double> > extractedFeatureVectorsTest = iterTest->second;
        for (unsigned int iterTestVec = 0; iterTestVec < extractedFeatureVectorsTest.size(); ++iterTestVec)
        {
            double confidence;
            char response = this->predict(extractedFeatureVectorsTest[iterTestVec], confidence, false);

            if ((iterTest->first + 'A') != response)
            {
                missClassified[iterTest->first].push_back(iterTestVec);
                errorCount[(int)(iterTest->first + 'A')]++;
                totalErrorCount++;
                logger->log->info("CBSvmClassifier::verifyModel()...Error at %d test->%d (%d,%d)\n", iterTest->first + 'A', iterTestVec, errorCount[(int)(iterTest->first + 'A')], totalErrorCount);
            }
            totalCount[(int)(iterTest->first + 'A')]++;
            totalNumTest++;
        }
    }

    assert(totalCount[(POSITIVE + 'A')] + totalCount[(NEGATIVE + 'A')] == totalNumTest);
    assert(errorCount[(POSITIVE + 'A')] + errorCount[(NEGATIVE + 'A')] == totalErrorCount);

    double testError = (double) totalErrorCount / (double) totalNumTest;
    logger->log->info("CBSvmClassifier::verifyModel()...Total Error %lf(%d/%d) \n", testError, totalErrorCount, totalNumTest);

    for (errorCountIter = errorCount.begin(); errorCountIter != errorCount.end(); ++errorCountIter)
    {
        double error = (double) errorCountIter->second / (double) totalCount[errorCountIter->first];
        logger->log->info("CBSvmClassifier::verifyModel()...Error for label %d -> %lf (%d,%d)\n", errorCountIter->first, error, errorCountIter->second, totalCount[errorCountIter->first]);
    }

    return testError;
}

std::vector<std::vector<double> > CBSvmClassifier::createBootstrap(std::vector<std::vector<double> > &featureVectors, unsigned int numberVec)
{
    logger->log->info("CBSvmClassifier::createBootstrap...CORRECT RANSAMPLING!!! MAYBE DOUBLED DRAWing if additonal is called!");
    assert(featureVectors.size() > 0);
    assert(featureVectors.size() >= numberVec);

    std::vector<std::vector<double> > bootstrapped;

    boost::uniform_int<> exampleSet(0, featureVectors.size() - 1);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);

    logger->log->debug("CBSvmClassifier::createBootstrap...select %d", numberVec);
    std::vector<int> randNumDrawn;
    for (unsigned int iterVec = 0; iterVec < (unsigned int) numberVec; ++iterVec)
    {
        int randIdx = getRandom();
        assert(randIdx >= 0 && randIdx < (int)featureVectors.size());
        logger->log->debug("CBSvmClassifier::createBootstrap...selected %d", randIdx);

        if (std::find(randNumDrawn.begin(), randNumDrawn.end(), randIdx) != randNumDrawn.end())
        {
            logger->log->debug("CBSvmClassifier::createBootstrap...already selected => %d ... no reject it!", randIdx);
            iterVec--;
            continue;
        }
        randNumDrawn.push_back(randIdx);
        bootstrapped.push_back(featureVectors[randIdx]);
    }

    assert(bootstrapped.size() == numberVec);
    return bootstrapped;
}

//if ratio == -1 then take all examples (true are labels obes, false are all other ones) so if 50 x 7 class then 50 pos and 280 neg
//if ration == 0.5 then 50% pos and 50% neg
// ratio how large is the portion of positves  so if 70 x 5 class then 70% then 70 pos and 17 neg
std::vector<std::vector<double> > CBSvmClassifier::createOneVsAllset(int label, std::map<int, std::vector<std::vector<double> > > &extractedFeatureVectors)
{
    std::map<int, std::vector<std::vector<double> > >::iterator iterCategories2;
    std::vector<std::vector<double> > featureVector;
    std::vector<std::vector<double> > negativeExampleFeatureVector;

    for (iterCategories2 = extractedFeatureVectors.begin(); iterCategories2 != extractedFeatureVectors.end(); ++iterCategories2)
    {
        std::vector<std::vector<double> > t = iterCategories2->second;
        for (unsigned int iterVec = 0; iterVec < t.size(); ++iterVec)
        {

            if (label == iterCategories2->first)
            {
                //POSITIVES
                t[iterVec].insert(t[iterVec].begin(), POSITIVE + 'A');
                featureVector.push_back(t[iterVec]);
            }
            else
            {
                //NEGATIVES
                t[iterVec].insert(t[iterVec].begin(), NEGATIVE + 'A');
                featureVector.push_back(t[iterVec]);

            }
        }
    }
    return featureVector;
}
