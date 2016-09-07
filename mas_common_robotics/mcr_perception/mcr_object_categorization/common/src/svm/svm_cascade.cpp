/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/

#include "svm/svm_cascade.h"

CSvmCascade::CSvmCascade()
{
    // TODO Auto-generated constructor stub
    this->logger = &CLogger::getInstance();
    this->numStages = 1;
    this->numIterations = 1;
    this->trainingType = 1;
}

void CSvmCascade::setHomePath(std::string homePath)
{
    this->homePath = homePath;
}

void CSvmCascade::loadCascadeModel()
{
    logger->log->info("CSvmCascade::loadCascadeModel...");
    assert(this->cascade.size() > 0);
    std::map<int, CBSvmClassifier>::iterator iterClassifier;

    for (iterClassifier = this->cascade.begin(); iterClassifier != this->cascade.end(); ++iterClassifier)
    {
        iterClassifier->second.setHomePath(this->homePath);
        iterClassifier->second.loadModel();
    }
}

void CSvmCascade::saveCascadeModel()
{
    logger->log->info("CSvmCascade::loadCascadeModel...");
    assert(this->cascade.size() > 0);
    std::map<int, CBSvmClassifier>::iterator iterClassifier;

    for (iterClassifier = this->cascade.begin(); iterClassifier != this->cascade.end(); ++iterClassifier)
    {
        iterClassifier->second.saveModel();
    }
}

void CSvmCascade::setTrainSet(std::map<int, std::vector<std::vector<double> > > fv)
{
    this->trainSet = fv;
}

void CSvmCascade::setTestSet(std::map<int, std::vector<std::vector<double> > > fv)
{
    this->testSet = fv;
}

void CSvmCascade::setNumStages(unsigned int numStages)
{
    this->numStages = numStages;
}

void CSvmCascade::setNumIterations(unsigned int numIterations)
{
    this->numIterations = numIterations;
}

std::map<int, std::vector<std::vector<double> > > CSvmCascade::createValidationSet(double ratio)
{
    logger->log->info("CSvmCascade::createValidationSet...");
    assert(ratio > 0);
    assert(this->trainSet.size() > 0);

    std::map<int, std::vector<std::vector<double> > > bootstrapped;

    std::map<int, std::vector<std::vector<double> > >::iterator iterSet;

    for (iterSet = this->trainSet.begin(); iterSet != this->trainSet.end(); ++iterSet)
    {
        int validationSetSize = iterSet->second.size() * ratio;

        assert(validationSetSize > 0 && validationSetSize < iterSet->second.size());

        boost::uniform_int<> exampleSet(0, iterSet->second.size() - 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);

        //draw samples from set.
        std::vector<int> randNumDrawn;
        for (unsigned int iterVec = 0; iterVec < (unsigned int) validationSetSize; ++iterVec)
        {
            int randIdx = getRandom();
            assert(randIdx >= 0 && randIdx < (int)iterSet->second.size());
            if (std::find(randNumDrawn.begin(), randNumDrawn.end(), randIdx) != randNumDrawn.end())
            {
                iterVec--;
                continue;
            }
            randNumDrawn.push_back(randIdx);
            bootstrapped[iterSet->first].push_back(iterSet->second[randIdx]);
        }

        //Delete drawn samples from training set
        int originalTrainSetSize = iterSet->second.size();
        for (unsigned int iterDel = 0; iterDel < randNumDrawn.size(); ++iterDel)
        {
            this->trainSet[iterSet->first].erase(this->trainSet[iterSet->first].begin() + randNumDrawn[iterDel]);
        }

        assert(bootstrapped[iterSet->first].size() == validationSetSize);
        assert((originalTrainSetSize - this->trainSet[iterSet->first].size()) == validationSetSize);
    }
    return bootstrapped;
}

//real cascade
void CSvmCascade::train0()
{
    assert(this->numStages == 1);
    assert(this->numStages > 0);
    assert(this->numIterations > 0);
    assert(this->trainSet.size() > 0);
    assert(this->testSet.size() > 0);
    this->trainingType = 0;

    std::map<int, std::vector<std::vector<double> > > currTrainSet = this->trainSet;
    std::map<int, std::vector<std::vector<double> > > currTestSet;
    std::map<int, std::vector<std::vector<double> > > currValiSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d ", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d ", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    currTestSet = this->trainSet;
    currTrainSet = this->trainSet;
    currValiSet = this->testSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    for (unsigned int iStage = 0; iStage < this->numStages; ++iStage)
    {
        for (unsigned int iIteration = 0; iIteration < this->numIterations; ++iIteration)
        {
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d ", iStage, iIteration);
            this->cascade[iIteration] = CBSvmClassifier();
            std::string strId = std::string(this->id + "_" + boost::lexical_cast<std::string>(iIteration));
            this->cascade[iIteration].setId(strId);
            this->cascade[iIteration].setHomePath(this->homePath);
            this->cascade[iIteration].setTrainSet(currTrainSet);
            this->cascade[iIteration].setTestSet(currTestSet);

            std::map<int, std::vector<int> > missClassified;
            this->cascadeStrongClassifierErr[iIteration] = this->cascade[iIteration].train(missClassified);
            unsigned int numbCl = this->cascade[iIteration].getNumClassifers();

            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: Positives failed == %d", iStage, iIteration, missClassified[POSITIVE].size());
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: Negatives failed == %d", iStage, iIteration, missClassified[NEGATIVE].size());
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: NumClassifiers == %d", iStage, iIteration, numbCl);

            //create positive set only with correct classified ones
            this->logger->log->info("CSvmCascade::train()....enhance positive set...");
            currTrainSet[POSITIVE] = this->createSet3(POSITIVE, currTrainSet[POSITIVE], currTestSet[POSITIVE], missClassified[POSITIVE], -1);
            //create negative set only with misclassified negative ones
            this->logger->log->info("CSvmCascade::train()....enhance negative set...");
            currTrainSet[NEGATIVE] = this->createSet3(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());

            //currTrainSet[POSITIVE].size() * (numbCl - (numbCl / (this->numStages - iStage))));

            //assert(currTrainSet[POSITIVE].size()>0);
            //assert(currTrainSet[NEGATIVE].size()>0);

            std::map<int, std::vector<int> > verifyValidationmissClassified;
            this->logger->log->info("CSvmCascade::train()....Test set Verify=%lf", this->verifyModel(this->testSet, verifyValidationmissClassified));
            verifyValidationmissClassified.clear();
            this->logger->log->info("CSvmCascade::train()....Validation set Verify=%lf", this->verifyModel(currValiSet, verifyValidationmissClassified));
            currValiSet[POSITIVE] = this->createSet3(POSITIVE, currValiSet[POSITIVE], currValiSet[POSITIVE], verifyValidationmissClassified[POSITIVE], -1);
            currValiSet[NEGATIVE] = this->createSet3(NEGATIVE, currValiSet[NEGATIVE], currValiSet[NEGATIVE], verifyValidationmissClassified[NEGATIVE], -1);

            if (currValiSet[POSITIVE].size() == 0 || currValiSet[NEGATIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....curr valiset = 0!!!! stop stagging");
                break;
            }

            if (missClassified[NEGATIVE].size() == 0 && missClassified[POSITIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....iStage  zero error in NEG!!!! stop stagging");
                break;
            }
            if (numbCl <= 1 && missClassified[NEGATIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....num Classifiers is one !!! stop stagging");
                break;
            }
            if (currTrainSet[POSITIVE].size() == 0 || currTrainSet[NEGATIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....train set set size == 0!!! stop stagging");
                break;
            }
            this->logger->log->info("--------------------------------------------------------------");
        }
        //this->logger->log->info("CSvmCascade::train()....enhance negative set...");
        //currTrainSet[NEGATIVE] = this->createSet2(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());
        //std::map<int, std::vector<int> > verifyValidationmissClassified;
        //this->logger->log->info("CSvmCascade::train()....Validation set Verify=%lf", this->verifyModel(this->validationSet, verifyValidationmissClassified));
        //this->logger->log->info("CSvmCascade::train()....Test set Verify=%lf", this->verifyModel(this->testSet, verifyValidationmissClassified));
        this->logger->log->info("*******************************************************");
    }
}

//in parallel pos and neg misclassifications
void CSvmCascade::train1()
{
    assert(this->numStages == 1);
    assert(this->numStages > 0);
    assert(this->numIterations > 0);
    assert(this->trainSet.size() > 0);
    assert(this->testSet.size() > 0);
    this->trainingType = 1;

    std::map<int, std::vector<std::vector<double> > > currTrainSet = this->trainSet;
    std::map<int, std::vector<std::vector<double> > > currTestSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d ", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d ", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    this->validationSet = currTestSet = this->createValidationSet(0.7f);
    currTrainSet = this->trainSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    for (unsigned int iStage = 0; iStage < this->numStages; ++iStage)
    {
        for (unsigned int iIteration = 0; iIteration < this->numIterations; ++iIteration)
        {
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d ", iStage, iIteration);
            this->cascade[iIteration] = CBSvmClassifier();
            std::string strId = std::string(this->id + "_" + boost::lexical_cast<std::string>(iIteration));
            this->cascade[iIteration].setId(strId);
            this->cascade[iIteration].setHomePath(this->homePath);
            this->cascade[iIteration].setTrainSet(currTrainSet);
            this->cascade[iIteration].setTestSet(currTestSet);

            std::map<int, std::vector<int> > missClassified;
            this->cascadeStrongClassifierErr[iIteration] = this->cascade[iIteration].train(missClassified);
            unsigned int numbCl = this->cascade[iIteration].getNumClassifers();

            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: Positives failed == %d", iStage, iIteration, missClassified[POSITIVE].size());
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: Negatives failed == %d", iStage, iIteration, missClassified[NEGATIVE].size());
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d: NumClassifiers == %d", iStage, iIteration, numbCl);

            assert(currTrainSet[NEGATIVE].size() > currTrainSet[POSITIVE].size());

            //create positive set only with correct classified ones
            this->logger->log->info("CSvmCascade::train()....enhance positive set...");
            currTrainSet[POSITIVE] = this->createSet2(POSITIVE, currTrainSet[POSITIVE], currTestSet[POSITIVE], missClassified[POSITIVE], -1);
            //create negative set only with misclassified negative ones
            this->logger->log->info("CSvmCascade::train()....enhance negative set...");
            currTrainSet[NEGATIVE] = this->createSet2(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());
            //currTrainSet[POSITIVE].size() * (numbCl - (numbCl / (this->numStages - iStage))));

            //assert(currTrainSet[POSITIVE].size()>0);
            //assert(currTrainSet[NEGATIVE].size()>0);

            //std::map<int, std::vector<int> > verifymissClassified;
            //this->logger->log->info("CSvmCascade::train()....Verify=%lf", this->verifyModel(this->testSet, verifymissClassified));

            std::map<int, std::vector<int> > verifyValidationmissClassified;
            this->logger->log->info("CSvmCascade::train()....Validation set Verify=%lf", this->verifyModel(this->validationSet, verifyValidationmissClassified));
            verifyValidationmissClassified.clear();
            this->logger->log->info("CSvmCascade::train()....Test set Verify=%lf", this->verifyModel(this->testSet, verifyValidationmissClassified));

            if (missClassified[NEGATIVE].size() == 0 && missClassified[POSITIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....iStage WOOOWWWWOOOOWW zero error in NEG && POS!!!! stop stagging");
                break;
            }
            if (numbCl <= 1 && missClassified[NEGATIVE].size() == 0)
            {
                this->logger->log->info("CSvmCascade::train()....num Classifiers is one !!! stop stagging");
                break;
            }
            this->logger->log->info("--------------------------------------------------------------");
        }
        //this->logger->log->info("CSvmCascade::train()....enhance negative set...");
        //currTrainSet[NEGATIVE] = this->createSet2(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());
        //std::map<int, std::vector<int> > verifyValidationmissClassified;
        //this->logger->log->info("CSvmCascade::train()....Validation set Verify=%lf", this->verifyModel(this->validationSet, verifyValidationmissClassified));
        //this->logger->log->info("CSvmCascade::train()....Test set Verify=%lf", this->verifyModel(this->testSet, verifyValidationmissClassified));
        this->logger->log->info("*******************************************************");
    }
}

//Matrix like
//during iterations > do positive misclassifications -> iteratively improve them!
//durging stage > negative misclassificaiton -> add all missclassified negative ones
void CSvmCascade::train2()
{
    assert(this->numStages > 0);
    assert(this->numIterations > 0);
    assert(this->trainSet.size() > 0);
    assert(this->testSet.size() > 0);
    this->trainingType = 2;

    std::map<int, std::vector<std::vector<double> > > currTrainSet = this->trainSet;
    std::map<int, std::vector<std::vector<double> > > currTestSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    this->validationSet = currTestSet = this->createValidationSet(0.7f);
    currTrainSet = this->trainSet;

    this->logger->log->info("CSvmCascade::train...Positive: TrainSet size = %d ValidationSet size %d", currTrainSet[POSITIVE].size(), currTestSet[POSITIVE].size());
    this->logger->log->info("CSvmCascade::train...Negative: TrainSet size = %d ValidationSet size %d", currTrainSet[NEGATIVE].size(), currTestSet[NEGATIVE].size());

    for (unsigned int iStage = 0; iStage < this->numStages; ++iStage)
    {
        std::map<int, std::vector<int> > missClassified;
        CBSvmClassifier currCascadeIteration;
        //POSITIVE PART
        for (unsigned int iIteration = 0; iIteration < this->numIterations; ++iIteration)
        {
            this->logger->log->info("CSvmCascade::train()....iStage %d, iIteration %d ", iStage, iIteration);
            currCascadeIteration = CBSvmClassifier();
            std::string strId = std::string(this->id + "_" + boost::lexical_cast<std::string>(iIteration));
            currCascadeIteration.setId(strId);
            currCascadeIteration.setHomePath(this->homePath);
            currCascadeIteration.setTrainSet(currTrainSet);
            currCascadeIteration.setTestSet(currTestSet);

            missClassified.clear();
            currCascadeIteration.train(missClassified);
            unsigned int numbCl = currCascadeIteration.getNumClassifers();

            this->logger->log->info("CSvmCascade::train()....POS iStage %d, iIteration %d: Positives failed == %d", iStage, iIteration, missClassified[POSITIVE].size());
            this->logger->log->info("CSvmCascade::train()....POS iStage %d, iIteration %d: Negatives failed == %d", iStage, iIteration, missClassified[NEGATIVE].size());
            this->logger->log->info("CSvmCascade::train()....POS iStage %d, iIteration %d: NumClassifiers == %d", iStage, iIteration, numbCl);

            assert(currTrainSet[NEGATIVE].size() > currTrainSet[POSITIVE].size());

            //create positive set only with correct classified ones
            this->logger->log->info("CSvmCascade::train()....enhance positive set...");
            currTrainSet[POSITIVE] = this->createSet2(POSITIVE, currTrainSet[POSITIVE], currTestSet[POSITIVE], missClassified[POSITIVE], -1);
            //create negative set only with misclassified negative ones
            if (currTrainSet[POSITIVE].size() > currTrainSet[NEGATIVE].size())
                ;
            {
                this->logger->log->info("CSvmCascade::train()....too many positves > negatives... break...go in negative stage!");
                break;
            }
            //currTrainSet[NEGATIVE] = this->createSet2(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());
            //currTrainSet[POSITIVE].size() * (numbCl - (numbCl / (this->numStages - iStage))));

            //std::map<int, std::vector<int> > verifymissClassified;
            //this->logger->log->info("CSvmCascade::train()....Verify=%lf", this->verifyModel(this->testSet, verifymissClassified));

            if (missClassified[POSITIVE].size() == 0) //missClassified[NEGATIVE].size() == 0 &&
            {
                this->logger->log->info("CSvmCascade::train()....iStage WOOOWWWWOOOOWW zero error in POS!!!! stop stagging");
                break;
            }
        }
        ////////////////////NEGATIVE PART
        this->logger->log->info("CSvmCascade::train()....enhance negative set...");
        currTrainSet[NEGATIVE] = this->createSet2(NEGATIVE, currTrainSet[NEGATIVE], currTestSet[NEGATIVE], missClassified[NEGATIVE], currTrainSet[POSITIVE].size());

        this->cascade[iStage] = CBSvmClassifier();
        std::string strId = std::string(this->id + "_" + boost::lexical_cast<std::string>(iStage));
        this->cascade[iStage].setId(strId);
        this->cascade[iStage].setHomePath(this->homePath);
        this->cascade[iStage].setTrainSet(currTrainSet);
        this->cascade[iStage].setTestSet(currTestSet);
        missClassified.clear();
        this->cascade[iStage].train(missClassified);
        unsigned int numbCl = this->cascade[iStage].getNumClassifers();

        this->logger->log->info("CSvmCascade::train()....NEG **iStage %d** : Positives failed == %d", iStage, missClassified[POSITIVE].size());
        this->logger->log->info("CSvmCascade::train()....NEG **iStage %d** : Negatives failed == %d", iStage, missClassified[NEGATIVE].size());
        this->logger->log->info("CSvmCascade::train()....NEG **iStage %d** : NumClassifiers == %d", iStage, numbCl);

        if (missClassified[POSITIVE].size() == 0 && missClassified[NEGATIVE].size() == 0)
        {
            this->logger->log->info("CSvmCascade::train()....iStage WOOOWWWWOOOOWW zero error in NEG/POS!!!! stop stagging");
            break;
        }
        this->logger->log->info("--------------------------------------------------------------");
    }
}

char CSvmCascade::predict(std::vector<double> query, double &confidence, bool isLabels)
{
    assert(this->numIterations > 0);
    assert(this->numStages > 0);
    assert(this->cascade.size() == this->cascadeStrongClassifierErr.size());
    assert(this->trainingType == 0 || this->trainingType == 1 || this->trainingType == 2);

    //usal cascade predictor
    if (this->trainingType == 0)
    {
        for (unsigned int iIteration = 0; iIteration < this->cascade.size(); ++iIteration)
        {
            //std::cout << "CSvmCascade::predict...iStage " << iStage << std::endl;
            if (this->cascade.find(iIteration) == this->cascade.end())
            {
                break;
            }

            if (this->cascade[iIteration].predict(query, confidence, isLabels) == NEGATIVE + 'A')
            {
                this->logger->log->debug("CSvmCascade::predict...subClassifier %d is negative", iIteration);
                ;
                return NEGATIVE + 'A';
            }
            this->logger->log->debug("CSvmCascade::predict...subClassifier %d is positive", iIteration);
        }
        return POSITIVE + 'A';
    }

    if (this->trainingType == 1)
    {
        std::map<int, double> res;

        for (unsigned int iIteration = 0; iIteration < this->cascade.size(); ++iIteration)
        {
            //  std::cout << "CSvmCascade::predict...iIteration " << iIteration << std::endl;
            if (this->cascade.find(iIteration) == this->cascade.end())
            {
                break;
            }
            double conf;
            int currResult = this->cascade[iIteration].predict(query, conf, isLabels);
            res[currResult] = res[currResult] + 1; //(this->cascadeStrongClassifierErr[iIteration] * conf);

            //this->logger->log->info("CSvmCascade::train()....subclassifier %d .... label %d ", iIteration, currResult);
        }

        if (res[POSITIVE + 'A'] > res[NEGATIVE + 'A'])
        {
            confidence = res[POSITIVE + 'A'] / (double) this->cascade.size();
            //this->logger->log->info("CSvmCascade::train()....final: %d", POSITIVE + 'A');
            return POSITIVE + 'A';
        }

        else
        {
            confidence = res[NEGATIVE + 'A'] / (double)(this->cascade.size());
            //this->logger->log->info("CSvmCascade::train()....final: %d", NEGATIVE + 'A');
            return NEGATIVE + 'A';
        }

    }
}

double CSvmCascade::verifyModel(std::map<int, std::vector<std::vector<double> > > set, std::map<int, std::vector<int> > &missClassified)
{
    logger->log->info("CSvmCascade::verifyModel....");
    assert(set.size() > 0);

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

    for (iterTest = set.begin(); iterTest != set.end(); ++iterTest)
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
                logger->log->info("CSvmCascade::verifyModel()...Error at %d test->%d (%d,%d)\n", iterTest->first + 'A', iterTestVec, errorCount[(int)(iterTest->first + 'A')], totalErrorCount);
            }
            totalCount[(int)(iterTest->first + 'A')]++;
            totalNumTest++;
        }
    }

    assert(totalCount[(POSITIVE + 'A')] + totalCount[(NEGATIVE + 'A')] == totalNumTest);
    assert(errorCount[(POSITIVE + 'A')] + errorCount[(NEGATIVE + 'A')] == totalErrorCount);

    double trainError = (double) totalErrorCount / (double) totalNumTest;
    logger->log->info("CSvmCascade::verifyModel()...Test Total Error %lf(%d/%d) \n", trainError, totalErrorCount, totalNumTest);

    for (errorCountIter = errorCount.begin(); errorCountIter != errorCount.end(); ++errorCountIter)
    {
        double error = (double) errorCountIter->second / (double) totalCount[errorCountIter->first];
        logger->log->info("CSvmCascade::verifyModel()...Test Error for label %d -> %lf (%d,%d)\n", errorCountIter->first, error, errorCountIter->second, totalCount[errorCountIter->first]);
    }

    return trainError;
}

std::vector<std::vector<double> > CSvmCascade::createSet(int label, std::vector<std::vector<double> > set, std::vector<int> idx, int minSize)
{
    std::vector<std::vector<double> > subSet;
    if (idx.size() == 0)
    {
        return subSet;
    }

    assert(set.size() > 0);

    if (label == POSITIVE)
    {
        for (unsigned int iterIdx = 0; iterIdx < idx.size(); ++iterIdx)
        {
            assert(idx[iterIdx] < (int)set.size());
            set.erase(set.begin() + idx[iterIdx]);
        }
        subSet = set;
    }
    if (label == NEGATIVE)
    {
        for (unsigned int iterIdx = 0; iterIdx < idx.size(); ++iterIdx)
        {
            assert(idx[iterIdx] < (int)set.size());
            subSet.push_back(set[idx[iterIdx]]);
        }

        //fill rest with negatives...
        if (subSet.size() < minSize)
        {
            boost::uniform_int<> exampleSet(0, set.size() - 1);
            boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
            do
            {
                int randIdx = getRandom();
                assert(randIdx >= 0 && randIdx < (int)set.size());
                subSet.push_back(set[randIdx]);
            }
            while (subSet.size() <= minSize);
        }

        //fill rest with missclassified negatives...
        /*if (subSet.size() < minSize)
         {
         boost::uniform_int<> exampleSet(0, idx.size() - 1);
         boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
         do
         {
         int randIdx = getRandom();
         assert(randIdx >=0 && randIdx < (int)idx.size());

         assert(idx[randIdx]<(int)set.size());
         subSet.push_back(set[idx[randIdx]]);
         } while (subSet.size() <= minSize);
         }*/
    }
    return subSet;
}

std::vector<std::vector<double> > CSvmCascade::createSet2(int label, std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, std::vector<int> idx, int minSize)
{
    std::vector<std::vector<double> > subSet;
    if (idx.size() == 0)
    {
        return subSet;
    }

    assert(trainSet.size() > 0);
    assert(testSet.size() > 0);
    assert(testSet.size() >= idx.size());

    this->logger->log->info("CSvmCascade::createSet2()....add misclassification");
    for (unsigned int iterIdx = 0; iterIdx < idx.size(); ++iterIdx)
    {
        assert(idx[iterIdx] < (int)testSet.size());
        trainSet.push_back(testSet[idx[iterIdx]]);
    }
    this->logger->log->info("CSvmCascade::createSet2()....add misclassification done");

    //fill rest with negatives...
    /*if (subSet.size() < minSize)
     {
     boost::uniform_int<> exampleSet(0, testSet.size() - 1);
     boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
     do
     {
     int randIdx = getRandom();
     assert(randIdx >=0 && randIdx < (int)testSet.size());
     trainSet.push_back(testSet[randIdx]);
     } while (trainSet.size() <= minSize);
     }*/

    //fill rest with missclassified negatives...

    if ((int) trainSet.size() < (int) minSize)
    {
        this->logger->log->info("CSvmCascade::createSet2()....adding to compensatate train set size %d < %d", trainSet.size(), minSize);
        boost::uniform_int<> exampleSet(0, idx.size() - 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
        do
        {
            int randIdx = getRandom();
            assert(randIdx >= 0 && randIdx < (int)idx.size());

            assert(idx[randIdx] < (int)testSet.size());
            trainSet.push_back(testSet[idx[randIdx]]);
        }
        while (trainSet.size() <= minSize);
        this->logger->log->info("CSvmCascade::createSet2()....adding to compensatate train set size done %d < %d", trainSet.size(), minSize);
    }

    subSet = trainSet;

    return subSet;
}

//create set out of only misclassified
std::vector<std::vector<double> > CSvmCascade::createSet3(int label, std::vector<std::vector<double> > trainSet, std::vector<std::vector<double> > testSet, std::vector<int> idx, int minSize)
{
    std::vector<std::vector<double> > subSet;
    if (idx.size() == 0)
    {
        return subSet;
    }

    assert(trainSet.size() > 0);
    assert(testSet.size() > 0);
    assert(testSet.size() >= idx.size());

    this->logger->log->info("CSvmCascade::createSet3()....add misclassification");
    for (unsigned int iterIdx = 0; iterIdx < idx.size(); ++iterIdx)
    {
        assert(idx[iterIdx] < (int)testSet.size());
        subSet.push_back(testSet[idx[iterIdx]]);
    }
    this->logger->log->info("CSvmCascade::createSet3()....add misclassification done");

    //fill rest with negatives...
    /*if (subSet.size() < minSize)
     {
     boost::uniform_int<> exampleSet(0, testSet.size() - 1);
     boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
     do
     {
     int randIdx = getRandom();
     assert(randIdx >=0 && randIdx < (int)testSet.size());
     trainSet.push_back(testSet[randIdx]);
     } while (trainSet.size() <= minSize);
     }*/

    //fill rest with missclassified negatives...

    if ((int) subSet.size() < (int) minSize)
    {
        this->logger->log->info("CSvmCascade::createSet2()....adding to compensatate train set size %d < %d", subSet.size(), minSize);
        boost::uniform_int<> exampleSet(0, idx.size() - 1);
        boost::variate_generator<boost::mt19937&, boost::uniform_int<> > getRandom(rng, exampleSet);
        do
        {
            int randIdx = getRandom();
            assert(randIdx >= 0 && randIdx < (int)idx.size());

            assert(idx[randIdx] < (int)testSet.size());
            subSet.push_back(testSet[idx[randIdx]]);
        }
        while (subSet.size() <= minSize);
        this->logger->log->info("CSvmCascade::createSet2()....adding to compensatate train set size done %d < %d", subSet.size(), minSize);
    }
    return subSet;
}

CSvmCascade::~CSvmCascade()
{
    // TODO Auto-generated destructor stub
}
