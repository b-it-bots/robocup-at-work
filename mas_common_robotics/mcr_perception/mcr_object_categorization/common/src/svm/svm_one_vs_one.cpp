/*
*  Created on: Mar 18, 2011
*      Author: Christian Mueller
*
*/


#include "svm/svm_one_vs_one.h"

CSvmOneVsOne::CSvmOneVsOne()
{
    this->gridSearchIteration = GRID_SEARCH_ITERATIONS;
}

CSvmOneVsOne::CSvmOneVsOne(COCSettings &ocSettings, string id, int gridSearchIteration) : COneVsOne(ocSettings, id)
{
    this->gridSearchIteration = gridSearchIteration;
}

void CSvmOneVsOne::initEvaluation()
{
    this->loadOneVsOneCfg();
    this->loadModel();
}

double CSvmOneVsOne::train()
{
    double error;
    char startLabel = 'A';
    int svmNumOfTrainingExamples = -1;

    this->createClassifierCombinations();
    this->svms.resize(this->numCombination);

    for (int iSvm = 0; iSvm < svms.size(); iSvm++)
    {
        this->svms.at(iSvm) =  CSvm(this->ocSettings);
        this->svms.at(iSvm).setGridSearchIterations(this->gridSearchIteration);
    }

    this->combinedTrainingExamples.resize(this->numCombination);
    this->combinedTestingExamples.resize(this->numCombination);
    this->combineTrainingSet();
    this->combineTestingSet();

    //FeatureSelection
    featureSelector.resize(this->svms.size());
    for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
    {
        featureSelector.at(iSvm) = new CEntropyScore(this->combinedTrainingExamples.at(iSvm));
        featureSelector.at(iSvm)->setBestofThreshold(this->featureSelectionBestofThreshold);
        //featureSelector.at(iSvm)= new CAdaScore(this->combinedTrainingExamples.at(iSvm));
        //featureSelector.at(iSvm)= new CFScore(this->combinedTrainingExamples.at(iSvm));
        //featureSelector.at(iSvm)= new CMiScore(this->combinedTrainingExamples.at(iSvm));
        featureSelector.at(iSvm)->computeFeatureSelection();
    }

    //Create Filter
    for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
    {
        featureSelector.at(iSvm)->createFeatureFilter(FILTER_BEST_OF);//2
    }

    for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
    {
        combinedTrainingExamples.at(iSvm) = featureSelector.at(iSvm)->filterFeatures(combinedTrainingExamples.at(iSvm));
        combinedTestingExamples.at(iSvm) = featureSelector.at(iSvm)->filterFeatures(combinedTestingExamples.at(iSvm));
    }

    for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
    {
        cout << "SVM number: " << iSvm << endl;
        cout << (char)(this->classifierCombinations.at(iSvm)[0] + startLabel) << " vs. " << (char)(this->classifierCombinations.at(iSvm)[1] + startLabel) << endl;
        this->classifierAccuracy.at(iSvm) = this->svms.at(iSvm).parameterGridSearch(combinedTrainingExamples.at(iSvm), combinedTestingExamples.at(iSvm), svmNumOfTrainingExamples, true, false, this->id, iSvm);
        this->svms.at(iSvm).saveSvmModel(id, iSvm);
    }

    this->saveOneVsOneCfg();

    error =  this->evaluate(this->testingExamples);
    cout << "Complete Test error: " << error << endl;

    return error;
}

void CSvmOneVsOne::combineTrainingSet()
{
    char startLabel = 'A';
    int idxClass0 = 0;
    int idxClass1 = 1;
    int idxLabel = 0;
    char class0Label;
    char class1Label;
    char convertClass0ToClassALabel = 'A';
    char convertClass1ToClassBLabel = 'B';

    for (int iSvm = 0; iSvm < this->numCombination; iSvm++)
    {

        class0Label = this->classifierCombinations.at(iSvm)[idxClass0];
        class1Label = this->classifierCombinations.at(iSvm)[idxClass1];

        for (int iExample = 0; (iExample < this->sortedTrainingExamples.at(class0Label).size()) || (iExample < this->sortedTrainingExamples.at(class1Label).size()); iExample++)
        {
            //this->trainingExamples.size();  )

            if (iExample < this->sortedTrainingExamples.at(class0Label).size())
            {
                this->combinedTrainingExamples.at(iSvm).push_back(this->sortedTrainingExamples.at(class0Label).at(iExample));
                this->combinedTrainingExamples.at(iSvm).back()[idxLabel] = convertClass0ToClassALabel;
            }

            if (iExample < this->sortedTrainingExamples.at(class1Label).size())
            {
                this->combinedTrainingExamples.at(iSvm).push_back(this->sortedTrainingExamples.at(class1Label).at(iExample));
                this->combinedTrainingExamples.at(iSvm).back()[idxLabel] = convertClass1ToClassBLabel;
            }

        }
    }
}

//TODO put combineTestSet in oneVs one?!
void CSvmOneVsOne::combineTestingSet()
{
    char startLabel = 'A';
    int idxClass0 = 0;
    int idxClass1 = 1;
    int idxLabel = 0;
    char class0Label;
    char class1Label;
    char convertClass0ToClassALabel = 'A';
    char convertClass1ToClassBLabel = 'B';

    for (int iSvm = 0; iSvm < this->numCombination; iSvm++)
    {

        class0Label = this->classifierCombinations.at(iSvm)[idxClass0];
        class1Label = this->classifierCombinations.at(iSvm)[idxClass1];

        for (int iExample = 0; (iExample < this->sortedTestingExamples.at(class0Label).size()) || (iExample < this->sortedTestingExamples.at(class1Label).size()); iExample++)
        {
            //this->trainingExamples.size();  )

            if (iExample < this->sortedTestingExamples.at(class0Label).size())
            {
                this->combinedTestingExamples.at(iSvm).push_back(this->sortedTestingExamples.at(class0Label).at(iExample));
                this->combinedTestingExamples.at(iSvm).back()[idxLabel] = convertClass0ToClassALabel;
            }

            if (iExample < this->sortedTestingExamples.at(class1Label).size())
            {
                this->combinedTestingExamples.at(iSvm).push_back(this->sortedTestingExamples.at(class1Label).at(iExample));
                this->combinedTestingExamples.at(iSvm).back()[idxLabel] = convertClass1ToClassBLabel;
            }

        }
    }
}

void CSvmOneVsOne::saveOneVsOneCfg()
{
    fstream out;
    out.open(string(ocSettings.path +
                    ocSettings.folderTraining +
                    ocSettings.trainingSvmOneVsOneFileNameCfg +
                    id +
                    ocSettings.currentEvalNumber +
                    ocSettings.dotCfg).c_str(), ios::out | ios::binary);

    if (out.is_open())
    {
        out << this->numK << endl
            << this->numClasses << endl
            << this->featureSelectionBestofThreshold << endl
            << this->featureSelector.at(0)->getFeatureVectorDim() << endl
            << this->numCombination << endl;

        for (int iSvm = 0; iSvm < this->classifierAccuracy.size(); iSvm++)
        {
            out << this->classifierAccuracy.at(iSvm) << " ";
        }
        out << endl;

        for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
            for (int iK = 0; iK < this->numK; iK++)
                out << this->classifierCombinations.at(iSvm)[iK] << " ";
        out << endl;

        for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
        {
            for (int iFeature = 0; iFeature < this->featureSelector.at(iSvm)->getRankedFeatures().size(); iFeature++)
            {
                out << this->featureSelector.at(iSvm)->getRankedFeatures()[iFeature] << " ";
            }
            out << endl;
        }

        for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
        {
            out << this->featureSelector.at(iSvm)->getActiveFilter() << endl;
            for (int iFeature = 0; iFeature < this->featureSelector.at(iSvm)->getRankedFeatures().size(); iFeature++)
            {
                out << this->featureSelector.at(iSvm)->getActiveFeatures()[iFeature] << " ";
            }
            out << endl;
        }
        cout << "*One Vs. One Configuration saved: " << string(ocSettings.path +
                ocSettings.folderTraining +
                ocSettings.trainingSvmOneVsOneFileNameCfg +
                id +
                ocSettings.currentEvalNumber +
                ocSettings.dotCfg).c_str() << endl;
        out.close();
    }
    else
        cout << "*Could not write training Svm OneVsOne Cfg" << endl;

}

void CSvmOneVsOne::loadOneVsOneCfg()
{
    fstream in;
    int value = 0;
    bool bvalue = false;
    int featureVectorDim;
    in.open(string(ocSettings.path +
                   ocSettings.folderTraining +
                   ocSettings.trainingSvmOneVsOneFileNameCfg +
                   this->id +
                   ocSettings.currentEvalNumber +
                   ocSettings.dotCfg).c_str(), ios::in | ios::binary);

    if (in.is_open())
    {
        in >> this->numK;
        in >> this->numClasses;
        in >> this->featureSelectionBestofThreshold;
        in >> featureVectorDim;
        in >> this->numCombination;

        this->classifierAccuracy.clear();
        this->classifierAccuracy.resize(this->numCombination);

        for (int iSvm = 0; iSvm < this->numCombination; iSvm++)
        {
            in >> this->classifierAccuracy.at(iSvm);
        }

        this->classifierCombinations.clear();
        this->classifierCombinations.resize(this->numCombination);

        for (int iComb = 0; iComb < this->numCombination; iComb++)
        {
            this->classifierCombinations.at(iComb) = (int*)malloc(sizeof(int) * this->numK);
            for (int iK = 0; iK < this->numK; iK++)
            {
                in >> this->classifierCombinations.at(iComb)[iK];
            }
        }

        featureSelector.clear();
        featureSelector.resize(this->numCombination);
        vector<int> tmpRankedFeatures;
        for (int iSvm = 0; iSvm < this->numCombination; iSvm++)
        {
            featureSelector.at(iSvm) = new CEntropyScore();
            featureSelector.at(iSvm)->setBestofThreshold(this->featureSelectionBestofThreshold);
            featureSelector.at(iSvm)->setFeatureVectorDim(featureVectorDim);
            tmpRankedFeatures.clear();
            for (int iFeature = 0; iFeature < featureVectorDim; iFeature++)
            {
                in >> value;
                tmpRankedFeatures.push_back(value);
            }
            this->featureSelector.at(iSvm)->setRankedFeatures(tmpRankedFeatures);

        }
        vector<bool> tmpActiveFeatures;
        for (int iSvm = 0; iSvm < this->numCombination; iSvm++)
        {
            tmpActiveFeatures.clear();
            in >> value;
            this->featureSelector.at(iSvm)->setActiveFilter(value);
            for (int iFeature = 0; iFeature < featureVectorDim; iFeature++)
            {
                in >> bvalue;
                tmpActiveFeatures.push_back(bvalue);
            }
            this->featureSelector.at(iSvm)->setActiveFeatures(tmpActiveFeatures);

        }

        cout << "*One Vs. One Configuration loaded: " << string(ocSettings.path +
                ocSettings.folderTraining +
                ocSettings.trainingSvmOneVsOneFileNameCfg +
                this->id +
                ocSettings.currentEvalNumber +
                ocSettings.dotCfg).c_str() << endl;

        in.close();
    }
    else
        cout << "*Could not write training Svm OneVsOne Cfg" << endl;

}

void CSvmOneVsOne::loadModel()
{
    if (this->numClasses > 1 && this->numCombination > 0)
    {
        this->svms.clear();
        this->svms.resize(this->numCombination);

        for (int iSvm = 0; iSvm < svms.size(); iSvm++)
        {
            this->svms.at(iSvm) =  CSvm(this->ocSettings);
            this->svms.at(iSvm).loadSvmModel(this->id, iSvm);
        }
    }
    else
        cout << "*Could not read SVM one vs one models" << endl;
}

char CSvmOneVsOne::predict(std::vector<double> testExample, bool isLabeled, double* prob)
{
    char result;
    char startLabel = 'A';

    if (!isLabeled)
    {
        testExample.insert(testExample.begin(), 0);
        isLabeled = true;
    }
    int maxVots = this->numCombination - 1;

    std::vector<int> majorityVotingResult;
    std::vector<char> labelResults;

    majorityVotingResult.resize(this->numClasses);
    labelResults.resize(this->svms.size());

    std::vector< vector<double> > probabilityResults;

    if (this->svms.at(0).isProbability())
        probabilityResults.resize(this->numClasses);


    std::vector<double> filteredTestExample;

    for (int iSvm = 0; iSvm < this->svms.size(); iSvm++)
    {
        double **prob_estimates = (double**)malloc(sizeof(double*));
        *prob_estimates = NULL;
        filteredTestExample.clear();
        filteredTestExample = this->featureSelector.at(iSvm)->filterFeatures(testExample);
        labelResults.at(iSvm) = this->svms.at(iSvm).predict(filteredTestExample, prob_estimates, isLabeled);
        majorityVotingResult.at(this->classifierCombinations.at(iSvm)[labelResults.at(iSvm) - startLabel])++;

        if (*prob_estimates != NULL)
        {
            if (DEBUG)
                cout << (char)(this->classifierCombinations.at(iSvm)[labelResults.at(iSvm) - startLabel] + startLabel) << ": " << prob_estimates[0][labelResults.at(iSvm) - startLabel] << endl; //<<" "<< prob_estimates[0][0]<<" "<<prob_estimates[0][1]<<endl;
            //probabilityResults.at(this->classifierCombinations.at(iSvm)[labelResults.at(iSvm)-startLabel]).push_back(prob_estimates[0][labelResults.at(iSvm)-startLabel]);
            probabilityResults.at(this->classifierCombinations.at(iSvm)['A' - startLabel]).push_back((prob_estimates[0]['A' - startLabel]) * (1 - this->classifierAccuracy.at(iSvm)));
            probabilityResults.at(this->classifierCombinations.at(iSvm)['B' - startLabel]).push_back((prob_estimates[0]['B' - startLabel]) * (1 - this->classifierAccuracy.at(iSvm)));

        }
        free(*prob_estimates);
        free(prob_estimates);

    }

    int maxVotsResult = 0;
    int idxMaxVots = -1;
    for (int iResult = 0; iResult < this->numClasses; iResult++)
    {
        if (majorityVotingResult.at(iResult) > maxVotsResult)
        {
            maxVotsResult = majorityVotingResult.at(iResult);
            idxMaxVots =  iResult;
        }
    }

    //check for tie!
    vector<int> tieClasses;
    for (int iResult = 0; iResult < this->numClasses; iResult++)
    {
        if (majorityVotingResult.at(iResult) == maxVotsResult)
        {
            tieClasses.push_back(iResult);
        }
    }

    if (tieClasses.size() > 1)
    {
        if (DEBUG)
            cout << "A tie has occurred...";
        if (probabilityResults.size() > 0)
        {
            if (DEBUG)
                cout << "Max Probability ...";
            double maxProbability = 0;
            int maxProbabilityClass = -1;
            vector<double> totalClassProbability;
            totalClassProbability.resize(this->numClasses);
            for (int iClass = 0; iClass < tieClasses.size(); iClass++)
            {
                for (int iResult = 0; iResult < probabilityResults.at(tieClasses.at(iClass)).size(); iResult++)
                {
                    if (iResult == 0)
                        totalClassProbability.at(iClass) = probabilityResults.at(tieClasses.at(iClass)).at(iResult);
                    else
                        totalClassProbability.at(iClass) += probabilityResults.at(tieClasses.at(iClass)).at(iResult);
                }
            }

            for (int iClass = 0; iClass < totalClassProbability.size(); iClass++)
            {
                if (totalClassProbability.at(iClass) > maxProbability)
                {
                    maxProbability = totalClassProbability.at(iClass);
                    maxProbabilityClass = iClass;
                }
            }
            idxMaxVots = maxProbabilityClass;

        }
        else
        {
            srand(time(NULL));
            idxMaxVots = rand() % tieClasses.size();
            if (DEBUG)
                cout << "Random guessing...";
        }
    }

    double probResultOfFinalLabelResult = -1;
    for (int iResult = 0; iResult < probabilityResults.at(idxMaxVots).size(); iResult++)
    {
        if (iResult == 0)
        {
            probResultOfFinalLabelResult = probabilityResults.at(idxMaxVots).at(iResult);
        }
        else
        {
            probResultOfFinalLabelResult += probabilityResults.at(idxMaxVots).at(iResult);
        }
    }
    //
    if (prob != NULL && this->svms.at(0).isProbability())
        *prob = (probResultOfFinalLabelResult / probabilityResults.at(idxMaxVots).size());
    char finalVotingResult = (idxMaxVots + startLabel);
    if (DEBUG)
        cout << "Result: " << finalVotingResult << " -- Confidence by Vots" << ((double)maxVotsResult / (double)maxVots) << ", by Probability " << probResultOfFinalLabelResult << endl;
    return finalVotingResult;
}

double CSvmOneVsOne::evaluate(std::vector< vector<double> > testingExamples)
{
    int idxLabel = 0;
    char expectedLabel;
    vector<int> testResult;

    this->loadOneVsOneCfg();
    this->loadModel();
    for (int iTest = 0; iTest < testingExamples.size(); iTest++)
    {
        expectedLabel = testingExamples.at(iTest).at(idxLabel);
        testResult.push_back(this->predict(testingExamples.at(iTest), true) == expectedLabel ? 0 : 1);
    }

    return toolBox.averageErrorRate(testResult);

}
