/*
 * CPNeuralNetwork.cpp
 *
 *  Created on: Apr 22, 2011
 *      Author:  Christian Mueller
 */


#include <climits>

#include "prob_neural_network/p_neural_network.h"
#include "file_settings.h"


#define PI 3.14159265

CPNeuralNetwork::CPNeuralNetwork()
{

    // TODO Auto-generated constructor stub
    this->stdDeviation = 0.05;/////0.05f;//0.05 is STD!!! // worked fine 0.2f; //smoothness parameter!
    this->trainTestSetRatio = -1;
    this->logger = &CLogger::getInstance();
    this->logger->log->debug("CPNeuralNetwork::CPNeuralNetwork ...stdDeviation = %lf\n", this->stdDeviation);
    //JS distance measure:
    //0.3 ok
    //0.2 better than 0.3
    //0.05 even better

    //Eucl distance measure Distance:
    //0.3f
    //0.5 real numbers like 0.8 cup 0.4 can...
}

void CPNeuralNetwork::addModel(std::map<int, std::map<int, std::vector<double> > > &model)
{
    this->logger->log->debug("CPNeuralNetwork::addModel ...\n");
    if (model.size() > 0)
    {
        this->model = model;
    }
    else
    {
        this->logger->log->warn("...but empty\n");
    }
}

void CPNeuralNetwork::addModel(int label, std::map<int, std::vector<double> > &model)
{
    this->logger->log->debug("CPNeuralNetwork::addModel ...\n");
    if (model.size() > 0)
    {
        this->model[label] = model;
    }
    else
    {
        this->logger->log->warn("...but empty\n");
    }
}

void CPNeuralNetwork::init(float trainTestSetRatio)
{
    std::cout << "CPNeuralNetwork::init()..." << std::endl;
    this->trainTestSetRatio = trainTestSetRatio;

    if (this->model.size() == 0)
        return;

    if (this->trainTestSetRatio < (0 + std::numeric_limits<float>::epsilon()))
    {
        this->trainModel = this->model;
        this->logger->log->debug("CPNeuralNetwork::init...complete examples used as trainExamples\n");
    }
    else
    {
        std::map<int, std::map<int, std::vector<double> > >::iterator iterModel;
        std::map<int, std::vector<double> >::iterator iterPattern;

        for (iterModel = this->model.begin(); iterModel != this->model.end(); ++iterModel)
        {
            std::map<int, std::vector<double> > pattern;
            pattern = iterModel->second;

            int numberOfTrainExamples = (int) ceil(float((float) pattern.size() * this->trainTestSetRatio));
            if (numberOfTrainExamples <= 0)
            {
                this->trainModel = this->model;
                this->logger->log->debug("CPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterModel->first].c_str(),
                                         this->trainModel[iterModel->first].size(), this->model[iterModel->first].size());

            }
            else //create Train and test examples
            {
                int currNumberofTrainExamples = 0;
                for (iterPattern = pattern.begin(); iterPattern != pattern.end(); ++iterPattern)
                {
                    if (currNumberofTrainExamples < numberOfTrainExamples)
                    {
                        this->trainModel[iterModel->first][iterPattern->first] = iterPattern->second;
                    }
                    else
                    {
                        this->testModel[iterModel->first][iterPattern->first] = iterPattern->second;
                    }
                    currNumberofTrainExamples++;
                }
                this->logger->log->debug("CPNeuralNetwork::initHierarchy... %s : complete %d of %d examples added\n", CFileSettings::labels[iterModel->first].c_str(),
                                         this->trainModel[iterModel->first].size(), this->model[iterModel->first].size());
            }
        }

    }
}

/* network consists of neurons
 * neurons consists of submodels
 */
std::pair<int, double> CPNeuralNetwork::evaluate(std::map<int, std::map<int, std::vector<double> > > query)
{
    std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons;
    std::map<int, std::vector<double> >::iterator iterPattern;
    this->logger->log->debug(" CPNeuralNetwork::evaluate....(TrainModelSize=%d)\n", trainModel.size());

    //neuron, responseValue;
    //std::map<int, double> response;

    std::pair<int, double> maxResponse;
    maxResponse.first = 0;
    maxResponse.second = 0;

    if (trainModel.size() == 0)
    {
        this->logger->log->error("CPNeuralNetwork::evaluate...TrainModel empty!!=> nothing to compare with!\n");
    }

    for (iterNeurons = trainModel.begin(); iterNeurons != trainModel.end(); ++iterNeurons)
    {
        std::pair<int, double> response;
        response.first = iterNeurons->first;
        std::map<int, std::vector<double> > modelPattern = iterNeurons->second;
        std::map<int, std::vector<double> > queryPattern = query[iterNeurons->first];
        //response[iterNeurons->first] = computeResponse(queryPattern,
        //  modelPattern);
        response.second = computeResponse(queryPattern, modelPattern);

        this->logger->log->info("CPNeuralNetwork::evaluate...Final response to label %d = %lf\n", response.first, response.second);

        if (response.second > maxResponse.second)
        {
            maxResponse = response;
        }
    }

    return maxResponse;
}

std::pair<int, double> CPNeuralNetwork::evaluate(std::map<int, std::map<int, std::vector<double> > > query, int labelToCompareWith)
{
    std::pair<int, double> response;

    if (query.find(labelToCompareWith) == query.end() || this->trainModel.find(labelToCompareWith) == this->trainModel.end())
    {
        this->logger->log->error("CPNeuralNetwork::evaluate...No such Label to compare with\n");
        response.first = -1;
        response.second = -1;
        return response;
    }

    response.first = labelToCompareWith;
    response.second = computeResponse(query[labelToCompareWith], this->trainModel[labelToCompareWith]);

    return response;
}

double CPNeuralNetwork::computeResponse(std::map<int, std::vector<double> > queryPattern, std::map<int, std::vector<double> > modelPattern)
{
    //std::cout << " CPNeuralNetwork::computeResponse ....\n";
    std::map<int, std::vector<double> >::iterator iterPatterns;

    double totalResponse = 0.0;
    //std::cout << "   Pattern responses:\n";
    for (iterPatterns = modelPattern.begin(); iterPatterns != modelPattern.end(); ++iterPatterns)
    {
        double response;

        //Eucl
        //response = this->kernelGaussian(getEuclideanDistance(
        //      iterPatterns->second, queryPattern[iterPatterns->first]), this->stdDeviation);

        //JS
        //std::cout<<"getJensenShannonDivergence"<<iterPatterns->first<<std::endl;
        response = this->kernelGaussian(getJensenShannonDivergence(this->normalizeHist(iterPatterns->second), this->normalizeHist(queryPattern[iterPatterns->first])), this->stdDeviation);

        //  CFileSettings::coutStdVector(queryPattern[iterPatterns->first],true);
        //std::cout << "   Pattern " << iterPatterns->first << " = " << response
        //<< "\n";
        totalResponse += response;
    }

    return ((double) totalResponse / (double) modelPattern.size());
}
double CPNeuralNetwork::kernelGaussian(double u, double stdDerivation)
{
    /*this->kernelGaussian(((query[i] - dd[d]) / h));*/
    //return (1.0f / sqrt(2.0f * PI)) * exp((-0.5f) * pow(u, 2));

    return exp((double) pow(u, 2.0) / (-2.0 * pow(stdDerivation, 2.0)));
}

double CPNeuralNetwork::mean(std::vector<double> histo)
{
    double mean = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        mean += histo[k];
    }

    //  std::cout << "histo Size()" << histo.size() << "\n";
    return (mean / (double) histo.size());
}

double CPNeuralNetwork::variance(std::vector<double> histo, double mean)
{
    double var = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        var += pow((histo[k] - mean), 2);
    }
    return (var / (double) histo.size());
}

double CPNeuralNetwork::getKLD(std::vector<double> a, std::vector<double> b)
{
    double kld = 0;

    for (unsigned int m = 0; m < a.size(); m++)
    {
        if (a[m] == 0)
        {
            a[m] = std::numeric_limits<float>::epsilon();
        }
        if (b[m] == 0)
        {
            b[m] = std::numeric_limits<float>::epsilon();
        }

        kld += a[m] * this->log_2((a[m] / b[m]));
    }

    return kld;
}

double CPNeuralNetwork::getEuclideanDistance(std::vector<double> a, std::vector<double> b)
{

    std::vector<double> delta;
    double deltaSum = 0;

    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (unsigned int m = 0; m < a.size(); m++)
        {
            delta.at(m) = (a.at(m) - b.at(m));
        }

        for (unsigned int m = 0; m < a.size(); m++)
        {

            deltaSum += (pow(delta.at(m), 2));
        }

        return sqrt(deltaSum);
    }
    else
    {
        this->logger->log->warn("CPNeuralNetwork::getEuclideanDistance => Feature Vector length different %d != %d !", a.size(), b.size());
        return -1;
    }
}

double CPNeuralNetwork::getJensenShannonDivergence(std::vector<double> a, std::vector<double> b)
{

    std::vector<double> delta;

    if (a.size() == b.size())
    {

        delta.resize(a.size());
        for (unsigned int m = 0; m < a.size(); m++)
        {
            delta[m] = ((a[m] + b[m]) / 2.0f);
        }

        double firstTerm = this->diffEntropy(delta);
        double aVal, bVal;

        aVal = this->diffEntropy(a);

        bVal = this->diffEntropy(b);

        //std::cout << "aVal " << aVal << "          " << "bVal " << bVal << "\n";
        double secondTerm = (aVal + bVal) / 2.0f;

        return (firstTerm - secondTerm);
    }
    else
    {
        this->logger->log->warn("CPNeuralNetwork::getJensenShannonDivergence => Feature Vector length different %d != %d !", a.size(), b.size());
        return -1;
    }
}

double CPNeuralNetwork::diffEntropy(std::vector<double> a)
{
    double entropy = 0;
    for (unsigned int m = 0; m < a.size(); m++)
    {
        double res;
        res = a[m] * log_2(a[m]);
        //std::cout<<"entropy"<< a[m]<<" "<< entropy<<"\n";
        if (res != res) // check for nan
        {
            //std::cout<<" Y "<<std::endl;
            res = 0;
        }
        entropy += res;
    }

    return (entropy * (-1));
}

double CPNeuralNetwork::getCosineDistance(std::vector<double> a, std::vector<double> b)
{

    double nominator = 0.0f;
    double aProduct = 0.0f;
    double bProduct = 0.0f;
    double denominator = 0.0f;

    if (a.size() == b.size())
    {
        for (unsigned int m = 0; m < a.size(); m++)
        {
            nominator += (a[m] * b[m]);
        }

        for (unsigned int m = 0; m < a.size(); m++)
        {

            aProduct += pow(a.at(m), 2);
            bProduct += pow(b.at(m), 2);
        }
        denominator = (sqrt(aProduct) * sqrt(bProduct));
        //  std::cout<<"Cosine..."<<nominator << "/"<<denominator<<"="<<(nominator / denominator)<<std::endl;
        if (denominator == 0)
        {
            //  std::cout << "!!! DIV/0" << std::endl;
            denominator = std::numeric_limits<double>::epsilon();
        }

        return (1 - (nominator / denominator));
    }
    else
    {
        this->logger->log->warn("Something is wrong with the vector length");
        return -1;
    }
}

std::vector<double> CPNeuralNetwork::normalizeHist(std::vector<double> histo)
{
    //Normalize histogram to 1 !
    if (histo.size() > 0)
    {
        double sum = 0;
        for (unsigned k = 0; k < (unsigned) histo.size(); k++)
        {
            sum += histo[k];
        }

        if (sum > 0)
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = (double) histo[k] / (double) sum;
            }
        }
        else
        {
            for (unsigned k = 0; k < (unsigned) histo.size(); k++)
            {
                histo[k] = 0;
            }
        }
    }
    else
    {
        histo.clear();
        histo.push_back(0);
    }

    return histo;
}

void CPNeuralNetwork::verifyPNN(std::map<int, CObjectDecomposition> objects)
{
    this->logger->log->info("CPNeuralNetwork::verify init...\n");
    if (this->model.size() == 0)
    {
        return;
    }

    std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons;
    std::map<int, std::vector<double> >::iterator iterPattern;
    std::map<int, CObjectDecomposition>::iterator iDecomp;

    //label, sum
    std::map<int, int> correctAll;
    //label, sum
    std::map<int, int> incorrectAll;

    std::map<int, int>::iterator iterResp;

    int correct = 0;
    int incorrect = 0;
    int total = 0;

    for (iDecomp = objects.begin(); iDecomp != objects.end(); ++iDecomp)
    {

        std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > & objDesc = iDecomp->second.getObjects();
        for (unsigned int iterObjDesc = 0; iterObjDesc < objDesc.size(); ++iterObjDesc)
        {
            if (iDecomp->first == this->evaluate(objDesc[iterObjDesc].estimatedRelatedModels).first)
            {
                this->logger->log->info("CPNeuralNetwork::verifyl...Correct(%s)\n", CFileSettings::labels[iDecomp->first].c_str());
                correct++;
                correctAll[iDecomp->first]++;
            }
            else
            {
                this->logger->log->info("CPNeuralNetwork::verifyl...Oh Oh(%s)\n", CFileSettings::labels[iDecomp->first].c_str());
                incorrect++;
                incorrectAll[iDecomp->first]++;
            }
            total++;
        }

    }
    this->logger->log->info("CPNeuralNetwork::verify...Total Correct: %lf Incorrect: %lf\n", (double) correct / (double) total, (double) incorrect / (double) total);
    for (iterResp = correctAll.begin(); iterResp != correctAll.end(); ++iterResp)
    {
        this->logger->log->info("CPNeuralNetwork::verify...Label %s Correct: %lf\n", CFileSettings::labels[iterResp->first].c_str(),
                                (double) correctAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]));
    }

    for (iterResp = incorrectAll.begin(); iterResp != incorrectAll.end(); ++iterResp)
    {
        this->logger->log->info("CPNeuralNetwork::verify...Label %s Incorrect: %lf\n ", CFileSettings::labels[iterResp->first].c_str(),
                                (double) incorrectAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]));
    }
}

void CPNeuralNetwork::verifyPNN()
{
    this->logger->log->info("CPNeuralNetwork::verifyHPNN....\n");
    if (this->model.size() == 0)
    {
        return;
    }

    if (this->trainModel.size() > 0)
    {
        this->logger->log->info("CPNeuralNetwork::verifyPNN...TrainData:\n");
        this->verifyPNNModel(this->trainModel);
    }
    if (this->testModel.size() > 0)
    {
        this->logger->log->info("CPNeuralNetwork::verifyPNN...TestData:\n");
        this->verifyPNNModel(this->testModel);
    }
}

void CPNeuralNetwork::verifyPNNModel(std::map<int, std::map<int, std::vector<double> > > toVerifyModel)
{
    /*this->logger->log->info("verifyPNNModel::verify init...\n");
     if (toVerifyModel.size() == 0)
     {
     return;
     }

     std::map<int, std::map<int, std::vector<double> > >::iterator iterNeurons;
     std::map<int, std::vector<double> >::iterator iterPattern;

     //label, sum
     std::map<int, int> correctAll;
     //label, sum
     std::map<int, int> incorrectAll;

     std::map<int, int>::iterator iterResp;

     int correct = 0;
     int incorrect = 0;
     int total = 0;

     for (iterNeurons = toVerifyModel.begin(); iterNeurons != toVerifyModel.end(); ++iterNeurons)
     {

     for (unsigned int iterObjDesc = 0; iterObjDesc < objDesc.size(); ++iterObjDesc)
     {
     if (iDecomp->first == this->evaluate(
     objDesc[iterObjDesc].estimatedRelatedModels).first)
     {
     this->logger->log->info("Correct(%d)\n", iDecomp->first);
     correct++;
     correctAll[iDecomp->first]++;
     }
     else
     {
     this->logger->log->info("Oh Oh(%d)\n", iDecomp->first);
     incorrect++;
     incorrectAll[iDecomp->first]++;
     }
     total++;
     }

     }
     this->logger->log->info(
     "CPNeuralNetwork::verify...Total Correct: %lf Incorrect: %lf\n",
     (double) correct / (double) total, (double) incorrect
     / (double) total);
     for (iterResp = correctAll.begin(); iterResp != correctAll.end(); ++iterResp)
     {
     this->logger->log->info(
     "CPNeuralNetwork::verify...Label %s Correct: %lf\n",
     CFileSettings::labels[iterResp->first].c_str(),
     (double) correctAll[iterResp->first]
     / (double) ((double) correctAll[iterResp->first]
     + (double) incorrectAll[iterResp->first]));
     }

     for (iterResp = incorrectAll.begin(); iterResp != incorrectAll.end(); ++iterResp)
     {
     this->logger->log->info(
     "CPNeuralNetwork::verify...Label %s Incorrect: %lf\n ",
     CFileSettings::labels[iterResp->first].c_str(),
     (double) incorrectAll[iterResp->first]
     / (double) ((double) correctAll[iterResp->first]
     + (double) incorrectAll[iterResp->first]));
     }
     */
}

double CPNeuralNetwork::log_2(double n)
{
    // log(n)/log(2) is log2.
    return log((double) n) / log((double) 2);
}

CPNeuralNetwork::~CPNeuralNetwork()
{
    // TODO Auto-generated destructor stub
}
