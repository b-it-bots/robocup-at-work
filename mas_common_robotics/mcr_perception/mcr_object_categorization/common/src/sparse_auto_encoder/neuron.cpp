/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */


#include <map>
#include <ctime>
#include <cmath>
#include <assert.h>
#include <climits>

#include "logger.h"
#include "file_settings.h"
#include "sparse_auto_encoder/neuron.h"
#include "sparse_auto_encoder/sae_tool_box.h"

CNeuron::CNeuron()
{
    this->logger = &CLogger::getInstance();
    this->sparsity = 0.0f;
}

CNeuron::CNeuron(int numInputs, double bias, double sparseParameter)
{
    this->logger = &CLogger::getInstance();
    this->numInputs = numInputs;
    this->bias = bias;
    this->sparseParameter = sparseParameter;
    this->sparsity = 0.0f;
}


float randomFloat(float min, float max)
{
    assert(max > min);
    float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;
    return (random * range) + min;
}



void CNeuron::init()
{
    // logger->log->debug("CNeuron::init...");
    assert(this->numInputs > 0);
    weightVector.clear();
    weightVector.resize(this->numInputs);
    for (unsigned int i = 0 ; i < this->numInputs; ++i)
    {
        //weightVector.push_back(1);
        //Random...between 0 1
        //weightVector.push_back( (float) rand()/RAND_MAX);
        weightVector[i] = ((float) randomFloat(0.0, 0.2));
        //  std::cout<<" "<<weightVector.back();
    }
}

void CNeuron::init(std::vector<double> mean, std::vector<double> variance)
{
    assert(this->numInputs > 0);
    assert(mean.size() == this->numInputs);
    assert(variance.size() == this->numInputs);

    //std::cout<<"init CNeuron...mean";
    // CFileSettings::coutStdVector(mean,true);
    // std::cout<<"init CNeuron...var";
    // CFileSettings::coutStdVector(variance,true);
    //logger->log->debug("CNeuron::init...");
    weightVector.clear();
    weightVector.resize(this->numInputs);
    for (unsigned int i = 0 ; i < this->numInputs; ++i)
    {
        weightVector[i] = (CSAEToolBox::randDouble(mean[i] - sqrt(variance[i]), mean[i] + sqrt(variance[i])));
    }
}

void CNeuron::input(std::vector<double> inputVector)
{
    assert(inputVector.size() == weightVector.size());
    this->inputVector = inputVector;
    this->response = std::numeric_limits<double>::epsilon();
}


double CNeuron::fire(std::vector<double> inputVector)
{
    this->input(inputVector);
    assert(this->inputVector.size() == this->numInputs);
    this->response = this->activationFunctionTanh();
    //this->response = this->activationFunctionSig();
    this->sparsity = this->sparsity * (1 - this->sparseParameter) + this->sparseParameter * this->response;

    logger->log->debug("CNeuron::fire...Response=%lf Sparsity=%lf Bias=%lf", this->response, this->sparsity, this->bias);
    inputVector.clear();
    return response;
}

double CNeuron::fire()
{
    assert(this->inputVector.size() == this->numInputs);
    return fire(this->inputVector);
}
double CNeuron::activationFunctionTanh()
{
    double sum = 0.0;
    assert(this->inputVector.size() == this->weightVector.size());

    for (unsigned int iterInput = 0; iterInput < this->numInputs; ++iterInput)
    {
        sum += this->weightVector[iterInput] * this->inputVector[iterInput] + this->bias;
        //   this->logger->log->debug("sum %lf",sum);
    }


    return tanh(sum);
}


double CNeuron::activationFunctionSig()
{
    double sum = 0.0;
    assert(this->inputVector.size() == this->weightVector.size());

    for (unsigned int iterInput = 0; iterInput < this->numInputs; ++iterInput)
    {
        sum += this->weightVector[iterInput] * this->inputVector[iterInput];
        //  this->logger->log->debug("sum %lf",sum);
    }
    sum += this->bias;

    return (1.0) / (1.0 + exp(-1.0 * sum));
}

CNeuron::~CNeuron() {}
