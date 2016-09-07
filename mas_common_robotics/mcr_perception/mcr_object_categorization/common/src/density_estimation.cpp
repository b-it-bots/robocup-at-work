/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */



#include<iostream>
#include<cstdlib>
#include<algorithm>
#include<cmath>
#include<map>
#include<limits>
#include<fstream>
#include<stdio.h>
#include<numeric>

#include<climits>
#include<cmath>
#include<assert.h>

#include "density_estimation.h"

CDensityEstimation::CDensityEstimation()
{
    // TODO Auto-generated constructor stub
    this->logger = &CLogger::getInstance();
    this->optimizeBinSizeMethod = CDE_OPT_SIZE_METHODE;
    this->isInitEstimation = false;
    this->isInitOptBin = false;
    this->isCommonBinSize = false;
}

CDensityEstimation::CDensityEstimation(int optBinSizeMethod)
{
    // TODO Auto-generated constructor stub
    this->logger = &CLogger::getInstance();
    this->optimizeBinSizeMethod = optBinSizeMethod;
    this->isInitEstimation = false;
    this->isInitOptBin = false;
    this->isCommonBinSize = false;
}

void CDensityEstimation::setManualInit(int concept, bool isInit)
{
    this->logger->log->debug("!!CDensityEstimation::manualSetInit!!\n");
    if (concept == 1)
    {
        this->isInitEstimation = isInit;
        this->isInitOptBin = isInit;
    }
}

void CDensityEstimation::addData(std::vector<double> &data, int label,
                                 std::string filename)
{
    this->logger->log->debug(
        "CDensityEstimation::addDensityDistribution()....(%d,%d,%s)\n",
        data.size(), label, filename.c_str());
    if (data.size() > 0)
    {
        SDensityDistribution d;
        d.data = data;
        d.label = label;
        d.filename = filename;
        this->densityDistributions.push_back(d);
    }
    else
    {
        this->logger->log->warn(
            "CDensityEstimation::addDensityDistribution: empty! \n");

        SDensityDistribution d;
        d.data.push_back(0);
        d.label = label;
        d.filename = filename;
        this->densityDistributions.push_back(d);

    }
    this->isInitEstimation = false;
    this->isInitOptBin = false;
}

/*computes for each density destribution in the DB the histogram according to the opt bin size
 * currently one opt binsize for all dd's
 * Single level histograms*/
std::vector<std::vector<double> > CDensityEstimation::initEstimation(
    bool isCommonBinSize)
{
    this->logger->log->debug(
        "CDensityEstimation::initEstimation....(densityDistributions.size()=%d)\n",
        this->densityDistributions.size());

    this->isCommonBinSize = isCommonBinSize;

    std::vector<std::vector<double> > estimations;

    SBinConfig commonBc;
    if (this->isCommonBinSize)
    {
        if (!this->isInitOptBin)
        {
            commonBc = this->optimizeCommonBinSize(this->densityDistributions, this->optimizeBinSizeMethod);
        }
        else
        {
            commonBc = this->commonBinSizeCfg;
        }
    }

    if (this->densityDistributions.size() > 0)
    {
        for (unsigned int iterDD = 0; iterDD
                < this->densityDistributions.size(); ++iterDD)
        {
            SBinConfig bc;
            if (!this->isCommonBinSize)
            {
                bc = this->optimizeBinSize(
                         this->densityDistributions[iterDD].data,
                         this->optimizeBinSizeMethod);
            }
            else
            {
                bc = commonBc;
            }

            if (this->densityDistributions[iterDD].data.size() == 1
                    && this->densityDistributions[iterDD].data[0]
                    > -(std::numeric_limits<double>::epsilon())
                    && this->densityDistributions[iterDD].data[0]
                    < std::numeric_limits<double>::epsilon())
            {
                this->logger->log->debug(
                    "CHECK HERE maybe faulty case handling!!!");
                for (unsigned int i = 0; i < bc.binSize; ++i)
                {
                    this->densityDistributions[iterDD].densityDistribution.push_back(
                        0);
                }
                estimations.push_back(this->densityDistributions[iterDD].densityDistribution);
                this->densityDistributions[iterDD].bc = bc;
                continue;
            }

            //assert(this->densityDistributions.size()>0);
            //assert(this->densityDistributions[iterDD].data.size()>0);
            this->logger->log->debug(
                "CDensityEstimation::initEstimation ...(%d %lf)   Optimal bin size found %lf %lf %lf (%lf/%lf)\n",
                this->densityDistributions[iterDD].data.size(),
                this->densityDistributions[iterDD].data[0], bc.binSize,
                bc.var, bc.binWidth, bc.minValue, bc.maxValue);
            //  std::cout<<"\n this->densityDistributions.size()>0 "<<this->densityDistributions.size()<<"\n";
            if (bc.binSize > 0)
            {

                std::vector<double> estimation = this->normalizeHistogram(
                                                     this->computeHistogram(bc,
                                                             this->densityDistributions[iterDD].data));

                estimations.push_back(estimation);

                this->logger->log->debug(
                    "CDensityEstimation::initEstimation ... Estimation for ID = %d => \n",
                    iterDD);
                for (unsigned int iii = 0; iii < estimation.size(); iii++)
                {
                    this->logger->log->debug("%lf ", estimation[iii]);
                }
                this->logger->log->debug("\n");

                this->densityDistributions[iterDD].densityDistribution
                = estimation;
                this->densityDistributions[iterDD].bc = bc;
            }
        }

        this->isInitOptBin = true;
        this->isInitEstimation = true;
    }
    else
    {
        this->logger->log->debug(
            "CDensityEstimation::initEstimation ... Empty estimation\n");
        std::vector<double> emptyEstimation;
        emptyEstimation.push_back(0);
        estimations.push_back(emptyEstimation);

        this->logger->log->debug("CHECK HERE maybe faulty case handling!!!");
        this->densityDistributions.resize(1);
        this->densityDistributions[0].densityDistribution = emptyEstimation;
        this->isInitOptBin = true;
        this->isInitEstimation = true;
        //  exit(1);
    }

    assert(estimations.size() == this->densityDistributions.size());
    return estimations;
}

//Only one DensityDistribution is available!!! is used!
//a vector of estimations for the query are returned. Each estimation is the estimation with the corresponding binsize configuration of the density distribution.
std::vector<std::vector<double> > CDensityEstimation::estimateDensityDistribution(
    std::vector<double> query)
{

    std::vector< std::vector<double> > estimations;
    std::vector<double> estimation;

    if (this->isInitEstimation && this->isInitOptBin)
    {



        if (this->densityDistributions.size() > 0)
        {

            for (unsigned int iterDD = 0; iterDD
                    < this->densityDistributions.size(); ++iterDD)
            {

                if (this->densityDistributions[iterDD].bc.binSize > 0)
                {

                    //we take now the bin cfg of the db to estimate the histogram of the query!
                    estimation = this->computeHistogram(
                                     this->densityDistributions[iterDD].bc, query);

                    estimation = this->normalizeHistogram(estimation);

                    estimations.push_back(estimation);
                    if (this->isCommonBinSize)
                    {
                        this->logger->log->debug(
                            "CDensityEstimation::estimateDensityDistribution ... !!!BREAK estimation of query since this->isCommonBinSize==true BUT HERE IS NO BReAK CHECK THAT!!!\n");
                        //  std::cout<<"CDensityEstimation::estimateDensityDistribution ... !!!BREAK estimation of query\n";
                        break;
                    }
                    ///!!!!!!!!!!!!!!!!!!
                }
                else
                {
                    this->logger->log->debug(
                        "CDensityEstimation::estimateDensityDistribution ... CHECK HERE!!!! Empty estimation if binsize==0 1!!! Num DD: %d \n",
                        this->densityDistributions.size());
                    estimation.push_back(0);
                    estimations.push_back(estimation);
                }
            }
        }
        else
        {
            std::vector<double> emptyEstimation;
            this->logger->log->debug(
                "CDensityEstimation::estimateDensityDistribution ... CHECK HERE!!!! Empty estimation if binsize==0 2!!!\n");
            emptyEstimation.push_back(0);
            estimation = emptyEstimation;
            estimations.push_back(estimation);
        }
    }
    else
    {
        this->logger->log->debug(
            "CDensityEstimation::estimateDensityDistribution ... cannot estimate since not estimation initialized!!!!\n");
        assert(this->isInitEstimation && this->isInitOptBin);
    }

    if (this->isCommonBinSize)
    {
        assert(estimations.size() == 1);
    }
    else
    {
        assert(estimations.size() == this->densityDistributions.size());
    }
    //  this->logger->log->info(
    //          "CDensityEstimation::estimateDensityDistribution ... estimation.size()==%d",
    //          estimation.size());
    return estimations;
}

std::vector<double> CDensityEstimation::computeHistogram(SBinConfig &bc,
        std::vector<double> dd)
{

    std::vector<double> histo;
    histo.clear();
    if (bc.binSize > 0)
    {

        double minBin = bc.minValue;
        double maxBin = bc.minValue + bc.binWidth;

        histo.resize(bc.binSize);

        for (unsigned k = 0; k < (unsigned) bc.binSize; k++)
        {

            for (unsigned j = 0; j < dd.size(); j++)
            {
                if (dd[j] >= minBin && dd[j] < maxBin)
                {
                    histo[k]++;
                }
            }

            minBin = maxBin;
            maxBin = maxBin + bc.binWidth;
        }
    }
    else
    {
        histo.push_back(0);
    }
    return histo;
}

std::vector<double> CDensityEstimation::normalizeHistogram(
    std::vector<double> histo)
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

SBinConfig CDensityEstimation::optimizeCommonBinSize()
{
    this->logger->log->debug("CDensityEstimation::optimizeCommonBinSize...commonBinSize optimization\n");
    std::vector<double> combinedData;
    for (unsigned int iterDD = 0; iterDD < this->densityDistributions.size(); ++iterDD)
    {
        for (unsigned int iterDDVec = 0; iterDDVec < this->densityDistributions[iterDD].data.size(); ++iterDDVec)
        {
            combinedData.push_back(this->densityDistributions[iterDD].data[iterDDVec]);
        }
    }

    this->commonBinSizeCfg = this->optimizeBinSize(combinedData, this->optimizeBinSizeMethod);
    /*
        for(unsigned int i = 0 ; i < this->densityDistributions.size(); ++i)
        {
            this->densityDistributions[iterDD].bc = commonBinCfg;
        }
        */
    return this->commonBinSizeCfg;
}


SBinConfig CDensityEstimation::optimizeCommonBinSize(std::vector<SDensityDistribution> &dd, int method)
{
    this->logger->log->debug("CDensityEstimation::optimizeCommonBinSize...commonBinSize optimization\n");
    std::vector<double> combinedData;
    for (unsigned int iterDD = 0; iterDD < dd.size(); ++iterDD)
    {
        for (unsigned int iterDDVec = 0; iterDDVec < dd[iterDD].data.size(); ++iterDDVec)
        {
            combinedData.push_back(dd[iterDD].data[iterDDVec]);
        }
    }
    this->commonBinSizeCfg = this->optimizeBinSize(combinedData, method);
    return this->commonBinSizeCfg;
}

SBinConfig CDensityEstimation::optimizeBinSize(std::vector<double> dd,
        int method)
{
    std::vector<int> binSizeToEvaluate;
    std::vector<SBinConfig> bcs;
    std::map<std::vector<double>, int>::iterator iterd;
    int numSeqences = 1;
    SBinConfig binConfig;
    ///concat all dds

    if (dd.size() == 0)
    {
        this->logger->log->warn(
            "CDensityEstimation::optimizeBinSize... no data\n");
        binConfig.maxValue = 0;
        binConfig.minValue = 0;
        binConfig.binWidth = 0;
        binConfig.binSize = 0;
        binConfig.cost = 0;
        binConfig.mean = 0;
        binConfig.var = 0;
        return binConfig;
    }

    //numSeqences = this->densityDistributions.size();

    if (method == 1)
    {
        /*bin size from 2 to 150*/
        for (unsigned int i = 2; i < 1000; i++)
        {
            binSizeToEvaluate.push_back(i);
        }

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        for (unsigned int i = 0; i < binSizeToEvaluate.size(); i++)
        {

            SBinConfig bc;
            std::vector<double> histo;

            double binWidth = (x_max - x_min) / (double) binSizeToEvaluate[i];
            bc.maxValue = x_max;
            bc.minValue = x_min;
            bc.binWidth = binWidth;
            bc.binSize = binSizeToEvaluate[i];

            histo = this->computeHistogram(bc, dd);

            bc.mean = this->mean(histo);
            bc.var = this->variance(histo, bc.mean);
            bc.cost = this->computeCostFunction(bc.binWidth, bc.mean, bc.var,
                                                numSeqences);
            this->logger->log->debug(
                "Cost=%lf for binwidth %lf for binsize %lf mean %lf var %lf \n",
                bc.cost, bc.binWidth, bc.binSize, bc.mean, bc.var);
            bcs.push_back(bc);
        }

        double minCost = std::numeric_limits<double>::infinity();
        for (unsigned int ii = 0; ii < bcs.size(); ++ii)
        {
            if (bcs[ii].cost < minCost)
            {
                binConfig = bcs[ii];
                minCost = bcs[ii].cost;
            }
        }
        //this->isInitOptBin = true;
    }

    //sqrt ROOT
    if (method == 2)
    {
        SBinConfig bc;
        std::vector<double> histo;

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        int numBins = ceil(sqrt(dd.size()));
        double binWidth = (x_max - x_min) / (double) numBins;
        bc.maxValue = x_max;
        bc.minValue = x_min;
        bc.binWidth = binWidth;
        bc.binSize = numBins;
        histo = this->computeHistogram(bc, dd);
        bc.mean = this->mean(histo);
        bc.var = this->variance(histo, bc.mean);

        binConfig = bc;
        this->isInitOptBin = true;
    }

    //scotts
    if (method == 3)
    {
        SBinConfig bc;
        std::vector<double> histo;

        double meanDD = this->mean(dd);
        double varDD = this->variance(dd, meanDD);

        this->logger->log->debug(" means %lf var %lf \n", meanDD, varDD);
        double binWidth = (3.5 * sqrt(varDD)) / (pow((double) dd.size(), 1.0
                          / 3.0));

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        //original
        //int numBins = (x_max - x_min) / (double) binWidth;

        //check for div zero (e.g if dd.size()==1 so var is 0 and mean = dd[0])
        int numBins;
        if (binWidth > std::numeric_limits<double>::epsilon())
        {
            numBins = ceil((x_max - x_min) / ((double) binWidth));
        }
        else
        {
            numBins = 1;
        }

        bc.maxValue = x_max;
        bc.minValue = x_min;
        bc.binWidth = binWidth;
        bc.binSize = numBins;

        //  std::cout<<" eiwidth "<< binWidth<< " binsuze "<<numBins<<"\n";

        histo = this->computeHistogram(bc, dd);
        bc.mean = this->mean(histo);
        bc.var = this->variance(histo, bc.mean);

        binConfig = bc;
        this->isInitOptBin = true;
    }

    //fixed binSize
    if (method == 4)
    {
        SBinConfig bc;
        std::vector<double> histo;

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        int numBins = CDE_FIXED_BIN_SIZE_HISTOGRAM;
        double binWidth = (x_max - x_min) / (double) numBins;
        bc.maxValue = x_max;
        bc.minValue = x_min;
        bc.binWidth = binWidth;
        bc.binSize = numBins;
        histo = this->computeHistogram(bc, dd);
        bc.mean = this->mean(histo);
        bc.var = this->variance(histo, bc.mean);

        binConfig = bc;
        this->isInitOptBin = true;
    }

    //fixed binSize and width
    if (method == 5)
    {
        SBinConfig bc;
        std::vector<double> histo;

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        int numBins = this->commonBinSize;
        double binWidth = this->commonBinWidth;
        bc.maxValue = x_max;
        bc.minValue = x_min;
        bc.binWidth = binWidth;
        bc.binSize = numBins;
        histo = this->computeHistogram(bc, dd);
        bc.mean = this->mean(histo);
        bc.var = this->variance(histo, bc.mean);

        binConfig = bc;
        this->isInitOptBin = true;
    }

    //Sturges' formula
    if (method == 6)
    {
        SBinConfig bc;
        std::vector<double> histo;

        double x_min = *std::min_element(dd.begin(), dd.end());
        double x_max = *std::max_element(dd.begin(), dd.end());

        //original
        //int numBins = (x_max - x_min) / (double) binWidth;

        int numBins = (log_2(dd.size() + 1));
        double binWidth = (x_max - x_min) / double(numBins);

        bc.maxValue = x_max;
        bc.minValue = x_min;
        bc.binWidth = binWidth;
        bc.binSize = numBins;
        histo = this->computeHistogram(bc, dd);
        bc.mean = this->mean(histo);
        bc.var = this->variance(histo, bc.mean);

        binConfig = bc;
        this->isInitOptBin = true;

    }

    return binConfig;
}

double CDensityEstimation::log_2(double n)
{
    // log(n)/log(2) is log2.
    return log((double) n) / log((double) 2);
}

double CDensityEstimation::mean(std::vector<double> histo)
{
    double mean = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        mean += histo[k];
    }

    //  std::cout << "histo Size()" << histo.size() << "\n";
    return (mean / (double) histo.size());
}

double CDensityEstimation::variance(std::vector<double> histo, double mean)
{
    double var = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        var += pow((histo[k] - mean), 2);
    }
    return (var / (double) histo.size());
}

double CDensityEstimation::computeCostFunction(double binWidth, double kMean,
        double vVariance, int numSeqences)
{
    //  double cost = (2 * kMean - vVariance) / pow((binWidth),2);


    double cost = (((double) 2 * kMean) - vVariance) / pow((binWidth), 2);
    //double cost = (((double)2 * kMean) - vVariance) / pow(((double) numSeqences
    //      * binWidth), 2);
    return cost;
}

CDensityEstimation::~CDensityEstimation()
{
    // TODO Auto-generated destructor stub
}
