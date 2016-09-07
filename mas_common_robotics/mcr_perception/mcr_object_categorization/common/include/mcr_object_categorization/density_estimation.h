/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef CDENSITYESTIMATION_H_
#define CDENSITYESTIMATION_H_

#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <fstream>
#include <numeric>
#include <vector>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>

#include "bin_size_config.h"
#include "logger.h"

#define CDE_OPT_SIZE_METHODE 3
#define CDE_FIXED_BIN_SIZE_HISTOGRAM 20
//define CDE_FIXED_BIN_SIZE_HISTOGRAM 20

struct SDensityDistribution
{
    //input data
    std::vector<double> data;
    std::vector<double> densityDistribution;
    int label;
    std::string filename;

    SBinConfig bc;

    SDensityDistribution() :
        label(-1), filename("temp")
    {
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & data;
        ar & densityDistribution;
        ar & label;
        ar & filename;
        ar & bc;
    }
};

class CDensityEstimation
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->densityDistributions;
        ar & this->optimizeBinSizeMethod;
        ar & this->isInitEstimation;
        ar & this->isInitOptBin;
        ar & this->isCommonBinSize;
        ar & this->commonBinSize;
        ar & this->commonBinWidth;
        ar & this->commonBinSizeCfg;
    }

    std::vector<SDensityDistribution> densityDistributions;
    int optimizeBinSizeMethod;
    bool isInitEstimation;
    bool isInitOptBin;
    bool isCommonBinSize;

    int commonBinSize;
    double commonBinWidth;
    CLogger* logger;

    SBinConfig commonBinSizeCfg;
public:
    CDensityEstimation();
    CDensityEstimation(int optBinSizeMethod);
    void addData(std::vector<double> &data, int label, std::string filename);
    std::vector<std::vector<double> > initEstimation(bool isCommonBinSize =
                false);
    std::vector<std::vector<double> > estimateDensityDistribution(std::vector<double> query);

    std::vector<double>
    computeHistogram(SBinConfig &bc, std::vector<double> dd);
    std::vector<double> normalizeHistogram(std::vector<double> histo);

    SBinConfig optimizeCommonBinSize();
    SBinConfig optimizeCommonBinSize(std::vector<SDensityDistribution> &densityDistributions, int method);
    SBinConfig optimizeBinSize(std::vector<double> dd, int method);

    double log_2(double n);
    double mean(std::vector<double> histo);
    double variance(std::vector<double> histo, double mean);
    double computeCostFunction(double binWidth, double kMean, double vVariance,
                               int numSeqences);

    std::vector<SDensityDistribution> getDensityDistributions()
    {
        return this->densityDistributions;
    }

    void setDensityDistributions(std::vector<SDensityDistribution> dDistr)
    {
        this->densityDistributions = dDistr;
    }

    void setManualInit(int concept = 1, bool isInit = false);
    virtual ~CDensityEstimation();
};

BOOST_CLASS_VERSION(SDensityDistribution, 1)
BOOST_CLASS_VERSION(CDensityEstimation, 1)
#endif /* CDENSITYESTIMATION_H_ */
