/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef CKDE_H
#define CKDE_H

#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>

#include <bin_size_config.h>


#define FIXED_BIN_SIZE_HISTOGRAM 20
class CKde
{
private:
    std::map<std::vector<double>, int> densityDistributions;
    std::map<std::vector<double>, int> estimatedDistributions;
    std::map<std::vector<double>, SBandwidthConfig> estimatedBandwidths;

    bool isInitEstimation;
    bool isInitOptBin;
    bool isInitOptBandwidth;

    /*Single binSize for a single estimation that means
     * vector= is the estimation
     * SBinConfig = regarding opt bin config */
    std::map<std::vector<double>, SBinConfig> binConfigs;

    /*Currently for each densityDistributions a optimized badnwidth */
    std::vector<SBandwidthConfig> vecBandwidthConfig;

    std::string filename;

    double log_2(double n);
    double diffEntropy(std::vector<double> a);
    double getKLD(std::vector<double> a, std::vector<double> b);
    double getJensenShannonDivergence(std::vector<double> a,
                                      std::vector<double> b);
public:
    CKde();
    std::vector<std::vector<double> > initEstimationHistogram(bool fixedBinSize = false);
    void initEstimationDensity();

    int getNumEstimatedDistributions()
    {
        return estimatedDistributions.size();
    }
    std::map<std::vector<double>, int> getEstimatedDistributions()
    {
        return estimatedDistributions;
    }


    std::vector<double>
    computeHistogram(SBinConfig &bc, std::vector<double> dd);
    std::vector<double> computeDensity(SBandwidthConfig &bc,
                                       std::vector<double> dd, std::vector<double> query);
    std::vector<double> estimateDensity(std::vector<double> query);
    std::vector<double> estimateHistogram(std::vector<double> query);

    double kernelGaussian(double u, double dim = 2);
    void addDensityDistribution(std::vector<double> dd, int label,
                                std::string filename);

    SBandwidthConfig optimizeBandwidth(std::vector<double> dd, int method = 1);
    SBinConfig optimizeBinSize(std::vector<double> dd, int method);

    double computeCostFunction(double binWidth, double kMean, double vVariance,
                               int numSeqences);
    double mean(std::vector<double> histo);
    double variance(std::vector<double> histo, double mean);
    double pseudoLikehoodLOOCV(std::vector<double> histo, int indexLeaveOut,
                               double bandwidth);
    void clear();

    std::vector<double> normalizeHist(std::vector<double> histo);
    double getEuclideanDistance(std::vector<double> a, std::vector<double> b);
    double getCosineDistance(std::vector<double> a, std::vector<double> b);
};

#endif
