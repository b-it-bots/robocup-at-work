/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef COBJECTDECOMPOSITION_H_
#define COBJECTDECOMPOSITION_H_

#include <iostream>
#include <cstdlib>
#include <vector>
#include <map>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "density_estimation.h"
#include "object_description.h"
#include "logger.h"



class CObjectDecomposition
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->objects;
        ar & this->nodeKdeEstimations;
        ar & this->isGeoShellMode;
        ar & this->isInit;
        ar & this->isGeoMode;
        ar & this->isNodeMode;
        ar & this->isNormalMode;
        ar & this->isNormalShellMode;
        ar & this->isNodeToCenterMode;
    }

    //idx of SObjectDescription vector corrspondes to std::map<int ,CKde> vector>
    std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > objects;

    std::vector<std::vector<double> > nodeKdeEstimations;

    bool isGeoShellMode;
    bool isInit;
    bool isGeoMode;
    bool isGeoNodeMode;
    bool isNodeMode;
    bool isNormalMode;
    bool isNormalShellMode;
    bool isNodeToCenterMode;
    CLogger* logger;
public:
    CObjectDecomposition();
    void addObject(SObjectDescription &object);

    //init estimation for objects
    void initGeoEstimation();

    //Indiviudual opt shell estimation for each object
    void initGeoShellEstimation(int numShell);
    //Common opt shell estimation for all objects
    void initGeoShellEstimationCommon(int numShell);

    //Indiviudual opt estimation for each object
    void initNormalEstimation();
    //Common opt estimation for all objects
    void initNormalEstimationCommon();

    void initNodeToCenterEstimation();

    //Indiviudual opt estimation for each object
    void initNormalShellEstimation(int numShell);
    //Common opt estimation for all objects
    //void initNormalShellEstimationCommon();
    std::map<int, std::vector<double> > initCombinedGeoShellNormalEstimation(int numShell, bool isCommon = false);
    std::map<int, std::vector<double> >
    initCombinedGeoShellNodeToCenterEstimation(int numShell, bool isCommon = false);

    std::map<int, std::vector<double> >
    initCombinedGeoShellNormalNormalShellEstimation(int numGeoShell, int numNormalShell, bool isCommon);

    void initNodeEstimation();

    SShellConfig optimalShellSize(SObjectDescription object, int numShells);
    SObjectDescription decomposeShell(SShellConfig shellConfig, SObjectDescription object);

    SObjectDescription decomposeNormalShell(SShellConfig shellConfig, SObjectDescription object);

    std::map<int, std::vector<double> > estimateNodeEstimation(SObjectDescription &query);
    std::map<int, std::vector<double> > estimateGeoHistogram(SObjectDescription &query);
    std::map<int, std::vector<double> > estimateGeoShellHistogram(SObjectDescription &query);
    std::map<int, std::vector<double> > estimateNormalHistogram(SObjectDescription &query);
    std::map<int, std::vector<double> > estimateNormalShellHistogram(SObjectDescription &query);
    std::map<int, std::vector<double> > estimateCombinedGeoShellNormalEstimation(SObjectDescription &query);

    std::map<int, std::vector<double> > estimateNodeToCenterDistanceHistogram(SObjectDescription &query);

    std::map<int, std::vector<double> > estimateGeoShellHistogramCommon(SObjectDescription &query);

    std::map<int, std::vector<double> >
    estimateCombinedGeoShellNodeToCenterEstimation(SObjectDescription &query);

    std::map<int, std::vector<double> >
    estimateCombinedGeoShellNormalNormalShellEstimation(SObjectDescription &query);

    std::vector<double> combineShellEstimations(std::vector<std::vector<double> > shellEstimations);
    std::vector<double> normalizeHist(std::vector<double> histo);

    void compareShell(std::map<int, std::vector<double> > allCombNormHistos);
    void compareNormalShell(std::map<int, std::vector<double> > allCombNormHistos);
    void compareNormals(std::map<int, std::vector<double> > allCombNormHistos);
    void compareCombinedShellNormal(std::map<int, std::vector<double> > allCombNormHistos);
    void compareCombinedGeoShellNormalNormalShell(std::map<int, std::vector<double> > allCombNormHistos);

    void
    compareEMDCommon(std::map<int, std::vector<double> > allCombNormHistos);

    double getEuclideanDistance(std::vector<double> a, std::vector<double> b);
    double getBhattacharyya(std::vector<double> a, std::vector<double> b);
    double getKLD(std::vector<double> a, std::vector<double> b);
    double getCosineDistance(std::vector<double> a, std::vector<double> b);
    double getJensenShannonDivergence(std::vector<double> a, std::vector<double> b);
    double diffEntropy(std::vector<double> a);
    double log_2(double n);

    std::vector<SObjectDescription, Eigen::aligned_allocator<SObjectDescription> > &getObjects()
    {
        return this->objects;
    }

    void setManualInitShell(bool isInit = false);
    void setManualInitNormal(bool isInit = false);
    void setManualInitShellNormal(bool isInit = false);
    virtual ~CObjectDecomposition();
};

BOOST_CLASS_VERSION(CObjectDecomposition, 1)

#endif /* COBJECTDECOMPOSITION_H_ */
