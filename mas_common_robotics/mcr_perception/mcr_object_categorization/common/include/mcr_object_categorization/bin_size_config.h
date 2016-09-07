/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef SBINSIZECONFIG_H_
#define SBINSIZECONFIG_H_

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>


struct SBinConfig
{
    double binSize;
    double binWidth;
    double minValue;
    double maxValue;
    double cost;
    double mean;
    double var;
    SBinConfig() :
        binSize(0), binWidth(0), minValue(0), maxValue(0), cost(0), mean(0),
        var(0)
    {
    }

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & binSize;
        ar & binWidth;
        ar & minValue;
        ar & maxValue;
        ar & cost;
        ar & mean;
        ar & var;
    }
};

struct SBandwidthConfig
{
    double bandwidth;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & bandwidth;
    }

};

BOOST_CLASS_VERSION(SBinConfig, 1)
BOOST_CLASS_VERSION(SBandwidthConfig, 1)


#endif /* SBINSIZECONFIG_H_ */
