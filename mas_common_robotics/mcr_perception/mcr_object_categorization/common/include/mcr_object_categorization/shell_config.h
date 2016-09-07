/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef SSHELLCONFIG_H_
#define SSHELLCONFIG_H_

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>


struct SShellConfig
{
    int shellSize; //num shells
    double shellWidth;
    double minValue; //minDistance
    double maxValue;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & shellSize;
        ar & shellWidth;
        ar & minValue;
        ar & maxValue;
    }

};

#endif /* SSHELLCONFIG_H_ */
