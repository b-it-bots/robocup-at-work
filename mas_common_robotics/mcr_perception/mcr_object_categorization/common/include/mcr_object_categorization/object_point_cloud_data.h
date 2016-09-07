/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef SObjectPointCloudData_H_
#define SObjectPointCloudData_H_


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>


struct SObjectPointCloudData
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZ> > > objectPointClouds;
    std::vector<int> objectLabels;
    std::vector<std::string> objectNames;
    //label, number of examples
    std::map<int, int> numExamples;

    //label, feature vector
    std::map<int, std::vector<std::vector<double> > > extractedFeatureVectors;


    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        //ar & objectPointClouds;
        ar & objectLabels;
        ar & objectNames;
        ar & numExamples;
        ar & extractedFeatureVectors;
    }
};

BOOST_CLASS_VERSION(SObjectPointCloudData, 1)

#endif /* SObjectPointCloudData_H_ */
