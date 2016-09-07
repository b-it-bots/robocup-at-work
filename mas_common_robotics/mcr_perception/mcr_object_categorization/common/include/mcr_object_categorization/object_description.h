/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */


#ifndef SObjectDescription_h
#define SObjectDescription_h

#include <iostream>
#include <cstdlib>
#include <vector>
#include <map>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "density_estimation.h"
#include "shell_config.h"
#include "johnson_apsp/apsp_graph.hpp"
#include "neural_gas/neural_gas.h"

struct SObjectDescription
{
    int label;

    pcl::PointCloud<pcl::PointXYZ> objectPointCloud;

    apsp_graph johnsonApsp;
    CNeuralGas neuralGas;
    std::vector<double> centroid;

    //id of node and position
    std::map<int, std::vector<double> > nodePosition;
    std::map<int, double> nodeMeanApspDistance;
    std::vector<double> meanApspDistances;

    std::map<int, std::vector<double> > nodeApspDistance;

    /////////////////////////

    std::vector<double> apspDistances; //total distances
    std::string filename;

    SShellConfig shellConfig;
    SShellConfig normalShellConfig;
    //a vector which consists of the nodes ids which are in 0 shell, 1 shell , 2 shell ....
    std::vector<std::vector<int> > decomposedNodes;
    std::vector<std::vector<double> > decomposedNodesWeights;
    std::vector<std::vector<int> > decomposedNormalShellNodes;
    std::vector<std::vector<double> > decomposedNormalShellNodesWeights;
    //each shell 0, 1 ,2, histo
    std::vector<std::vector<double> > kdeEstimations; //over distances combined over shells
    std::vector<double> combinedNormalizedKdeEstimations;

    std::vector<double> nodeToCenterDistances; // all nodes
    std::vector<double> kdeNodeToCenterDistanceEstimations;
    CDensityEstimation kdeNodeToCenterDistance;
    std::vector<double> combinedNormalizedKdeShellNodeToCenterEstimations;

    //number Shell, CKde // 1 first shell near center.... x shell farhter
    std::map<int, CDensityEstimation> kdeShell; //over distances

    ///////////////// Normals
    std::map<int, std::vector<double> > pointNormals; //total normals 0=x,1=y,2=z

    //node id, x y z, normals
    std::map<int, std::map<int, std::vector<double> > > nodePointNormals;//
    std::map<int, CDensityEstimation> kdeNormalXShell; //in shell manner
    std::map<int, CDensityEstimation> kdeNormalYShell; //in shell manner
    std::map<int, CDensityEstimation> kdeNormalZShell; //in shell manner
    std::vector<std::vector<double> > kdeNormalXEstimations;
    std::vector<std::vector<double> > kdeNormalYEstimations;
    std::vector<std::vector<double> > kdeNormalZEstimations;
    std::vector<double> combinedNormalXShellEstimation;
    std::vector<double> combinedNormalYShellEstimation;
    std::vector<double> combinedNormalZShellEstimation;
    //All combined combinedNormalXShellEstimation combinedNormalYShellEstimation ...
    std::vector<double> combinedNormalShellEstimation;

    //std::vector<std::vector<double> > pointNormals; //total normals 0=x,1=y,2=z
    std::map<int, CDensityEstimation> kdeNormals; //normals x=0, y=0, z=0;
    std::vector<std::vector<double> > kdeNormalEstimations;
    std::vector<double> combinedNormalizedKdeNormalEstimations;

    ///Shell Normals combination...
    std::vector<double> combinedNormalizedKdeShellNormalEstimations;

    //GeoShell Normals NormalShell combinaiton
    std::vector<double> combinedNormalizedGeoShellNormalNormalShellEstimations;

    std::map<int, std::map<int, std::vector<double> > > estimatedRelatedModels;

    std::map<int, std::vector<double> > nodeKdeEstimations;
    CDensityEstimation kdeNode;// over distances

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & label;
        //  ar & objectPointCloud;
        //  ar & johnsonApsp;
        //  ar & neuralGas;
        ar & centroid;

        ar & nodePosition;
        ar & nodeMeanApspDistance;
        ar & meanApspDistances;

        ar & nodeApspDistance;

        //ar & nodeKdeEstimations; //
        //ar & kdeNode;// over distances //Some it does not work to serialize it during saveing and loading: maybe the instance is not initialized

        ar & apspDistances; //total distances
        ar & filename;
        ar & shellConfig;
        ar & normalShellConfig;
        ar & decomposedNodes;
        ar & decomposedNodesWeights;
        ar & decomposedNormalShellNodes;
        ar & decomposedNormalShellNodesWeights;
        ar & kdeEstimations; //over distances combined over shells
        ar & combinedNormalizedKdeEstimations;
        ar & kdeShell;
        ar & pointNormals; //total normals 0=x,1=y,2=z
        ar & nodePointNormals;//
        ar & kdeNormalXShell; //in shell manner
        ar & kdeNormalYShell; //in shell manner
        ar & kdeNormalZShell; //in shell manner
        ar & kdeNormalXEstimations;
        ar & kdeNormalYEstimations;
        ar & kdeNormalZEstimations;
        ar & combinedNormalXShellEstimation;
        ar & combinedNormalYShellEstimation;
        ar & combinedNormalZShellEstimation;
        ar & combinedNormalShellEstimation;
        ar & kdeNormals;
        ar & kdeNormalEstimations;
        ar & combinedNormalizedKdeNormalEstimations;
        ar & combinedNormalizedKdeShellNormalEstimations;
        ar & combinedNormalizedGeoShellNormalNormalShellEstimations;
        ar & estimatedRelatedModels;

        ar & nodeToCenterDistances; // all nodes
        ar & kdeNodeToCenterDistanceEstimations;
        ar & kdeNodeToCenterDistance;
        ar & combinedNormalizedKdeShellNodeToCenterEstimations;

    }

};

BOOST_CLASS_VERSION(SObjectDescription, 1)

#endif
