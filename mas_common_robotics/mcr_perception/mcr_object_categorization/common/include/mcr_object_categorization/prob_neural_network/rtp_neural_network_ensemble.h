/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */
#ifndef CRTPNEURALNETWORKENSEMBLE_H_
#define CRTPNEURALNETWORKENSEMBLE_H_

#include <iostream>
#include <vector>
#include <utility>
#include <map>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>

#include "logger.h"
#include "prob_neural_network/rtp_neural_network.h"
#include "file_settings.h"

#define CRTPNeuralNetworkEnsemble_NUM_TREES 100

#define CRTPNeuralNetworkEnsemble_SELECTED_NUM_TREES 60

#define CRTPNNE_TRAIN_CHUNK_SIZE_PER_TREE 0.6

class CRTPNeuralNetworkEnsemble
{
private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & this->RTPNNForrest;
        //ar & this->hModel;
        //ar & this->hTrainModel;
        //ar & this->hTestModel;
        ar & this->trainTestSetRatioForrest;
        ar & this->hModelVerifications;
        ar & this->numCategories;
    }
protected:
    CLogger *logger;

    int numCategories;

    std::map<int, CRTPNeuralNetwork> RTPNNForrest;

    std::map<int, std::vector<SVerficationResult> > hModelVerifications;

    //label, object id, label', object id', feature vector
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hModel;

    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hTrainModel;

    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > hTestModel;

    float trainTestSetRatioForrest;
    float trainSetChunkSize;
    int numToSelectTrees;
    //std::vector<std::vector<int> > rootCombinations;
    void extractDiscriminativeTrees(unsigned int numTreesToExtract);
    void addDiscriminativeTrees(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees, bool replace = true);
    void addDiscriminativeTrees2(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees);
    void addDiscriminativeTrees3(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees);
    void addDiscriminativeTrees4(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees);
    int next_comb(int comb[], int k, int n);
    std::vector<int*> computeCombinations(int n, int k);
    std::vector<std::vector<int> > generateRootCombinations(unsigned int numberOfNeurons);
    void setTrainTestSet(float trainTestSetRatio);

    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > createTrainSetChunk(
        std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > &trainSet, float chunkSize);

    //trainset , testset
    std::vector<SVerficationResult> verifyRT(CRTPNeuralNetwork tree);
public:
    CRTPNeuralNetworkEnsemble();
    void initEnsemble(float trainTestSetRatio);
    std::pair<int, double> fEvaluate(std::map<int, std::map<int, std::vector<double> > > query, std::map<int, std::pair<int, double> > &treeReponses);

    void addHModel(int label, int pattern, std::map<int, std::map<int, std::vector<double> > > &model);

    SVerficationResult selfVerificationRTPNNModel();

    SVerficationResult verifyRTPNNModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > toVerifyModel);

    int getNumTrees()
    {
        std::cout << this->RTPNNForrest.size() << " " << hModelVerifications.size() << "\n" << hModelVerifications[0].size() << " " << hModelVerifications[1].size() << std::endl;
        return this->RTPNNForrest.size();
    }

    void save(std::string filename);
    void load(std::string filename);

    int getNumCategories()
    {
        return numCategories;
    }

    ~CRTPNeuralNetworkEnsemble();
};

//BOOST_CLASS_VERSION(CRTPNeuralNetworkEnsemble, 1)
BOOST_CLASS_VERSION(CRTPNeuralNetworkEnsemble, 1)

#endif /* CHPNEURALNETWORKENSEMBLE_H_ */
