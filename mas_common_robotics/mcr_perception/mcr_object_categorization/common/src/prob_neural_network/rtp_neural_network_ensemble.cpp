/*
 * CHPNeuralNetworkEnsemble.cpp
 *
 *  Created on: May 15, 2011
 *      Author:  Christian Mueller
 */


#include <climits>
#include <iostream>
#include <string>
#include <map>
#include <assert.h>
#include <ctime>

#include <boost/shared_ptr.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "prob_neural_network/rtp_neural_network_ensemble.h"

CRTPNeuralNetworkEnsemble::CRTPNeuralNetworkEnsemble()
{
    this->logger = &CLogger::getInstance();
    this->trainTestSetRatioForrest = -1;
    this->trainSetChunkSize = CRTPNNE_TRAIN_CHUNK_SIZE_PER_TREE;
    this->numToSelectTrees = CRTPNeuralNetworkEnsemble_SELECTED_NUM_TREES;
    this->numCategories = 0;
    // TODO Auto-generated constructor stub
}

CRTPNeuralNetworkEnsemble::~CRTPNeuralNetworkEnsemble()
{
    // TODO Auto-generated destructor stub
}

void CRTPNeuralNetworkEnsemble::addHModel(int label, int pattern, std::map<int, std::map<int, std::vector<double> > > &model)
{
    this->logger->log->debug("CRTPNeuralNetworkEnsemble::addHModel...");
    //model represents the featVec regarding all other labels and pattern cfg according the pattern of label.
    this->hModel[label][pattern] = model;
}

void CRTPNeuralNetworkEnsemble::initEnsemble(float trainTestSetRatio)
{
    this->logger->log->debug("CRTPNeuralNetworkEnsemble::init...(Trees=%d)\n", CRTPNeuralNetworkEnsemble_NUM_TREES);
    printf("CRTPNeuralNetworkEnsemble::init...(Trees=%d)\n", CRTPNeuralNetworkEnsemble_NUM_TREES);
    //this->rootCombinations = this->generateRootCombinations(NUM_CLASSES-1); //-1 since non class removal

    if (this->hModel.size() > 0)
    {
        this->numCategories = this->hModel.size();
        this->trainTestSetRatioForrest = trainTestSetRatio;
        CRTPNeuralNetwork tempTree;
        std::vector<SVerficationResult> tempHModelVerification;

        //this->hModelVerifications.resize(this->RTPNNForrest.size());
        this->setTrainTestSet(trainTestSetRatioForrest);

        for (unsigned int iterTree = 0; iterTree < CRTPNeuralNetworkEnsemble_NUM_TREES; ++iterTree)
        {
            //this->RTPNNForrest[iterTree].setHModel(this->hModel);
            this->logger->log->info("CRTPNeuralNetworkEnsemble::init...Tree %d", iterTree);
            tempTree = CRTPNeuralNetwork();

            //CompleteTrainSet
            //tempTree.setHModel(this->hTrainModel);

            //Randomly chosen Train set
            std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > currenTrainSetChunk = this->createTrainSetChunk(this->hTrainModel, this->trainSetChunkSize);
            tempTree.setHModel(currenTrainSetChunk);

            tempTree.initHierarchy(); // all examples are taken as traing, no test
            tempHModelVerification = this->verifyRT(tempTree);

            //std
            this->addDiscriminativeTrees3(iterTree, tempTree, tempHModelVerification, this->numToSelectTrees);
            //

            //toplists
            //this->addDiscriminativeTrees4(iterTree, tempTree, tempHModelVerification, this->numToSelectTrees);
            //

            //just add
            //this->RTPNNForrest[iterTree] = tempTree;
            //this->hModelVerifications[iterTree] = tempHModelVerification;
            //
        }

        //extractDiscriminativeTrees(CRTPNeuralNetworkEnsemble_SELECTED_NUM_TREES);

        //Now verify complete ensemble
        this->verifyRTPNNModel(this->hTestModel);
        this->logger->log->debug("CRTPNeuralNetworkEnsemble::init...done");
    }
    else
    {
        this->logger->log->warn("CRTPNeuralNetworkEnsemble::init...Nothing to initialize! Model is empty");
    }
}

std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > CRTPNeuralNetworkEnsemble::createTrainSetChunk(
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > &trainSet, float chunkSize)
{
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > finalTrainSetChunk;
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > >::iterator iterHModel;
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPattern;

    for (iterHModel = trainSet.begin(); iterHModel != trainSet.end(); ++iterHModel)
    {
        std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern;
        pattern = iterHModel->second;

        int numberOfTrainExamples = (int) ceil(float((float) pattern.size() * chunkSize));
        if (numberOfTrainExamples <= 0)
        {
            finalTrainSetChunk = trainSet;

            this->logger->log->info(" CRTPNeuralNetworkEnsemble::createTrainSetChunk... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                    finalTrainSetChunk[iterHModel->first].size(), trainSet[iterHModel->first].size());
        }
        else //create Train and test examples
        {
            int currNumberofTrainExamples = 0;
            do
            {
                iterPattern = pattern.begin();
                std::advance(iterPattern, rand() % pattern.size());
                finalTrainSetChunk[iterHModel->first][iterPattern->first] = iterPattern->second;
                currNumberofTrainExamples++;
            }
            while (currNumberofTrainExamples <= numberOfTrainExamples);

            this->logger->log->info("CRTPNeuralNetworkEnsemble::createTrainSetChunk... %s : TrainsetChunk %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                    finalTrainSetChunk[iterHModel->first].size(), trainSet[iterHModel->first].size());
        }
    }

    return finalTrainSetChunk;
}

//This method will add an tree only if it is under the top maxNumTrees, accordingly the worst tree is removed
void CRTPNeuralNetworkEnsemble::addDiscriminativeTrees(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees,
        bool replace)
{
    assert(this->hModelVerifications.size() == this->RTPNNForrest.size());

    std::map<int, std::vector<SVerficationResult> >::iterator iterTreeResults;
    if (tempHModelVerification[1].numberOfExamples == 0)
    {
        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....empty Model Verification found(due to no test examples)...stopped!\n");
        return;
    }

    if (this->hModelVerifications.size() > maxNumTrees)
    {
        int minTotalCorrectTreeNum = -1;
        double minTotalCorrect = 1;
        for (iterTreeResults = this->hModelVerifications.begin(); iterTreeResults != this->hModelVerifications.end(); ++iterTreeResults)
        {
            if (iterTreeResults->second[1].totalCorrect <= minTotalCorrect)
            {
                minTotalCorrect = iterTreeResults->second[1].totalCorrect;
                minTotalCorrectTreeNum = iterTreeResults->first;
            }
        }
        assert(minTotalCorrectTreeNum > -1);

        if (tempHModelVerification[1].totalCorrect > minTotalCorrect)
        {
            //delete worst of top maxNumTrees
            if (replace)
            {
                this->RTPNNForrest.erase(this->RTPNNForrest.find(minTotalCorrectTreeNum));
                this->hModelVerifications.erase(this->hModelVerifications.find(minTotalCorrectTreeNum));

            }
            //add new one
            this->RTPNNForrest[iterTree] = tempTree;
            this->hModelVerifications[iterTree] = tempHModelVerification;
            assert(this->hModelVerifications.size() == this->RTPNNForrest.size());

            if (replace)
            {
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced(Size=%d)!\n", minTotalCorrectTreeNum, minTotalCorrect, iterTree,
                                         tempHModelVerification[1].totalCorrect, this->RTPNNForrest.size());
            }
            else
            {
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf) is added (Size=%d)!\n", iterTree, tempHModelVerification[1].totalCorrect,
                                         this->RTPNNForrest.size());
            }
        }
    }
    else //just add
    {
        this->RTPNNForrest[iterTree] = tempTree;
        this->hModelVerifications[iterTree] = tempHModelVerification;
    }
}

//This method will add and replace a tree only if the tree is better then the worse in the forest accordingly the label
void CRTPNeuralNetworkEnsemble::addDiscriminativeTrees2(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees)
{
    assert(this->hModelVerifications.size() == this->RTPNNForrest.size());

    std::map<int, std::vector<SVerficationResult> >::iterator iterTreeResults;
    std::map<int, double>::iterator iterCorrectLabel;
    if (tempHModelVerification[1].numberOfExamples == 0)
    {
        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....empty Model Verification found(due to no test examples)...stopped!\n");
        return;
    }

    if (this->hModelVerifications.size() > maxNumTrees)
    {
        //label, value
        std::map<int, double> minCorrectLabel;

        //label, tree
        std::map<int, int> minCorrectTree;

        assert(tempHModelVerification.size() > 1);
        //init!
        for (iterCorrectLabel = tempHModelVerification[1].correctLabel.begin(); iterCorrectLabel != tempHModelVerification[1].correctLabel.end(); ++iterCorrectLabel)
        {
            minCorrectLabel[iterCorrectLabel->first] = 1;
        }

        assert(minCorrectLabel.size()
               == tempHModelVerification[1].correctLabel.size());

        for (iterTreeResults = this->hModelVerifications.begin(); iterTreeResults != this->hModelVerifications.end(); ++iterTreeResults)
        {
            for (iterCorrectLabel = iterTreeResults->second[1].correctLabel.begin(); iterCorrectLabel != iterTreeResults->second[1].correctLabel.end(); ++iterCorrectLabel)
            {

                if (iterCorrectLabel->second <= minCorrectLabel[iterCorrectLabel->first])
                {
                    minCorrectLabel[iterCorrectLabel->first] = iterCorrectLabel->second;
                    minCorrectTree[iterCorrectLabel->first] = iterTreeResults->first;
                }
            }
        }

        //iterate over all min correctLabel and check temp tree more accurate and replace it with old one if so.
        for (iterCorrectLabel = minCorrectLabel.begin(); iterCorrectLabel != minCorrectLabel.end(); ++iterCorrectLabel)
        {

            if (tempHModelVerification[1].correctLabel[iterCorrectLabel->first] > iterCorrectLabel->second) // && tempHModelVerification[1].totalCorrect > this->hModelVerifications[minCorrectTree[iterCorrectLabel->first]][1].totalCorrect)
            {
                //delete worst of top maxNumTrees
                if (this->RTPNNForrest.find(minCorrectTree[iterCorrectLabel->first]) != this->RTPNNForrest.end())
                {
                    this->RTPNNForrest.erase(this->RTPNNForrest.find(minCorrectTree[iterCorrectLabel->first]));
                }
                else
                {
                    this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)*Already replaced!!\n",
                                             minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first], iterCorrectLabel->first,
                                             this->RTPNNForrest.size());
                    continue;

                }
                if (this->hModelVerifications.find(minCorrectTree[iterCorrectLabel->first]) != this->hModelVerifications.end())
                {
                    this->hModelVerifications.erase(this->hModelVerifications.find(minCorrectTree[iterCorrectLabel->first]));
                }
                else
                {
                    this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)*Already replaced!!\n",
                                             minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first], iterCorrectLabel->first,
                                             this->RTPNNForrest.size());
                    continue;
                }
                //add new one
                this->RTPNNForrest[iterTree] = tempTree;
                this->hModelVerifications[iterTree] = tempHModelVerification;
                assert(this->hModelVerifications.size()
                       == this->RTPNNForrest.size());
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)!\n",
                                         minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first], iterCorrectLabel->first,
                                         this->RTPNNForrest.size());
            }
        }
    }
    else //just add
    {
        this->RTPNNForrest[iterTree] = tempTree;
        this->hModelVerifications[iterTree] = tempHModelVerification;
    }
}

//like addDiscriminativeTrees2 but also checking whether totalCorrect is better then
void CRTPNeuralNetworkEnsemble::addDiscriminativeTrees3(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees)
{
    assert(this->hModelVerifications.size() == this->RTPNNForrest.size());

    std::map<int, std::vector<SVerficationResult> >::iterator iterTreeResults;
    std::map<int, double>::iterator iterCorrectLabel;
    if (tempHModelVerification[1].numberOfExamples == 0)
    {
        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....empty Model Verification found(due to no test examples)...stopped!\n");
        return;
    }

    if (this->hModelVerifications.size() > maxNumTrees)
    {
        //label, value
        std::map<int, double> minCorrectLabel;

        //label, tree
        std::map<int, int> minCorrectTree;

        assert(tempHModelVerification.size() > 1);
        //init!
        for (iterCorrectLabel = tempHModelVerification[1].correctLabel.begin(); iterCorrectLabel != tempHModelVerification[1].correctLabel.end(); ++iterCorrectLabel)
        {
            minCorrectLabel[iterCorrectLabel->first] = 1;
        }

        assert(minCorrectLabel.size()
               == tempHModelVerification[1].correctLabel.size());

        for (iterTreeResults = this->hModelVerifications.begin(); iterTreeResults != this->hModelVerifications.end(); ++iterTreeResults)
        {
            for (iterCorrectLabel = iterTreeResults->second[1].correctLabel.begin(); iterCorrectLabel != iterTreeResults->second[1].correctLabel.end(); ++iterCorrectLabel)
            {

                if (iterCorrectLabel->second <= minCorrectLabel[iterCorrectLabel->first])
                {
                    minCorrectLabel[iterCorrectLabel->first] = iterCorrectLabel->second;
                    minCorrectTree[iterCorrectLabel->first] = iterTreeResults->first;
                }
            }
        }

        //iterate over all min correctLabel and check temp tree more accurate and replace it with old one if so.
        for (iterCorrectLabel = minCorrectLabel.begin(); iterCorrectLabel != minCorrectLabel.end(); ++iterCorrectLabel)
        {
            //check if current is better for specific label
            if (tempHModelVerification[1].correctLabel[iterCorrectLabel->first] > iterCorrectLabel->second) // && tempHModelVerification[1].totalCorrect > this->hModelVerifications[minCorrectTree[iterCorrectLabel->first]][1].totalCorrect)
            {
                if (this->hModelVerifications.find(minCorrectTree[iterCorrectLabel->first]) != this->hModelVerifications.end())
                {
                    //check if current is better in general then specific label
                    if (tempHModelVerification[1].totalCorrect > this->hModelVerifications[minCorrectTree[iterCorrectLabel->first]][1].totalCorrect)
                    {
                        //delete worst of top maxNumTrees
                        if (this->RTPNNForrest.find(minCorrectTree[iterCorrectLabel->first]) != this->RTPNNForrest.end())
                        {
                            this->RTPNNForrest.erase(this->RTPNNForrest.find(minCorrectTree[iterCorrectLabel->first]));
                        }
                        else
                        {
                            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)*Already replaced!!\n",
                                                     minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first],
                                                     iterCorrectLabel->first, this->RTPNNForrest.size());
                            continue;

                        }
                        if (this->hModelVerifications.find(minCorrectTree[iterCorrectLabel->first]) != this->hModelVerifications.end())
                        {
                            this->hModelVerifications.erase(this->hModelVerifications.find(minCorrectTree[iterCorrectLabel->first]));
                        }
                        else
                        {
                            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)*Already replaced!!\n",
                                                     minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first],
                                                     iterCorrectLabel->first, this->RTPNNForrest.size());
                            continue;
                        }
                        //add new one
                        this->RTPNNForrest[iterTree] = tempTree;
                        this->hModelVerifications[iterTree] = tempHModelVerification;
                        assert(this->hModelVerifications.size()
                               == this->RTPNNForrest.size());
                        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....tree(%d=>%lf with %d=>%lf) is replaced for Label %d (Size=%d)!\n",
                                                 minCorrectTree[iterCorrectLabel->first], iterCorrectLabel->second, iterTree, tempHModelVerification[1].correctLabel[iterCorrectLabel->first], iterCorrectLabel->first,
                                                 this->RTPNNForrest.size());

                    }
                }
                else
                {
                    continue;
                }

            }
        }
    }
    else //just add
    {
        //

        if (iterTree > maxNumTrees) //check whether forest size had already reach desired size => if so we are in best tree selection phase
        {
            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....less then needed trees -> try to add!!\n");
            this->addDiscriminativeTrees(iterTree, tempTree, tempHModelVerification, this->hModelVerifications.size() - 1, false);
            // this->hModelVerifications.size() - 1 to gurantee that we search for min accuracy tree and then check whether current one is better.
            //Do not replace(false)
        }
        else
        {
            this->RTPNNForrest[iterTree] = tempTree; //!!
            this->hModelVerifications[iterTree] = tempHModelVerification;//!!
        }
    }
}

//like addDiscriminativeTrees4 : maintain a list of top trees for each category!
void CRTPNeuralNetworkEnsemble::addDiscriminativeTrees4(unsigned int iterTree, CRTPNeuralNetwork tempTree, std::vector<SVerficationResult> tempHModelVerification, unsigned int maxNumTrees)
{
    assert(this->hModelVerifications.size() == this->RTPNNForrest.size());

    static int maxNumTopTreeCatgory = ceil((double) maxNumTrees / (double) this->numCategories);
    //label, tree id
    static std::map<int, std::vector<int> > topRTPNNForrest;
    //train and test result
    //for each tree on hModeVerfi. where[0] is train result and [1] is test result
    static std::map<int, std::vector<int> > topHModelVerifications;

    std::map<int, std::vector<int> >::iterator iterTopRTPNNForrest;
    std::map<int, std::vector<int> >::iterator iterTopHModelVerifications;

    std::map<int, std::vector<SVerficationResult> >::iterator iterTreeResults;
    std::map<int, CRTPNeuralNetwork>::iterator iterForrest;
    std::map<int, double>::iterator iterCorrectLabel;
    if (tempHModelVerification[1].numberOfExamples == 0)
    {
        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....empty Model Verification found(due to no test examples)...stopped!\n");
        return;
    }

    //if (this->hModelVerifications.size() > maxNumTopTreeCatgory) // maxNumTrees)
    this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....entering with tree %d!\n", iterTree);
    if (iterTree > maxNumTopTreeCatgory)
    {

        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....less then needed trees -> try to add!!\n");
        //this->addDiscriminativeTrees(iterTree, tempTree, tempHModelVerification, this->hModelVerifications.size() - 1, false);
        // this->hModelVerifications.size() - 1 to gurantee that we search for min accuracy tree and then check whether current one is better.
        //Do not replace(false)

        //iterator over categories...
        for (iterTopHModelVerifications = topHModelVerifications.begin(); iterTopHModelVerifications != topHModelVerifications.end(); ++iterTopHModelVerifications)
        {
            //iter over tops trees of categories
            for (unsigned int iterTops = 0; iterTops < iterTopHModelVerifications->second.size(); ++iterTops)
            {
                //if current tree better then take tree to the ensemble.
                if (tempHModelVerification[1].correctLabel[iterTopHModelVerifications->first]
                        > this->hModelVerifications[iterTopHModelVerifications->second[iterTops]][1].correctLabel[iterTopHModelVerifications->first])
                {

                    this->logger->log->error(
                        "CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....Tree %d too all Top Lists:: (cur tree for label %d better than tree %d in top list cat %d : %lf > %lf)!! totaltrees of cat %d / %d\n",
                        iterTree, iterTree, iterTopHModelVerifications->second[iterTops], iterTopHModelVerifications->first,
                        tempHModelVerification[1].correctLabel[iterTopHModelVerifications->first],
                        this->hModelVerifications[iterTopHModelVerifications->second[iterTops]][1].correctLabel[iterTopHModelVerifications->first], iterTopHModelVerifications->second.size(),
                        maxNumTopTreeCatgory);

                    this->RTPNNForrest[iterTree] = tempTree;
                    this->hModelVerifications[iterTree] = tempHModelVerification;

                    //replace it from the list of the current category!!!
                    //topRTPNNForrest[iterTopHModelVerifications->first][iterTops] = (iterTree);
                    //topHModelVerifications[iterTopHModelVerifications->first][iterTops] = (iterTree);

                    if (topRTPNNForrest[iterTopHModelVerifications->first].size() < maxNumTopTreeCatgory)
                    {
                        //maxNumTopTreeCatgory not reached for cat so push back
                        topRTPNNForrest[iterTopHModelVerifications->first].push_back(iterTree);
                        topHModelVerifications[iterTopHModelVerifications->first].push_back(iterTree);
                        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...adding tree %d", iterTree);
                    }
                    else
                    {
                        //maxNumTopTreeCatgory not reached for cat so replace
                        // todo?! this is error : iterator and here direct acces: if replace error comes : 20 tree is reaplced look in log
                        topRTPNNForrest[iterTopHModelVerifications->first][iterTops] = (iterTree);
                        topHModelVerifications[iterTopHModelVerifications->first][iterTops] = (iterTree);
                        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...replaceing with tree %d", iterTree);
                    }

                    assert(topRTPNNForrest[iterTopHModelVerifications->first].size() == topHModelVerifications[iterTopHModelVerifications->first].size());
                    /***todo
                     * should be break here so that only one --the first-- tree is replaced? or replace all with best tree but no variablity, or the worst tree replace
                     *   */
                    break;
                }
            }
        }

        /////
        //remove tree from ensemble if not in used by any top list since it is replaced by the loop before
        std::vector<int> treesToDelete;
        for (iterForrest = this->RTPNNForrest.begin(); iterForrest != this->RTPNNForrest.end(); ++iterForrest)
        {
            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...is tree %d unused?", iterForrest->first);
            bool isInUse = false;
            for (iterTopRTPNNForrest = topRTPNNForrest.begin(); iterTopRTPNNForrest != topRTPNNForrest.end(); ++iterTopRTPNNForrest)
            {
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...check for cat %d?", iterTopRTPNNForrest->first);
                //iter over tops trees of categories
                for (unsigned int iterTops = 0; iterTops < iterTopRTPNNForrest->second.size(); ++iterTops)
                {
                    if (iterForrest->first == iterTopRTPNNForrest->second[iterTops])
                    {
                        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...tree %d is in use!!!", iterForrest->first);
                        isInUse = true;
                    }
                    if (isInUse)
                    {
                        break;
                    }
                }
                if (isInUse)
                {
                    break;
                }
            }
            //Not in use so remove
            if (!isInUse)
            {
                treesToDelete.push_back(iterForrest->first);

            }
        }

        //remove unsed Trees Now
        for (int iterDeleteTree = 0; iterDeleteTree < treesToDelete.size(); ++iterDeleteTree)
        {
            if (this->RTPNNForrest.find(treesToDelete[iterDeleteTree]) != this->RTPNNForrest.end())
            {
                this->RTPNNForrest.erase(this->RTPNNForrest.find(treesToDelete[iterDeleteTree]));
                //this->RTPNNForrest.erase(iterForrest->first);
            }
            else
            {
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...oh oh sth is wrong no tree to delte!");
                continue;
            }
            if (this->hModelVerifications.find(treesToDelete[iterDeleteTree]) != this->hModelVerifications.end())
            {
                this->hModelVerifications.erase(this->hModelVerifications.find(treesToDelete[iterDeleteTree]));
                //this->hModelVerifications.erase(iterForrest->first);
            }
            else
            {
                this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees...oh oh sth is wrong no tree to delte!");
                continue;
            }

            assert(this->hModelVerifications.size() == this->RTPNNForrest.size());
            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees... tree %d not is use in any top list -> removed :: total trees:%d", treesToDelete[iterDeleteTree],
                                     this->RTPNNForrest.size());
        }

    }
    else //just add
    {
        this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees... too less trees due to removal of no usage of tree or init");
        if (iterTree > maxNumTrees) //check whether forest size had already reach desired size => if so we are in best tree selection phase
        {
            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees....less then needed trees -> try to add!!\n");
            //this->addDiscriminativeTrees(iterTree, tempTree, tempHModelVerification, this->hModelVerifications.size() - 1, false);
            // this->hModelVerifications.size() - 1 to gurantee that we search for min accuracy tree and then check whether current one is better.
            //Do not replace(false)

            //iterator over categories...
            for (iterTopHModelVerifications = topHModelVerifications.begin(); iterTopHModelVerifications != topHModelVerifications.end(); ++iterTopHModelVerifications)
            {
                //iter over tops trees of categories
                for (unsigned int iterTops = 0; iterTops < iterTopHModelVerifications->second.size(); ++iterTops)
                {
                    //if current tree better then take tree to the ensemble.
                    if (tempHModelVerification[1].correctLabel[iterTopHModelVerifications->first] > this->hModelVerifications[iterTops][1].correctLabel[iterTopHModelVerifications->first])
                    {
                        this->RTPNNForrest[iterTree] = tempTree;
                        this->hModelVerifications[iterTree] = tempHModelVerification;

                        if (topRTPNNForrest[iterTopHModelVerifications->first].size() < maxNumTopTreeCatgory)
                        {
                            //maxNumTopTreeCatgory not reached for cat so push back
                            topRTPNNForrest[iterTopHModelVerifications->first].push_back(iterTree);
                            topHModelVerifications[iterTopHModelVerifications->first].push_back(iterTree);
                        }
                        else
                        {
                            //maxNumTopTreeCatgory not reached for cat so replace
                            topRTPNNForrest[iterTopHModelVerifications->first][iterTops] = (iterTree);
                            topHModelVerifications[iterTopHModelVerifications->first][iterTops] = (iterTree);
                        }
                    }
                }
            }
        }
        else
        {
            this->RTPNNForrest[iterTree] = tempTree; //!!
            this->hModelVerifications[iterTree] = tempHModelVerification;//!!

            //Cat=0 is neg

            for (unsigned int iterCat = 1; iterCat < this->numCategories; ++iterCat)
            {
                topRTPNNForrest[iterCat].push_back(iterTree);
                topHModelVerifications[iterCat].push_back(iterTree);
            }
            this->logger->log->error("CRTPNeuralNetworkEnsemble::addDiscriminativeTrees(init)....Tree %d too all Top Lists!!\n", iterTree);
        }
    }
}

void CRTPNeuralNetworkEnsemble::extractDiscriminativeTrees(unsigned int numTreesToExtract)
{
    this->logger->log->info("CRTPNeuralNetworkEnsemble::extractDiscriminativeTrees....\n");
    std::map<int, std::vector<SVerficationResult> >::iterator iterTreeResults;
    std::map<int, std::vector<SVerficationResult> > discriminativehModelVerifications;
    std::map<int, CRTPNeuralNetwork> discriminativeRTPNNForrest;

    for (iterTreeResults = this->hModelVerifications.begin(); iterTreeResults != this->hModelVerifications.end(); ++iterTreeResults)
    {
        if (iterTreeResults->second[1].numberOfExamples == 0)
        {
            this->logger->log->warn("CRTPNeuralNetworkEnsemble::extractDiscriminativeTrees....empty Model Verification found(due to no test examples)...stopped!\n");
            return;
        }
    }

    for (unsigned int iter = 0; iter < numTreesToExtract && this->RTPNNForrest.size() > 0; ++iter)
    {

        int bestTreeNum = 0;
        double currentbestCorrect = 0;
        //double currentbestCorrect = 1;
        for (iterTreeResults = this->hModelVerifications.begin(); iterTreeResults != this->hModelVerifications.end(); ++iterTreeResults)
        {
            if (iterTreeResults->second[1].totalCorrect >= currentbestCorrect)
                //if (iterTreeResults->second[1].totalCorrect <= currentbestCorrect)

            {
                currentbestCorrect = iterTreeResults->second[1].totalCorrect;
                bestTreeNum = iterTreeResults->first;
            }
        }

        this->logger->log->info("CRTPNeuralNetworkEnsemble::extractDiscriminativeTrees....Tree %d selected with totalCorrect=%lf \n", bestTreeNum,
                                this->hModelVerifications[bestTreeNum][1].totalCorrect);

        discriminativeRTPNNForrest[bestTreeNum] = this->RTPNNForrest[bestTreeNum];
        discriminativehModelVerifications[bestTreeNum] = this->hModelVerifications[bestTreeNum];
        this->RTPNNForrest.erase(this->RTPNNForrest.find(bestTreeNum));
        this->hModelVerifications.erase(this->hModelVerifications.find(bestTreeNum));
    }

    assert(discriminativeRTPNNForrest.size() > 0);
    assert(discriminativeRTPNNForrest.size()
           == discriminativehModelVerifications.size());

    this->RTPNNForrest = discriminativeRTPNNForrest;
    this->hModelVerifications = discriminativehModelVerifications;
    this->logger->log->info("CRTPNeuralNetworkEnsemble::extractDiscriminativeTrees....%d\n", discriminativeRTPNNForrest.size());

}

//0 train result, 1 test result
std::vector<SVerficationResult> CRTPNeuralNetworkEnsemble::verifyRT(CRTPNeuralNetwork tree)
{
    std::vector<SVerficationResult> result;
    result.resize(2);

    this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRT....\n");

    if (this->hTrainModel.size() > 0)
    {
        this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRT...TrainData(%d):\n", this->hTrainModel.size());
        result[0] = tree.verifyHPNNModel(this->hTrainModel);
    }
    else
    {
        this->logger->log->warn("CRTPNeuralNetworkEnsemble::verifyRT...TrainData: NO TRAIN DATA -> this might lead to an exception\n");
    }
    if (this->hTestModel.size() > 0)
    {
        this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRT...TestData(%d):\n", this->hTestModel.size());
        result[1] = tree.verifyHPNNModel(this->hTestModel);
    }
    else
    {
        this->logger->log->warn("CRTPNeuralNetworkEnsemble::verifyRT...TestData: NO TEST DATA -> this might lead to an exception\n");
    }
    return result;
}

void CRTPNeuralNetworkEnsemble::setTrainTestSet(float trainTestSetRatio)
{
    this->trainTestSetRatioForrest = trainTestSetRatio;
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > >::iterator iterHModel;
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPattern;

    if (this->trainTestSetRatioForrest < (0 + std::numeric_limits<float>::epsilon()))
    {
        this->hTrainModel = hModel;
        this->logger->log->debug("CRTPNeuralNetworkEnsemble::setTrainTestSet...complete examples used as trainExamples\n");
    }
    else
    {
        for (iterHModel = this->hModel.begin(); iterHModel != this->hModel.end(); ++iterHModel)
        {
            std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern;
            pattern = iterHModel->second;

            int numberOfTrainExamples = (int) ceil(float((float) pattern.size() * this->trainTestSetRatioForrest));
            if (numberOfTrainExamples <= 0)
            {
                this->hTrainModel = hModel;

                this->logger->log->info(" CRTPNeuralNetworkEnsemble::setTrainTestSet... %s : complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                        this->hTrainModel[iterHModel->first].size(), this->hModel[iterHModel->first].size());

            }
            else //create Train and test examples

            {
                int currNumberofTrainExamples = 0;
                for (iterPattern = pattern.begin(); iterPattern != pattern.end(); ++iterPattern)
                {
                    if (currNumberofTrainExamples < numberOfTrainExamples)
                    {
                        this->hTrainModel[iterHModel->first][iterPattern->first] = iterPattern->second;
                    }
                    else
                    {
                        this->hTestModel[iterHModel->first][iterPattern->first] = iterPattern->second;
                    }
                    currNumberofTrainExamples++;
                }
                this->logger->log->info("CRTPNeuralNetworkEnsemble::setTrainTestSet... %s : Trainset complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                        this->hTrainModel[iterHModel->first].size(), this->hModel[iterHModel->first].size());
                this->logger->log->info("CRTPNeuralNetworkEnsemble::setTrainTestSet... %s : Testset complete %d of %d examples added\n", CFileSettings::labels[iterHModel->first].c_str(),
                                        this->hTestModel[iterHModel->first].size(), this->hModel[iterHModel->first].size());
            }
        }
    }

    if (this->hTrainModel.size() == 0)
    {
        this->logger->log->warn("CRTPNeuralNetworkEnsemble::setTrainTestSet... TrainSet is emtpy!\n");
    }

    if (this->hTestModel.size() == 0)
    {
        this->logger->log->warn("CRTPNeuralNetworkEnsemble::setTrainTestSet... Testset is emtpy!\n");
    }
}

/*
 * Evaluate the given query
 * return the label and the confidence*/
std::pair<int, double> CRTPNeuralNetworkEnsemble::fEvaluate(std::map<int, std::map<int, std::vector<double> > > query, std::map<int, std::pair<int, double> > &treeReponses)
{
    clock_t start = clock();
    this->logger->log->debug("CRTPNeuralNetworkEnsemble::fEvaluate...\n");
    double maxConfidence = 0;
    int maxLabel = 0;
    std::map<int, double>::iterator iterResult;
    std::pair<int, double> finalResult;

    std::map<int, double> fusionedResults;
    std::map<int, int> finalResultLabelCounts;
    //tree id, label, accuracy
    std::map<int, std::pair<int, double> > results;

    //results.resize(this->RTPNNForrest.size());

    std::map<int, CRTPNeuralNetwork>::iterator iterTree;
    for (iterTree = this->RTPNNForrest.begin(); iterTree != this->RTPNNForrest.end(); ++iterTree)
    {
        results[iterTree->first] = (iterTree->second.hEvaluate(query));
        treeReponses[iterTree->first] = results[iterTree->first];
        //  if(iterTree==1)
        //  results[iterTree].first = 1;

        this->logger->log->debug("CRTPNeuralNetworkEnsemble::fEvaluate...Tree %d vote %s conf %lf \n", iterTree->first, CFileSettings::labels[results[iterTree->first].first].c_str(),
                                 results[iterTree->first].second);
        //testData result is used as weighting but we need to test whether there was test data available
        //since if 70% is ratio and 2 examples are given so 2 train and 0 test!!!
        if (hModelVerifications[iterTree->first][1].numberOfExamples > 0)
        {
            results[iterTree->first].second = results[iterTree->first].second * exp(hModelVerifications[iterTree->first][1].totalCorrect) * exp(
                                                  hModelVerifications[iterTree->first][1].correctLabel[results[iterTree->first].first]);
            //// since sometimes it is very goodtree but bad for a certain class <--- this assumption failed!

            //With larger weight on correct labeling not total
            //results[iterTree->first].second = results[iterTree->first].second * hModelVerifications[iterTree->first][1].totalCorrect
            //      * (hModelVerifications[iterTree->first][1].correctLabel[results[iterTree->first].first] + hModelVerifications[iterTree->first][1].correctLabel[results[iterTree->first].first]
            //              * 0.75);
        }
        else
        {
            this->logger->log->warn("CRTPNeuralNetworkEnsemble::fEvaluate...No weighting applied since no testing data available. This might lead to wrong results!!!!\n");
        }

        this->logger->log->debug("CRTPNeuralNetworkEnsemble::fEvaluate...Tree %d weighted confidence %lf\n", iterTree->first, results[iterTree->first].second);
    }

    //Fusion results
    std::map<int, std::pair<int, double> >::iterator iterTreeResults;
    for (iterTreeResults = results.begin(); iterTreeResults != results.end(); ++iterTreeResults)
    {
        //std::cout<<"* LABEL "<<iterTreeResults->second.first<<" _>" <<iterTreeResults->second.second<<"\n";
        //std::cout<<"*added to LABEL "<<iterTreeResults->second.first <<" _>" <<fusionedResults[iterTreeResults->second.first]<<"\n";

        //fusionedResults[iterTreeResults->second.first] += iterTreeResults->second.second;
        fusionedResults[iterTreeResults->second.first] += exp(iterTreeResults->second.second);
        //fusionedResults[iterTreeResults->second.first] += 2*(iterTreeResults->second.second);
        finalResultLabelCounts[iterTreeResults->second.first]++; //otr ,ost votes
        //std::cout<<"*after added to LABEL "<<iterTreeResults->second.first <<" _>" <<fusionedResults[iterTreeResults->second.first]<<"\n";
    }

    //normalization
    for (iterResult = fusionedResults.begin(); iterResult != fusionedResults.end(); ++iterResult)
    {
        this->logger->log->debug("CRTPNeuralNetworkEnsemble::fEvaluate... final result normalization = false\n");
        //iterResult->second /= finalResultLabelCounts[iterResult->first];

        if (iterResult->second > maxConfidence)
        {
            maxConfidence = iterResult->second;
            maxLabel = iterResult->first;
        }
    }

    finalResult.first = maxLabel;
    finalResult.second = maxConfidence;

    clock_t end = clock();
    double time = static_cast<double>(end - start) / CLOCKS_PER_SEC;
    this->logger->log->debug("CRTPNeuralNetworkEnsemble::fEvaluate...Final vote %s conf %lf (in %lf sec)\n", CFileSettings::labels[finalResult.first].c_str(), finalResult.second, time);
    return finalResult;
}

int CRTPNeuralNetworkEnsemble::next_comb(int comb[], int k, int n)
{
    int i = k - 1;
    ++comb[i];
    while ((i >= 0) && (comb[i] >= n - k + 1 + i))
    {
        --i;
        ++comb[i];
    }

    if (comb[0] > n - k)
        return 0;

    for (i = i + 1; i < k; ++i)
        comb[i] = comb[i - 1] + 1;

    return 1;
}

std::vector<int*> CRTPNeuralNetworkEnsemble::computeCombinations(int n, int k)
{
    std::vector<int*> combinations;
    int* comb;
    comb = (int*) malloc(sizeof(int) * k);

    for (int i = 0; i < k; ++i)
        comb[i] = i;

    int *tempCom;
    tempCom = (int*) malloc(sizeof(int) * k);
    for (int iK = 0; iK < k; iK++)
    {
        tempCom[iK] = comb[iK];
    }
    combinations.push_back(tempCom);

    while (this->next_comb(comb, k, n))
    {
        int *tempCom;
        tempCom = (int*) malloc(sizeof(int) * k);
        for (int iK = 0; iK < k; iK++)
        {
            tempCom[iK] = comb[iK];
        }
        combinations.push_back(tempCom);
    }

    return combinations;
}

std::vector<std::vector<int> > CRTPNeuralNetworkEnsemble::generateRootCombinations(unsigned int numberOfNeurons)
{
    unsigned int maxK = numberOfNeurons; //number of classifers
    unsigned int numChildren = 2;

    this->logger->log->debug(" CRTPNeuralNetworkEnsemble::generateRootCombinations...start generating combinations...\n");

    std::vector<int*> combinations;
    std::vector<std::vector<int> > vecCombinations;

    combinations = this->computeCombinations(maxK, numChildren);
    for (unsigned int ic = 0; ic < combinations.size(); ic++)
    {
        std::vector<int> currComb;
        this->logger->log->debug(" CRTPNeuralNetworkEnsemble::generateRootCombinations...");
        for (unsigned int iK = 0; iK < numChildren; iK++)
        {
            currComb.push_back(combinations[ic][iK]);
            this->logger->log->debug(" %d ", combinations[ic][iK]);
        }
        this->logger->log->debug("\n");
        vecCombinations.push_back(currComb);
    }
    return vecCombinations;
}

SVerficationResult CRTPNeuralNetworkEnsemble::selfVerificationRTPNNModel()
{
    return this->verifyRTPNNModel(this->hModel);
}

SVerficationResult CRTPNeuralNetworkEnsemble::verifyRTPNNModel(std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > > toVerifyModel)
{
    this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...\n");
    SVerficationResult vr;
    std::map<int, std::map<int, std::map<int, std::map<int, std::vector<double> > > > >::iterator iterNeurons;
    std::map<int, std::map<int, std::map<int, std::vector<double> > > >::iterator iterPattern;

    //label, sum
    std::map<int, int> correctAll;

    //label, sum
    std::map<int, int> incorrectAll;

    std::map<int, int>::iterator iterResp;

    int correct = 0;
    int incorrect = 0;
    int total = 0;
    for (iterNeurons = toVerifyModel.begin(); iterNeurons != toVerifyModel.end(); ++iterNeurons)
    {
        std::map<int, std::map<int, std::map<int, std::vector<double> > > > pattern;
        pattern = iterNeurons->second;
        for (iterPattern = pattern.begin(); iterPattern != pattern.end(); ++iterPattern)
        {
            std::map<int, std::pair<int, double> > treeReponses;
            if (iterNeurons->first == this->fEvaluate(iterPattern->second, treeReponses).first)
            {
                this->logger->log->debug("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...Correct(%s)\n", CFileSettings::labels[iterNeurons->first].c_str());
                correct++;
                correctAll[iterNeurons->first]++;
            }
            else
            {
                this->logger->log->debug("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...Oh Oh(%s)\n", CFileSettings::labels[iterNeurons->first].c_str());
                incorrect++;
                incorrectAll[iterNeurons->first]++;
            }
            total++;
        }
    }

    vr.totalCorrect = (double) correct / (double) total;
    vr.totalFalse = (double) incorrect / (double) total;

    this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...Total Correct: %lf Incorrect: %lf", vr.totalCorrect, vr.totalFalse);

    for (iterResp = correctAll.begin(); iterResp != correctAll.end(); ++iterResp)
    {

        vr.correctLabel[iterResp->first] = (double) correctAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]);

        this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...Label %s Correct: %lf\n ", CFileSettings::labels[iterResp->first].c_str(), vr.correctLabel[iterResp->first]);
    }

    for (iterResp = incorrectAll.begin(); iterResp != incorrectAll.end(); ++iterResp)
    {
        vr.falseLabel[iterResp->first] = (double) incorrectAll[iterResp->first] / (double)((double) correctAll[iterResp->first] + (double) incorrectAll[iterResp->first]);

        this->logger->log->info("CRTPNeuralNetworkEnsemble::verifyRTPNNModel...Label %s Incorrect: %lf\n ", CFileSettings::labels[iterResp->first].c_str(), vr.falseLabel[iterResp->first]);

    }

    return vr;
}

void CRTPNeuralNetworkEnsemble::save(std::string filename)
{
    this->logger->log->info("CRTPNeuralNetworkEnsemble::save...\n");
    std::ofstream ofs((char*) filename.c_str());

    boost::archive::text_oarchive oa(ofs);
    // write class instance to archive
    oa << this;
    // archive and stream closed when destructors are called

}

void CRTPNeuralNetworkEnsemble::load(std::string filename)
{
    this->logger->log->info("CRTPNeuralNetworkEnsemble::load...\n");
    std::ifstream ifs((char*) filename.c_str());

    CRTPNeuralNetworkEnsemble t;
    boost::archive::text_iarchive ia(ifs);
    // write class instance to archive
    ia >> *this;

    //this = t;
    // archive and stream closed when destructors are called

}

