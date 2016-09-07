/*
 * Created on: Mar 18, 2011
 * Author: Christian Mueller
 */

#ifndef FILESETTINGS_H_
#define FILESETTINGS_H_
#include<vector>
#include<string>
#include<cstdlib>
#include<map>
#include<utility>

//#define NUM_CLASSES 8
#define NUM_CLASSES 8
#define LABEL_NEGATIVE 0
#define LABEL_CUP 1
#define LABEL_CAN 2
#define LABEL_BOX 3
#define LABEL_BOTTLE 4
#define LABEL_BOWL 5
#define LABEL_PLATE 6
#define LABEL_BALL 7
//#define LABEL_ADD0 8

#define TRAIN_NEGATIVE "queries/train/"
#define TRAIN_CUP "cup/train/"
#define TRAIN_CAN "can/train/"
#define TRAIN_BOWL "bowl/train/"
#define TRAIN_BOTTLE "bottle/train/"
#define TRAIN_BOX "box/train/"
#define TRAIN_PLATE "plate/train/"
#define TRAIN_BALL "ball/train/"
//#define TRAIN_ADD0 "stapler/train/"

#define NAME_NEGATIVE "query"
#define NAME_CUP "cup"
#define NAME_CAN "can"
#define NAME_BOWL "bowl"
#define NAME_BOTTLE "bottle"
#define NAME_BOX "box"
#define NAME_PLATE "plate"
#define NAME_BALL "ball"
//#define NAME_ADD0 "add0"

#define HOME_PATH "./ros/data/"

#define TRAIN_PATH "train/"
#define TEST_PATH "test/"

//combining geodsic shell approach with normals
#define KDESHELLNORMALESTIMATION_PATH "kdeShellNormalsEstimations/"
#define KDESHELLNORMALESTIMATION_POSTFIX "shellNormal.geo"
#define KDESHELLNORMALESTIMATION_TRAIN_POSTFIX "shellNormal.libsvm"

//geodesic shell approach
#define KDESHELLESTIMATION_PATH "kdeShellEstimations/"
#define KDESHELLESTIMATION_POSTFIX "shell.geo"
#define KDESHELLESTIMATION_TRAIN_POSTFIX "shell.libsvm"

//Normals approach
#define KDENORMALESTIMATION_PATH "kdeNormalsEstimations/"
#define KDENORMALESTIMATION_POSTFIX "normals.geo"
#define KDENORMALESTIMATION_TRAIN_POSTFIX "normal.libsvm"

//Normals with shell approach
#define KDENORMALSHELLESTIMATION_PATH "kdeSNormalsEstimations/"
#define KDENORMALSHELLESTIMATION_POSTFIX "snormals.geo"
#define KDENORMALSHELLESTIMATION_TRAIN_POSTFIX "snormal.libsvm"

#define KDENODEESTIMATION_PATH "nodeEstimation/"
#define KDENODEESTIMATION_POSTFIX "node.geo"
#define KDENODEESTIMATION_TRAIN_POSTFIX "node.libsvm"

#define KDEGEONODEESTIMATION_PATH "nodeEstimation/"
#define KDEGEONODEESTIMATION_TRAIN_UNSUP_POSTFIX "geoNode.geo"
#define KDEGEONODEESTIMATION_TRAIN_LIBSVM_POSTFIX "geoNode.libsvm"


#define BIN_CONFIG_NORMAL_PATH "kdeNormalBinConfig/"
#define BIN_CONFIG_NORMAL_POSTFIX "NormalBin.cfg"

#define BIN_CONFIG_SHELL_PATH "kdeShellBinConfig/"
#define BIN_CONFIG_SHELL_POSTFIX "shellBin.cfg"

#define RELATED_MODEL_ESTIMATIONS_PATH "relatedModelEstimations/"
#define RELATED_MODEL_ESTIMATIONS_POSTFIX "relModEst.geo"

#define SHELL_CONFIG_PATH "shellConfig/"
#define SHELL_CONFIG_POSTFIX "shell.cfg"

#define NEURAL_GAS_PATH "neuralGas/"
#define NEURAL_GAS_ASAP_POSTFIX "apsp.ng"
#define NEURAL_GAS_NEURALlGAS_POSTFIX "neuralGas.ng"

#define POINT_CLOUD_PATH "pointCloud/"
#define POINT_CLOUD_POSTIFX "pointCloud.pcd"

#define RTPNNENSEMBLE_PATH "rtpnnEnsemble/"
#define RTPNNENSEMBLE_POSTFIX "ensemble.rtp"
#define RTPNNENSEMBLE_FILENAME "rtpnn"

#define PNN_PATH "pnn/"
#define PNN_POSTFIX "pnn.pnn"
#define PNN_FILENAME "pnn"

#define KNN_PATH "knn/"
#define KNN_POSTFIX "knn.knn"
#define KNN_FILENAME "knn"

#define OBJECTDECOMPOSTION_PATH "objectDecompostion/"
#define OBJECTDECOMPOSTION_POSTFIX "objDecomp.od"
#define OBJECTDECOMPOSTION_FILENAME "objDecomp"

#define SVM_PATH "svm/"
#define SVM_CASCADE_PATH "svmCascade/"
#define SVM_MODEL_POSTFIX "model.svm"
#define SVM_MODEL_FILENAME "svm"
#define SVM_LINEAR_MODEL_FILENAME "svmLinear"
#define SVM_ONECLASS_MODEL_FILENAME "svmOneClass"
#define SVM_MODEL_CASCADE_FILENAME "svmCascade"




#define AUTOENCODER_MODEL_PATH "autoEncoder/"
#define AUTOENCODER_MODEL_POSTFIX "model.ae"
#define AUTOENCODER_MODEL_FILENAME "autoEncoder"

class CFileSettings
{
public:
    static std::map<int, std::string> trainPath;
    static std::map<int, std::string> labels;

    static void init()
    {

        CFileSettings::labels.insert(std::pair<int, std::string>(
                                         LABEL_NEGATIVE, NAME_NEGATIVE));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_CUP,
                                     NAME_CUP));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_CAN,
                                     NAME_CAN));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_BOX,
                                     NAME_BOX));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_BOTTLE,
                                     NAME_BOTTLE));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_BOWL,
                                     NAME_BOWL));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_PLATE,
                                     NAME_PLATE));
        CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_BALL,
                                     NAME_BALL));
        //CFileSettings::labels.insert(std::pair<int, std::string>(LABEL_ADD0,
        //              NAME_ADD0));

        CFileSettings::trainPath.insert(std::pair<int, std::string>(
                                            LABEL_NEGATIVE, TRAIN_NEGATIVE));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_CUP,
                                        TRAIN_CUP));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_CAN,
                                        TRAIN_CAN));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_BOX,
                                        TRAIN_BOX));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(
                                            LABEL_BOTTLE, TRAIN_BOTTLE));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_BOWL,
                                        TRAIN_BOWL));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(
                                            LABEL_PLATE, TRAIN_PLATE));
        CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_BALL,
                                        TRAIN_BALL));
        //CFileSettings::trainPath.insert(std::pair<int, std::string>(LABEL_ADD0,
        //              TRAIN_ADD0));
    }

    static void coutStdVector(std::vector<double> data, bool newLines = false);
    static void saveStdVector(std::string fileName, std::vector<double> data);
    //static void saveAppendStdVector(std::string fileName,
    //      std::vector<double> data, bool reCreateFile = false);
    static std::vector<std::vector<double> >
    loadStdVector(std::string fileName);

    static void
    saveLibSvmItem(bool isAppend, std::string fileName,
                   std::vector<int> labels,
                   std::vector<std::vector<double> > features);

    static void
    saveLibSvmItem2(bool isAppend, std::string fileName,
                    std::vector<int> labels,
                    std::vector<std::vector<double> > features);
    static void saveUnsupervisedItem(bool isAppend, std::string fileName, std::map<int, std::vector<double> > features);
    static void saveUnsupervisedItem(bool isAppend, std::string fileName, std::vector<std::vector<double> > features);

    static void tokenize(const std::string& str,
                         std::vector<std::string>& tokens, const std::string& delimiters =
                             " ");

};

#endif /* FILESETTINGS_H_ */

