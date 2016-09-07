#include "object_decomposition.h"
#include "prob_neural_network/rtp_neural_network_ensemble.h"
#include <fstream>
#include <iostream>


using namespace std;

int main(int argc, char **argv)
{
    string model_filename = "";
    string binary_ending = "_binary";
    map<int, CObjectDecomposition> obj_decomp_class;
    CRTPNeuralNetworkEnsemble rtpnn_ensemble;

    try
    {
        // Object Decompisition Model File
        model_filename = "./ros/config/objectDecompostion/objDecomp_objDecomp.od";
        cout << "convert file " << model_filename << " from text to binary" << std::endl;
        ifstream ifs_obj_decomp(model_filename.c_str());
        boost::archive::text_iarchive ia_obj_decomp(ifs_obj_decomp);
        ia_obj_decomp >> obj_decomp_class;


        ofstream ofs_obj_decomp(string(model_filename + binary_ending).c_str());
        boost::archive::binary_oarchive oa_obj_decomp(ofs_obj_decomp);
        oa_obj_decomp << obj_decomp_class;


        // RTPNN Ensemble Model File
        model_filename = "./ros/config/rtpnnEnsemble/rtpnn_ensemble.rtp";
        cout << "convert file " << model_filename << " from text to binary" << endl;
        ifstream ifs_rtpnn(model_filename.c_str());
        boost::archive::text_iarchive ia_rtpnn(ifs_rtpnn);
        ia_rtpnn >> rtpnn_ensemble;


        ofstream ofs_rtpnn(string(model_filename + binary_ending).c_str());
        boost::archive::binary_oarchive oa_rtpnn(ofs_rtpnn);
        oa_rtpnn << rtpnn_ensemble;
    }
    catch (boost::archive::archive_exception ae)
    {
        cout << "could not convert file " << ae.what() << endl;
        return -1;
    }

    return 0;
}
