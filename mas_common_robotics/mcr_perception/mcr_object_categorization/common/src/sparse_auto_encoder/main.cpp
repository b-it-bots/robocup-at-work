/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include <iostream>
#include <CAutoEncoder.h>
#include <CFileSettings.h>
#include <CStackedAutoEncoder.h>
#include <CLogger.h>
#include <vector>

void testAutoEncoder()
{
    std::vector<double> testVec;
    std::vector<double> encodeDecodeVec;
    std::vector<double> encodeVec;
    std::vector<double> decodeVec;

    testVec.push_back(0.1);
    testVec.push_back(0.01);
    testVec.push_back(0.09);
    testVec.push_back(0.05);
    testVec.push_back(0.05);
    testVec.push_back(0.3);
    testVec.push_back(0.1);
    testVec.push_back(0.1);
    testVec.push_back(0.15);
    testVec.push_back(0.05);

    CAutoEncoder autoEncoder; //(testVec.size(), 1, testVec.size());
//    autoEncoder.input(testVec);
//   autoEncoder.input(std::string("./data/1_11_geoNode.geo"));
    autoEncoder.input(std::string("./data/cupshell.geo"));
    autoEncoder.train(0.0001, 5);
    //autoEncoder.encode(std::string("./data/1_0_geoNode.geo"));
    //autoEncoder.encode(std::string("./data/1_0_geoNode.geo"));

    encodeDecodeVec =  autoEncoder.encodeDecode(testVec);
    for (unsigned int  i = 0; i < encodeDecodeVec.size(); ++i)
    {
        std::cout << "\nAE encode decode " << encodeDecodeVec[i] << "\n";
    }

    encodeVec =  autoEncoder.encode(testVec);
    for (unsigned int  i = 0; i < encodeVec.size(); ++i)
    {
        std::cout << "\nAE encode " << encodeVec[i] << "\n";
    }


    decodeVec =   autoEncoder.decode(encodeVec);
    for (unsigned int  i = 0; i < decodeVec.size(); ++i)
    {
        std::cout << "\nAE decode " << decodeVec[i] << "\n";
    }


    //normalize, sparse
    //Stoachsatoc graoient! loop random pick of set !!!! a
}


void testStackedAutoEncoder()
{
    CStackedAutoEncoder sae(1);

    sae.input(std::string("./data/1_11_geoNode.geo"));
    sae.train(0.01, 10); //150);

    std::vector<std::vector<double> > test = CFileSettings::loadStdVector("./data/1_11_geoNode.geo");

    /*for(int i = 0; i < test.size();++i)
    {
        CFileSettings::coutStdVector(sae.encode(test[i]),true);
    }*/

    std::vector<double> testVec = CFileSettings::loadStdVector("./data/1_11_geoNode.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/1_0_geoNode.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/3_5_1_geoNode.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/7_29_1_geoNode.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/6_13_1_geoNode.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

}

void testStackedAutoEncoder2()
{
    std::vector<double> encodeDecodeVec;
    std::vector<double> encodeVec;
    std::vector<double> decodeVec;


    /*
     std::vector<double> testVec;
    std::vector<double> encodeDecodeVec;
    std::vector<double> encodeVec;
    std::vector<double> decodeVec;

    testVec.push_back(0.1);
    testVec.push_back(0.01);
    testVec.push_back(0.09);
    testVec.push_back(0.05);
    testVec.push_back(0.05);
    testVec.push_back(0.3);
    testVec.push_back(0.1);
    testVec.push_back(0.1);
    testVec.push_back(0.15);
    testVec.push_back(0.05);*/

    CStackedAutoEncoder sae(1);

    sae.input(std::string("./data/1_0_geoNode.geo"));
    sae.trainOpt(0.01, 10); //150);

    // std::vector<std::vector<double> > test = CFileSettings::loadStdVector("./data/1_11_geoNode.geo");


    // std::vector<double> en = sae.encode(test[0]);
    //  for(int i = 0; i < test.size();++i)
    // {
    //   CFileSettings::coutStdVector(en,true);
    // }
    //std::cout<<"\n"<<en.size()<<"\n";


    //std::vector<double> testVec = CFileSettings::loadStdVector("./data/3_5_1_geoNode.geo")[0];

    std::cout << "\n";
    std::vector<std::vector<double> >testVecVec = CFileSettings::loadStdVector("./data/1_0_geoNode.geo");
    std::cout << "Error1 " <<  sae.encodeDecode(testVecVec) << "\n";

    std::vector<std::vector<double> >testVecVec11 = CFileSettings::loadStdVector("./data/1_1_geoNode.geo");
    std::cout << "Error11 " <<  sae.encodeDecode(testVecVec11) << "\n";

    std::vector<std::vector<double> >testVecVec111 = CFileSettings::loadStdVector("./data/1_11_geoNode.geo");
    std::cout << "Error111 " <<  sae.encodeDecode(testVecVec111) << "\n";

    std::vector<std::vector<double> >testVecVec2 = CFileSettings::loadStdVector("./data/3_5_1_geoNode.geo");
    std::cout << "Error2 " <<  sae.encodeDecode(testVecVec2) << "\n";

    std::vector<std::vector<double> >testVecVec3 = CFileSettings::loadStdVector("./data/7_29_1_geoNode.geo");
    std::cout << "Error3 " <<  sae.encodeDecode(testVecVec3) << "\n";
    /*


    CFileSettings::coutStdVector(testVec,true);

    encodeDecodeVec=  sae.encodeDecode(testVec);
    CFileSettings::coutStdVector(encodeDecodeVec,true);

    encodeVec =  sae.encode(testVec);
    CFileSettings::coutStdVector(encodeVec,true);

    decodeVec =  sae.decode(encodeVec);
    CFileSettings::coutStdVector((decodeVec),true);



    testVec = CFileSettings::loadStdVector("./data/3_5_1_geoNode.geo")[0];



    CFileSettings::coutStdVector(testVec,true);

    encodeDecodeVec=  sae.encodeDecode(testVec);
    CFileSettings::coutStdVector(encodeDecodeVec,true);

    encodeVec =  sae.encode(testVec);
    CFileSettings::coutStdVector(encodeVec,true);

    decodeVec =  sae.decode(encodeVec);
    CFileSettings::coutStdVector((decodeVec),true);
    */
}


void testStackedAutoEncoder3()
{
    CStackedAutoEncoder sae(1);

    sae.input(std::string("./data/cupshell.geo"));
    sae.trainOpt(0.01, 100); //150);

    //std::vector<std::vector<double> > test = CFileSettings::loadStdVector("./data/1_11_geoNode.geo");

    /*for(int i = 0; i < test.size();++i)
    {
        CFileSettings::coutStdVector(sae.encode(test[i]),true);
    }*/

    std::vector<double> testVec = CFileSettings::loadStdVector("./data/1_7_object4-1-12_shellNormal.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    CFileSettings::loadStdVector("1_14_object5-1-43_shellNormal.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/2_0_1_2_object4-2-9_relModEst.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/2_4_1_19_object1-2-40_relModEst.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/3_0_1_0_object0-3-4_relModEst.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

    testVec = CFileSettings::loadStdVector("./data/3_1_1_24_object3-3-22_relModEst.geo")[0];
    CFileSettings::coutStdVector(sae.encodeDecode(testVec), true);

}

int main()
{
    CLogger *logger = &CLogger::getInstance();

    logger->log->debug("START...");

    testAutoEncoder();
    //testStackedAutoEncoder3();

    logger->log->debug("DONE...");
    return 0;
}