/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#include "neural_gas/key_point_service_open_surf.h"

CKeyPointService_OpenSurf::CKeyPointService_OpenSurf()
{
    ;
}

void CKeyPointService_OpenSurf::extractedKeyPoints(std::string fileName,
        char label, std::vector<CKeyPoint> &keyPoints)
{
    int keyPointDim = 64;

    // Declare Ipoints and other stuff
    IpVec ipts;
    IplImage *img = cvLoadImage(string(fileName).c_str());//("imgs/sf.jpg");

    // Detect and describe interest points in the image
    clock_t start = clock();
    surfDetDes(img, ipts, false, 5, 4, 2, 0.00003f);// STANDARD-DEFAULT
    clock_t end = clock();

    std::cout << "OpenSURF found: " << ipts.size() << " interest points"
              << std::endl;
    std::cout << "OpenSURF took: " << float(end - start) / CLOCKS_PER_SEC
              << " seconds" << std::endl;

    cvNamedWindow("Example", 1);

    for (int i = 0; i < ipts.size(); i++)
        cvCircle(img, cvPoint(ipts.at(i).x, ipts.at(i).y), 1, cvScalar(100,
                 100, 255), 1);

    cvShowImage("Example", img);

    cvWaitKey(1);

    this->readKeyPointsOpenSurf(ipts, keyPointDim, keyPoints);

    cvReleaseImage(&img);
}

void CKeyPointService_OpenSurf::readKeyPointsOpenSurf(IpVec ipts,
        int keyPointDim, std::vector<CKeyPoint> &keyPoints)
{
    int numKeyPoints = 0;

    numKeyPoints = ipts.size();

    for (int i = 0; i < numKeyPoints; i++) //128
    {
        CKeyPoint sample(keyPointDim);
        sample.set_x_cord(ipts.at(i).x);
        sample.set_y_cord(ipts.at(i).y);
        sample.set_a(-1);
        sample.set_b(-1);
        sample.set_c(-1);

        for (int j = 0; j < keyPointDim; j++) //128
        {
            //  cout<< j<<"."<<temp<<endl;
            sample.add_feature(j, ipts.at(i).descriptor[j]);
        }
        //  cout<<i<<". feature"<<endl;
        keyPoints.push_back(sample);
    }
    cout << "*OpenSurf KeyPoints read" << endl;
}
