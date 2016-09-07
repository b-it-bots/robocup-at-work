/*
 * Created on: 19.04.2011
 * Author: Christian Mueller
 */

#ifndef __CKEYPOINTSERVICEOPENSURF_H__
#define __CKEYPOINTSERVICEOPENSURF_H__
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <opencv/cv.h>
#include <ctime>

//OpenSurf
#include "open_surf/surflib.h"
#include "open_surf/utils.h"
#include "neural_gas/key_point.h"


#define WINDOW_LIVE_IMAGE "Live Image Capture"
using namespace std;

class CKeyPointService_OpenSurf
{

public:
    CKeyPointService_OpenSurf();
    void extractedKeyPoints(string fileName, char label,
                            std::vector<CKeyPoint> &keyPoints);
    void readKeyPointsOpenSurf(IpVec ipts, int keyPointDim, std::vector <
                               CKeyPoint > &keyPoints);
};

#endif
