#ifndef BLOBDETECTION_H_
#define BLOBDETECTION_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cvblobs/BlobResult.h>

using namespace cv;

class BlobDetection
{
public:
    BlobDetection();
    virtual ~BlobDetection();
    int detectBlobs(const Mat &mat_input_image, Mat &debug_image, vector<vector<double> > &blobs);
    void updateDynamicVariables(bool debug_mode, int min_blob_area, int max_blob_area);

private:
    IplImage *gray_image_;
    IplImage *blob_image_;
    CBlobGetOrientation get_blob_orientation_;
    CBlobGetArea get_blob_area_;
    bool debug_mode_;
    CBlobResult blobs_;
    CBlob largest_blob_;
    int min_blob_area_;
    int max_blob_area_;
};

#endif /* BLOBDETECTION_H_ */
