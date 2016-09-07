#include <mcr_blob_detection/blob_detection.h>

BlobDetection::BlobDetection()
{
    gray_image_ = NULL;
    blob_image_ = NULL;
}

BlobDetection::~BlobDetection()
{

}

int BlobDetection::detectBlobs(const Mat &mat_input_image, Mat &debug_image, vector<vector<double> > &blobs)
{
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_theta = 0.0;
    double blob_area = 0.0;

    Mat gray_image;
    cvtColor(mat_input_image, gray_image, CV_BGR2GRAY);

    Mat gray_blurred;
    bilateralFilter(gray_image, gray_blurred, 8, 5, 4);

    Canny(gray_blurred, gray_blurred, 10, 100, 3);

    Mat gray_filter;
    adaptiveThreshold(gray_blurred, gray_filter, 255,
                      //CV_ADAPTIVE_THRESH_GAUSSIAN_C,
                      CV_ADAPTIVE_THRESH_MEAN_C,
                      CV_THRESH_BINARY, 9, 9);

    bitwise_not(gray_filter, gray_filter);

    Mat gray_dilate;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(25, 25), Point(-1, -1));
    dilate(gray_filter, gray_dilate, kernel);

    erode(gray_dilate, gray_dilate, kernel);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(gray_dilate, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    // Draw contours
    Mat drawing = Mat::zeros(gray_dilate.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(drawing, contours, i, cv::Scalar(255, 255, 255), CV_FILLED);
    }

    IplImage input_image = drawing;

    if (!&input_image)
    {
        return -2; // Image not found
    }

    blob_image_ = cvCreateImage(cvGetSize(&input_image), IPL_DEPTH_8U, 3);
    gray_image_ = cvCreateImage(cvGetSize(&input_image), IPL_DEPTH_8U, 1);

    cvCvtColor(&input_image, gray_image_, CV_BGR2GRAY);

    blobs_ = CBlobResult(gray_image_, NULL, 0);
    blobs_.Filter(blobs_, B_EXCLUDE, CBlobGetArea(), B_LESS, min_blob_area_);
    blobs_.Filter(blobs_, B_EXCLUDE, CBlobGetArea(), B_GREATER, max_blob_area_);

    if (debug_mode_)
    {
        cvMerge(gray_image_, gray_image_, gray_image_, NULL, blob_image_);
    }

    if (blobs_.GetNumBlobs() > 0)
    {

        blobs.resize(blobs_.GetNumBlobs()); // Resize vector to match the number of blobs
        for (int x = 0; x < blobs_.GetNumBlobs(); x++)
        {
            blobs[x].resize(4); // Resize vector width to hold 4 properties of the blob
            CBlob  temp_blob;
            temp_blob = blobs_.GetBlob(x);
            pose_x = ((temp_blob.MinX() + temp_blob.MaxX()) / 2);
            pose_y = ((temp_blob.MinY() + temp_blob.MaxY()) / 2);
            pose_theta = get_blob_orientation_(temp_blob);
            if (pose_theta > 180)
            {
                pose_theta = pose_theta - 180;
            }
            blob_area = get_blob_area_(temp_blob);
            blobs[x][0] = pose_x;
            blobs[x][1] = pose_y;
            blobs[x][2] = pose_theta;
            blobs[x][3] = blob_area;
            if (debug_mode_)
            {
                temp_blob.FillBlob(blob_image_, CV_RGB(0, 255, 0));
                Mat mat_img(blob_image_);
                mat_img.copyTo(debug_image);
            }
        }

        cvSetZero(gray_image_);
        cvSetZero(blob_image_);
        cvReleaseImage(&gray_image_);
        cvReleaseImage(&blob_image_);

        return 1; //Blobs Detected

    }
    else
    {
        return -1; //No Blobs Detected
    }

    return 0; //Unidentified Error

}

void BlobDetection::updateDynamicVariables(bool debug_mode, int min_blob_area, int max_blob_area)
{
    debug_mode_ = debug_mode;
    min_blob_area_ = min_blob_area;
    max_blob_area_ = max_blob_area;
    return;
}
