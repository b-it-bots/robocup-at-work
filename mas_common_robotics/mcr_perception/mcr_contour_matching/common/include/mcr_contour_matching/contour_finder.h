#ifndef CONTOUR_FINDER_H_
#define CONTOUR_FINDER_H_

#include <pcl/PCLPointCloud2.h>
#include <opencv2/core/core.hpp>

/**
 * Finds 2D contours after applying edge detection
 * Finds corresponding 3D contour using the given pointcloud
 */
class ContourFinder
{
public:
    /**
     * Constructor
     */
    ContourFinder();
    /**
     * Destructor
     */
    virtual ~ContourFinder();
    /**
     * Finds 2D contours in the image using edge detection. Displays the edges in the debug image
     *
     * @param image
     *          image in which to find the contours
     * @param debug_image
     *          the found edges are displayed in this image
     * @return found contours
     */
    std::vector<std::vector<cv::Point> > find2DContours(const cv::Mat &image, cv::Mat &debug_image);
    /**
     * Finds 3D contours corresponding to the given 2D contours in the input cloud
     *
     * @param contours
     *          input 2D contours
     * @param input_cloud
     *          input pointcloud
     * @return found 3D contours
     */
    std::vector<pcl::PCLPointCloud2::Ptr> get3DContours(const std::vector<std::vector<cv::Point> > &contours, pcl::PCLPointCloud2::Ptr input_cloud);

    /**
     * Set threshold1 for Canny edge detector
     * See http://docs.opencv.org/modules/imgproc/doc/feature_detection.html#canny
     *
     * @param canny_threshold
     * Threshold 1 for canny edge detection
     */
    void setCannyThreshold(double canny_threshold);

    /**
     * Set multiplier to calculate threshold2 for Canny edge detector
     * threshold2 = canny_multiplier * canny_threshold_
     * See http://docs.opencv.org/modules/imgproc/doc/feature_detection.html#canny
     *
     * @param canny_multiplier
     * Multiplier used to calculate threshold 2 for canny edge detection
     */
    void setCannyMultiplier(double canny_multiplier);


private:
    /**
     * Copy constructor.
     */
    ContourFinder(const ContourFinder &other);

    /**
     * Copy assignment operator.
     */
    ContourFinder &operator=(ContourFinder other);


private:
    /**
     * Threshold for canny edge detector
     */
    double canny_threshold_;
    /**
     * Multiplier for canny edge detector
     */
    double canny_multiplier_;
};
#endif
