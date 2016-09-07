#include <mcr_contour_matching/contour_finder.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ContourFinder::ContourFinder() : canny_threshold_(220), canny_multiplier_(3)
{
}

ContourFinder::~ContourFinder()
{
}

std::vector<std::vector<cv::Point> > ContourFinder::find2DContours(const cv::Mat &image, cv::Mat &debug_image)
{
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    cv::blur(gray_image, gray_image, cv::Size(3, 3));

    cv::Mat canny_output;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Canny edge detection
    cv::Canny(gray_image, canny_output, canny_threshold_, canny_threshold_ * canny_multiplier_, 3);
    canny_output.copyTo(debug_image);

    // Dilate edges so that the contours are expanded slightly
    // When converted to 3D this ensures that most points of the contour are on the same plane
    int dilation_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                      cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                      cv::Point(dilation_size, dilation_size));

    cv::dilate(canny_output, canny_output, element);

    canny_output.copyTo(debug_image);

    // find contours
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

    std::vector<std::vector<cv::Point> > filtered_contours;

    // filter the contours so we keep only those in the bottom half of the image
    // TODO: move this to another component
    for (int i = 0; i < contours.size(); i++)
    {
        cv::Point2f circle_center;
        float radius;

        cv::minEnclosingCircle((cv::Mat)contours[i], circle_center, radius);

        //if (circle_center.y > canny_output.rows / 2) {
        filtered_contours.push_back(contours.at(i));
        //  }
    }

    return filtered_contours;
}

std::vector<pcl::PCLPointCloud2::Ptr> ContourFinder::get3DContours(const std::vector<std::vector<cv::Point> > &contours, pcl::PCLPointCloud2::Ptr input_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*input_cloud, *xyz_input_cloud);

    // loop through points in the 2D contour and find their 3D positions in the given pointcloud
    std::vector<pcl::PCLPointCloud2::Ptr> pcl_contours;

    for (size_t i = 0; i < contours.size(); i++)
    {
        pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t j = 0; j < contours[i].size(); j++)
        {
            pcl::PointXYZ pcl_point = xyz_input_cloud->at(contours[i][j].x, contours[i][j].y);

            if ((!pcl_isnan(pcl_point.x)) && (!pcl_isnan(pcl_point.y)) && (!pcl_isnan(pcl_point.z)))
            {
                xyz_contour->points.push_back(pcl_point);
            }
        }

        // remove outliers in the pointcloud. this ensures the points are roughly on the same plane
        xyz_contour->header = xyz_input_cloud->header;

        if (xyz_contour->points.size() > 0)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(xyz_contour);
            sor.setMeanK(50);
            sor.setStddevMulThresh(3.0);
            sor.filter(*xyz_contour);
            pcl::toPCLPointCloud2(*xyz_contour, *pcl_contour);
            pcl_contours.push_back(pcl_contour);
        }
    }

    return pcl_contours;
}

void ContourFinder::setCannyThreshold(double canny_threshold)
{
    canny_threshold_ = canny_threshold;
}

void ContourFinder::setCannyMultiplier(double canny_multiplier)
{
    canny_multiplier_ = canny_multiplier;
}
