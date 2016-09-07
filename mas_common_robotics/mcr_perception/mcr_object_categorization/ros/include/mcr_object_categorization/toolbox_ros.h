/*
 *  CToolBox.h
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#ifndef CTOOLBOXROS_H
#define CTOOLBOXROS_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <limits.h>

#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/range_image/range_image.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/impl/mls.hpp>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>

#include "struct_planar_surface.h" //since we need the StructPlanarSurface
class CToolBoxROS
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool subsampling(pcl::PointCloud<pcl::PointXYZ> &cloud, const double dLeafsize);
    bool subsampling(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const double dLeafsize);
    bool subsampling(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const double dLeafsize);

    bool filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float thresholdDistance);

    double dotProduct(std::vector<double> &x, std::vector<double> &y);
    double dotProduct(float* x, float* y);
    double dotProduct(pcl::PointXYZRGBNormal &x, pcl::PointXYZRGBNormal &y);

    pcl::PointCloud<pcl::PointXYZ> filterDistance(pcl::PointCloud<pcl::PointXYZ> &cloud, float minDist, float maxDist, std::string axis);
    pcl::PointCloud<pcl::PointXYZRGB> filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float minDist, float maxDist, std::string axis);
    bool statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int meanK, float stddevMulThresh);
    bool statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ> &cloud, int meanK, float stddevMulThresh);

    pcl::PointCloud<pcl::PointXYZRGBNormal> movingLeastSquares(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float searchRadius = 0.005f);
    pcl::PointCloud<pcl::PointNormal> movingLeastSquares(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius);
    pcl::PointCloud<pcl::PointXYZ> movingLeastSquares2(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius);

    pcl::PointCloud<pcl::Normal> estimatingNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int KSearch = 50);
    pcl::PointCloud<pcl::Normal> estimatingNormals(pcl::PointCloud<pcl::PointXYZ> &cloud, int KSearch = 50);
    pcl::PointCloud<pcl::Normal> estimatingNormalsIntegral(pcl::PointCloud<pcl::PointXYZ> &cloud);

    pcl::PointCloud<pcl::Normal> estimatingNormals2(pcl::PointCloud<pcl::PointXYZ> &cloud, double searchRadius = 0.005);
    pcl::PointCloud<pcl::Normal> estimatingNormals2(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double searchRadius = 0.005);
    pcl::PointCloud<pcl::Normal> estimatingNormalsOMP(pcl::PointCloud<pcl::PointXYZ> &cloud, double radius = 0.03);

    //  pcl::PointCloud<pcl::FPFHSignature33> FPFHFeatureExtractor(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius);

    void markClusteredPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clusteredPointCloud,
                                 pcl::PointCloud<pcl::PointXYZRGBNormal> &markedPointCloud);

    void markClusteredPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusteredPointCloud, pcl::PointCloud<pcl::PointXYZ> &markedPointCloud);

    bool transformPointCloud(tf::TransformListener &tfListener, std::string &fromFrame, std::string &toFrame, const sensor_msgs::PointCloud2 &srcPointCloud,
                             sensor_msgs::PointCloud2 &transformedPointCloud, float duration = 1.0);

    int pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointXYZRGBNormal &point);
    int pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud);
    int pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZ> &convexHull, pcl::PointXYZ &point);

    pcl::PointXYZRGBNormal centroidHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, float area);
    float areaConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &hull);
    float avgValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis);
    float minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis);
    float minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis);
    float minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis, pcl::PointXYZ &min_point);
    float maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis);
    float maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis);
    float maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis, pcl::PointXYZ &max_point);

    int overlapConvexHull2d(StructPlanarSurface &surface1, StructPlanarSurface &surface2);
    int overlapConvexHull2d2(StructPlanarSurface &surface1, StructPlanarSurface &surface2);

    bool distanceBetweenPlane2d(StructPlanarSurface &surface1, StructPlanarSurface &surface2, float distanceThreshold);
    bool isObjectPlane(StructPlanarSurface &surface, pcl::PointCloud<pcl::PointXYZRGBNormal> &object, float objectHeightThreshold,
                       float objectPlaneHeightDifferenceThreshold);

    pcl::PointCloud<pcl::Normal> filterNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud);

    pcl::PointCloud<pcl::PointXYZ> normalizePointCloud(pcl::PointCloud<pcl::PointXYZ> &point_cloud);

    pcl::PointCloud<pcl::Boundary> estimatingBoundaryPoints(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    pcl::PointXYZ pointCloudCentroid(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
    pcl::PointXYZ pointCloudCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud);
    pcl::PointXYZRGB pointCloudCentroid(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud);
    pcl::PointXYZRGB pointCloudCentroid2(pcl::PointCloud<pcl::PointXYZ> &point_cloud);
    pcl::PointXYZ pointCloudBoundingBoxCentroid(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointCloud<pcl::PointXYZ> &boundingBox);

    std::vector<double> normalizePoint3D(std::vector<double> &point);

    float angleBetweenPoints(std::vector<double> &point1, std::vector<double> &point2);

    void get3DPointsWithinHull(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLFull, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLHull,
                               const double dMinHeight, const double dMaxHeight, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLSegmentOutput);

    pcl::PointXYZRGBNormal getNearestNeighborPlane(StructPlanarSurface &plane, pcl::PointXYZRGBNormal &queryPoint);

    double euclDistanceBtwPoints(pcl::PointXYZ &p1, pcl::PointXYZ &p2);

    int pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointXYZ &point);

    double mean(std::vector<double> &histo);
    double variance(std::vector<double> &histo, double mean);

    std::vector<std::vector<double> > convertPointCloudToStdVec(pcl::PointCloud<pcl::PointXYZ> &pointCloud);

    int findMinValue(std::vector<double> &vec)
    {
        assert(vec.size() > 0);
        int minIdx = -1;
        int minValue = 0;

        minValue = std::numeric_limits<double>::infinity();
        for (unsigned int i = 0; i < vec.size(); i++)
        {
            if (vec[i] < minValue)
            {
                minValue = vec[i];
                minIdx = i;
            }
        }
        return minIdx;
    }

    int findMaxValue(std::vector<double> &vec)
    {
        assert(vec.size() > 0);
        int maxIdx = -1;
        int maxValue = 0;

        maxValue = std::numeric_limits<double>::epsilon();
        for (unsigned int i = 0; i < vec.size(); i++)
        {
            if (vec[i] > maxValue)
            {
                maxValue = vec[i];
                maxIdx = i;
            }
        }
        return maxIdx;
    }

    double meanSquaredError(std::vector<double> &inputVector1, std::vector<double> &inputVector2)
    {
        assert(inputVector1.size() == inputVector2.size());

        double sumError = 0.0;
        for (unsigned int i = 0; i < inputVector1.size(); ++i)
        {
            sumError += pow(inputVector1[i] - inputVector2[i], 2);
        }
        return sqrt(sumError / (double) inputVector1.size());
    }

    double meanSquareError(std::vector<double> &inputVector1)
    {
        double sumError = 0.0;
        for (unsigned int i = 0; i < inputVector1.size(); ++i)
        {
            sumError += pow(inputVector1[i], 2);
        }
        return sqrt(sumError / (double) inputVector1.size());
    }

    double meanError(std::vector<double> &vec)
    {
        double sum = 0;

        for (unsigned int i = 0; i < vec.size(); i++)
        {
            sum += pow(vec.at(i), 2);
        }
        return (double) sum / ((double) vec.size());
    }
    void stringTokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters)
    {
        // Skip delimiters at beginning.
        std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        // Find first "non-delimiter".
        std::string::size_type pos = str.find_first_of(delimiters, lastPos);

        while (std::string::npos != pos || std::string::npos != lastPos)
        {
            // Found a token, add it to the vector.
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            // Skip delimiters.  Note the "not_of"
            lastPos = str.find_first_not_of(delimiters, pos);
            // Find next "non-delimiter"
            pos = str.find_first_of(delimiters, lastPos);
        }
    }

    double randNumber(double low, double high);
    void extractHighestLowestPoint(pcl::PointCloud<pcl::PointXYZRGBNormal> &pointCloud, pcl::PointXYZ &highestPoint, pcl::PointXYZ &lowestPoint);
    double euclDistanceBtwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2);

    void enNoisePointCloud(pcl::PointCloud<pcl::PointXYZ> &pointCloud, double addNoise);
    void getNearestKPoints(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, pcl::PointXYZRGB &queryPoint, int k, std::vector<int> &k_indicies,
                           std::vector<float> &k_distances);
    void getNearestKPoints(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointXYZ &queryPoint, int k, std::vector<int> &k_indicies,
                           std::vector<float> &k_distances);
    void getNearestKPoints(pcl::KdTreeFLANN<pcl::PointXYZ> &tree, pcl::PointXYZ &queryPoint, int k, std::vector<int> &k_indicies,
                           std::vector<float> &k_distances);
};

#endif

