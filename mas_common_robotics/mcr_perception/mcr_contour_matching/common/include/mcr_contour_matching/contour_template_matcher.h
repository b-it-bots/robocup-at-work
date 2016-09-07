#ifndef CONTOUR_TEMPLATE_MATCHER_H_
#define CONTOUR_TEMPLATE_MATCHER_H_

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

/**
 * Finds the best match from a list of 3D contours with a given template pointcloud
 */
class ContourTemplateMatcher
{
public:
    /**
     * Constructor
     */
    ContourTemplateMatcher();

    /**
     * Destructor
     */
    virtual ~ContourTemplateMatcher();

    /**
     * Finds the contour from contours that best matches template_cloud.
     * Both the contours and template_cloud are transformed to their respective
     * eigenvector spaces. For a given contour, the closest points to each point in the
     * template cloud is found and the distances between the contour and template points are summed.
     * The contour with the minimum sum of distances is said to match the template cloud the best.
     * The sum of distances is returned as the matching error.
     *
     * @param contours
     * List of contours that are to be compared to template cloud.
     *
     * @param template_cloud
     * Template contour which is being matched.
     *
     * @param matched_contour
     * Best matching contour from contours. This is passed as a reference and modified in place.
     *
     * @return matching error
     * Sum of distances from each point in template pointcloud to closest point in best matching contour.
     */
    double match(const std::vector<pcl::PCLPointCloud2::Ptr> &contours, const pcl::PCLPointCloud2::ConstPtr &template_cloud, pcl::PCLPointCloud2::Ptr &matched_contour);


private:
    /**
     * Copy constructor.
     */
    ContourTemplateMatcher(const ContourTemplateMatcher &other);

    /**
     * Copy assignment operator.
     */
    ContourTemplateMatcher &operator=(ContourTemplateMatcher other);


private:
    /**
     * pcl::PCA object for transforming pointcloud to its eigenvector space using PCA.
     */
    pcl::PCA<pcl::PointXYZ> pca_;
};
#endif
