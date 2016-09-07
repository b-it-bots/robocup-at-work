#include <mcr_contour_matching/contour_template_matcher.h>
#include <pcl/kdtree/kdtree_flann.h>

ContourTemplateMatcher::ContourTemplateMatcher()
{
}

ContourTemplateMatcher::~ContourTemplateMatcher()
{
}

double ContourTemplateMatcher::match(const std::vector<pcl::PCLPointCloud2::Ptr> &contours, const pcl::PCLPointCloud2::ConstPtr &template_cloud, pcl::PCLPointCloud2::Ptr &matched_contour)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*template_cloud, *xyz_template_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr template_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pca_.setInputCloud(xyz_template_cloud);

    // transform template pointcloud to its eigenvector space
    pca_.project(*xyz_template_cloud, *template_transformed);

    double min_sum = std::numeric_limits<double>::max();
    int min_index = -1;

    for (int i = 0; i < contours.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_contour(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*(contours.at(i)), *xyz_contour);
        pca_.setInputCloud(xyz_contour);

        // transform contour pointcloud to its eigenvector space
        pca_.project(*xyz_contour, *projected);

        double sum_of_distances = 0.0;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(projected);

        // for each point in the template, find the closest point in the contour
        for (int j = 0; j < template_transformed->points.size(); j++)
        {
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(template_transformed->points.at(j), 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                sum_of_distances += pointNKNSquaredDistance.at(0);
            }
        }

        // find the contour with the minimum sum of distances
        if (sum_of_distances < min_sum)
        {
            min_sum = sum_of_distances;
            min_index = i;
        }
    }

    *matched_contour = *(contours.at(min_index));
    return min_sum;
}
