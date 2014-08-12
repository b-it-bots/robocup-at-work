#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class BoundingBox
{

  public:

    typedef Eigen::Vector3f Point;
    typedef std::vector<Point, Eigen::aligned_allocator<Point> > Points;

    inline const Point& getCenter() const { return center_; }

    inline const Points& getVertices() const { return vertices_; }

    inline Eigen::Vector3f getDimensions() const { return dimensions_; }

    inline float getVolume() const { return dimensions_[0] * dimensions_[1] * dimensions_[2]; }

    /** Create a bounding box around the cloud, restricting it to be parallel to
      * the plane defined by the normal. */
    static BoundingBox create(const typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                              const Eigen::Vector3f& normal);

    /** Create a bounding box around the point vector, restricting it to be
      * parallel to the plane defined by the normal. */
    static BoundingBox create(const typename pcl::PointCloud<pcl::PointXYZRGB>::VectorType& points,
                              const Eigen::Vector3f& normal);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Point center_;
    Points vertices_;
    Eigen::Vector3f dimensions_;

};

#endif
