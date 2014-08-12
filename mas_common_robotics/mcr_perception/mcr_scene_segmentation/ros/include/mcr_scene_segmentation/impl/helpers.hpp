#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <mcr_perception_msgs/PlanarPolygon.h>
#include <mcr_perception_msgs/BoundingBox.h>

#include "mcr_scene_segmentation/aliases.h"
#include "mcr_scene_segmentation/bounding_box.h"

/** Convert from PCL PlanarPolygon to ROS message. */
void convertPlanarPolygon(const PlanarPolygon& polygon, mcr_perception_msgs::PlanarPolygon& polygon_msg)
{
  for (int i = 0; i < 4; ++i)
  {
    polygon_msg.coefficients[i] = polygon.getCoefficients()[i];
  }
  for (const auto& point : polygon.getContour())
  {
    geometry_msgs::Point32 pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    polygon_msg.contour.push_back(pt);
  }
}

/** Convert from ROS message to PCL PlanarPolygon. */
void convertPlanarPolygon(const mcr_perception_msgs::PlanarPolygon& polygon_msg, PlanarPolygon& polygon)
{
  PointCloud::VectorType contour;
  Eigen::Vector4f coefficients(polygon_msg.coefficients.elems);
  for (const auto& point : polygon_msg.contour)
  {
    PointT pt;
    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    contour.push_back(pt);
  }
  polygon = PlanarPolygon(contour, coefficients);
}

double computePlanarPolygonArea(const PlanarPolygon& polygon)
{
  const auto& normal = polygon.getCoefficients();
  const auto& points = polygon.getContour();

  // Find axis with largest normal component and project onto perpendicular plane
  int k0, k1, k2;
  k0 = (std::fabs(normal[0]) > std::fabs(normal[1])) ? 0 : 1;
  k0 = (std::fabs(normal[k0]) > std::fabs(normal[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;

  double area = 0;
  for (size_t i = 0; i < points.size(); i++)
  {
    size_t j = (i + 1) % points.size();
    float p1[3] = { points[i].x, points[i].y, points[i].z };
    float p2[3] = { points[j].x, points[j].y, points[j].z };
    area += p1[k1] * p2[k2] - p1[k2] * p2[k1];
  }

  return std::fabs(area) / (2 * std::fabs(normal[k0]));
}

/** Convert from BoundingBox object to ROS message. */
void convertBoundingBox(const BoundingBox& bounding_box, mcr_perception_msgs::BoundingBox& bounding_box_msg)
{
  const auto& center = bounding_box.getCenter();
  bounding_box_msg.center.x = center[0];
  bounding_box_msg.center.y = center[1];
  bounding_box_msg.center.z = center[2];
  bounding_box_msg.dimensions.x = bounding_box.getDimensions()[0];
  bounding_box_msg.dimensions.y = bounding_box.getDimensions()[1];
  bounding_box_msg.dimensions.z = bounding_box.getDimensions()[2];
  for (const auto& vertex : bounding_box.getVertices())
  {
    geometry_msgs::Point pt;
    pt.x = vertex[0];
    pt.y = vertex[1];
    pt.z = vertex[2];
    bounding_box_msg.vertices.push_back(pt);
  }
}

#endif /* HELPERS_HPP */

