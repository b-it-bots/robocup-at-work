#include <algorithm>
#include <polyclipping/clipper.hpp>

#include "mcr_scene_segmentation/polyclipping.h"

void clipPlanarPolygon(PlanarPolygon& planar_polygon, double clip_by)
{
  using namespace ClipperLib;

  // Step 1: prepare a transformation to align the z-axis with plane normal.
  Eigen::Vector3f normal = planar_polygon.getCoefficients().head<3>();
  Eigen::Vector3f perpendicular(-normal[1], normal[0], normal[2]);
  Eigen::Affine3f transform = pcl::getTransFromUnitVectorsZY(normal, perpendicular);
  Eigen::Affine3f inverse_transform = transform.inverse(Eigen::Isometry);

  // Step 2: iterate through points transforming them, projecting on the plane
  // and pushing to a 2D polygon.
  Polygon polygon;
  const float SCALE = 1000.0;
  float z;
  for (const auto& point : planar_polygon.getContour())
  {
    Eigen::Vector3f p = transform * point.getVector3fMap();
    polygon.push_back(IntPoint(p[0] * SCALE, p[1] * SCALE));
    z = p[2];
  }
  // Check the orientation of the polygon and reverse if it is wrong.
  if (!ClipperLib::Orientation(polygon))
    std::reverse(polygon.begin(), polygon.end());

  if (!ClipperLib::Orientation(polygon))
  {
    throw std::runtime_error("Polygon orientation is wrong.");
  }

  // Step 3: offset polygon.
  Polygons polygons;
  polygons.push_back(polygon);
  OffsetPolygons(polygons, polygons, -SCALE * clip_by);

  // Step 4: create a new planar polygon based on the produced clipped polygon.
  PointCloud::VectorType points;
  for (size_t i = 0; i < polygons[0].size(); i++)
  {
    auto& poly_point = polygons[0][i];
    PointT point;
    Eigen::Vector3f p;
    p << poly_point.X / SCALE, poly_point.Y / SCALE, z;
    point.getVector3fMap() = inverse_transform * p;
    points.push_back(point);
  }
  planar_polygon = PlanarPolygon(points, planar_polygon.getCoefficients());
}

