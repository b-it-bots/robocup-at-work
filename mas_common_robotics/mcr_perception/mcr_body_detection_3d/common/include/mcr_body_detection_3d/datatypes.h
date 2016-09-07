/* Copyright [2013] <Bonn-Rhein-Sieg University>
 *
 * datatypes.h
 *
 *  Created on: May 22, 2013
 *      Author: Frederik Hegger
 */

#ifndef MCR_BODY_DETECTION_3D_DATATYPES_H
#define MCR_BODY_DETECTION_3D_DATATYPES_H

struct Person
{
    Person() : position_x(0.0), position_y(0.0), position_z(0.0), orientation_yaw(0.0), height(0.0), width(0.0),
        depth(0.0), probability(0.0) {}
    double position_x;
    double position_y;
    double position_z;
    double orientation_yaw;
    double height;
    double width;
    double depth;
    double probability;
};

struct Segment3D
{
    Segment3D() : number_of_segments(0), probability(0.0) {}
    pcl::PointCloud<pcl::PointNormal> pcl_cloud;
    unsigned int number_of_segments;
    double probability;
};

struct Segment3DProperties
{
    Segment3DProperties() : probability(0.0) {}
    pcl::PointXYZ centroid;
    double probability;
};

struct Vertex
{
    Vertex() : related_segment_id(0), id(0), probability(0.0) {}
    pcl::PointXYZ point;
    unsigned int related_segment_id;
    unsigned int id;
    double probability;
};

struct Edge
{
    Edge() : eucl_distance(0.0) {}
    double eucl_distance;
};


#endif  // MCR_BODY_DETECTION_3D_DATATYPES_H
