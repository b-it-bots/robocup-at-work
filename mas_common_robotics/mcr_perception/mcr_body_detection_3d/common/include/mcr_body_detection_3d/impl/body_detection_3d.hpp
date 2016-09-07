/*
 * body_detection_3d.hpp
 *
 *  Created on: 02.12.2010
 *      Author: Frederik Hegger
 */

#ifndef BODY_DETECTION_3D_H_
#define BODY_DETECTION_3D_H_


#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mcr_algorithms/geometry/conversions.h>
#include <mcr_algorithms/geometry/geometric_distances.hpp>
#include <mcr_algorithms/geometry/geometric_properties.hpp>
#include <mcr_algorithms/machine_learning/random_trees.h>
#include <mcr_algorithms/segmentation/pointcloud_segmentation.hpp>
#include <mcr_algorithms/statistics/minmax.hpp>
#include <mcr_algorithms/wrapper/pcl_wrapper.hpp>

#include "mcr_body_detection_3d/datatypes.h"

/*##################################################
 *      NAMESPACES
  ###################################################*/
using namespace std;
using namespace pcl;


/*##################################################
 *      TYPEDEFS
  ###################################################*/
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> StatisticalGraph;
typedef StatisticalGraph::vertex_descriptor VertexID;
typedef StatisticalGraph::edge_descriptor EdgeID;


/*##################################################
 *      CLASS
  ###################################################*/
class BodyDetection3D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BodyDetection3D();
    ~BodyDetection3D();

    void loadModel(const string &filename);
    vector<Person> getPersonList(const PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud_input);

    template <typename PointT>
    int getPointCloudFeatureVector(const pcl::PointCloud<PointT> &pcl_cloud_segment, vector<double> &feature_vector, const double &sample_label);
    unsigned int getNumberOfPointCloudFeatures();

    // getter functions
    pcl::PointCloud<pcl::PointXYZRGB> getClassifiedCloud();

    // setter functions
    void setMinimumClustersPerPerson(const unsigned int &minimum_clusters_per_person)
    {
        this->minimum_clusters_per_person_ = minimum_clusters_per_person;
    };
    void setHistogramBinSize(const double &bin_size)
    {
        this->histogram_bin_size_ = bin_size;
    };

private:
    void preprocessAndSegmentPointCloud(const PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud_input, std::vector<Segment3DProperties> &body_segments, std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > &pcl_cloud_classified_3d_segments_);
    void createStatisticalGraph(std::vector<Segment3DProperties> &body_parts, StatisticalGraph &connectivity_graph);
    void splitGraphIntoConnectedComponents(StatisticalGraph &graph, vector<StatisticalGraph> &clustered_sub_graphs);
    void cluster3DSegmentsAccordingToGraph(vector<StatisticalGraph> &clustered_sub_graphs, std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > &pcl_cloud_classified_3d_segments_, std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > &clustered_3d_segments);
    void classify3DSegment(const std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > &clustered_3d_segments, vector<Person> &classified_persons);

    VertexID addVertex(pcl::PointXYZ &point, unsigned int &related_segment_id, double &probability, StatisticalGraph &graph);
    VertexID addVertex(pcl::PointXYZ &point, unsigned int &index, unsigned int &related_segment_id, double &probability, StatisticalGraph &graph, std::map<unsigned int, VertexID> &added_points);
    EdgeID addEdge(VertexID &vertex_start, VertexID &vertex_end, double &distance, StatisticalGraph &graph);

    template <typename PointT>
    int getPointCloudFeatureVector(const pcl::PointCloud<PointT> &pcl_cloud_segment, vector<double> &feature_vector);

    RandomTrees *random_tree_;
    PointCloudSegmentation<pcl::PointNormal> *cloud_segmentation_;

    PointCloud<PointNormal>::Ptr pcl_cloud_point_normals_;
    PointCloud<PointXYZ>::Ptr pcl_cloud_downsampled_;
    PointCloud<PointXYZ>::Ptr pcl_cloud_pass_through_tmp_;
    PointCloud<PointXYZ>::Ptr pcl_cloud_pass_through_;

    std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > pcl_cloud_segments_;
    std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > pcl_cloud_classified_3d_segments_;

    // passthrough filter parameters
    string height_axis_;
    double min_range_height_;
    double max_range_height_;
    string depth_axis_;
    double min_range_depth_;
    double max_range_depth_;

    // clustering parameters
    int min_cluster_size_;
    int max_cluster_size_;
    double cluster_tolerance_;

    // downsampling parameters
    double downsampling_leaf_size_;

    // normal parameters
    double normals_search_radius_;

    // segmentation
    double slice_height_;

    // feature parameter
    unsigned int histogram_bin_size_;
    double histogram_min_value_;
    double histogram_max_value_;

    // merging parameter
    double minimum_clusters_per_person_;

    unsigned int vertex_id_counter_;
};

BodyDetection3D::BodyDetection3D()
{
    // Downsampling parameters
    this->downsampling_leaf_size_ = 0.03;

    // Passthrough parameters
    this->height_axis_ = "z";
    this->min_range_height_ = 0.15;
    this->max_range_height_ = 2.0;
    this->depth_axis_ = "x";
    this->min_range_depth_ = 0.0;
    this->max_range_depth_ = 4.5;

    // cluster parameters
    this->min_cluster_size_ = 20;
    this->max_cluster_size_ = 100000;
    this->cluster_tolerance_ = 0.10;

    // normals parameter
    this->normals_search_radius_ = 0.04;

    // segmentation parameters
    this->slice_height_ = 0.2;

    // feature parameter
    this->histogram_bin_size_ = 7;
    this->histogram_min_value_ = -1.0;
    this->histogram_max_value_ = 1.0;

    // merging parameter
    minimum_clusters_per_person_ = 4;

    //create instances
    this->random_tree_ = new RandomTrees(this->getNumberOfPointCloudFeatures());

    this->cloud_segmentation_ = new PointCloudSegmentation<pcl::PointNormal>(this->min_range_height_, this->max_range_height_, this->slice_height_,
            this->cluster_tolerance_, this->min_cluster_size_, this->max_cluster_size_);

    this->pcl_cloud_pass_through_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);;
    this->pcl_cloud_pass_through_tmp_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);;
    this->pcl_cloud_downsampled_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);;
    this->pcl_cloud_point_normals_ = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);;
}

BodyDetection3D::~BodyDetection3D()
{
    if (this->random_tree_ != NULL)
        delete this->random_tree_;
    if (this->cloud_segmentation_ != NULL)
        delete this->cloud_segmentation_;
}

void BodyDetection3D::loadModel(const string &filename)
{
    // load models for random forest classifier
    if (this->random_tree_->loadModel(filename.c_str()) == 0)
        std::cout << "\tloaded random forest model " << filename << " successfully";
    else
    {
        std::cerr << "\tcould not load random forest model " << filename.c_str();
        exit(0);
    }
}

pcl::PointCloud<pcl::PointXYZRGB> BodyDetection3D::getClassifiedCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB> output;

    int color = 0;

    for (unsigned int i = 0; i < this->pcl_cloud_classified_3d_segments_.size(); ++i)
    {
        color = rand() % 1000;
        output.header = this->pcl_cloud_classified_3d_segments_[i].header;

        for (unsigned int j = 0; j < this->pcl_cloud_classified_3d_segments_[i].size(); ++j)
        {
            pcl::PointXYZRGB point_rgb;

            point_rgb.x = this->pcl_cloud_classified_3d_segments_[i].points[j].x;
            point_rgb.y = this->pcl_cloud_classified_3d_segments_[i].points[j].y;
            point_rgb.z = this->pcl_cloud_classified_3d_segments_[i].points[j].z;
            point_rgb.rgb = color;

            output.points.push_back(point_rgb);
        }
    }

    return output;
}


vector<Person> BodyDetection3D::getPersonList(const PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud_input)
{
    std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > clustered_3d_segments;
    std::vector<Segment3DProperties> body_segments;
    StatisticalGraph connectivity_graph;
    vector<StatisticalGraph> clustered_sub_graphs;
    vector<vector<double> > graph_feature_vector;
    vector<Person> person_list;


    this->preprocessAndSegmentPointCloud(pcl_cloud_input, body_segments, pcl_cloud_classified_3d_segments_);

    this->createStatisticalGraph(body_segments, connectivity_graph);

    this->splitGraphIntoConnectedComponents(connectivity_graph, clustered_sub_graphs);

    this->cluster3DSegmentsAccordingToGraph(clustered_sub_graphs, pcl_cloud_classified_3d_segments_, clustered_3d_segments);

    this->classify3DSegment(clustered_3d_segments, person_list);


    return person_list;
}

void BodyDetection3D::preprocessAndSegmentPointCloud(const PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud_input, std::vector<Segment3DProperties> &body_segments, std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > &pcl_cloud_classified_3d_segments_)
{
    PointCloud<PointNormal> pcl_cloud_slice;
    vector<PointIndices> cluster_indices;

    pcl_cloud_slice.header = pcl_cloud_input->header;   // TODO: needed????

    // cut down pointcloud
    PCLWrapper<pcl::PointXYZ>::passThroughFilter(pcl_cloud_input, this->pcl_cloud_pass_through_tmp_, this->depth_axis_, this->min_range_depth_, this->max_range_depth_);
    PCLWrapper<pcl::PointXYZ>::passThroughFilter(this->pcl_cloud_pass_through_tmp_, this->pcl_cloud_pass_through_, this->height_axis_, this->min_range_height_, this->max_range_height_);

    // subsampling
    PCLWrapper<pcl::PointXYZ>::downsampling(this->pcl_cloud_pass_through_, this->pcl_cloud_downsampled_, this->downsampling_leaf_size_);

    if (pcl_cloud_downsampled_->points.empty())
        return;

    // get normals
    PCLWrapper<pcl::PointXYZ>::computeNormals(this->pcl_cloud_downsampled_, this->pcl_cloud_point_normals_, this->normals_search_radius_);

    // segment scene by layering and euclidean clustering
    pcl_cloud_segments_.clear();
    this->cloud_segmentation_->getSegments(this->pcl_cloud_point_normals_, pcl_cloud_segments_);

    pcl_cloud_classified_3d_segments_.clear();

    // get each slices, cluster it and classify each segment
    for (unsigned int i = 0; i < pcl_cloud_segments_.size(); ++i)
    {
        //reject small segments
        if (pcl_cloud_segments_[i].size() <= (unsigned int) this->min_cluster_size_)
            continue;

        vector<double> feature_vector;
        this->getPointCloudFeatureVector(pcl_cloud_segments_[i], feature_vector);

        //classification
        LabelMap class_result;
        class_result = this->random_tree_->classify(feature_vector);

        if (class_result['1'] > class_result['2'])
        {
            Segment3DProperties body_part;
            Eigen::Vector4f centroid;

            GeometricProperties::getCentroid3D(pcl_cloud_segments_[i], centroid);

            body_part.centroid.x = centroid[0];
            body_part.centroid.y = centroid[1];
            body_part.centroid.z = centroid[2];
            body_part.probability = class_result['1'];

            body_segments.push_back(body_part);
            pcl_cloud_classified_3d_segments_.push_back(pcl_cloud_segments_[i]);
        }
    }
}

void BodyDetection3D::createStatisticalGraph(std::vector<Segment3DProperties> &body_parts, StatisticalGraph &connectivity_graph)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>);
    KdTreeFLANN<pcl::PointXYZ>::Ptr cluster_tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    std::map<unsigned int, VertexID>::iterator iter;
    VertexID query_point_id, nearst_point_id;
    std::map<unsigned int, VertexID> added_points;
    double eucl_dist = 0;
    unsigned int kk = 3;
    std::vector<int> indices(kk);
    std::vector<float> distances(kk);

    this->vertex_id_counter_ = 0;

    for (unsigned int i = 0; i < body_parts.size(); ++i)
        centroids->points.push_back(body_parts[i].centroid);

    if (centroids->points.size() < kk)
        return;

    cluster_tree->setInputCloud(centroids);

    for (unsigned int j = 0; j < centroids->points.size(); ++j)
    {
        if (!cluster_tree->nearestKSearch(j, kk, indices, distances))
            continue;

        if (indices.size() >= kk)
        {
            query_point_id = this->addVertex(centroids->points[j], j, j, body_parts[j].probability, connectivity_graph, added_points);

            // connect to k-nearest neighbors
            for (unsigned int n = 1; n < kk; ++n)
            {
                eucl_dist = GeometricDistances::getEuclideanDistance3D(centroids->points[j], centroids->points[indices[n]]);

                if (eucl_dist > 0.5 || eucl_dist == 0.0)
                    continue;

                unsigned int index = (unsigned int)indices[n];
                nearst_point_id = this->addVertex(centroids->points[index], index, index, body_parts[index].probability, connectivity_graph, added_points);

                this->addEdge(query_point_id, nearst_point_id, eucl_dist, connectivity_graph);
            }
        }
    }
}

void BodyDetection3D::splitGraphIntoConnectedComponents(StatisticalGraph &graph, vector<StatisticalGraph> &clustered_sub_graphs)
{
    std::vector<int> components(num_vertices(graph));
    int num = connected_components(graph, &components[0]);
    vector<StatisticalGraph> sub_graphs(num);
    std::map<unsigned int, VertexID> added_vertices;

    this->vertex_id_counter_ = 0;

    StatisticalGraph::edge_iterator edge_iter, edge_end;
    for (tie(edge_iter, edge_end) = edges(graph); edge_iter != edge_end; ++edge_iter)
    {
        EdgeID edge_id = *edge_iter;
        Edge & edge = graph[edge_id];

        VertexID start = source(*edge_iter, graph);
        VertexID end = target(*edge_iter, graph);
        Vertex vertex_s = graph[start];
        Vertex vertex_e = graph[end];

        VertexID v_s = this->addVertex(vertex_s.point, vertex_s.id, vertex_s.related_segment_id, vertex_s.probability, sub_graphs[components[start]], added_vertices);
        VertexID v_e = this->addVertex(vertex_e.point, vertex_e.id, vertex_e.related_segment_id, vertex_e.probability, sub_graphs[components[start]], added_vertices);

        this->addEdge(v_s, v_e, edge.eucl_distance, sub_graphs[components[start]]);
    }

    clustered_sub_graphs = sub_graphs;
}

void BodyDetection3D::cluster3DSegmentsAccordingToGraph(vector<StatisticalGraph> &clustered_sub_graphs, std::vector<pcl::PointCloud<PointNormal>, Eigen::aligned_allocator<pcl::PointCloud<PointNormal> > > &pcl_cloud_classified_3d_segments_, std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > &clustered_3d_segments)
{
    std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > tmp_segments(clustered_sub_graphs.size());

    for (unsigned int i = 0; i < clustered_sub_graphs.size(); ++i)
    {
        StatisticalGraph::vertex_iterator vertexIt, vertexEnd;
        boost::tie(vertexIt, vertexEnd) = vertices(clustered_sub_graphs[i]);

        double sum_probability = 0;
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
            VertexID vertexID = *vertexIt; // dereference vertexIt, get the ID
            Vertex & vertex = clustered_sub_graphs[i][vertexID];

            tmp_segments[i].pcl_cloud.header = pcl_cloud_classified_3d_segments_[vertex.related_segment_id].header;
            tmp_segments[i].pcl_cloud += pcl_cloud_classified_3d_segments_[vertex.related_segment_id];

            tmp_segments[i].number_of_segments++;
            sum_probability += vertex.probability;
        }

        tmp_segments[i].probability = sum_probability / tmp_segments[i].number_of_segments;
    }

    clustered_3d_segments.clear();
    for (unsigned int j = 0; j < tmp_segments.size(); ++j)
    {
        if (tmp_segments[j].number_of_segments > 0)
            clustered_3d_segments.push_back(tmp_segments[j]);
    }
}


void BodyDetection3D::classify3DSegment(const std::vector<Segment3D, Eigen::aligned_allocator<Segment3D > > &clustered_3d_segments, vector<Person> &classified_persons)
{
    Eigen::Vector4f centroid;
    pcl::PointNormal min_x, max_x, min_y, max_y, min_z, max_z;
    double distance = 0, angle = 0;

    // clear person list
    classified_persons.clear();

    // iterate over clustered graphs
    for (unsigned int i = 0; i < clustered_3d_segments.size(); ++i)
    {
        GeometricProperties::getCentroid3D(clustered_3d_segments[i].pcl_cloud, centroid);
        MinMax::determineMinMax3D(clustered_3d_segments[i].pcl_cloud, min_x, max_x, min_y, max_y, min_z, max_z);

        // establish person message and add it to the person list
        Person person;

        person.position_x = centroid[0];
        person.position_y = centroid[1];
        person.position_z = centroid[2];

        Conversions::cartesian2polar2D(person.position_x, person.position_y, distance, angle);
        person.orientation_yaw = angle;

        person.height = GeometricDistances::getEuclideanDistance3D(min_z, max_z);
        person.width = GeometricDistances::getEuclideanDistance3D(min_y, max_y);
        person.depth = GeometricDistances::getEuclideanDistance3D(min_x, max_x);
        person.probability = clustered_3d_segments[i].probability;

        if (clustered_3d_segments[i].number_of_segments < minimum_clusters_per_person_)
            continue;

        classified_persons.push_back(person);
    }
}

VertexID BodyDetection3D::addVertex(pcl::PointXYZ &point, unsigned int &index, unsigned int &related_segment_id, double &probability, StatisticalGraph &graph, std::map<unsigned int, VertexID> &added_points)
{
    std::map<unsigned int, VertexID>::iterator iter;
    VertexID vertex_id;

    iter = added_points.find(index);
    if (iter != added_points.end())
        vertex_id = iter->second;
    else
    {
        vertex_id = this->addVertex(point, related_segment_id, probability, graph);
        added_points[index] = vertex_id;
    }

    return vertex_id;
}

VertexID BodyDetection3D::addVertex(pcl::PointXYZ &point, unsigned int &related_segment_id, double &probability, StatisticalGraph &graph)
{
    Vertex v;

    v.point = point;
    v.related_segment_id = related_segment_id;
    v.id = this->vertex_id_counter_++;
    v.probability = probability;

    return boost::add_vertex(v, graph);
}

EdgeID BodyDetection3D::addEdge(VertexID &vertex_start, VertexID &vertex_end, double &distance, StatisticalGraph &graph)
{
    EdgeID edge_id;
    bool ok;
    boost::tie(edge_id, ok) = boost::add_edge(vertex_start, vertex_end, graph);

    if (ok)
        graph[edge_id].eucl_distance = distance;

    return edge_id;
}


unsigned int BodyDetection3D::getNumberOfPointCloudFeatures()
{
    vector<double> feature_vector;
    pcl::PointCloud<pcl::PointNormal> pcl_cloud_temp;
    const unsigned int cloud_size = 10;

    pcl_cloud_temp.points.resize(cloud_size);

    for (unsigned int i = 0; i < cloud_size; ++i)
        pcl_cloud_temp.points[i].x = pcl_cloud_temp.points[i].y = pcl_cloud_temp.points[i].z = pcl_cloud_temp.points[i].normal[0] = pcl_cloud_temp.points[i].normal[1] = pcl_cloud_temp.points[i].normal[2] = 1.5;

    return this->getPointCloudFeatureVector(pcl_cloud_temp, feature_vector);
}

template <typename PointT>
int BodyDetection3D::getPointCloudFeatureVector(const pcl::PointCloud<PointT> &pcl_cloud_segment, vector<double> &feature_vector, const double &sample_label)
{
    unsigned int feature_counter = 0;
    feature_counter = this->getPointCloudFeatureVector(pcl_cloud_segment, feature_vector);
    feature_vector.insert(feature_vector.begin(), sample_label);

    return feature_counter;
}

template <typename PointT>
int BodyDetection3D::getPointCloudFeatureVector(const pcl::PointCloud<PointT> &pcl_cloud_segment, vector<double> &feature_vector)
{
    double step_size = fabs(this->histogram_min_value_ - this->histogram_max_value_) / this->histogram_bin_size_;
    unsigned int feature_counter = 0;

    feature_vector.clear();

    double x_min, x_max, y_min, y_max, z_min, z_max;
    MinMax::determineMinMax3D(pcl_cloud_segment, x_min, x_max, y_min, y_max, z_min, z_max);
    feature_vector.push_back(fabs(x_min - x_max));
    ++feature_counter;
    feature_vector.push_back(fabs(y_min - y_max));
    ++feature_counter;
    feature_vector.push_back(fabs(z_min - z_max));
    ++feature_counter;

    double step = this->histogram_min_value_;
    double histogram[this->histogram_bin_size_][3];
    unsigned int count_x = 0, count_y = 0, count_z = 0;
    for (unsigned int i = 0; i < this->histogram_bin_size_; ++i, step += step_size)
    {
        histogram[i][0] = 0.0;
        histogram[i][1] = 0.0;
        histogram[i][2] = 0.0;

        for (unsigned int j = 0; j < pcl_cloud_segment.points.size(); ++j)
        {
            if (pcl_cloud_segment.points[j].normal[0] >= step && pcl_cloud_segment.points[j].normal[0] < (step + step_size))
                histogram[i][0] += 1.0;
            if (pcl_cloud_segment.points[j].normal[1] >= step && pcl_cloud_segment.points[j].normal[1] < (step + step_size))
                histogram[i][1] += 1.0;
            if (pcl_cloud_segment.points[j].normal[2] >= step && pcl_cloud_segment.points[j].normal[2] <= (step + step_size))
                histogram[i][2] += 1.0;
        }

        count_x += histogram[i][0];
        count_y += histogram[i][1];
        count_z += histogram[i][2];
    }

    //normalize histogram and collect in the return vector
    // histogram over x normals
    for (unsigned int j = 0; j < this->histogram_bin_size_; ++j)
    {
        feature_vector.push_back((histogram[j][0] / count_x));
        ++feature_counter;
    }
    // histogram over y normals
    for (unsigned int j = 0; j < this->histogram_bin_size_; ++j)
    {
        feature_vector.push_back((histogram[j][1] / count_y));
        ++feature_counter;
    }
    // histogram over y normals
    for (unsigned int j = 0; j < this->histogram_bin_size_; ++j)
    {
        feature_vector.push_back((histogram[j][2] / count_z));
        ++feature_counter;
    }

    return feature_counter;
}

#endif /* BODY_DETECTION_3D_H_ */

