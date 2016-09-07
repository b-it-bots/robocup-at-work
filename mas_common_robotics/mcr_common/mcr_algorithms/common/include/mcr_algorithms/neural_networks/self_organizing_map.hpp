/*
 * self_organizing_map.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: Frederik Hegger
 */

#ifndef SELF_ORGANIZING_MAP_H_
#define SELF_ORGANIZING_MAP_H_

//########### INCLUDES #############################
#include <ros/ros.h>
#include <boost/random.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "mcr_algorithms/geometry/geometric_distances.hpp"

//########### DEFINES ##############################
#define DEBUG_LEVEL             0
#define WAIT_IMG                100
#define DEBUG_WINDOW_NAME       "SOM"

//########### NAMESPACES ###########################
using namespace std;

//########### NEW DATA STRUCTURES ##################
struct Point
{
    Point() : x(0), y(0), z(0) {};
    double x;
    double y;
    double z;
};

struct Neuron
{
    Point position;
};

struct Connection
    {};

//########### TYPEDEFS #############################
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Neuron, Connection> SOMMap;
typedef SOMMap::vertex_descriptor NeuronID;
typedef SOMMap::edge_descriptor ConnectionID;

//########### CLASS #############################
class SelfOrganizingMap
{
public:
    // supported distance functions
    enum { EUCLIDEAN_DISTANCE, MANHATTEN_DISTANCE };
    //supported
    enum { CUBE_3D, CUBE_3D_RANDOMIZED };

    SelfOrganizingMap();
    ~SelfOrganizingMap();

    template <typename PointT>
    void updateMap(vector<PointT, Eigen::aligned_allocator<PointT> > training_data2, unsigned int max_iterations, double initial_learning_rate);
    void initMapModel(unsigned int num_neurons, double min_x, double max_x, double min_y, double max_y, double mean_z);
    void pruneMap(double distance_threshold);

    //DEBUG FUNCTION ToDo:
    SOMMap getMap()
    {
        return this->data_map_;
    };
private:
    NeuronID addNeuron(Point position);
    bool findNeuron(Point position, NeuronID& found_neuron);
    ConnectionID addConnection(NeuronID &neuron_start, NeuronID &neuron_end);

    template <typename PointT>
    NeuronID findBMU(PointT sample);

    //Debug Functions ToDo:
    void drawMap();

    SOMMap data_map_;
    SOMMap model_;
    unsigned int distance_function_;
    unsigned int model_type_;
    boost::mt19937 random_number_generator_;
    double map_radius_;

    //Debug Variable
    IplImage* som_img_;
};

SelfOrganizingMap::SelfOrganizingMap()
{
    //set default values
    this->distance_function_ = MANHATTEN_DISTANCE;
    this->model_type_ = CUBE_3D;
    this->random_number_generator_.seed(time(0));

    this->som_img_ = 0;
    if (DEBUG_LEVEL > 0)
    {
        this->som_img_ = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
        cvNamedWindow(DEBUG_WINDOW_NAME, 1);
    }
}

SelfOrganizingMap::~SelfOrganizingMap()
{
    //cleanup
    if (DEBUG_LEVEL > 0)
    {
        cvDestroyWindow(DEBUG_WINDOW_NAME);
        cvReleaseImage(&this->som_img_);
    }
}

void SelfOrganizingMap::initMapModel(unsigned int num_neurons, double min_x, double max_x, double min_y, double max_y, double mean_z)
{
    this->data_map_.clear();

    //ToDo::TEMP MODEL ONLY FOR TESTING
    cout << "min x: " << min_x << " max_x: " << max_x << " min_y: " << min_y << " max_y: " << max_y << " mean z: " << mean_z << endl;
    double sqrt_neurons = sqrt((double)num_neurons);
    double width = fabs(min_x - max_x);
    double height = fabs(min_y - max_y);

    cout << "num neurons: " << num_neurons << "sqrt neurons: " << sqrt_neurons << " width: " << width << " height: " << height << endl;

    double step_size_x = (width / sqrt_neurons);
    double step_size_y = (height / sqrt_neurons);
    double start_x = min_x;
    double start_y = min_y;
    double end_x = max_x;
    double end_y = max_y;

    cout << "step_size_x " << step_size_x << " step_size_y " << step_size_y << " start_x " << start_x << " start_y " << start_y <<
         " end_x " << end_x << " end_y " << end_y  << endl;

    Point neighbor_left, neighbor_right, neighbor_bottom, neighbor_top;
    NeuronID found_neuron_id;

    for (double x = start_x; x < end_x; x += step_size_x)
    {
        for (double y = start_y; y < end_y; y += step_size_y)
        {
            Point p;

            p.x = mean_z;
            p.y = x;
            p.z = y;

            this->addNeuron(p);
        }
    }

    cout << "num neurons: " << num_vertices(this->data_map_) << endl;

    if (DEBUG_LEVEL > 10)
        getchar();

    SOMMap::vertex_iterator neuron_iter, neuron_end;
    for (boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
    {
        NeuronID neuron_id = *neuron_iter;

        neighbor_left = neighbor_right = neighbor_bottom = neighbor_top = this->data_map_[neuron_id].position;

        neighbor_left.y  -= step_size_x;
        neighbor_right.y += step_size_x;
        neighbor_top.z   -= step_size_y;
        neighbor_bottom.z  += step_size_y;

        if (neighbor_left.y >= start_x)
            if (this->findNeuron(neighbor_left, found_neuron_id))
                this->addConnection(neuron_id, found_neuron_id);

        if (neighbor_right.y <= end_x)
            if (this->findNeuron(neighbor_right, found_neuron_id))
                this->addConnection(neuron_id, found_neuron_id);

        if (neighbor_bottom.z >= start_y)
            if (this->findNeuron(neighbor_bottom, found_neuron_id))
                this->addConnection(neuron_id, found_neuron_id);

        if (neighbor_top.z <= end_y)
            if (this->findNeuron(neighbor_top, found_neuron_id))
                this->addConnection(neuron_id, found_neuron_id);
    }

    if (this->model_type_ == CUBE_3D_RANDOMIZED)
    {
        for (boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
        {
            NeuronID neuron_id = *neuron_iter;

            boost::uniform_int<> generate_x(min_x, max_x);
            boost::uniform_int<> generate_y(min_y, max_y);
            this->data_map_[neuron_id].position.x = generate_x(this->random_number_generator_);
            this->data_map_[neuron_id].position.y = generate_y(this->random_number_generator_);
        }
    }

    this->map_radius_ = max(fabs((double)end_x - (double)start_x), fabs((double)end_y - (double)start_y)) / 2    ;
    cout << "map radius: " << this->map_radius_ << endl;
}

NeuronID SelfOrganizingMap::addNeuron(Point position)
{
    Neuron n;

    n.position = position;

    return boost::add_vertex(n, this->data_map_);
}

bool SelfOrganizingMap::findNeuron(Point position, NeuronID& found_neuron)
{
    SOMMap::vertex_iterator neuron_iter, neuron_end;
    for (boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
    {
        NeuronID neuron_id = *neuron_iter;

        if (this->data_map_[neuron_id].position.x == position.x && this->data_map_[neuron_id].position.y == position.y && this->data_map_[neuron_id].position.z == position.z)
        {
            found_neuron = neuron_id;
            return true;
        }
    }

    return false;
}

ConnectionID SelfOrganizingMap::addConnection(NeuronID &neuron_start, NeuronID &neuron_end)
{
    ConnectionID connection_id;
    bool edge_exists = false;
    bool egde_added = false;

    //check if edge already exists
    boost::tie(connection_id, edge_exists) = boost::edge(neuron_start, neuron_end, this->data_map_);
    if (edge_exists)
        return connection_id;

    //add egde
    boost::tie(connection_id, egde_added) = boost::add_edge(neuron_start, neuron_end, this->data_map_);

    return connection_id;
}

template<typename PointT>
void SelfOrganizingMap::updateMap(vector<PointT, Eigen::aligned_allocator<PointT> > training_data2, unsigned int max_iterations, double initial_learning_rate)
{
    vector<Point> training_data;
    //convert to own repre
    for (unsigned int i = 0; i < training_data2.size(); ++i)
    {
        Point p;

        p.x = training_data2[i].x;
        p.y = training_data2[i].y;
        p.z = training_data2[i].z;

        training_data.push_back(p);
    }


    double current_learning_rate = initial_learning_rate;
    double time_constant = (double)max_iterations / log(this->map_radius_);

    for (unsigned int cur_iteration = max_iterations, iter_count = 0; cur_iteration > 1; --cur_iteration, ++iter_count)
    {
        //cout << "iteration: " << cur_iteration << endl;

        //clear image
        if (DEBUG_LEVEL > 10 || (DEBUG_LEVEL > 9 && cur_iteration == 2))
        {
            cvSet(this->som_img_, cvScalar(0, 0, 0));

            //draw training data
            for (unsigned int j = 0; j < training_data.size(); ++j)
                cvCircle(this->som_img_, cvPoint(training_data[j].x, training_data[j].y), 1, cvScalar(0, 255, 0), 1);

            this->drawMap();
        }

        //get a random training sample
        boost::uniform_int<> generate(0, (training_data.size() - 1));
        int index = generate(this->random_number_generator_);
        Point sample = training_data[index];

        //draw random sample
        if (DEBUG_LEVEL > 10 || (DEBUG_LEVEL > 9 && cur_iteration == 2))
        {
            cvCircle(this->som_img_, cvPoint(sample.x, sample.y), 4, cvScalar(0, 255, 255), 5);

            cvShowImage(DEBUG_WINDOW_NAME, this->som_img_);
            cvWaitKey(WAIT_IMG);
            //getchar();
        }

        //find best matching neuron for the training sample
        NeuronID bm_neuron = this->findBMU(sample);

        //draw best matching neuron
        if (DEBUG_LEVEL > 10 || (DEBUG_LEVEL > 9 && cur_iteration == 2))
        {
            cvCircle(this->som_img_, cvPoint(this->data_map_[bm_neuron].position.x, this->data_map_[bm_neuron].position.y), 3, cvScalar(0, 0, 255), 3);

            cvShowImage(DEBUG_WINDOW_NAME, this->som_img_);
            cvWaitKey(WAIT_IMG);
        }

        //get neighborhood of best matching neuron, dependent of a decreasing radius, BUT first try with direct neighborhood ToDo:

        double neighborhood_radius = this->map_radius_ * exp(-(double)iter_count / time_constant);

        //cout << "learning rate: " << current_learning_rate << " time constant: " << time_constant << " neighborhood radius: " << neighborhood_radius << endl;


        this->data_map_[bm_neuron].position.x += current_learning_rate * (sample.x - this->data_map_[bm_neuron].position.x);
        this->data_map_[bm_neuron].position.y += current_learning_rate * (sample.y - this->data_map_[bm_neuron].position.y);
        this->data_map_[bm_neuron].position.z += current_learning_rate * (sample.z - this->data_map_[bm_neuron].position.z);
        //cout << "BMU x: " << this->data_map_[bm_neuron].position.x << " y: " << this->data_map_[bm_neuron].position.y << endl;
        //cout << "sample x: " << sample.x << " y: " << sample.y << "vec index: " << index << "vec:size:" << training_data.size() << endl;

        if (iter_count < (max_iterations / 1.5))
        {
            SOMMap::adjacency_iterator neighbor_iter, neighbor_end;
            for (boost::tie(neighbor_iter, neighbor_end) = adjacent_vertices(bm_neuron, this->data_map_); neighbor_iter != neighbor_end; ++neighbor_iter)
                ///////////////////////////SOMMap::vertex_iterator neuron_iter, neuron_end;
                /////////////////////////for(boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
            {
                NeuronID neuron_id = *neighbor_iter; // dereference vertexIt, get the ID

                double cur_distance;
                if (this->distance_function_ == EUCLIDEAN_DISTANCE)
                    cur_distance = GeometricDistances::getEuclideanDistance3D(this->data_map_[bm_neuron].position, this->data_map_[neuron_id].position);
                else if (this->distance_function_ == MANHATTEN_DISTANCE)
                    cur_distance = GeometricDistances::getManhattanDistance3D(this->data_map_[bm_neuron].position, this->data_map_[neuron_id].position);

                //check if distance between bmu and current neuron is with in the neighborhood radius
                //if((cur_distance * cur_distance) < (neighborhood_radius * neighborhood_radius))
                {
                    /*
                     *
                     * neighborhood function 1/(1 + (dist(j, k) / (unitwidth^2 * decreasing_paramete^2))
                     *
                     *
                     */

                    //calculate impact to this neuron
                    double impact = exp(-((cur_distance * cur_distance) / (2 * (neighborhood_radius * neighborhood_radius))));

                    //cout << "impact: " << impact << endl;

                    //adjust the weight of the current neuron

                    //  cout << "OLD x: " << this->data_map_[neuron_id].position.x << " y: " << this->data_map_[neuron_id].position.y << endl;

                    //cout << "diff : " << cur_distance << endl;
                    //cout << "diff half: " << 0.5 * cur_distance << endl;

                    //              cout << "diff x: " << (sample.x - this->data_map_[neuron_id].position.x) << "\ty: " << (sample.y - this->data_map_[neuron_id].position.y) << endl;

                    if (DEBUG_LEVEL > 10 || (DEBUG_LEVEL > 9 && cur_iteration == 2))
                    {
                        cvCircle(this->som_img_, cvPoint(this->data_map_[neuron_id].position.x, this->data_map_[neuron_id].position.y), 3, cvScalar(255, 0, 0), 3);
                        //cvShowImage(DEBUG_WINDOW_NAME, this->som_img_);
                        //cvWaitKey(WAIT_IMG);
                        //getchar();
                    }

                    this->data_map_[neuron_id].position.x += current_learning_rate * (sample.x - this->data_map_[neuron_id].position.x);
                    this->data_map_[neuron_id].position.y += current_learning_rate * (sample.y - this->data_map_[neuron_id].position.y);
                    this->data_map_[neuron_id].position.z += current_learning_rate * (sample.z - this->data_map_[neuron_id].position.z);

                    //cout << "NEW x: " << this->data_map_[neuron_id].position.x << " y: " << this->data_map_[neuron_id].position.y << endl;

                    if (iter_count < (max_iterations / 3))
                    {
                        SOMMap::adjacency_iterator neighbor_iter2, neighbor_end2;
                        for (boost::tie(neighbor_iter2, neighbor_end2) = adjacent_vertices(neuron_id, this->data_map_); neighbor_iter2 != neighbor_end2; ++neighbor_iter2)
                            ///////////////////////////SOMMap::vertex_iterator neuron_iter, neuron_end;
                            /////////////////////////for(boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
                        {
                            NeuronID neuron_id2 = *neighbor_iter2; // dereference vertexIt, get the ID
                            this->data_map_[neuron_id2].position.x += current_learning_rate * (sample.x - this->data_map_[neuron_id2].position.x);
                            this->data_map_[neuron_id2].position.y += current_learning_rate * (sample.y - this->data_map_[neuron_id2].position.y);
                            this->data_map_[neuron_id2].position.z += current_learning_rate * (sample.z - this->data_map_[neuron_id2].position.z);

                            if (iter_count < (max_iterations / 6))
                            {
                                SOMMap::adjacency_iterator neighbor_iter3, neighbor_end3;
                                for (boost::tie(neighbor_iter3, neighbor_end3) = adjacent_vertices(neuron_id2, this->data_map_); neighbor_iter3 != neighbor_end3; ++neighbor_iter3)
                                    ///////////////////////////SOMMap::vertex_iterator neuron_iter, neuron_end;
                                    /////////////////////////for(boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
                                {
                                    NeuronID neuron_id3 = *neighbor_iter3; // dereference vertexIt, get the ID
                                    this->data_map_[neuron_id3].position.x += current_learning_rate * (sample.x - this->data_map_[neuron_id3].position.x);
                                    this->data_map_[neuron_id3].position.y += current_learning_rate * (sample.y - this->data_map_[neuron_id3].position.y);
                                    this->data_map_[neuron_id3].position.z += current_learning_rate * (sample.z - this->data_map_[neuron_id3].position.z);
                                }
                            }
                        }
                    }

                }
            }
        }

        //update learning rate
        current_learning_rate = initial_learning_rate * exp(-(double)iter_count / (double)cur_iteration);

        if (DEBUG_LEVEL > 10)
            getchar();
        /*
        if(DEBUG_LEVEL > 10)
        {
            this->drawMap();
            cvShowImage(DEBUG_WINDOW_NAME, this->som_img_);
            cvWaitKey(WAIT_IMG);
        }*/


    }
}

void SelfOrganizingMap::pruneMap(double distance_threshold)
{
    double distance = 0;
    vector<ConnectionID> edges_to_be_removed;

    SOMMap::edge_iterator connection_iter, connection_end;
    for (tie(connection_iter, connection_end) = edges(this->data_map_); connection_iter != connection_end; ++connection_iter)
    {
        ConnectionID connection_id = *connection_iter;

        NeuronID start_neuron = source(*connection_iter, this->data_map_);
        NeuronID end_neuron = target(*connection_iter, this->data_map_);

        distance = GeometricDistances::getEuclideanDistance3D(this->data_map_[start_neuron].position, this->data_map_[end_neuron].position);

        cout << "DISTANCE: " << distance << endl;

        //check if there are edges above a certain threshold, if yes remove this edges
        if (distance > distance_threshold)
        {
            edges_to_be_removed.push_back(connection_id);
            ROS_DEBUG("DELETED edge neuron");
            /*
                        //check if start and end neuron of the edge are still connected to other neurons, if not delete the respective neuron
                        SOMMap::adjacency_iterator neighbour_it, neighbour_end;
                        boost::tie(neighbour_it, neighbour_end) = adjacent_vertices(start_neuron, this->data_map_);
                        if(neighbour_it == neighbour_end)
                        {
                            remove_vertex(start_neuron, this->data_map_);
                            ROS_DEBUG("DELETED start neuron");
                        }

                        boost::tie(neighbour_it, neighbour_end) = adjacent_vertices(end_neuron, this->data_map_);
                        if(neighbour_it == neighbour_end)
                        {
                            remove_vertex(end_neuron, this->data_map_);
                            ROS_DEBUG("DELETED end neuron");
                        }*/
        }
    }

    for (unsigned int i = 0; i < edges_to_be_removed.size(); ++i)
        remove_edge(edges_to_be_removed[i], this->data_map_);

}

template <typename PointT>
NeuronID SelfOrganizingMap::findBMU(PointT sample)
{
    double min_weight = DBL_MAX, current_weight = 0;
    NeuronID min_weight_neuron_id;

    //iterate over all neurons in the current data map
    SOMMap::vertex_iterator neuron_iter, neuron_end;
    for (boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
    {
        NeuronID neuron_id = *neuron_iter; // dereference vertexIt, get the ID

        if (this->distance_function_ == EUCLIDEAN_DISTANCE)
            current_weight = GeometricDistances::getEuclideanDistance3D(sample, this->data_map_[neuron_id].position);
        else if (this->distance_function_ == MANHATTEN_DISTANCE)
            current_weight = GeometricDistances::getManhattanDistance3D(sample, this->data_map_[neuron_id].position);

        if (current_weight < min_weight)
        {
            min_weight_neuron_id = neuron_id;
            min_weight = current_weight;
        }
    }

    return min_weight_neuron_id;
}

void SelfOrganizingMap::drawMap()
{
    //draw neurons
    SOMMap::vertex_iterator neuron_iter, neuron_end;
    for (boost::tie(neuron_iter, neuron_end) = vertices(this->data_map_); neuron_iter != neuron_end; ++neuron_iter)
    {
        NeuronID neuron_id = *neuron_iter; // dereference vertexIt, get the ID

        cvCircle(this->som_img_, cvPoint(this->data_map_[neuron_id].position.x, this->data_map_[neuron_id].position.y), 2, cvScalar(255, 255, 255), 2);
    }

    //draw connections between neurons
    SOMMap::edge_iterator connection_iter, connection_end;
    for (boost::tie(connection_iter, connection_end) = edges(this->data_map_); connection_iter != connection_end; ++connection_iter)
    {
        NeuronID neuron_start = source(*connection_iter, this->data_map_);
        NeuronID neuron_end = target(*connection_iter, this->data_map_);

        cvLine(this->som_img_, cvPoint(this->data_map_[neuron_start].position.x, this->data_map_[neuron_start].position.y),
               cvPoint(this->data_map_[neuron_end].position.x, this->data_map_[neuron_end].position.y), cvScalar(255, 255, 255), 1);
    }
}
/*
int main(int argc, char** argv)
{
    //ros::init(argc, argv, "test_som");
    //ros::NodeHandle n;

    SelfOrganizingMap *som = new SelfOrganizingMap();
    vector<Point> training_data;
    Point p;

    boost::mt19937 random_number_generator;
    random_number_generator.seed(time(0));
    for(unsigned int i=0; i < 30; ++i)
    {
        boost::uniform_int<> generate(300, 340); //245, 395
        boost::uniform_int<> generate2(0, 40);  //0, 150

        p.x = generate(random_number_generator);
        p.y = generate2(random_number_generator);
        p.z = 0;

        training_data.push_back(p);
    }

    for(unsigned int i=0; i < 150; ++i)
    {
        boost::uniform_int<> generate(250, 390);  //20, 620
        boost::uniform_int<> generate2(40, 150); //150, 480

        p.x = generate(random_number_generator);
        p.y = generate2(random_number_generator);
        p.z = 0;

        training_data.push_back(p);
    }

    int min_x, max_x, min_y, max_y;
    min_x = min_y = INT_MAX;
    max_x = max_y = -INT_MAX;
    for(unsigned int i=0; i < training_data.size(); ++i)
    {
        if(training_data[i].x < min_x)
            min_x = training_data[i].x;
        if(training_data[i].x > max_x)
            max_x = training_data[i].x;
        if(training_data[i].y < min_y)
            min_y = training_data[i].y;
        if(training_data[i].y > max_y)
            max_y = training_data[i].y;
    }

    cout << "train size: " << training_data.size() << endl;

    //ros::Time timer_start = ros::Time::now();
    cout << "load" << endl;
    som->initMapModel(training_data.size(), min_x, max_x, min_y, max_y, 0.0);
    cout << "update" << endl;
    som->updateMap(training_data, 10000, 0.9);
    //cout << "took " << (ros::Time::now() - timer_start) << endl; // TODO: remove for release version

    if(DEBUG_LEVEL > 9)
        getchar();

    delete som;

    return 0;
}*/

#endif /* SELF_ORGANIZING_MAP_H_ */
