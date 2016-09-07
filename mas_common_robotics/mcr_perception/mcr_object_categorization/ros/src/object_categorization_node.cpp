/*
 *
 *  Created on: 21.04.2011
 *      Author: Christian Mueller
 */

#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <sstream>
#include <omp.h>
#include <deque>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <ros/publisher.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include <mcr_perception_msgs/Object.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/GetObjectList.h>

#include "toolbox_ros.h"
#include "object_categorization_geometric.h"
#include "file_settings.h"

#define BASE_LINK_FRAME "/base_link"

#define TRACKING 0
#define DO_CHECKPERSPECTIVE_CHANGE 0

#define LEARN_NEW_CAT 0
int newClassLabel = NUM_CLASSES;

ros::Publisher markerObjectCategorizationGeometricPublisher;
ros::Publisher testPublisher;
CObjectCategorizationGeometric *objectCategorizationGeometric;
CToolBoxROS toolBox;
std::string homePath;

boost::mutex mutexStampUpdateTrackedObjects;
boost::mutex mutexCategorizationThread;
boost::mutex mutexConcatResultCategorizationThread;
boost::mutex mutexCentroidMovementsThread;
bool runningCategorizationThread = false;
size_t queueSize = 10;  //10
int tag;

mcr_perception_msgs::ObjectList object_list;

ros::NodeHandle* g_nh_ptr = NULL;
ros::Subscriber sub_pointcloud;
ros::Publisher pub_object_list;
ros::ServiceServer srv_get_objects;

static unsigned int runningCounter = 0;
struct SPointCloudObjects
{
    std::map<int, pcl::PointCloud<pcl::PointXYZ> > objects;
    std::vector<int> observeredObjects;  // id of last frame observed objects
    std::map<int, pcl::PointXYZRGB> centroids;
    std::map<int, float> centroidMovements;  //this is the distance of the neareset objectcentroid which the current query could be associated to.
    std::map<int, int> confidence;
    pcl::PointCloud<pcl::PointXYZRGB> centroidsPCD;

    std::map<int, std::deque<std::pair<int, double> > > classifications;

    int stamp;

} trackedObjects;

bool checkPerspectiveChange(std::vector<int> &observeredObjects, float thd)
{
    float meanDistance = 0;
    int counts = 0;

    mutexCentroidMovementsThread.lock();
    for (unsigned int i = 0; i < observeredObjects.size(); ++i)
    {
        if (trackedObjects.centroidMovements.find(observeredObjects[i]) != trackedObjects.centroidMovements.end())
        {
            meanDistance += trackedObjects.centroidMovements[observeredObjects[i]];
            counts++;
        }
    }
    mutexCentroidMovementsThread.unlock();

    if (meanDistance / (double) counts > thd)
    {
        std::cout << "checkPerspectiveChange....TRUE!!!!!!!!!!!!!!!" << meanDistance / (double) counts << "<" << thd << std::endl;
        return true;
    }
    else
    {
        std::cout << "checkPerspectiveChange....FALSE!!!!!!!!!!!!!!!" << meanDistance / (double) counts << "<" << thd << std::endl;
        return false;
    }

}

std::pair<int, double> queryTracked(int id)
{
    std::pair<int, double> result;
    std::map<int, int> resultCounts;
    std::map<int, std::vector<double> > resultConfidence;

    std::map<int, int>::iterator iterResultCounts;
    std::map<int, std::vector<double> >::iterator iterResultConfidence;

    result.first = 0;
    result.second = 1;

    if (trackedObjects.classifications.find(id) == trackedObjects.classifications.end())
    {
        return result;
    }

    mutexConcatResultCategorizationThread.lock();

    std::cout << "Final decision Making of Object " << id << ":";
    for (unsigned int i = 0; i < trackedObjects.classifications[id].size(); ++i)
    {
        resultCounts[trackedObjects.classifications[id][i].first]++;
        resultConfidence[trackedObjects.classifications[id][i].first].push_back(trackedObjects.classifications[id][i].second);
        std::cout << " -> " << trackedObjects.classifications[id][i].first;
    }
    std::cout << std::endl;
    mutexConcatResultCategorizationThread.unlock();

    int maxCount = 0;
    int maxLabel = 0;
    for (iterResultCounts = resultCounts.begin(); iterResultCounts != resultCounts.end(); ++iterResultCounts)
    {
        if (iterResultCounts->second > maxCount)
        {
            maxCount = iterResultCounts->second;
            maxLabel = iterResultCounts->first;
        }
    }
    result.first = maxLabel;

    double avgConfidence = 0.0;
    std::vector<double> maxLabelConfidences = resultConfidence[result.first];

    for (unsigned int i = 0; i < maxLabelConfidences.size(); ++i)
    {
        avgConfidence += maxLabelConfidences[i];
    }

    //= avg of individual confidence od classifications * the ratio of the most classified label compare to the total count of classfication for the analyzied object.
    result.second = ((avgConfidence / (double) maxLabelConfidences.size()) * (resultCounts[result.first] / (double) trackedObjects.classifications[id].size()));

    std::cout << "Final decision Making of Object " << id << ": " << result.first << " " << result.second << std::endl;
    return result;
}
bool concatClassificationResult(int id, std::pair<int, double> result)
{
    if (trackedObjects.classifications.find(id) == trackedObjects.classifications.end())
    {
        return false;
    }

    mutexConcatResultCategorizationThread.lock();
    trackedObjects.classifications[id].push_back(result);

    if (trackedObjects.classifications[id].size() > queueSize)
    {
        trackedObjects.classifications[id].pop_front();
    }

    std::cout << "Running Classifications of object " << id << "are :";
    for (unsigned int i = 0; i < trackedObjects.classifications[id].size(); ++i)
    {
        std::cout << " -> " << trackedObjects.classifications[id][i].first;
    }
    std::cout << std::endl;
    mutexConcatResultCategorizationThread.unlock();

    return true;
}

void setStampTrackedObjects(unsigned int stamp, std::vector<int> &observedObjectID)
{
    mutexStampUpdateTrackedObjects.lock();
    trackedObjects.stamp = stamp;
    trackedObjects.observeredObjects = observedObjectID;
    mutexStampUpdateTrackedObjects.unlock();
}

int getStampTrackedObjects()
{
    unsigned int stamp;
    mutexStampUpdateTrackedObjects.lock();
    stamp = trackedObjects.stamp;
    mutexStampUpdateTrackedObjects.unlock();
    return stamp;
}

bool getObserveredObjects(int stamp, std::map<int, pcl::PointCloud<pcl::PointXYZ> > &objects, std::vector<int> &observeredObjects)
{
    if (stamp <= trackedObjects.stamp)
    {
        mutexStampUpdateTrackedObjects.lock();
        objects = trackedObjects.objects;
        observeredObjects = trackedObjects.observeredObjects;
        mutexStampUpdateTrackedObjects.unlock();
        return true;
    }
    else
    {
        return false;
    }
}

void decayTrackedObjects()
{
    std::map<int, int>::iterator iterConfidence;
    int step = 5;
    //now decrease confidence for all;
    std::vector<int> toRemoveID;
    for (iterConfidence = trackedObjects.confidence.begin(); iterConfidence != trackedObjects.confidence.end(); ++iterConfidence)
    {
        trackedObjects.confidence[iterConfidence->first] = trackedObjects.confidence[iterConfidence->first] - (step);
        std::cout << "condfidece of ob" << iterConfidence->first << "= >" << trackedObjects.confidence[iterConfidence->first] << std::endl;
        if (trackedObjects.confidence[iterConfidence->first] <= 0)
        {
            toRemoveID.push_back(iterConfidence->first);
        }
    }

    //now remove bad confidence
    std::cout << "remove object total id" << trackedObjects.centroids.size() << std::endl;
    for (unsigned int i = 0; i < toRemoveID.size(); ++i)
    {
        trackedObjects.centroids.erase(trackedObjects.centroids.find(toRemoveID[i]));
        trackedObjects.objects.erase(trackedObjects.objects.find(toRemoveID[i]));
        trackedObjects.confidence.erase(trackedObjects.confidence.find(toRemoveID[i]));
        trackedObjects.centroidMovements.erase(trackedObjects.centroidMovements.find(toRemoveID[i]));
        trackedObjects.classifications.erase(trackedObjects.classifications.find(toRemoveID[i]));
        std::cout << "remove object with id" << toRemoveID[i] << std::endl;
    }
    std::cout << "remove object total id" << trackedObjects.centroids.size() << std::endl;

}

int trackObjects(pcl::PointCloud<pcl::PointXYZ> query)
{
    double maxDist = 0.1;
    int step = 5;
    int max = 50;
    int init = max;
    int queryID = -1;
    int stepIncreaseIfFound = step;  //0
    double initCentroidMovementValue = 1.0;  // it is 1.0 so it can be recognitzed as movement in the  checkPerspectiveChange();

    std::map<int, pcl::PointXYZRGB>::iterator iterCentroid;
    std::map<int, int>::iterator iterConfidence;
    pcl::PointXYZRGB centroid = toolBox.pointCloudCentroid2(query);

    //init tracking with first found object
    if (trackedObjects.objects.size() == 0)
    {
        unsigned int r = rand() % 255;
        unsigned int g = rand() % 255;
        unsigned int b = rand() % 255;

        uint32_t urgb = r << 16 | g << 8 | b;
        float color = *(reinterpret_cast<float *>(&urgb));

        trackedObjects.objects[runningCounter] = query;
        trackedObjects.centroids[runningCounter] = centroid;
        trackedObjects.centroids[runningCounter].rgb = color;
        trackedObjects.confidence[runningCounter] = init;
        trackedObjects.classifications[runningCounter].clear();
        trackedObjects.centroidMovements[runningCounter] = initCentroidMovementValue;
        runningCounter++;
        std::cout << "new object with id" << runningCounter << std::endl;
        queryID = runningCounter;
    }
    else
    {
        //nearest object to current entering object
        double minDistance = 999.0;
        int minDistanceCentroidId = -1;
        double currentDist;

        for (iterCentroid = trackedObjects.centroids.begin(); iterCentroid != trackedObjects.centroids.end(); ++iterCentroid)
        {
            currentDist = toolBox.euclDistanceBtwPoints(centroid, iterCentroid->second);
            if (currentDist <= minDistance)
            {
                minDistance = currentDist;
                minDistanceCentroidId = iterCentroid->first;
            }
        }

        std::cout << "Min Distance of current object is " << minDistance << std::endl;
        //found an object
        if (minDistance < maxDist)
        {
            std::cout << "found object with id" << minDistanceCentroidId << " PointCloud Distance to nearest One: " << minDistance << std::endl;
            //////////////
            //////////////
            trackedObjects.centroids[minDistanceCentroidId].x = ((trackedObjects.centroids[minDistanceCentroidId].x + centroid.x) / 2.0);
            trackedObjects.centroids[minDistanceCentroidId].y = ((trackedObjects.centroids[minDistanceCentroidId].y + centroid.y) / 2.0);
            trackedObjects.centroids[minDistanceCentroidId].z = ((trackedObjects.centroids[minDistanceCentroidId].z + centroid.z) / 2.0);
            trackedObjects.objects[minDistanceCentroidId] = query;
            trackedObjects.centroidMovements[minDistanceCentroidId] = minDistance;
            if (trackedObjects.confidence[minDistanceCentroidId] < max)
            {
                trackedObjects.confidence[minDistanceCentroidId] = trackedObjects.confidence[minDistanceCentroidId] + (step + stepIncreaseIfFound);
            }
            queryID = minDistanceCentroidId;
        }
        else  //didnt find maybe an new object so add
        {
            int key = 0;
            bool found = false;
            do
            {
                if (trackedObjects.centroids.find(key) == trackedObjects.centroids.end())
                {
                    found = true;
                    unsigned int r = rand() % 255;
                    unsigned int g = rand() % 255;
                    unsigned int b = rand() % 255;

                    uint32_t urgb = r << 16 | g << 8 | b;
                    float color = *(reinterpret_cast<float *>(&urgb));

                    trackedObjects.objects[key] = query;
                    trackedObjects.centroids[key] = centroid;
                    trackedObjects.centroids[key].rgb = color;
                    trackedObjects.confidence[key] = init;
                    trackedObjects.classifications[key].clear();
                    trackedObjects.centroidMovements[key] = initCentroidMovementValue;
                    queryID = key;

                    std::cout << "didnt find so add object with id" << key << std::endl;
                }
                key++;

            }
            while (!found);
        }
    }

    trackedObjects.centroidsPCD.header = query.header;
    trackedObjects.centroidsPCD.height = 1;
    trackedObjects.centroidsPCD.width = 0;
    trackedObjects.centroidsPCD.points.clear();

    for (iterCentroid = trackedObjects.centroids.begin(); iterCentroid != trackedObjects.centroids.end(); ++iterCentroid)
    {
        trackedObjects.centroidsPCD.points.push_back(trackedObjects.centroids[iterCentroid->first]);
        trackedObjects.centroidsPCD.width++;
    }

    return queryID;
}

void objectCategorizer(std::map<int, pcl::PointCloud<pcl::PointXYZ> > pointClouds, std::vector<int> observedObjectID)
{
    //clock_t begin_time = clock();

    if (mutexCategorizationThread.try_lock())
    {
        //std::cout << "objectCategorizer()..................THREAD STARTED for objectCategorizer()...Num PointClouds  " << pointClouds.size() << " to categorize " << observedObjectID.size() << std::endl;

        if (observedObjectID.size() > 0)
        {
            visualization_msgs::MarkerArray pointArray;
            pcl::PointCloud < pcl::PointXYZRGB > totalColoredPointCloud;
            //sensor_msgs::PointCloud2 pointCloudMsg;

            for (unsigned int i = 0; i < observedObjectID.size(); i++)
            {
                int id = observedObjectID[i];
                pcl::PointCloud < pcl::PointXYZ > pointCloud;
                pointCloud = (pointClouds[id]);

                //checkPoinCloud Difference ... this is an indication that the object perspective might have changed if object pcd is different.
                ROS_DEBUG("pointCloudVectorCallback...OBJECT NUMBER  %d/%d", (int) i, (int) observedObjectID.size());

                //ROS_INFO("objectCategorizer...CALL >>>>QUERY2<<<<!!!!")
                std::pair<int, double> res = objectCategorizationGeometric->query(pointCloud);
                concatClassificationResult(id, res);
                res = queryTracked(id);

                //Now create label markers.
                visualization_msgs::Marker points;
                points.ns = "object_category_lables";
                points.header.frame_id = BASE_LINK_FRAME;

                geometry_msgs::Point p;

                ////
                //In order to update display position of the object since in the meanwhile of categorization the position might have been changed!
                if (trackedObjects.objects.find(id) != trackedObjects.objects.end())
                {
                    pointCloud = trackedObjects.objects[id];
                }
                ////
                pcl::PointXYZ centroid = toolBox.pointCloudCentroid(pointCloud);
                points.id = i;
                points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                points.action = visualization_msgs::Marker::ADD;
                points.scale.x = 0.07;
                points.scale.y = 0.07;
                points.scale.z = 0.07;
                points.color.g = 1.0f;
                points.color.r = 1.0f;
                points.color.b = 1.0f;

                points.color.a = 1.0;
                points.pose.orientation.w = 1.0;

                //std::cout << "Centroid : " << centroid << "\n";
                points.pose.position.x = centroid.x;
                points.pose.position.y = centroid.y;  //+0.05;
                points.pose.position.z = centroid.z + 0.09;
                points.text = std::string(
                                  CFileSettings::labels[res.first] + "\n" + boost::lexical_cast < std::string > (static_cast<int>((int)(res.second * 100.0f))) + "%")
                              .c_str();
                points.lifetime = ros::Duration(3);

                points.header.stamp = ros::Time::now();
                pointArray.markers.push_back(points);

                markerObjectCategorizationGeometricPublisher.publish(pointArray);

            }
        }
        else
        {
            ROS_INFO("no object found");
        }

        mutexCategorizationThread.unlock();
        //std::cout << "objectCategorizer()..................THREAD DONE for objectCategorizer()...tool "<<float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl << std::endl;
    }
}

void pointCloudVectorTrackingCallback(const mcr_perception_msgs::ObjectListPtr& pointCloudVectorMsg)
{
    pcl::PointCloud < pcl::PointXYZ > pointCloud;
    ROS_DEBUG("pointCloudVectorTrackingCallback.........................");
    clock_t begin_time = clock();
    static int currentTimeStamp;

    if (pointCloudVectorMsg->objects.size() > 0)
    {
        visualization_msgs::MarkerArray pointArray;
        pcl::PointCloud < pcl::PointXYZRGB > totalColoredPointCloud;
        sensor_msgs::PointCloud2 pointCloudMsg;

        std::vector<int> observedObjectID;
        for (unsigned int i = 0; i < pointCloudVectorMsg->objects.size(); i++)
        {
            pcl::fromROSMsg(pointCloudVectorMsg->objects[i].pointcloud, pointCloud);

            ROS_DEBUG("pointCloudVectorCallback...OBJECT NUMBER  %d/%d", (int) i, (int) pointCloudVectorMsg->objects.size());

            observedObjectID.push_back(trackObjects(pointCloud));

            pcl::toROSMsg(trackedObjects.centroidsPCD, pointCloudMsg);  //this is currently the surface
            pcl_conversions::fromPCL(pointCloud.header, pointCloudMsg.header);
            pointCloudMsg.header.frame_id = "/base_link";  //totalColoredPointCloud.header.frame_id;
            pointCloudMsg.header.stamp = ros::Time::now();
            testPublisher.publish(pointCloudMsg);
        }
        decayTrackedObjects();
        currentTimeStamp = ros::Time::now().sec;
        std::cout << "pointCloudVectorTrackingCallback....stamped at " << currentTimeStamp << std::endl;
        setStampTrackedObjects(currentTimeStamp, observedObjectID);

        /*start thread objectCategorizer */
        std::vector<int> observeredObjects;
        std::map<int, pcl::PointCloud<pcl::PointXYZ> > objects;
        getObserveredObjects(currentTimeStamp, objects, observeredObjects);

        if (checkPerspectiveChange(observeredObjects, 0.005) || !DO_CHECKPERSPECTIVE_CHANGE)  //0.015 //this checks if the objects have moved, respectively the camera position as moved.
            boost::thread(objectCategorizer, objects, observeredObjects);
        /**********************************/
        ROS_DEBUG("pointCloudVectorTrackingCallback...........................................................tracked and started ... took %lf",
                  float(clock() - begin_time) / CLOCKS_PER_SEC);
    }
}

void pointCloudVectorGasCallback(const mcr_perception_msgs::ObjectListPtr& pointCloudVectorMsg)
{
    pcl::PointCloud < pcl::PointXYZ > pointCloud;
    clock_t begin_time = clock();
    ROS_DEBUG("pointCloudVectorCallback...");

    if (pointCloudVectorMsg->objects.size() > 0)
    {
        visualization_msgs::MarkerArray pointArray;
        pcl::PointCloud < pcl::PointXYZRGB > totalColoredPointCloud;
        sensor_msgs::PointCloud2 pointCloudMsg;

        object_list.objects.clear();

        for (unsigned int i = 0; i < pointCloudVectorMsg->objects.size(); i++)
        {
            pcl::fromROSMsg(pointCloudVectorMsg->objects[i].pointcloud, pointCloud);

            ROS_DEBUG("pointCloudVectorCallback...OBJECT NUMBER  %d/%d with %d points", (int) i, (int) pointCloudVectorMsg->objects.size(),
                      (int) pointCloud.points.size());

            std::pair<int, double> res = objectCategorizationGeometric->query(pointCloud);

            //Now create label markers.
            visualization_msgs::Marker points;
            points.ns = "object_category_lables";
            points.header.frame_id = BASE_LINK_FRAME;

            geometry_msgs::Point p;
            pcl::PointXYZ centroid = toolBox.pointCloudCentroid(pointCloud);
            points.id = i;
            points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            points.action = visualization_msgs::Marker::ADD;
            points.scale.x = 0.07;
            points.scale.y = 0.07;
            points.scale.z = 0.07;
            points.color.g = 1.0f;
            points.color.r = 1.0f;
            points.color.b = 1.0f;

            points.color.a = 1.0;
            points.pose.orientation.w = 1.0;

            //std::cout << "Centroid : " << centroid << "\n";
            points.pose.position.x = centroid.x;
            points.pose.position.y = centroid.y;  //+0.05;
            points.pose.position.z = centroid.z + 0.09;
            points.text =
                std::string(
                    CFileSettings::labels[res.first] + "\n" + boost::lexical_cast < std::string > (static_cast<int>((int)(res.second * 100.0f))) + "%")
                .c_str();
            points.lifetime = ros::Duration(3);

            points.header.stamp = ros::Time::now();
            pointArray.markers.push_back(points);

            // create msg
            if (std::string(CFileSettings::labels[res.first]) != "query")
            {
                mcr_perception_msgs::Object object;
                object.name = std::string(CFileSettings::labels[res.first]);
                object.probability = res.second;
                object.pose.header.frame_id = BASE_LINK_FRAME;
                object.pose.header.stamp = ros::Time::now();
                object.pose.pose.position.x = centroid.x;
                object.pose.pose.position.y = centroid.y;
                object.pose.pose.position.z = centroid.z;

                object_list.objects.push_back(object);

                ROS_INFO("found object: %s with %f", std::string(CFileSettings::labels[res.first]).c_str(), res.second);
            }

            //Now color point cloud according category
            /*
             pcl::PointCloud<pcl::PointXYZRGB> coloredPointCloud;
             copyPointCloud(pointCloud, coloredPointCloud);

             for (unsigned int iterPoint = 0; iterPoint < coloredPointCloud.points.size(); ++iterPoint)
             {
             coloredPointCloud.points[iterPoint].rgb = color;
             }

             totalColoredPointCloud.header = coloredPointCloud.header;
             totalColoredPointCloud += coloredPointCloud;
             //totalColoredPointCloud.header.frame_id = coloredPointCloud.header.frame_id;
             // pcl::toROSMsg(coloredPointCloud, pointCloudMsg); //this is currently the surface
             // pointCloudMsg.header.frame_id = coloredPointCloud.header.frame_id;
             // pointCloudMsg.header.stamp = ros::Time::now();

             ///pointCloudCategorizationGeometricPublisher.publish(pointCloudMsg);


             pcl::toROSMsg(totalColoredPointCloud, pointCloudMsg); //this is currently the surface
             pointCloudMsg.header.frame_id = BASE_LINK_FRAME; //totalColoredPointCloud.header.frame_id;
             pointCloudMsg.header.stamp = ros::Time::now();

             pointCloudCategorizationGeometricPublisher.publish(pointCloudMsg);
             */
            markerObjectCategorizationGeometricPublisher.publish(pointArray);
        }
        pub_object_list.publish(object_list);
    }
    else
    {
        ROS_INFO("NO objects found!");
    }
    ROS_DEBUG("[pointCloudVectorCallback...total DONE in %lf", float(clock() - begin_time) / CLOCKS_PER_SEC);
}
void trainAdditionalObjectCatergorizationGeometric(std::string databaseTraining)
{
    ;
}
void saveObjectCategorizationGeometricModel(std::string filename)
{
    try
    {
        printf("saveObjectCategorizationGeometricModel...%s\n", filename.c_str());
        std::ofstream ofs(filename.c_str());
        const CObjectCategorizationGeometric t = *objectCategorizationGeometric;

        boost::archive::text_oarchive oa(ofs);
        //boost::archive::binary_oarchive oa(ofs);
        // write class instance to archive
        oa << t;

    }
    catch (boost::archive::archive_exception ae)
    {
        printf("saveObjectCategorizationGeometricModel...%s", ae.what());
    }
}

void loadObjectCategorizationGeometricModel(std::string filename)
{
    try
    {
        printf("loadObjectCategorizationGeometricModel...%s\n", filename.c_str());
        std::ifstream ifs(filename.c_str());  //("filename");

        CObjectCategorizationGeometric t;
        boost::archive::text_iarchive ia(ifs);

        // write class instance to archive
        ia >> t;

        std::cout << "t : " << t.getTrainingData().extractedFeatureVectors.size() << "\n";
        *objectCategorizationGeometric = t;
    }
    catch (boost::archive::archive_exception ae)
    {
        printf("loadObjectCategorizationGeometricModeln...%s", ae.what());
    }

}

void trainObjectCategorizationGeometric(std::string databaseTraining, std::string databaseTesting, bool trainPseudoCatgory)
{
    ROS_INFO("trainObjectCategorizationGeometric...");

    /*objectCategorizationGeometric->inputObjectPointClouds("data7.3_std/", 1, 1, true);
     std::vector< pcl::PointCloud<pcl::PointXYZ> > clouds =  objectCategorizationGeometric->getTrainingData().objectPointClouds;

     std::cout<<"SIZEEE"<< clouds.size();
     sensor_msgs::PointCloud2 pointCloudMsg;
     //unsigned int i=0;
     for(unsigned int i = 0; i < clouds.size(); ++i )
     {
     std::cout<<"could " << clouds[i].points.size()<<std::endl;
     pcl::toROSMsg(clouds[i], pointCloudMsg); //this is currently the surface
     //pointCloudMsg.header = clouds[i].header;
     pointCloudMsg.header.frame_id = "/base_link"; //totalColoredPointCloud.header.frame_id;
     pointCloudMsg.header.stamp = ros::Time::now();
     testPublisher.publish(pointCloudMsg);
     std::cout<<" "<<i<<std::endl;
     sleep(1);
     }
     return;*/

    if (trainPseudoCatgory)
    {
        ROS_INFO("trainObjectCategorizationGeometric...PseudoCatgories train...");

        objectCategorizationGeometric->inputObjectPointCloudsPseudoCategories(databaseTraining, 45, true);  //45
        //objectCategorizationGeometric->inputObjectPointCloudsPseudoCategories(databaseTesting, 2,true);
        objectCategorizationGeometric->trainPseudoCategories();  //50

        ROS_INFO("trainObjectCategorizationGeometric...PseudoCatgories trained");
        return;
    }
    else
    {
        objectCategorizationGeometric->loadPseudoCategories();
        ROS_INFO("trainCat...pseudoCategories loaded");
    }

    objectCategorizationGeometric->setDoTrainSvm(true);  //t
    objectCategorizationGeometric->setDoTrainAutoEncoder(false);  //f
    objectCategorizationGeometric->setDoTrainSvmLinear(false);  //f

    objectCategorizationGeometric->setDoQuerySvm(true);  //t
    objectCategorizationGeometric->setDoQueryAutoEncoder(false);  //f
    objectCategorizationGeometric->setDoQuerySvmLinear(false);  //f

    //Test set
    objectCategorizationGeometric->inputObjectPointClouds(homePath + std::string("data7.3_std/"), 15, 3, true);  //15 ("data7.3_std/", 15, 2,false);//20
    objectCategorizationGeometric->inputObjectPointClouds(homePath + std::string("data7.3_less/"), 15, 3, true);  //15 test set
    objectCategorizationGeometric->inputObjectPointClouds("data8_1_sub/", 90, 2, true);  //90 validation set
    //Test stapler(8)
    //objectCategorizationGeometric.inputObjectPointClouds("data1.0_addExternalTest/", 1, 2, true); //15

    //Train sets
    std::cout << " tag/2 " << tag / 2 << std::endl;

    objectCategorizationGeometric->inputObjectPointClouds(databaseTraining, tag / 2, 1, true);  //50 (databaseTraining, 50, 1,true);//50 //25 for AE
    objectCategorizationGeometric->inputObjectPointClouds(databaseTesting, tag / 2, 1, true);  //50(databaseTesting, 50, 1,true);//20   //25 for AE

    //Train stapler(8)
    //objectCategorizationGeometric.inputObjectPointClouds("data1.0_addExternalTest/", 1, 1,true); //50(databaseTesting, 50, 1,true);//20
    ROS_INFO("trainCat...categories loaded");

    objectCategorizationGeometric->setTag(boost::lexical_cast < std::string > (tag));
    objectCategorizationGeometric->train();

    //Not necessary
    //saveObjectCategorizationGeometricModel();
    saveObjectCategorizationGeometricModel("plRF.ot");
}

void loadObjectCategorizationGeometric()
{
    ROS_INFO("loadObjectCategorizationGeometric...loadModel");
    objectCategorizationGeometric->setDoQuerySvm(true);
    objectCategorizationGeometric->setDoQuerySvmLinear(false);
    objectCategorizationGeometric->setDoQueryAutoEncoder(false);
    objectCategorizationGeometric->loadModel();
}

void loadAndTrainObjectCategorizationGeometric()
{
    loadObjectCategorizationGeometricModel("/home/mca/Development/workspace/object_categorization_node/plRF.ot");
    objectCategorizationGeometric->loadTraintrain();

    //loadObjectCategorizationGeometric();
    //objectCategorizationGeometric->evaluateSvm2();
    //objectCategorizationGeometric->evaluate();
    //objectCategorizationGeometric->evaluateQuery2();
    //objectCategorizationGeometric->evaluateSvm3();
}

void evaluateObjectCategorizationGeometric()
{
    std::string databaseTr = homePath + std::string("data7.6_Pan1_shuffled/");
    std::string databaseTe = homePath + std::string("data7.5_Pan1_shuffled/");
    //objectCategorizationGeometric->inputObjectPointClouds(homePath + std::string("data7.3_std/"), 15, 3, true);//15 ("data7.3_std/", 15, 2,false);//20
    //objectCategorizationGeometric->inputObjectPointClouds(homePath + std::string("data7.3_less/"), 15, 3, true);//15

    objectCategorizationGeometric->inputObjectPointClouds("data8_1_sub/", 90, 2, true);
    //objectCategorizationGeometric->inputObjectPointClouds(databaseTr, 50, 1, true);//50 (databaseTraining, 50, 1,true);//50 //25 for AE
    //objectCategorizationGeometric->inputObjectPointClouds(databaseTe, 50, 1, true); //50(databaseTesting, 50, 1,true);//20   //25 for AE
    //loadObjectCategorizationGeometricModel("/home/mca/Development/workspace/object_categorization_node/plRF.ot");
    loadObjectCategorizationGeometric();
    //objectCategorizationGeometric->evaluate();
    //objectCategorizationGeometric->evaluate();
    //objectCategorizationGeometric->evaluateSvm();

    //loadObjectCategorizationGeometricModel("/home/mca/Development/workspace/object_categorization_node/plRF.ot");
    //loadObjectCategorizationGeometric();
    objectCategorizationGeometric->evaluateQuery1();
    //objectCategorizationGeometric->evaluateSvm3();
}

void newCategoryLearner(const mcr_perception_msgs::ObjectListPtr& pointCloudVectorMsg)
{
    pcl::PointCloud < pcl::PointXYZ > pointCloud;
    int label = newClassLabel;
    ROS_INFO("pointCloudVectorCallback...");
    static std::vector<pcl::PointCloud<pcl::PointXYZ> > tolearn;

    if (pointCloudVectorMsg->objects.size() > 0)
    {
        visualization_msgs::MarkerArray pointArray;
        pcl::PointCloud < pcl::PointXYZRGB > totalColoredPointCloud;
        sensor_msgs::PointCloud2 pointCloudMsg;

        int choice = 1;
        for (unsigned int i = 0; i < pointCloudVectorMsg->objects.size(); i++)
        {
            testPublisher.publish(pointCloudVectorMsg->objects[i].pointcloud);

            //  std::cout << "Take the Example or reject ? 0 1" << std::endl;
            //std::cin >> choice;
            if (choice)
            {
                pcl::fromROSMsg(pointCloudVectorMsg->objects[i].pointcloud, pointCloud);
                tolearn.push_back(pointCloud);
            }

            //std::cout << "Finish? 0 1" << std::endl;
            //std::cin >> choice;
            std::cout << "Number of examples " << tolearn.size() << std::endl;
            if (tolearn.size() > 100)           //50 if (choice)
            {
                std::vector < pcl::PointCloud<pcl::PointXYZ> > tolearnSub;
                objectCategorizationGeometric->setDoQuerySvm(true);
                objectCategorizationGeometric->setDoQuerySvmLinear(false);
                objectCategorizationGeometric->setDoQueryAutoEncoder(false);
                objectCategorizationGeometric->loadModel();

                std::map<int, std::vector<std::vector<double> > > extractedTrainingFeatureVectors;
                for (unsigned int i = 0; i < tolearn.size(); ++i)
                {
                    extractedTrainingFeatureVectors[label].push_back(objectCategorizationGeometric->queryPseudo2(tolearn[i]).second);

                    pcl::PointCloud < pcl::PointXYZ > newPointCloud = tolearn[i];

                    {
                        float minLeafSize = 0.01;
                        float maxLeafSize = 0.25;  //0.5;
                        int numStep = 2;  // number of points clouds to add == number of subsampling steps;

                        float step = ((maxLeafSize - minLeafSize) / double(numStep));

                        for (float curLeafSize = minLeafSize; curLeafSize < maxLeafSize; curLeafSize = curLeafSize + step)
                        {
                            pcl::PointCloud < pcl::PointXYZ > newPointCloud = tolearn[i];
                            std::pair<std::string, pcl::PointCloud<pcl::PointXYZ> > sample;

                            //Do subsampling....
                            toolBox.subsampling(newPointCloud, curLeafSize);

                            if (newPointCloud.points.size() > 80)
                            {
                                std::cout << "CV : augmenpub_object_listtSetWithScalePyramid.....add " << curLeafSize << " " << maxLeafSize << std::endl;
                                extractedTrainingFeatureVectors[label].push_back(objectCategorizationGeometric->queryPseudo2(newPointCloud).second);

                            }
                            else
                            {
                                std::cout << "CV::augmentSetWithScalePyramid...kick out\n";
                            }
                        }

                    }
                }

                //for (unsigned int j = 0; j < 30; ++j)
                //extractedTrainingFeatureVectors[label].push_back(extractedTrainingFeatureVectors[label][0]);

                objectCategorizationGeometric->updateModelSvmLinear(extractedTrainingFeatureVectors);
                std::cout << "DONE!!!" << std::endl;
                exit(1);
            }
        }

    }
}

bool start(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    sub_pointcloud = g_nh_ptr->subscribe("object_candidates", 1, pointCloudVectorGasCallback);
    ;

    ROS_INFO("3D object categorization ENABLED");
    return true;
}

bool stop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    sub_pointcloud.shutdown();
    ROS_INFO("3D object categorization DISABLED");
    return true;
}

bool get_categorized_objects(mcr_perception_msgs::GetObjectList::Request &req, mcr_perception_msgs::GetObjectList::Response &res)
{
    res.objects = object_list.objects;

    object_list.objects.clear();

    return true;
}

int main(int argc, char **argv)
{
    tag = 100;              // boost::lexical_cast<unsigned int>(argv[1]);

    ros::init(argc, argv, "object_categorization");
    ROS_INFO("... started");

    ros::NodeHandle nodeHandle("~");
    g_nh_ptr = &nodeHandle;

    ros::ServiceServer srv_start = nodeHandle.advertiseService("start", start);
    ros::ServiceServer srv_stop = nodeHandle.advertiseService("stop", stop);
    ros::ServiceServer srv_get_objects = nodeHandle.advertiseService("get_categorized_objects", get_categorized_objects);

    pub_object_list = nodeHandle.advertise < mcr_perception_msgs::ObjectList > ("categorized_objects", 10);
    markerObjectCategorizationGeometricPublisher = nodeHandle.advertise < visualization_msgs::MarkerArray > ("/visualization_marker_array", 10);

    //load launch file parameter
    if (nodeHandle.getParam("home_path", homePath) == false) ROS_INFO(" ... could not read parameter /homePath from launch file");
    else ROS_INFO(" ... home path set to %s ", homePath.c_str());

    //init logger and categorizer with home path
    CLogger *logger = &CLogger::getInstance(homePath + std::string("/"));

    objectCategorizationGeometric = new CObjectCategorizationGeometric();

    //set new home folder which is from previous loaded launch file
    objectCategorizationGeometric->setHomePath(homePath + std::string("/ros/config/"));

    loadObjectCategorizationGeometric();

    ROS_INFO("initialized successfull ...");

    ros::spin();

    return 0;
}
