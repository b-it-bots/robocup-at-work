/*
 * Created on: Apr 21, 2011
 * Author: Christian Mueller
 */

#ifndef CObjectPoseNormalization_H
#define CObjectPoseNormalization_H

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <time.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <climits>
#include <malloc.h>

#include <opencv/cv.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include "toolbox_ros.h"
#include "pca.h"

#define DEG_TO_RAD(val) (val * (M_PI/180))
#define RAD_TO_DEG(val) (val * (180/M_PI))

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#define YZ_PLANE 0
#define XY_PLANE 1
#define XZ_PLANE 2

#define STEP_ANGLE_ALIGNEMENT 5.0
//10.0 high
//2.0 STD
#define POSE_NORM_SUBSAMPLING 0.1
//0.1
//0.05
//0.1std

template<typename PointT>
class CObjectPoseNormalization
{
protected:
    CToolBoxROS toolBox;
    ros::Publisher pub;
    ros::Publisher principleAxis_pub;
    bool doPublish;
public:
    void setNodeHandle(ros::NodeHandle *node)
    {
        doPublish = true;
        pub = node->advertise<sensor_msgs::PointCloud2>("object_perception/poseNormalization", 1);

        principleAxis_pub = node->advertise<visualization_msgs::Marker>("object_perception/poseNormalization/principleAxis", 10);
        std::cout << "SETNODE____2\n";
    }

    pcl::PointCloud<PointT> pointCloudModel_0; // model, input point cloud
    std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> > > pointCloudModel_1; // reflective symmetric counterpart of point cloud (X-Axis=0 Y-Axis=1 Z-Axis=2)

    //template<typename PointT>
    void orbitPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, const Eigen::Vector3f &offset, const Eigen::Quaternionf &rotation)
    {
        Eigen::Translation3f translation(offset);
        Eigen::Affine3f t;
        t = rotation * translation;
        pcl::transformPointCloud(cloud_in, cloud_out, t);
    }

    //template<typename PointT>
    void rotate(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, double degree, int axis)
    {
        Eigen::Matrix3f m;

        switch (axis)
        {
        case X_AXIS: // X axis rotate
            m = Eigen::AngleAxisf(DEG_TO_RAD(degree), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
            break;
        case Y_AXIS: //Y axis rotate
            m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(DEG_TO_RAD(degree), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
            break;
        case Z_AXIS: //Z axis rotate
            m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(DEG_TO_RAD(degree), Eigen::Vector3f::UnitZ());
            break;
        default:
            assert(axis == X_AXIS || axis == Y_AXIS || axis == Z_AXIS);
            break;
        }
        Eigen::Quaternionf rotation(m);

        Eigen::Vector3f offset(0.0, 0.0, 0.0);

        orbitPointCloud(cloud_in, cloud_out, offset, rotation);
    }

    void translate(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, double translate, int axis)
    {
        Eigen::Matrix3f m;

        m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());

        Eigen::Vector3f offset;
        switch (axis)
        {
        case X_AXIS: // X axis rotate
            offset = Eigen::Vector3f(translate, 0.0, 0.0);
            break;
        case Y_AXIS: //Y axis rotate
            offset = Eigen::Vector3f(0.0, translate, 0.0);
            break;
        case Z_AXIS: //Z axis rotate
            offset = Eigen::Vector3f(0.0, 0.0, translate);
            break;
        default:
            assert(axis == X_AXIS || axis == Y_AXIS || axis == Z_AXIS);
            break;
        }
        Eigen::Quaternionf rotation(m);

        orbitPointCloud(cloud_in, cloud_out, offset, rotation);
    }

    //template<typename PointT>
    pcl::PointCloud<PointT> transformReflection(int axis)
    {
        assert(this->pointCloudModel_0.points.size() > 0);

        pcl::PointCloud<PointT> reflection = this->pointCloudModel_0;
        switch (axis)
        {
        case X_AXIS:
            this->pointCloudModel_1[X_AXIS] = this->pointCloudModel_0;
            for (unsigned iterPoint = 0; iterPoint < this->pointCloudModel_1[X_AXIS].points.size(); ++iterPoint)
            {
                this->pointCloudModel_1[X_AXIS].points[iterPoint].y *= (-1.0);
            }
            reflection = this->pointCloudModel_1[X_AXIS];
            break;
        case Y_AXIS:
            this->pointCloudModel_1[Y_AXIS] = this->pointCloudModel_0;
            for (unsigned iterPoint = 0; iterPoint < this->pointCloudModel_1[Y_AXIS].points.size(); ++iterPoint)
            {
                this->pointCloudModel_1[Y_AXIS].points[iterPoint].x *= (-1.0);
            }
            reflection = this->pointCloudModel_1[Y_AXIS];
            break;

        case Z_AXIS:
            this->pointCloudModel_1[Z_AXIS] = this->pointCloudModel_0;
            for (unsigned iterPoint = 0; iterPoint < this->pointCloudModel_1[Z_AXIS].points.size(); ++iterPoint)
            {
                this->pointCloudModel_1[Z_AXIS].points[iterPoint].z *= (-1.0);
            }
            reflection = this->pointCloudModel_1[Z_AXIS];
            break;
        }
        return reflection;
    }

    void transformReflection(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, int axis)
    {
        assert(cloud_in.points.size() > 0);

        cloud_out = cloud_in;

        switch (axis)
        {
        case XZ_PLANE:
//#pragma omp parallel for shared(cloud_out)
            for (unsigned iterPoint = 0; iterPoint < cloud_out.points.size(); ++iterPoint)
            {
                cloud_out.points[iterPoint].y *= (-1.0);
            }
            break;
        case YZ_PLANE:
//#pragma omp parallel for shared(cloud_out)
            for (unsigned iterPoint = 0; iterPoint < cloud_out.points.size(); ++iterPoint)
            {
                cloud_out.points[iterPoint].x *= (-1.0);
            }
            break;

        case XY_PLANE:
//#pragma omp parallel for shared(cloud_out)
            for (unsigned iterPoint = 0; iterPoint < cloud_out.points.size(); ++iterPoint)
            {
                cloud_out.points[iterPoint].z *= (-1.0);
            }
            break;
        default:
            assert(axis == X_AXIS || axis == Y_AXIS || axis == Z_AXIS);
            break;
        }
    }

    int findMinValue(std::vector<double> vec)
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

    template<typename PointNormalT>
    double computeZetaDistance(PointNormalT p1, PointNormalT p2)
    {
        unsigned int numAngleNormals = 3;
        double distance = 0;
        double normalizeP1 = 0;
        double normalizeP2 = 0;

        //  unsigned int i;
//#pragma omp parallel for private(i)  shared(distance,p1)
        for (unsigned int i = 0; i < numAngleNormals; ++i)
        {
//#pragma omp atomic
            distance += p1.normal[i] * p2.normal[i];
//#pragma omp atomic
            normalizeP1 += pow(p1.normal[i], 2);
//#pragma omp atomic
            normalizeP2 += pow(p2.normal[i], 2);
        }
        return acos(distance / (sqrt(normalizeP1) * sqrt(normalizeP2)));
    }

    double computeZetaPointCloudDistance(pcl::PointCloud<PointT> pointCloud1, pcl::PointCloud<PointT> pointCloud2)
    {
        assert(pointCloud1.points.size() == pointCloud2.points.size());

        //ComputeNormals
        pcl::PointCloud<pcl::Normal> pointCloudNormals1 = this->toolBox.estimatingNormals(pointCloud1, 10); //10
        pcl::PointCloud<pcl::Normal> pointCloudNormals2 = this->toolBox.estimatingNormals(pointCloud2, 10); //10

        double distance = 0;
        //unsigned int iterPoint;
        //#pragma omp parallel for private(iterPoint)  shared(distance,pointCloudNormals1,pointCloudNormals2)
        for (unsigned int  iterPoint = 0; iterPoint < pointCloud1.points.size(); ++iterPoint)
        {
//#pragma omp atomic
            distance += (tanh((3.0 / 2.0) * (computeZetaDistance(pointCloudNormals1.points[iterPoint], pointCloudNormals2.points[iterPoint]) - M_PI)) + 1.0);
            //distance += ((computeZetaDistance(pointCloudNormals1.points[iterPoint], pointCloudNormals2.points[iterPoint]) / (double) M_PI));
        }

        return distance / (double) pointCloud1.points.size();
    }

    double computeZetaPointCloudDistanceTEST(pcl::PointCloud<PointT> pointCloud1, pcl::PointCloud<PointT> pointCloud2, int axis)
    {
        assert(pointCloud1.points.size() == pointCloud2.points.size());

        //ComputeNormals
        pcl::PointCloud<pcl::Normal> pointCloudNormals1 = this->toolBox.estimatingNormals(pointCloud1, 10); //10
        pcl::PointCloud<pcl::Normal> pointCloudNormals2 = this->toolBox.estimatingNormals(pointCloud2, 10); //10

        double distance = 0;
        for (unsigned int iterPoint = 0; iterPoint < pointCloud1.points.size(); ++iterPoint)
        {
            distance += (tanh(
                             (3.0 / 2.0)
                             * ((pointCloudNormals1.points[iterPoint].normal[axis] * pointCloudNormals2.points[iterPoint].normal[axis])
                                / (fabs(pointCloudNormals1.points[iterPoint].normal[axis]) * fabs(pointCloudNormals2.points[iterPoint].normal[axis])) - M_PI)) + 1.0);
            //distance += ((computeZetaDistance(pointCloudNormals1.points[iterPoint], pointCloudNormals2.points[iterPoint]) / (double) M_PI));
        }

        return distance / (double) pointCloud1.points.size();
    }

    double computeEuclideanDistance(PointT p1, PointT p2)
    {
        double x, y, z;

        x = pow((p1.x - p2.x), 2);
        y = pow((p1.y - p2.y), 2);
        z = pow((p1.z - p2.z), 2);

        return sqrt(x + y + z);
    }

    double computeEuclideanPointCloudDistance(pcl::PointCloud<PointT> pointCloud1, pcl::PointCloud<PointT> pointCloud2)
    {
        assert(pointCloud1.points.size() == pointCloud2.points.size());

        double distance = 0;
        for (unsigned int iterPoint = 0; iterPoint < pointCloud1.points.size(); ++iterPoint)
        {
            distance += computeEuclideanDistance(pointCloud1.points[iterPoint], pointCloud2.points[iterPoint]);
        }
        return (distance / (double) pointCloud1.points.size() * 2.0f);
        //return (distance / (double) pointCloud1.points.size());
    }

    double computeDistance(pcl::PointCloud<PointT> pointCloud1, pcl::PointCloud<PointT> pointCloud2, int axis)
    {
        double eucl = this->computeEuclideanPointCloudDistance(pointCloud1, pointCloud2);
        double zeta = this->computeZetaPointCloudDistanceTEST(pointCloud1, pointCloud2, axis); //this->computeZetaPointCloudDistance(pointCloud1, pointCloud2);

        //std::cout << "Eucl = " << eucl << "  zeta = " << zeta << "\n";
        return (eucl + zeta); ///2.0;
    }

    CObjectPoseNormalization()
    {
        this->pointCloudModel_1.resize(3);
        this->doPublish = false;
    }

    void setPointCloudModel(pcl::PointCloud<PointT> pointCloud)
    {
        this->pointCloudModel_0 = pointCloud;
    }

    void normalizePositionScale(pcl::PointCloud<PointT> &pointCloud)
    {
        assert(pointCloud.size() > 0);

        pcl::PointXYZ centroid;

        //PointCloud centroid
        centroid = this->toolBox.pointCloudCentroid(pointCloud);
        //Bounding Box centroid of pointcloud
        //pcl::PointCloud<pcl::PointXYZ> boundingBox;
        //centroid = this->toolBox.pointCloudBoundingBoxCentroid(pointCloud, boundingBox);

        //Eigen::Vector4f centroidPCL;
        //pcl::compute3DCentroid (pointCloud, centroidPCL);
        //std::cout<<"SELF centroid::"<< centroid << "  PCL::"<<centroidPCL<<"\n";

        //do translation that centroid is coord frame origin!
        std::vector<double> distances;
        double maxDistance = 0;
        double t = 0;

        pcl::PointXYZ normalizedCentroid; // :p
        normalizedCentroid.x = 0;
        normalizedCentroid.y = 0;
        normalizedCentroid.z = 0;

        //Translation to centroid
        for (unsigned int it = 0; it < pointCloud.points.size(); it++)
        {
            pointCloud.points[it].x = pointCloud.points[it].x - centroid.x;
            pointCloud.points[it].y = pointCloud.points[it].y - centroid.y;
            pointCloud.points[it].z = pointCloud.points[it].z - centroid.z;

            distances.push_back(this->toolBox.euclDistanceBtwPoints(pointCloud.points[it], normalizedCentroid)); // cent 0,0,0
        }

        //Normalization to unit sphere
        maxDistance = *std::max_element(distances.begin(), distances.end());
        t = ((double) 1.0 / maxDistance);
        for (unsigned int it = 0; it < pointCloud.points.size(); it++)
        {

            pointCloud.points[it].x = normalizedCentroid.x + (pointCloud.points[it].x - normalizedCentroid.x) * t;
            pointCloud.points[it].y = normalizedCentroid.y + (pointCloud.points[it].y - normalizedCentroid.y) * t;
            pointCloud.points[it].z = normalizedCentroid.z + (pointCloud.points[it].z - normalizedCentroid.z) * t;

            //  std::cout << pointCloud.points[it] << "-----" << this->toolBox.euclDistanceBtwPoints(pointCloud.points[it], normalizedCentroid) << "\n";
        }

        ////    sample
        //  std::cout<<"Normalized Subsampling before "<<pointCloud.points.size()<<std::endl;
        this->toolBox.subsampling(pointCloud, POSE_NORM_SUBSAMPLING); //0.05 good //after size norm e.g. if 0.1 then abt ten points in a line describe the height since object is 1 heigt
        //  std::cout<<"Normalized Subsampling after "<<pointCloud.points.size()<<std::endl;
        ///
    }

    pcl::PointCloud<PointT> alignPointCloud(pcl::PointCloud<PointT> &pointCloud)
    {

        //  pcl_visualization::CloudViewer viewer("out");
        assert(pointCloud.points.size() > 0);
        std::vector<int> rotationSequence;
        rotationSequence.push_back(Z_AXIS);
        rotationSequence.push_back(Y_AXIS);
        rotationSequence.push_back(Z_AXIS);

        /*rotationSequence.push_back(Y_AXIS);
         rotationSequence.push_back(X_AXIS);
         rotationSequence.push_back(Y_AXIS);*/

        /*rotationSequence.push_back(Y_AXIS); worked alignemt
         rotationSequence.push_back(Z_AXIS);
         rotationSequence.push_back(Y_AXIS);*/

        double maxAngleAlignment = 180.0f;
        double stepAngleAlignment = STEP_ANGLE_ALIGNEMENT; //2.0 was trained with

        pcl::PointCloud<PointT> pointCloudModel = pointCloud;
        pcl::PointCloud<pcl::PointXYZ> boundingBox;
        this->toolBox.pointCloudBoundingBoxCentroid(pointCloud, boundingBox);
        //pointCloudModel = boundingBox;
        //pcl::PointCloud<PointT> pointCloudObjectModel = pointCloud;

        /*int t;
         std::cin >> t;
         if (t == 0)
         {
         std::cout << "contin...";
         }*/

        if (doPublish)
        {
            this->publishPointCloud(pointCloudModel);
            this->publishPrincipleAxis(boundingBox);
        }

        //  viewer.showCloud(pointCloudModel);
        //  sleep(6);
        pcl::PointCloud<PointT> pointCloudModelBest;
        pcl::PointCloud<PointT> pointCloudObjectModelBest;
        for (unsigned int iterRotAxis = 0; iterRotAxis < rotationSequence.size(); ++iterRotAxis)
        {
            //std::cout << "NEW Rotate Axis" << rotationSequence[iterRotAxis] << "\n";

            //      int z;
            //   std::cin>>z;
            //   if(z==0)
            //   {
            //       std::cout<<"contin...";
            //   }

            double lowestOverlapError = std::numeric_limits<double>::infinity();

            double lowestOverlapErrorAngle = std::numeric_limits<double>::infinity();
            pcl::PointCloud<PointT> pointCloudModelReflection;
            pcl::PointCloud<PointT> pointCloudObjectModelReflection;

            if (doPublish)
            {
                this->publishPointCloud(pointCloudModel);
            }
            //viewer.showCloud(pointCloudModel);
            //sleep(3);
            //Do symmetric reflection according the rotation axis,
            switch (rotationSequence[iterRotAxis])
            {
            case X_AXIS:
                this->transformReflection(pointCloudModel, pointCloudModelReflection, XY_PLANE);
                //this->transformReflection(pointCloudObjectModel, pointCloudObjectModelReflection, XY_PLANE);
                break;
            case Y_AXIS:
                this->transformReflection(pointCloudModel, pointCloudModelReflection, YZ_PLANE);
                //this->transformReflection(pointCloudObjectModel, pointCloudObjectModelReflection, YZ_PLANE);
                break;
            case Z_AXIS:
                this->transformReflection(pointCloudModel, pointCloudModelReflection, XZ_PLANE); //X_AXIS
                //this->transformReflection(pointCloudObjectModel, pointCloudObjectModelReflection, XZ_PLANE);
                break;
            default:
                assert(rotationSequence[iterRotAxis] == X_AXIS || rotationSequence[iterRotAxis] == Y_AXIS || rotationSequence[iterRotAxis] == Z_AXIS);
                break;
            }

            //      viewer.showCloud(pointCloudModelReflection);
            if (doPublish)
            {
                this->publishPointCloud(pointCloudModelReflection);
            }
            //std::cout << "reflectioed..";

            //  sleep(5);
            /*int w;
             std::cin >> w;
             if (w == 0)
             {
             std::cout << "contin...";
             }*/

            for (unsigned int iterAngle = 0; iterAngle < maxAngleAlignment; iterAngle += stepAngleAlignment)
            {
                pcl::PointCloud<PointT> temp;
                temp = pointCloudModel;
                temp += pointCloudModelReflection;
                //viewer.showCloud(temp);
                //  viewer.showCloud(pointCloudModelReflection);
                rotate(pointCloudModel, pointCloudModel, (1.0) * stepAngleAlignment, rotationSequence[iterRotAxis]);
                rotate(pointCloudModelReflection, pointCloudModelReflection, (-1.0) * stepAngleAlignment, rotationSequence[iterRotAxis]);

//              rotate(pointCloudObjectModel, pointCloudObjectModel, (1) * stepAngleAlignment, rotationSequence[iterRotAxis]);
//              rotate(pointCloudObjectModelReflection, pointCloudObjectModelReflection, (-1) * stepAngleAlignment, rotationSequence[iterRotAxis]);

                double currentOverlapError = computeDistance(pointCloudModel, pointCloudModelReflection, iterRotAxis);              //computeEuclideanPointCloudDistance(pointCloudModel, pointCloudModelReflection);

                if (currentOverlapError < lowestOverlapError)
                {
                    lowestOverlapError = currentOverlapError;
                    pointCloudModelBest = pointCloudModel;
                    //  pointCloudObjectModelBest = pointCloudObjectModel;
                    lowestOverlapErrorAngle = iterAngle;
                }
                //std::cout << "Align " << iterAngle << "° => Error=" << currentOverlapError << "\n";
                //sleep(0.7);
                if (doPublish)
                {
                    this->publishPointCloud(temp);
                }
            }
            //std::cout << "Best at " << lowestOverlapErrorAngle << " " << lowestOverlapError << " \n";
            pointCloudModel = pointCloudModelBest;
            //sleep(3);
            //          viewer.showCloud(pointCloudModelBest);
            //          if (doPublish)
            //          {
            //                  this->publishPointCloud(pointCloudModel);
            //          }
            //      sleep(3);
            /*
             int u;
             std::cin>>u;
             if(u==0)
             {
             std::cout<<"contin...";
             }*/

        }

        //std::cout << "NormalizePose...done()" << std::endl;
        //      viewer.showCloud(pointCloudModelBest);
        //sleep(5);
        //std::cout << "NormalizePose...organize done()" << std::endl;
        //rotates frame accordingly the principle axis of the object(maybe not necessry)

        //pointCloudModelBest = this->organizeAxis(pointCloudModelBest);

        //  sleep(2);
        if (doPublish)
        {
            this->publishPointCloud(pointCloudModelBest);
            this->publishPointCloud(pointCloudModelBest);
            //this->publishPrincipleAxis(pointCloudModelBest);/*
        }
        //  sleep(5);
        // viewer.showCloud(pointCloudModelBest);

        /*   int z;
         std::cin >> z;
         if (z == 0)
         {
         std::cout << "contin...";
         }*/

        //std::cout << "NormalizePose...done()" << std::endl;
        //sleep(2);
        //kill this if object
        //pointCloudModelBest = pointCloudObjectModelBest; //pointCloudObjectModel;
        return pointCloudModelBest;
        //pointCloudModel  = pointCloudModelBest;
    }

    pcl::PointCloud<PointT> normalizePose(bool align = true)
    {
        pcl::PointCloud<PointT> pointCloud = this->pointCloudModel_0;

        //std::cout << "CObjectPoseNormalization::normalizePose...Normalize Scale and position....\n";
        this->normalizePositionScale(pointCloud);
        std::cout << "p" << std::flush;
        //std::cout << "CObjectPoseNormalization::normalizePose...Align Pointcloud....\n";

        if (align)
        {
            std::cout << "a" << std::flush;
            pointCloud = this->alignPointCloud(pointCloud);
        }

        return pointCloud;
    }

    pcl::PointCloud<PointT> organizeAxis(pcl::PointCloud<PointT> pointCloud)
    {
        pcl::PointCloud<PointT> organizedPointCloud = pointCloud;
        pcl::PointCloud<PointT> reflectionX;
        pcl::PointCloud<PointT> reflectionY;
        pcl::PointCloud<PointT> reflectionZ;

        this->transformReflection(pointCloud, reflectionX, XY_PLANE);
        this->transformReflection(pointCloud, reflectionY, YZ_PLANE);
        this->transformReflection(pointCloud, reflectionZ, XZ_PLANE);

        std::vector<double> differences;
        double diffX = this->computeEuclideanPointCloudDistance(pointCloud, reflectionX);
        differences.push_back(diffX);
        double diffY = this->computeEuclideanPointCloudDistance(pointCloud, reflectionY);
        differences.push_back(diffY);
        double diffZ = this->computeEuclideanPointCloudDistance(pointCloud, reflectionZ);
        differences.push_back(diffZ);

        //Z Y X

        int newXAxis = findMinValue(differences);
        //std::cout << "X" << newXAxis << " " << differences[newXAxis] << "\n";
        differences[newXAxis] = std::numeric_limits<double>::infinity();

        int newYAxis = findMinValue(differences);
        //std::cout << "Y" << newYAxis << " " << differences[newYAxis] << "\n";
        differences[newYAxis] = std::numeric_limits<double>::infinity();

        int newZAxis = findMinValue(differences);
        //std::cout << "Z" << newZAxis << " " << differences[newZAxis] << "\n";
        differences[newZAxis] = std::numeric_limits<double>::infinity();

        /*
         int newYAxis = findMinValue(differences);
         std::cout<<"Y"<<newYAxis<<" "<<    differences[newYAxis]<<"\n";
         differences[newYAxis] = std::numeric_limits<double>::infinity();
         int newZAxis = findMinValue(differences);
         std::cout<<"Z"<<newZAxis<<" "<<    differences[newZAxis]<<"\n";
         differences[newZAxis] = std::numeric_limits<double>::infinity();


         int newXAxis = findMinValue(differences);
         std::cout<<"X"<<newXAxis<<" "<<    differences[newXAxis]<<"\n";
         differences[newXAxis] = std::numeric_limits<double>::infinity();
         */

        //std::cout<<newXAxis<<" "<<newYAxis<<" "<<newZAxis<<std::endl;
        for (unsigned int iter = 0; iter < pointCloud.points.size(); ++iter)
        {

            if (newXAxis == 0)
                organizedPointCloud.points[iter].x = pointCloud.points[iter].x;
            if (newXAxis == 1)
                organizedPointCloud.points[iter].y = pointCloud.points[iter].x;
            if (newXAxis == 2)
                organizedPointCloud.points[iter].z = pointCloud.points[iter].x;

            if (newYAxis == 0)
                organizedPointCloud.points[iter].x = pointCloud.points[iter].y;
            if (newYAxis == 1)
                organizedPointCloud.points[iter].y = pointCloud.points[iter].y;
            if (newYAxis == 2)
                organizedPointCloud.points[iter].z = pointCloud.points[iter].y;

            if (newZAxis == 0)
                organizedPointCloud.points[iter].x = pointCloud.points[iter].z;
            if (newZAxis == 1)
                organizedPointCloud.points[iter].y = pointCloud.points[iter].z;
            if (newZAxis == 2)
                organizedPointCloud.points[iter].z = pointCloud.points[iter].z;

            //  organizedPointCloud.points[iter].x = pointCloud.points[iter].data[newXAxis];
            //  organizedPointCloud.points[iter].y = pointCloud.points[iter].data[newYAxis];
            //  organizedPointCloud.points[iter].z = pointCloud.points[iter].data[newZAxis];
        }

        return organizedPointCloud;
    }

    void publishPointCloud(pcl::PointCloud<PointT> pointCloud)
    {
        sensor_msgs::PointCloud2 pointsCloudMsg;
        pcl::toROSMsg(pointCloud, pointsCloudMsg);
        pointsCloudMsg.header.frame_id = "/world";
        pointsCloudMsg.header.stamp = ros::Time::now();
        this->pub.publish(pointsCloudMsg);

    }

    void publishPrincipleAxis(pcl::PointCloud<PointT> pointCloud)
    {
        visualization_msgs::Marker line_list;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.header.frame_id = "/world";
        line_list.color.r = 0.5;
        line_list.color.a = 0.5;
        line_list.pose.orientation.w = 1.0;
        line_list.scale.x = 0.025;
        line_list.header.stamp = ros::Time::now();

        std::vector<std::vector<double> > pointCloudVec = this->toolBox.convertPointCloudToStdVec(pointCloud);

        unsigned dim = 3;
        CPca principleAxis;
        principleAxis.init(pointCloudVec, dim);
        std::vector<std::vector<double> > eigenVectors;
        std::vector<double> eigenValues;

        principleAxis.computePca(eigenVectors, eigenValues);

        pcl::PointXYZ centroid = toolBox.pointCloudCentroid(pointCloud);
        geometry_msgs::Point p;

        int maxEigen = this->toolBox.findMaxValue(eigenValues);

        p.x = eigenVectors[maxEigen][0];            //+ p.x;
        p.y = eigenVectors[maxEigen][1];            //+ p.y;
        p.z = eigenVectors[maxEigen][2];            //+ p.z;
        line_list.points.push_back(p);

        p.x = -1.0 * eigenVectors[maxEigen][0];         //+ p.x;
        p.y = -1.0 * eigenVectors[maxEigen][1];         //+ p.y;
        p.z = -1.0 * eigenVectors[maxEigen][2];         //+ p.z;
        //  std::cout << p.x << " " << p.y << " " << p.z << " " << "\n";
        line_list.points.push_back(p);
        /*
         //draw all axis!!
         p.x = centroid.x;
         p.y = centroid.y;
         p.z = centroid.z;
         line_list.points.push_back(p);

         p.x = eigenVectors[0][0] + p.x;
         p.y = eigenVectors[0][1] + p.y;
         p.z = eigenVectors[0][2] + p.z;
         std::cout << p.x << " " << p.y << " " << p.z << " " << "\n";

         line_list.points.push_back(p);

         p.x = centroid.x;
         p.y = centroid.y;
         p.z = centroid.z;
         line_list.points.push_back(p);

         p.x = eigenVectors[1][0] + p.x;
         p.y = eigenVectors[1][1] + p.y;
         p.z = eigenVectors[1][2] + p.z;
         std::cout << p.x << " " << p.y << " " << p.z << " " << "\n";

         line_list.points.push_back(p);

         p.x = centroid.x;
         p.y = centroid.y;
         p.z = centroid.z;
         line_list.points.push_back(p);

         p.x = eigenVectors[2][0] + p.x;
         p.y = eigenVectors[2][1] + p.y;
         p.z = eigenVectors[2][2] + p.z;
         std::cout << p.x << " " << p.y << " " << p.z << " " << "\n";

         line_list.points.push_back(p);
         */
        this->principleAxis_pub.publish(line_list);
    }

    void computePrincipleAxis(pcl::PointCloud<PointT> pointCloud)
    {

        /*assert(pointCloud.points.size()>0);

         std::vector < std::vector<double> > pointCloudVec = this->toolBox.convertPointCloudToStdVec(pointCloud);

         CvMat* featureVector;
         CvMat* avgVector;
         CvMat* eigenVectors;
         CvMat* eigenValues;

         featureVector = cvCreateMat(pointCloudVec.size(), pointCloudVec[0].size(), CV_32FC1);
         avgVector = cvCreateMat(1, pointCloudVec[0].size(), CV_32FC1);
         eigenVectors = cvCreateMat(pointCloudVec[0].size(), pointCloudVec[0].size(), CV_32FC1);
         eigenValues = cvCreateMat(1, pointCloudVec[0].size(), CV_32FC1);

         for (int iExample = 0; iExample < pointCloudVec.size(); iExample++)
         {
         for (int iFeature = 0; iFeature < pointCloudVec[0].size(); iFeature++)
         {
         *((float*) CV_MAT_ELEM_PTR( *featureVector, iExample, iFeature )) = pointCloudVec[iExample][iFeature];
         }
         }

         cvCalcPCA(featureVector,avgVector,eigenValues,eigenVectors,CV_PCA_DATA_AS_ROW);



         std::cout<<" EigenValue 0 " <<CV_MAT_ELEM( *eigenValues, float, 0, 0)<<std::endl;
         std::cout<<" EigenValue 1 " <<CV_MAT_ELEM( *eigenValues, float, 0, 1)<<std::endl;
         std::cout<<" EigenValue 2 " <<CV_MAT_ELEM( *eigenValues, float, 0, 2)<<std::endl;



         cv::Mat eigenValuesN(eigenValues);*/
    }
};

#endif

