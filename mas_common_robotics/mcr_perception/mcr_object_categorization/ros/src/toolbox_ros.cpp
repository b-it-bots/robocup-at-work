/*
 *  CToolBox.cpp
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#include "toolbox_ros.h"

double CToolBoxROS::euclDistanceBtwPoints(pcl::PointXYZ &p1, pcl::PointXYZ &p2)
{
    double x, y, z;

    x = pow((p1.x - p2.x), 2);
    y = pow((p1.y - p2.y), 2);
    z = pow((p1.z - p2.z), 2);

    return sqrt(x + y + z);
}

double CToolBoxROS::euclDistanceBtwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2)
{
    double x, y, z;

    x = pow((p1.x - p2.x), 2);
    y = pow((p1.y - p2.y), 2);
    z = pow((p1.z - p2.z), 2);

    return sqrt(x + y + z);
}

void CToolBoxROS::enNoisePointCloud(pcl::PointCloud<pcl::PointXYZ> &pointCloud, double addNoise)
{
    for (unsigned int i = 0; i < pointCloud.points.size(); ++i)
    {
        pointCloud.points[i].x += this->randNumber(-(pointCloud.points[i].x * addNoise), pointCloud.points[i].x * addNoise);
        pointCloud.points[i].y += this->randNumber(-(pointCloud.points[i].y * addNoise), pointCloud.points[i].y * addNoise);
        pointCloud.points[i].z += this->randNumber(-(pointCloud.points[i].z * addNoise), pointCloud.points[i].z * addNoise);
    }
}

pcl::PointCloud<pcl::PointXYZ> CToolBoxROS::filterDistance(pcl::PointCloud<pcl::PointXYZ> &cloud, float minDist, float maxDist, std::string axis)
{
    pcl::PassThrough < pcl::PointXYZ > pass;

    pcl::PointCloud < pcl::PointXYZ > cloud_filtered;

    pass.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZ> > (cloud));
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(minDist, maxDist);

    pass.filter(cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> CToolBoxROS::filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float minDist, float maxDist, std::string axis)
{
    pcl::PassThrough < pcl::PointXYZRGB > pass;

    pcl::PointCloud < pcl::PointXYZRGB > cloud_filtered;

    pass.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGB> > (cloud));
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(minDist, maxDist);

    pass.filter(cloud_filtered);

    return cloud_filtered;
}

//spherical distance filtering
bool CToolBoxROS::filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float thresholdDistance)
{
    float pointDistance;
    pcl::PointCloud < pcl::PointXYZRGB > filteredDistanceCloud;

    pcl::PointIndices filteredDistanceIndices;
    for (size_t i = 0; i < (cloud.points.size()); i++)
    {
        pointDistance = sqrt(pow(fabs(cloud.points[i].x), 2) + pow(fabs(cloud.points[i].y), 2) + pow(fabs(cloud.points[i].z), 2));

        if (pointDistance > thresholdDistance)
        {
            filteredDistanceIndices.indices.push_back(i);
        }
    }

    if (filteredDistanceIndices.indices.size() > 0)
    {
        pcl::ExtractIndices < pcl::PointXYZRGB > extract;
        extract.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGB> > (cloud));
        extract.setIndices(boost::make_shared < pcl::PointIndices > (filteredDistanceIndices));
        extract.setNegative(true);
        extract.filter(filteredDistanceCloud);

        cloud.width = filteredDistanceCloud.width;
        cloud.height = filteredDistanceCloud.height;
        cloud.points = filteredDistanceCloud.points;
        //filteredDistanceIndices.indices.clear();
        return true;
    }

    return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZ> &cloud, const double dLeafsize)
{
    pcl::PointCloud < pcl::PointXYZ > cloud_filtered;

    pcl::VoxelGrid < pcl::PointXYZ > sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
    sor.filter(cloud_filtered);
    cloud = cloud_filtered;

    return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const double dLeafsize)
{
    pcl::PointCloud < pcl::PointXYZRGB > cloud_filtered;

    pcl::VoxelGrid < pcl::PointXYZRGB > sor;
    sor.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGB> > (cloud));
    sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
    sor.filter(cloud_filtered);
    cloud = cloud_filtered;

    return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const double dLeafsize)
{
    pcl::PointCloud < pcl::PointXYZRGBNormal > cloud_filtered;

    pcl::VoxelGrid < pcl::PointXYZRGBNormal > sor;
    sor.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGBNormal> > (cloud));
    sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
    sor.filter(cloud_filtered);
    cloud = cloud_filtered;

    return false;
}

bool CToolBoxROS::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ> &cloud, int meanK, float stddevMulThresh)
{
    pcl::PointCloud < pcl::PointXYZ > cloud_filtered;

    pcl::StatisticalOutlierRemoval < pcl::PointXYZ > sor;
    sor.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZ> > (cloud));
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(cloud_filtered);

    sor.setNegative(false);  //get inliers = true
    sor.filter(cloud_filtered);

    cloud = cloud_filtered;

    return false;
}

bool CToolBoxROS::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int meanK, float stddevMulThresh)
{
    pcl::PointCloud < pcl::PointXYZRGB > cloud_filtered;

    pcl::StatisticalOutlierRemoval < pcl::PointXYZRGB > sor;
    sor.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGB> > (cloud));
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_filtered << std::endl;

    sor.setNegative(false);  //get inliers = true
    sor.filter(cloud_filtered);

    cloud = cloud_filtered;

    return false;
}

pcl::PointCloud<pcl::PointXYZRGBNormal> CToolBoxROS::movingLeastSquares(pcl::PointCloud<pcl::PointXYZRGB> &cloud, float searchRadius)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> >();
    pcl::PointCloud < pcl::PointXYZRGBNormal > mls_points_with_normals;

    tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> >();
    tree->setInputCloud(cloud.makeShared());

    pcl::MovingLeastSquares < pcl::PointXYZRGB, pcl::PointXYZRGBNormal > movingLeastSquaresExtractor;
    movingLeastSquaresExtractor.setComputeNormals(true);
    movingLeastSquaresExtractor.setInputCloud(cloud.makeShared());
    movingLeastSquaresExtractor.setPolynomialFit(true);
    movingLeastSquaresExtractor.setSearchRadius(searchRadius);
    movingLeastSquaresExtractor.setPolynomialOrder(3);
    movingLeastSquaresExtractor.setSearchMethod(tree);
    movingLeastSquaresExtractor.process(mls_points_with_normals);

    return mls_points_with_normals;
}

pcl::PointCloud<pcl::PointXYZ> CToolBoxROS::movingLeastSquares2(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    pcl::PointCloud < pcl::PointXYZ > mls_points;

    tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    tree->setInputCloud(cloud.makeShared());

    pcl::MovingLeastSquares < pcl::PointXYZ, pcl::PointXYZ > movingLeastSquaresExtractor;
    movingLeastSquaresExtractor.setComputeNormals(true);
    movingLeastSquaresExtractor.setInputCloud(cloud.makeShared());
    movingLeastSquaresExtractor.setPolynomialFit(true);
    movingLeastSquaresExtractor.setSearchRadius(searchRadius);
    movingLeastSquaresExtractor.setPolynomialOrder(3);
    movingLeastSquaresExtractor.setSearchMethod(tree);
    movingLeastSquaresExtractor.process(mls_points);

    return mls_points;
}

pcl::PointCloud<pcl::PointNormal> CToolBoxROS::movingLeastSquares(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    pcl::PointCloud < pcl::PointNormal > mls_normals;

    tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    tree->setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZ> > (cloud));

    pcl::MovingLeastSquares < pcl::PointXYZ, pcl::PointNormal > movingLeastSquaresExtractor;
    movingLeastSquaresExtractor.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZ> > (cloud));
    movingLeastSquaresExtractor.setPolynomialFit(true);
    movingLeastSquaresExtractor.setSearchRadius(searchRadius);
    movingLeastSquaresExtractor.setPolynomialOrder(3);
    movingLeastSquaresExtractor.setSearchMethod(tree);
    movingLeastSquaresExtractor.process(mls_normals);

    return mls_normals;
}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int KSearch)
{
    pcl::PointCloud < pcl::Normal > cloud_normals;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> >();
    pcl::NormalEstimation < pcl::PointXYZRGB, pcl::Normal > normalEstimation;

    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(cloud.makeShared());

    if (KSearch != -1)
        normalEstimation.setKSearch(KSearch);
    //if(searchRadius!=-1)
    //normalEstimation.setRadiusSearch(searchRadius);
    normalEstimation.compute(cloud_normals);
    //  for(unsigned int i =0; i < cloud_normals.points.size(); ++i)
    //  {
    //      std::cout<<cloud_normals.points[i]<<"\n";
    //  }

    return cloud_normals;
}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormalsIntegral(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    std::cout << "CToolBoxROS::estimatingNormalsIntegral...not itegrated!!" << std::endl;
    assert(cloud.points.size() > 0);

    //pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    pcl::PointCloud < pcl::Normal > normals;

    /*  ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
     ne.setMaxDepthChangeFactor(0.02f);
     ne.setNormalSmoothingSize(10.0f);
     ne.setInputCloud(cloud);
     ne.compute(normals);
     */
    return normals;
}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormalsOMP(pcl::PointCloud<pcl::PointXYZ> &cloud, double radius)
{
    std::cout << "CToolBoxROS::estimatingNormalsIntegral...not itegrated!!" << std::endl;
    assert(cloud.points.size() > 0);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP < pcl::PointXYZ, pcl::Normal > ne;
    ne.setInputCloud(cloud.makeShared());

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(radius);

    // Compute the features
    ne.compute(*cloud_normals);

    return *cloud_normals;
}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormals(pcl::PointCloud<pcl::PointXYZ> &cloud, int KSearch)
{
    assert(cloud.points.size() > 0);
    pcl::PointCloud < pcl::Normal > cloud_normals;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    pcl::NormalEstimation < pcl::PointXYZ, pcl::Normal > normalEstimation;

    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(cloud.makeShared());

    if (KSearch != -1)
        normalEstimation.setKSearch(KSearch);
    //if(searchRadius!=-1)
    //normalEstimation.setRadiusSearch(searchRadius);
    //std::cout<<"cloud.points.size() "<<cloud.points.size();
    normalEstimation.compute(cloud_normals);

    return cloud_normals;

}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormals2(pcl::PointCloud<pcl::PointXYZ> &cloud, double searchRadius)
{
    pcl::PointCloud < pcl::Normal > cloud_normals;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
    pcl::NormalEstimation < pcl::PointXYZ, pcl::Normal > normalEstimation;

    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZ> > (cloud));

    normalEstimation.setRadiusSearch(searchRadius);

    normalEstimation.compute(cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormals2(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double searchRadius)
{
    pcl::PointCloud < pcl::Normal > cloud_normals;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> >();
    pcl::NormalEstimation < pcl::PointXYZRGB, pcl::Normal > normalEstimation;

    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGB> > (cloud));

    normalEstimation.setRadiusSearch(searchRadius);

    normalEstimation.compute(cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::Boundary> CToolBoxROS::estimatingBoundaryPoints(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{

    pcl::PointCloud < pcl::Boundary > cloud_boundary;
    std::cout << "not implemented";
    /*
     pcl::Boundary<pcl::PointXYZRGB, pcl::pcl::Boundary> normalEstimation;

     normalEstimation.setSearchMethod(tree);
     normalEstimation.setInputCloud(boost::make_shared<pcl::PointCloud<
     pcl::PointXYZRGB> >(cloud));

     if (KSearch != -1)
     normalEstimation.setKSearch(KSearch);
     //if(searchRadius!=-1)
     //normalEstimation.setRadiusSearch(searchRadius);
     normalEstimation.compute(cloud_normals);*/

    return cloud_boundary;

}

pcl::PointCloud<pcl::Normal> CToolBoxROS::filterNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud)
{
    pcl::PointCloud < pcl::Normal > cloud_normals;

    cloud_normals.points.resize(cloud.points.size());
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        cloud_normals.points[i].normal[0] = cloud.points[i].normal[0];
        cloud_normals.points[i].normal[1] = cloud.points[i].normal[1];
        cloud_normals.points[i].normal[2] = cloud.points[i].normal[2];
    }

    return cloud_normals;

}
/*
 pcl::PointCloud<pcl::FPFHSignature33> CToolBox::FPFHFeatureExtractor(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius)
 {
 pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
 pcl::PointCloud<pcl::Normal> normalEstimation;
 pcl::PointCloud<pcl::FPFHSignature33> cloudFeatures;
 pcl::KdTreeANN<pcl::PointXYZ> tree;

 fpfhEstimation.setSearchMethod(boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > (tree));
 fpfhEstimation.setRadiusSearch (searchRadius);

 normalEstimation = this->estimatingNormals(cloud,0,searchRadius);

 fpfhEstimation.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
 fpfhEstimation.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (normalEstimation));
 fpfhEstimation.compute(cloudFeatures);

 return cloudFeatures;
 }*/

double CToolBoxROS::dotProduct(std::vector<double> &x, std::vector<double> &y)
{
    double result = 0;

    if (x.size() != y.size())
    {
        std::cout << "CToolBoxROS::dotProduct... vector length !=  \n";
        return result;
    }

    x = normalizePoint3D(x);
    y = normalizePoint3D(y);

    for (unsigned int i = 0; i < x.size(); ++i)
    {
        result += x[i] * y[i];
    }
    return result;
}

//float[3]
double CToolBoxROS::dotProduct(float* x, float* y)
{

    std::vector<double> vecX;
    std::vector<double> vecY;

    vecX.resize(3);
    vecY.resize(3);

    vecX[0] = x[0];
    vecX[1] = x[1];
    vecX[2] = x[2];

    vecY[0] = y[0];
    vecY[1] = y[1];
    vecY[2] = y[2];

    return this->dotProduct(vecX, vecY);
}

double CToolBoxROS::dotProduct(pcl::PointXYZRGBNormal &x, pcl::PointXYZRGBNormal &y)
{
    std::vector<double> vecX;
    std::vector<double> vecY;

    vecX.resize(3);
    vecY.resize(3);

    vecX[0] = x.x;
    vecX[1] = x.y;
    vecX[2] = x.z;

    vecY[0] = y.x;
    vecY[1] = y.y;
    vecY[2] = y.z;

    return this->dotProduct(vecX, vecY);
}

double CToolBoxROS::variance(std::vector<double> &histo, double mean)
{
    double var = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        var += pow((histo[k] - mean), 2);
    }
    return (var / (double) histo.size());
}

double CToolBoxROS::mean(std::vector<double> &histo)
{
    double mean = 0;

    for (unsigned k = 0; k < histo.size(); k++)
    {
        mean += histo[k];
    }

    return (mean / (double) histo.size());
}

void CToolBoxROS::markClusteredPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &clusteredPointCloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal> &markedPointCloud)
{
    int color = 0;

    if (clusteredPointCloud.size() > 0)
    {
        markedPointCloud.points.clear();

        for (unsigned int iterCluster = 0; iterCluster < clusteredPointCloud.size(); iterCluster++)
        {
            color = rand() % 10000;

            ////////////////
            pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> >();
            tree->setInputCloud(boost::make_shared < pcl::PointCloud<pcl::PointXYZRGBNormal> > (clusteredPointCloud.at(iterCluster)));

            //////////////////////
            for (unsigned int iterPoint = 0; iterPoint < clusteredPointCloud.at(iterCluster).points.size(); iterPoint++)
            {
                pcl::PointXYZRGBNormal pointRGB;
                pointRGB = clusteredPointCloud.at(iterCluster).points.at(iterPoint);

                pointRGB.rgb = color;
                ///////////////////////////////////////7

                int k = 10;  //25; //30
                if (clusteredPointCloud.at(iterCluster).size() > k)
                {

                    //                  int counterK = 0;
                    std::vector<int> k_indicies;
                    std::vector<float> k_distances;
                    k_distances.resize(k);
                    k_indicies.resize(k);
                    tree->nearestKSearch(pointRGB, k, k_indicies, k_distances);

                    std::vector<double> neighborCurvaturesDiff;
                    for (unsigned int iK = 0; iK < k; ++iK)
                    {
                        neighborCurvaturesDiff.push_back(pointRGB.curvature - clusteredPointCloud.at(iterCluster).points[k_indicies[iK]].curvature);
                    }
                    //std::cout << pointRGB << std::endl;
                    double meanCurDiff = mean(neighborCurvaturesDiff);
                    double stdDevCurDiff = sqrt(this->variance(neighborCurvaturesDiff, meanCurDiff));
                    //  std::cout << " |->" << stdDevCurDiff << std::endl;

                    //  if(pointRGB.curvature  < 1 && pointRGB.normal[0]!=0)
                    //  pointRGB.rgb = 1000;
                    /*int k = 10;
                     int counterK = 0;
                     std::vector<int> k_indicies;
                     std::vector<float> k_distances;
                     k_distances.resize(k);
                     k_indicies.resize(k);
                     tree->nearestKSearch(pointRGB, k, k_indicies, k_distances);

                     // std::cout<<pointRGB<<std::endl;
                     bool found = false;
                     for (unsigned int iK = 0; iK < k; ++iK) {
                     if (pointRGB.curvature != 0) {
                     std::cout<<pointRGB<<std::endl;
                     std::cout << ">> "
                     << (dotProduct(
                     clusteredPointCloud.at(iterCluster).points[k_indicies[iK]].normal,
                     pointRGB.normal)) << " ";
                     if (dotProduct(
                     clusteredPointCloud.at(iterCluster).points[k_indicies[iK]].normal,
                     pointRGB.normal) < 0.5) {

                     std::cout << "Wooaahh" << std::endl;
                     counterK++;

                     }

                     }
                     }

                     if (counterK >=k*0.5 )
                     pointRGB.rgb = 1000;
                     else*/

                    //if (abs(pointRGB.curvature) < 1.0)
                    if (stdDevCurDiff > 0.4)  //0.4
                        pointRGB.rgb = 100;  //color;
                }

                //////////////////////////////////////////
                markedPointCloud.points.push_back(pointRGB);
            }
        }

        markedPointCloud.header.frame_id = clusteredPointCloud.at(0).header.frame_id;
        markedPointCloud.height = markedPointCloud.points.size();
        markedPointCloud.width = 1;
    }
}

void CToolBoxROS::markClusteredPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusteredPointCloud, pcl::PointCloud<pcl::PointXYZ> &markedPointCloud)
{
    if (clusteredPointCloud.size() > 0)
    {
        markedPointCloud.points.clear();

        for (unsigned int iterCluster = 0; iterCluster < clusteredPointCloud.size(); iterCluster++)
        {
            for (unsigned int iterPoint = 0; iterPoint < clusteredPointCloud.at(iterCluster).points.size(); iterPoint++)
            {
                pcl::PointXYZ pointRGB;
                pointRGB = clusteredPointCloud.at(iterCluster).points.at(iterPoint);
                markedPointCloud.points.push_back(pointRGB);
            }
        }

        markedPointCloud.header.frame_id = clusteredPointCloud.at(0).header.frame_id;
        markedPointCloud.height = markedPointCloud.points.size();
        markedPointCloud.width = 1;
    }
}

bool CToolBoxROS::transformPointCloud(tf::TransformListener &tfListener, std::string &fromFrame, std::string &toFrame,
                                      const sensor_msgs::PointCloud2 &srcPointCloud, sensor_msgs::PointCloud2 &transformedPointCloud, float duration)
{
    bool success_tf = false;

    sensor_msgs::PointCloud pointCloudMsgConvert;
    sensor_msgs::PointCloud pointCloudMsgTransformed;

    sensor_msgs::convertPointCloud2ToPointCloud(srcPointCloud, pointCloudMsgConvert);

    try
    {
        tfListener.waitForTransform(toFrame, fromFrame, srcPointCloud.header.stamp, ros::Duration(duration));
        tfListener.transformPointCloud(std::string(toFrame), pointCloudMsgConvert, pointCloudMsgTransformed);
        success_tf = true;
    }
    catch (tf::TransformException &ex)
    {
        success_tf = false;
        return success_tf;
    }

    sensor_msgs::convertPointCloudToPointCloud2(pointCloudMsgTransformed, transformedPointCloud);

    return success_tf;
}

//1 for interior points and 0 for exterior points
int CToolBoxROS::pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointXYZRGBNormal &point)
{
    unsigned int convexHullSize = convexHull.points.size();

    unsigned int i, j, c = 0;
    for (i = 0, j = convexHullSize - 1; i < convexHullSize; j = i++)
    {
        if ((((convexHull.points[i].y <= point.y) && (point.y < convexHull.points[j].y))
                || ((convexHull.points[j].y <= point.y) && (point.y < convexHull.points[i].y)))
                && (point.x
                    < (convexHull.points[j].x - convexHull.points[i].x) * (point.y - convexHull.points[i].y)
                    / (convexHull.points[j].y - convexHull.points[i].y) + convexHull.points[i].x))

            c = !c;
    }
    return c;
}

int CToolBoxROS::pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZ> &convexHull, pcl::PointXYZ &point)
{
    unsigned int convexHullSize = convexHull.points.size();

    unsigned int i, j, c = 0;
    for (i = 0, j = convexHullSize - 1; i < convexHullSize; j = i++)
    {
        if ((((convexHull.points[i].y <= point.y) && (point.y < convexHull.points[j].y))
                || ((convexHull.points[j].y <= point.y) && (point.y < convexHull.points[i].y)))
                && (point.x
                    < (convexHull.points[j].x - convexHull.points[i].x) * (point.y - convexHull.points[i].y)
                    / (convexHull.points[j].y - convexHull.points[i].y) + convexHull.points[i].x))

            c = !c;
    }
    return c;
}

int CToolBoxROS::pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointXYZ &point)
{
    unsigned int convexHullSize = convexHull.points.size();

    unsigned int i, j, c = 0;
    for (i = 0, j = convexHullSize - 1; i < convexHullSize; j = i++)
    {
        if ((((convexHull.points[i].y <= point.y) && (point.y < convexHull.points[j].y))
                || ((convexHull.points[j].y <= point.y) && (point.y < convexHull.points[i].y)))
                && (point.x
                    < (convexHull.points[j].x - convexHull.points[i].x) * (point.y - convexHull.points[i].y)
                    / (convexHull.points[j].y - convexHull.points[i].y) + convexHull.points[i].x))

            c = !c;
    }
    return c;
}

//1 for interior points and 0 for exterior points
int CToolBoxROS::pointInsideConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &convexHull, pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud)
{
    for (unsigned int iter = 0; iter < point_cloud.points.size(); iter++)
    {
        if (this->pointInsideConvexHull2d(convexHull, point_cloud.points[iter]))
            return 1;
    }
    return 0;
}

//low computation comparison based on centroid
int CToolBoxROS::overlapConvexHull2d(StructPlanarSurface &surface1, StructPlanarSurface &surface2)
{

    StructPlanarSurface surfaceLarger;
    StructPlanarSurface surfaceSmaller;
    //find the largest surface
    if (surface1.area > surface2.area)
    {
        surfaceLarger = surface1;
        surfaceSmaller = surface2;
    }
    else
    {
        surfaceLarger = surface2;
        surfaceSmaller = surface1;
    }

    return this->pointInsideConvexHull2d(surfaceLarger.convexHull, surfaceSmaller.centroid);

}

pcl::PointXYZRGBNormal CToolBoxROS::getNearestNeighborPlane(StructPlanarSurface &plane, pcl::PointXYZRGBNormal &queryPoint)
{
    //  std::cout<<"ENTER";
    int k = 1;
    std::vector<int> k_indicies;
    std::vector<float> k_distances;

    k_distances.resize(k);
    k_indicies.resize(k);
    plane.tree->nearestKSearch(queryPoint, k, k_indicies, k_distances);

    return plane.pointCloud.points[k_indicies[0]];
}

void CToolBoxROS::getNearestKPoints(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, pcl::PointXYZRGB &queryPoint, int k, std::vector<int> &k_indicies,
                                    std::vector<float> &k_distances)
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> >();
    tree->setInputCloud(pointCloud.makeShared());

    k_distances.resize(k);
    k_indicies.resize(k);
    tree->nearestKSearch(queryPoint, k, k_indicies, k_distances);
}

void CToolBoxROS::getNearestKPoints(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointXYZ &queryPoint, int k, std::vector<int> &k_indicies,
                                    std::vector<float> &k_distances)
{

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> >();
    tree->setInputCloud(pointCloud.makeShared());

    k_distances.resize(k);
    k_indicies.resize(k);
    tree->nearestKSearch(queryPoint, k, k_indicies, k_distances);
}

void CToolBoxROS::getNearestKPoints(pcl::KdTreeFLANN<pcl::PointXYZ> &tree, pcl::PointXYZ &queryPoint, int k, std::vector<int> &k_indicies,
                                    std::vector<float> &k_distances)
{
    k_distances.resize(k);
    k_indicies.resize(k);
    tree.nearestKSearch(queryPoint, k, k_indicies, k_distances);
}

//computationally costly
int CToolBoxROS::overlapConvexHull2d2(StructPlanarSurface &surface1, StructPlanarSurface &surface2)
{
    StructPlanarSurface surfaceLarger;
    StructPlanarSurface surfaceSmaller;
    //find the largest surface
    if (surface1.area > surface2.area)
    {
        surfaceLarger = surface1;
        surfaceSmaller = surface2;
    }
    else
    {
        surfaceLarger = surface2;
        surfaceSmaller = surface1;
    }

    //the largest surface is used to be checked whether the small one is intersecting the larger one
    int k = 1;
    std::vector<int> k_indicies;
    std::vector<float> k_distances;
    for (unsigned int iter = 0; iter < surfaceLarger.convexHull.points.size(); iter++)
    {

        k_distances.clear();
        k_indicies.clear();

        k_distances.resize(k);
        k_indicies.resize(k);
        //find from smaller surfaces the nearstest point to a convexHull point of the larger surface.
        //the nearest point is used to be checked whether the point is inside the large surface convex hull
        //if so there is a overlap! -> return 1
        surfaceSmaller.tree->nearestKSearch(surfaceLarger.convexHull.points[iter], k, k_indicies, k_distances);

        if (this->pointInsideConvexHull2d(surfaceLarger.convexHull, surfaceSmaller.pointCloud.points[k_indicies[0]]))
        {
            return 1;
        }
    }
    return 0;
}

//non-self-intersecting polygon
float CToolBoxROS::areaConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &hull)
{

    unsigned int sizeHull = hull.points.size();
    float area = 0;

    for (unsigned int iter_point = 0; iter_point < sizeHull - 2; ++iter_point)
    {
        //std::cout<<"HULL points "<< hull.points[iter_point]<<std::endl;
        area += (hull.points[iter_point].x * hull.points[iter_point + 1].y - hull.points[iter_point + 1].x * hull.points[iter_point].y);
    }
    if (sizeHull > 0)
    {
        area += (hull.points[sizeHull - 1].x * hull.points[0].y - hull.points[0].x * hull.points[sizeHull - 1].y);
    }

    return fabs(0.5f * area);

}
pcl::PointXYZ CToolBoxROS::pointCloudCentroid(pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{

    unsigned int sizePointCloud = point_cloud.points.size();

    pcl::PointXYZ centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
    {
        if (!isnan(point_cloud.points[iter_point].x))
        {
            centroid.x += (point_cloud.points[iter_point].x);
            centroid.y += (point_cloud.points[iter_point].y);
            centroid.z += (point_cloud.points[iter_point].z);
        }
    }

    centroid.x /= sizePointCloud;
    centroid.y /= sizePointCloud;
    centroid.z /= sizePointCloud;

    return centroid;
}

pcl::PointXYZRGB CToolBoxROS::pointCloudCentroid(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud)
{
    unsigned int sizePointCloud = point_cloud.points.size();

    pcl::PointXYZRGB centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
    {
        if (!isnan(point_cloud.points[iter_point].x))
        {
            centroid.x += (point_cloud.points[iter_point].x);
            centroid.y += (point_cloud.points[iter_point].y);
            centroid.z += (point_cloud.points[iter_point].z);
        }
    }

    centroid.x /= sizePointCloud;
    centroid.y /= sizePointCloud;
    centroid.z /= sizePointCloud;

    return centroid;
}

pcl::PointXYZRGB CToolBoxROS::pointCloudCentroid2(pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{

    unsigned int sizePointCloud = point_cloud.points.size();

    pcl::PointXYZRGB centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
    {
        centroid.x += (point_cloud.points[iter_point].x);
        centroid.y += (point_cloud.points[iter_point].y);
        centroid.z += (point_cloud.points[iter_point].z);
    }

    centroid.x /= sizePointCloud;
    centroid.y /= sizePointCloud;
    centroid.z /= sizePointCloud;

    return centroid;
}

pcl::PointXYZ CToolBoxROS::pointCloudBoundingBoxCentroid(pcl::PointCloud<pcl::PointXYZ> &pointCloud, pcl::PointCloud<pcl::PointXYZ> &boundingBox)
{
    double pointMinX = std::numeric_limits<double>::infinity();
    double pointMinY = std::numeric_limits<double>::infinity();
    double pointMinZ = std::numeric_limits<double>::infinity();

    double pointMaxX = std::numeric_limits<double>::epsilon();
    double pointMaxY = std::numeric_limits<double>::epsilon();
    double pointMaxZ = std::numeric_limits<double>::epsilon();

    pcl::PointXYZ ptMinX;
    pcl::PointXYZ ptMinY;
    pcl::PointXYZ ptMinZ;

    pcl::PointXYZ ptMaxX;
    pcl::PointXYZ ptMaxY;
    pcl::PointXYZ ptMaxZ;

    //  pcl::PointCloud<pcl::PointXYZ> boundingBox;

    for (unsigned int iterPoint = 0; iterPoint < pointCloud.points.size(); ++iterPoint)
    {
        if (pointCloud.points[iterPoint].x > pointMaxX)
        {
            pointMaxX = pointCloud.points[iterPoint].x;
            ptMaxX = pointCloud.points[iterPoint];
        }
        if (pointCloud.points[iterPoint].y > pointMaxY)
        {
            pointMaxY = pointCloud.points[iterPoint].y;
            ptMaxY = pointCloud.points[iterPoint];
        }
        if (pointCloud.points[iterPoint].z > pointMaxZ)
        {
            pointMaxZ = pointCloud.points[iterPoint].z;
            ptMaxZ = pointCloud.points[iterPoint];
        }

        if (pointCloud.points[iterPoint].x < pointMinX)
        {
            pointMinX = pointCloud.points[iterPoint].x;
            ptMinX = pointCloud.points[iterPoint];
        }
        if (pointCloud.points[iterPoint].y < pointMinY)
        {
            pointMinY = pointCloud.points[iterPoint].y;
            ptMinY = pointCloud.points[iterPoint];
        }
        if (pointCloud.points[iterPoint].z < pointMinZ)
        {
            pointMinZ = pointCloud.points[iterPoint].z;
            ptMinZ = pointCloud.points[iterPoint];
        }
    }

    boundingBox.points.push_back(pcl::PointXYZ(pointMinX, pointMaxY, pointMinZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMaxX, pointMinY, pointMinZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMinX, pointMinY, pointMinZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMaxX, pointMaxY, pointMinZ));

    boundingBox.points.push_back(pcl::PointXYZ(pointMinX, pointMaxY, pointMaxZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMaxX, pointMinY, pointMaxZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMinX, pointMinY, pointMaxZ));
    boundingBox.points.push_back(pcl::PointXYZ(pointMaxX, pointMaxY, pointMaxZ));

    //boundingBox.points.push_back();

    boundingBox.height = 1;
    boundingBox.width = 8;

    return this->pointCloudCentroid(boundingBox);
}

pcl::PointXYZ CToolBoxROS::pointCloudCentroid(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud)
{

    unsigned int sizePointCloud = point_cloud.points.size();

    pcl::PointXYZ centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
    {
        centroid.x += (point_cloud.points[iter_point].x);
        centroid.y += (point_cloud.points[iter_point].y);
        centroid.z += (point_cloud.points[iter_point].z);
    }

    centroid.x /= sizePointCloud;
    centroid.y /= sizePointCloud;
    centroid.z /= sizePointCloud;

    return centroid;
}

//non-self-intersecting polygon
pcl::PointXYZRGBNormal CToolBoxROS::centroidHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, float area)
{
    unsigned int sizePointCloud = point_cloud.points.size();

    pcl::PointXYZRGBNormal centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
    {
        centroid.z += point_cloud.points[iter_point].z;
    }
    centroid.z /= sizePointCloud;

    for (unsigned int iter_point = 0; iter_point < sizePointCloud - 1; iter_point++)
    {
        centroid.x += (point_cloud.points[iter_point].x + point_cloud.points[iter_point + 1].x)
                      * (point_cloud.points[iter_point].x * point_cloud.points[iter_point + 1].y
                         - point_cloud.points[iter_point + 1].x * point_cloud.points[iter_point].y);
        centroid.y += (point_cloud.points[iter_point].y + point_cloud.points[iter_point + 1].y)
                      * (point_cloud.points[iter_point].x * point_cloud.points[iter_point + 1].y
                         - point_cloud.points[iter_point + 1].x * point_cloud.points[iter_point].y);
    }

    centroid.x = centroid.x / (6 * area);
    centroid.y = centroid.y / (6 * area);

    return centroid;
}

float CToolBoxROS::avgValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float sumValue = 0;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            sumValue += point_cloud.points[iter_point].x;
        }

        return sumValue / sizePointCloud;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            sumValue += point_cloud.points[iter_point].y;
        }

        return sumValue / sizePointCloud;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            sumValue += point_cloud.points[iter_point].z;
        }

        return sumValue / sizePointCloud;
    }

    return 0;
}

float CToolBoxROS::minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float minValue = 9999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x < minValue)
                minValue = point_cloud.points[iter_point].x;
        }

        return minValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y < minValue)
                minValue = point_cloud.points[iter_point].y;
        }

        return minValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z < minValue)
                minValue = point_cloud.points[iter_point].z;
        }

        return minValue;
    }

    return 0;
}

float CToolBoxROS::minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float minValue = 9999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x < minValue)
                minValue = point_cloud.points[iter_point].x;
        }

        return minValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y < minValue)
                minValue = point_cloud.points[iter_point].y;
        }

        return minValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z < minValue)
                minValue = point_cloud.points[iter_point].z;
        }

        return minValue;
    }

    return 0;
}

float CToolBoxROS::minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis, pcl::PointXYZ &min_point)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float minValue = 9999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x < minValue)
            {
                minValue = point_cloud.points[iter_point].x;
                min_point = point_cloud.points[iter_point];
            }
        }

        return minValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y < minValue)
            {
                minValue = point_cloud.points[iter_point].y;
                min_point = point_cloud.points[iter_point];
            }
        }

        return minValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z < minValue)
            {
                minValue = point_cloud.points[iter_point].z;
                min_point = point_cloud.points[iter_point];
            }
        }

        return minValue;
    }

    return 0;
}

float CToolBoxROS::maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZRGBNormal> &point_cloud, int axis)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float maxValue = -999999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x > maxValue)
                maxValue = point_cloud.points[iter_point].x;
        }

        return maxValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y > maxValue)
                maxValue = point_cloud.points[iter_point].y;
        }

        return maxValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z > maxValue)
                maxValue = point_cloud.points[iter_point].z;
        }

        return maxValue;
    }

    return 0;
}
float CToolBoxROS::maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis, pcl::PointXYZ &max_point)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float maxValue = -999999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x > maxValue)
            {
                maxValue = point_cloud.points[iter_point].x;
                max_point = point_cloud.points[iter_point];
            }
        }

        return maxValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y > maxValue)
            {
                maxValue = point_cloud.points[iter_point].y;
                max_point = point_cloud.points[iter_point];
            }
        }

        return maxValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z > maxValue)
            {
                maxValue = point_cloud.points[iter_point].z;
                max_point = point_cloud.points[iter_point];
            }
        }

        return maxValue;
    }

    return 0;
}

float CToolBoxROS::maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> &point_cloud, int axis)
{
    unsigned int sizePointCloud = point_cloud.points.size();
    float maxValue = -999999;
    if (axis == 0)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].x > maxValue)
                maxValue = point_cloud.points[iter_point].x;
        }

        return maxValue;
    }

    if (axis == 1)
    {
        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].y > maxValue)
                maxValue = point_cloud.points[iter_point].y;
        }

        return maxValue;
    }

    if (axis == 2)
    {

        for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++)
        {
            if (point_cloud.points[iter_point].z > maxValue)
                maxValue = point_cloud.points[iter_point].z;
        }

        return maxValue;
    }

    return 0;
}

// low computational cost required: we take the smaller plane then we search for the closest point from the that plane to the other(larger plane)
bool CToolBoxROS::distanceBetweenPlane2d(StructPlanarSurface &surface1, StructPlanarSurface &surface2, float distanceThreshold)
{
    //ROS_INFO("[ToolBox]DistanceBetweenPlane2d started...");
    ros::Time start, finish;
    start = ros::Time::now();
    //float distance;
    float minDistance = 5;  //since 5m is max range of kinect, however ToDo limit.h
    StructPlanarSurface surfaceLarger;
    StructPlanarSurface surfaceSmaller;
    //find the largest surface
    if (surface1.pointCloud.size() > surface2.pointCloud.size())
    {
        surfaceLarger = surface1;
        surfaceSmaller = surface2;
    }
    else
    {
        surfaceLarger = surface2;
        surfaceSmaller = surface1;
    }

    int k = 1;
    std::vector<int> k_indicies;
    std::vector<float> k_distances;
    for (unsigned int iter = 0; iter < surfaceSmaller.pointCloud.points.size(); iter++)
    {

        k_distances.clear();
        k_indicies.clear();

        k_distances.resize(k);
        k_indicies.resize(k);

        surfaceLarger.tree->nearestKSearch(surfaceSmaller.pointCloud.points[iter], k, k_indicies, k_distances);

        if (k_distances[0] <= minDistance)
        {
            minDistance = k_distances[0];  //0 since we are looking for the nearest one!
        }
    }
    finish = ros::Time::now();
    //  ROS_INFO("...distanceBetweenPlane2d(%f) took %lf", minDistance, (finish.toSec() - start.toSec() ));
    return (minDistance < distanceThreshold);
}

//checks whether an "object" is just a planar surface which is not that interesting, if we a just interested in objects
bool CToolBoxROS::isObjectPlane(StructPlanarSurface &surface, pcl::PointCloud<pcl::PointXYZRGBNormal> &object, float objectHeightThreshold,
                                float objectPlaneHeightDifferenceThreshold)
{

    float zMinObject = 5.0f;  //min height of the object
    float zMaxObject = 0.0f;

    for (unsigned int iter_point = 0; iter_point < object.points.size(); iter_point++)
    {
        if (object.points[iter_point].z < zMinObject)
            zMinObject = object.points[iter_point].z;

        if (object.points[iter_point].z > zMaxObject)
            zMaxObject = object.points[iter_point].z;
    }

    ROS_WARN("object height %f/%f ; diff %f/%f  ", fabs(zMaxObject - zMinObject), objectHeightThreshold, (zMinObject - surface.plane_height),
             objectPlaneHeightDifferenceThreshold);

    //Todo here we just look at "flying" objects, this is true for many cases, but what about flying labels of transparent bottles? They are still objects. in that case they are neglected!!
    //if(fabs(zMaxObject-zMinObject) > objectHeightThreshold)
    //  return false;

    if ((zMinObject - surface.plane_height) < objectPlaneHeightDifferenceThreshold || (zMinObject - surface.plane_height) < 0.0)
        return false;

    return true;
}

pcl::PointCloud<pcl::PointXYZ> CToolBoxROS::normalizePointCloud(pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{

    pcl::PointCloud < pcl::PointXYZ > point_cloudNorm;
    point_cloudNorm.points.resize(point_cloud.points.size());

    for (unsigned int iterPoints = 0; iterPoints < point_cloud.points.size(); iterPoints++)
    {
        double a = sqrt(
                       (point_cloud.points[iterPoints].x * point_cloud.points[iterPoints].x) + (point_cloud.points[iterPoints].y * point_cloud.points[iterPoints].y)
                       + (point_cloud.points[iterPoints].z * point_cloud.points[iterPoints].z));

        point_cloudNorm.points[iterPoints].x = point_cloud.points[iterPoints].x / a;
        point_cloudNorm.points[iterPoints].y = point_cloud.points[iterPoints].y / a;
        point_cloudNorm.points[iterPoints].z = point_cloud.points[iterPoints].z / a;

    }

    return point_cloudNorm;
}

std::vector<double> CToolBoxROS::normalizePoint3D(std::vector<double> &point)
{
    std::vector<double> normalizedpoint;

    normalizedpoint.resize(point.size());
    double a;
    double sum = 0;

    for (unsigned int i = 0; i < point.size(); i++)
    {
        sum += (point[i] * point[i]);
    }

    a = sqrt(sum);

    for (unsigned int i = 0; i < point.size(); i++)
    {
        normalizedpoint[i] = (point[i] / a);
    }

    return normalizedpoint;
}

float CToolBoxROS::angleBetweenPoints(std::vector<double> &point1, std::vector<double> &point2)
{
    float angle = 0;
    if (point1.size() != point2.size())
        return angle;

    std::vector<double> normalizedpoint1;
    std::vector<double> normalizedpoint2;

    normalizedpoint1 = this->normalizePoint3D(point1);
    normalizedpoint2 = this->normalizePoint3D(point2);

    double sum = 0;
    for (unsigned int i = 0; i < normalizedpoint1.size(); i++)
    {
        sum += normalizedpoint1[i] + normalizedpoint2[i];
    }

    angle = acos(sum);

    return angle;
}

void CToolBoxROS::get3DPointsWithinHull(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLFull,
                                        const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLHull, const double dMinHeight, const double dMaxHeight,
                                        pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLSegmentOutput)
{
    pcl::PointIndices object_indices;
    pcl::ExtractPolygonalPrismData < pcl::PointXYZRGBNormal > hull_limiter;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCLFullPtr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    *cloudPCLFullPtr = cloudPCLFull;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCLHullPtr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);  // HERE SOMETIMES AND ERROR OCCURS !
    *cloudPCLHullPtr = cloudPCLHull;

    hull_limiter.setInputCloud(cloudPCLFullPtr);
    hull_limiter.setInputPlanarHull(cloudPCLHullPtr);
    hull_limiter.setHeightLimits(dMinHeight, dMaxHeight);
    hull_limiter.segment(object_indices);

    pcl::ExtractIndices < pcl::PointXYZRGBNormal > extract;
    extract.setInputCloud(cloudPCLFullPtr);
    extract.setIndices(boost::make_shared < pcl::PointIndices > (object_indices));
    extract.setNegative(false);
    extract.filter(cloudPCLSegmentOutput);
}

std::vector<std::vector<double> > CToolBoxROS::convertPointCloudToStdVec(pcl::PointCloud<pcl::PointXYZ> &pointCloud)
{
    std::vector<std::vector<double> > convCloud;

    assert(pointCloud.points.size() > 0);

    for (unsigned int p = 0; p < pointCloud.points.size(); p++)
    {
        std::vector<double> convPoint;
        convPoint.push_back(pointCloud.points[p].x);
        convPoint.push_back(pointCloud.points[p].y);
        convPoint.push_back(pointCloud.points[p].z);
        convCloud.push_back(convPoint);
    }

    assert(convCloud.size() > 0);

    return convCloud;
}

double CToolBoxROS::randNumber(double low, double high)
{
    double temp;

    /* swap low & high around if the user makes no sense */
    if (low > high)
    {
        temp = low;
        low = high;
        high = temp;
    }

    /* calculate the random number & return it */
    temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0)) * (high - low) + low;
    return temp;
}

void CToolBoxROS::extractHighestLowestPoint(pcl::PointCloud<pcl::PointXYZRGBNormal> &pointCloud, pcl::PointXYZ &highestPoint, pcl::PointXYZ &lowestPoint)
{

    highestPoint.x = std::numeric_limits<double>::epsilon();
    highestPoint.y = std::numeric_limits<double>::epsilon();
    highestPoint.z = std::numeric_limits<double>::epsilon();

    lowestPoint.x = std::numeric_limits<double>::infinity();
    lowestPoint.y = std::numeric_limits<double>::infinity();
    lowestPoint.z = std::numeric_limits<double>::infinity();
    ;
    for (unsigned int iterPoint = 0; iterPoint < pointCloud.points.size(); ++iterPoint)
    {
        if (pointCloud.points[iterPoint].z > highestPoint.z)
        {
            highestPoint.x = pointCloud.points[iterPoint].x;
            highestPoint.y = pointCloud.points[iterPoint].y;
            highestPoint.z = pointCloud.points[iterPoint].z;
        }

        if (pointCloud.points[iterPoint].z < lowestPoint.z)
        {
            lowestPoint.x = pointCloud.points[iterPoint].x;
            lowestPoint.y = pointCloud.points[iterPoint].y;
            lowestPoint.z = pointCloud.points[iterPoint].z;
        }

    }
}

