#include <mcr_pose_estimation/pca_pose_estimator_ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>

PCAPoseEstimatorRos::PCAPoseEstimatorRos() : pointcloud_msg_received_(false)
{
    ros::NodeHandle nh("~");
    sub_pointcloud_ = nh.subscribe("input/pointcloud", 1, &PCAPoseEstimatorRos::pointcloudCallback, this);
    pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("output/pose", 1);
}

PCAPoseEstimatorRos::~PCAPoseEstimatorRos()
{
}

void PCAPoseEstimatorRos::update()
{
    if (pointcloud_msg_received_)
    {
        estimatePose();
        pointcloud_msg_received_ = false;
    }
}

void PCAPoseEstimatorRos::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    ROS_INFO("[pca_pose_estimator] Received pointcloud message");
    pointcloud_msg_ = msg;
    pointcloud_msg_received_ = true;
}

// Based on code by Nicola Fioraio here:
// http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
void PCAPoseEstimatorRos::estimatePose()
{
    pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*pointcloud_msg_, *pcl_input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*pcl_input_cloud, *xyz_input_cloud);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*xyz_input_cloud, centroid);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*xyz_input_cloud, centroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

    // swap largest and second largest eigenvector so that y-axis aligns with largest eigenvector and z with the second largest
    eigen_vectors.col(1).swap(eigen_vectors.col(2));
    eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

    Eigen::Matrix4f eigen_vector_transform(Eigen::Matrix4f::Identity());
    eigen_vector_transform.block<3, 3>(0, 0) = eigen_vectors.transpose();
    eigen_vector_transform.block<3, 1>(0, 3) = -(eigen_vector_transform.block<3, 3>(0, 0) * centroid.head<3>());

    // transform cloud to eigenvector space
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*xyz_input_cloud, transformed_cloud, eigen_vector_transform);

    // find mean diagonal
    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(transformed_cloud, min_point, max_point);
    Eigen::Vector3f mean_diag = (max_point.getVector3fMap() + min_point.getVector3fMap()) / 2.0;

    // orientation and position of bounding box of cloud
    Eigen::Quaternionf orientation(eigen_vectors);
    Eigen::Vector3f position = eigen_vectors * mean_diag + centroid.head<3>();

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = position(0);
    pose_stamped.pose.position.y = position(1);
    pose_stamped.pose.position.z = position(2);
    pose_stamped.pose.orientation.w = orientation.w();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();
    pose_stamped.header = pointcloud_msg_->header;

    pub_pose_.publish(pose_stamped);
}
