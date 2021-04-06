#include "PoseEstimator.h"
#include "../Utils/Utils.h"
#include <Eigen/Eigenvalues>
#include <vector>

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PoseEstimator::GetClusters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

    ComputeClusters(cloud);
    int size = clustersIndices.size();

    for(int i = 0; i < size; i++) {
        clusters.push_back((Utils::SelectByIndex(cloud, clustersIndices[i], false)));
    }

    return clusters;
}

std::vector<Eigen::Affine3f> PoseEstimator::DefineTransformations()
{
    for(auto &cluster : clusters)
    {
        auto eigens = ComputeRotationAndTranslation(cluster);

        Eigen::Matrix3f eigenVectors = std::get<0>(eigens);
        Eigen::Vector3f eigenValues = std::get<1>(eigens);
        Eigen::Vector4f centroid = std::get<2>(eigens);

        for(int i = 0; i < eigenVectors.cols(); i++)
        {
            eigenVectors.col(i).normalize();
        }

        Eigen::Affine3f trans = CreateAffineTransformation(cluster, eigenVectors, centroid);
        transformations.push_back(trans);
    }

    return transformations;
}

void PoseEstimator::ComputeClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    clustering.setInputCloud (cloud);
    clustering.extract(clustersIndices);
}

Eigen::Affine3f PoseEstimator::CreateAffineTransformation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Matrix3f &rotation, Eigen::Vector4f &centroid) {

    Eigen::Matrix4f trans;
    trans.col(3) = centroid;

    for (int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            trans(i, j) = rotation(j, i);
        }
    }

    Eigen::Vector4f scale;
    scale << 0, 0, 0, 1;
    trans.row(3) = scale;

    Eigen::Affine3f convertedTrans;
    convertedTrans.matrix() = trans;

    return convertedTrans;
}

std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector4f> PoseEstimator::ComputeRotationAndTranslation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    Eigen::EigenSolver<Eigen::Matrix3f> eigenSolver;
    Eigen::Vector4f centroid;
    Eigen::Matrix3f covarianceMatrix;

    pcl::compute3DCentroid(*cloud, centroid);
    pcl::computeCovarianceMatrix(*cloud, centroid, covarianceMatrix);

    eigenSolver.compute(covarianceMatrix);

    Eigen::Vector3f eigenValues = eigenSolver.eigenvalues().real();
    Eigen::Matrix3f eigenVectors = eigenSolver.eigenvectors().real();

    return {eigenVectors, eigenValues, centroid};
}
