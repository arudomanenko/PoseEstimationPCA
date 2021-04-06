#ifndef WOODPLANKDETECTOR_OBSTACLEDETECTOR_H
#define WOODPLANKDETECTOR_OBSTACLEDETECTOR_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

/**
    * @struct ClusteringOptions
    * Create setting for clustering algoritm
*/
struct ClusteringOptions
{
    float tolerance;
    int minClusterSize;
    int maxClusterSize;
};

/**
    * @class PoseEstimator
    * Class for clustering point cloud and for pose estimation for every cluster
*/
class PoseEstimator {
public:
    /**
     * Constructor for PoseEstimator
     * @param clusteringOptions - options for clustering algoritm
     */
    explicit PoseEstimator(ClusteringOptions clusteringOptions)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        clustering.setClusterTolerance(clusteringOptions.tolerance);
        clustering.setMinClusterSize(clusteringOptions.minClusterSize);
        clustering.setMaxClusterSize(clusteringOptions.maxClusterSize);
        clustering.setSearchMethod(tree);
    }

    /**
     * Compute clusters and returns it
     * @param cloud - pointer to input cloud for clustering
     * @return vector of pointers to  pointers to clusters
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> GetClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * Computing rotation and translation
     * @param clusteringOptions - options for clustering algoritm
     * @return rotation and translation as affine trasformation
     */
    std::vector<Eigen::Affine3f> DefineTransformations();

private:
    std::vector<pcl::PointIndices> clustersIndices;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<Eigen::Affine3f> transformations;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;

    /**
     * Compute clusters
     * @param cloud - pointer to input cloud for clustering
     */
    void ComputeClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
     * Convert matrix of transformation to affine transformation
     * @param cloud - pointer to input cloud
     * @param rotation - rotation matrix
     * @param centriod - position of center of point cloud
     * @return affine transformation
     */
    static Eigen::Affine3f CreateAffineTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                      Eigen::Matrix3f &rotation, Eigen::Vector4f &centroid);

    /**
     * Compute rotation and translation of cloud
     * @param cloud - pointer to input cloud
     * @return rotation and translation of cloud
     */
    static std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector4f> ComputeRotationAndTranslation(
            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
};


#endif //WOODPLANKDETECTOR_OBSTACLEDETECTOR_H
