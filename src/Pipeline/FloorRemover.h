#ifndef WOODPLANKDETECTOR_PLANESREMOVER_H
#define WOODPLANKDETECTOR_PLANESREMOVER_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/**
    * @struct RansacOptions
    * Create setting for RANSAC algoritm
*/
struct RansacOptions
{
    float distanceThreshold;
    int maxIterations;
};

/**
    * @class FloorRemover
    * Class for segmenting floor plane and removing it
*/
class FloorRemover
{
public:
    /**
     * Constructor for FloorRemover
     * @param ransacOptions - options for RANSAC algoritm
     */
    explicit FloorRemover(RansacOptions ransacOptions)
    : coefficients(new pcl::ModelCoefficients), inliers(new pcl::PointIndices)
    {
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations(ransacOptions.maxIterations);
        seg.setDistanceThreshold(ransacOptions.distanceThreshold);
    }

    /**
     * Segmenting floor
     * @param cloud - input cloud
     * @return cloud without floor plane
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr RemoveFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

private:
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
};

#endif //WOODPLANKDETECTOR_PLANESREMOVER_H
