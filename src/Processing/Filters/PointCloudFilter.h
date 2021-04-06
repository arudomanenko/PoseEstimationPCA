#ifndef WOODPLANKDETECTOR_POINTCLOUDFILTER_H
#define WOODPLANKDETECTOR_POINTCLOUDFILTER_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>

/**
    * @struct OutlierRemovalOptions
    * Create setting for outlier removal algoritm
*/
struct OutlierRemovalOptions
{
    float threshold;
    int minKnearest;
};

/**
    * @class PointCloudFilter
    * Class for filtering point cloud
*/
class PointCloudFilter
{
public:
    /**
    * Constructor for PointCloudFilter
    * @param outliersRemovalOptions - options for outlier removal algoritm
    * @param leafSize - leaf size for voxel grid
    */
    explicit PointCloudFilter(OutlierRemovalOptions outliersRemovalOptions, std::vector<float> &leafSize)
    : filtered(new pcl::PointCloud<pcl::PointXYZ>) {

        outlierRemover.setMeanK (outliersRemovalOptions.minKnearest);
        outlierRemover.setStddevMulThresh (outliersRemovalOptions.threshold);

        grid.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
    }

    /**
    * Down sample input cloud
    * @param cloud - pointer to input cloud
    */
    void DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
    * Remove outliers from input cloud
    * @param cloud - pointer to input cloud
    */
    void RemoveOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
private:
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierRemover;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
};

#endif //WOODPLANKDETECTOR_POINTCLOUDFILTER_H
