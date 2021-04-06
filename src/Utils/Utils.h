#ifndef WOODPLANKDETECTOR_UTILS_H
#define WOODPLANKDETECTOR_UTILS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <tuple>

/**
    * @class Utils
    * Class for useful methods
*/
class Utils {
public:
    /**
        * Convert matrix of transformation to affine transformation
        * @param cloud - pointer to input cloud
        * @param indices - pointer to indices of desired point cloud
        * @param isInvert - select desired indices of exclude it
        * @return pointer to desired point cloud
    */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr SelectByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::PointIndices::Ptr &indices, bool isInvert);

    /**
        * Convert matrix of transformation to affine transformation
        * @param cloud - pointer to input cloud
        * @param indices - indices of desired point cloud
        * @param isInvert - select desired indices of exclude it
        * @return pointer to desired point cloud
    */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr SelectByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::PointIndices &indices, bool isInvert);
};

#endif //WOODPLANKDETECTOR_UTILS_H
