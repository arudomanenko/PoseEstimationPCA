#include "FloorRemover.h"
#include "../Utils/Utils.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr FloorRemover::RemoveFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);

    return Utils::SelectByIndex(cloud, inliers, true);
}