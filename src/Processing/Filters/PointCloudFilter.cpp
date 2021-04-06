#include "PointCloudFilter.h"

void PointCloudFilter::DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    grid.setInputCloud(cloud);
    grid.filter(*filtered);

    pcl::copyPointCloud(*filtered, *cloud);
}

void PointCloudFilter::RemoveOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    outlierRemover.setInputCloud(cloud);
    outlierRemover.filter (*filtered);

    pcl::copyPointCloud(*filtered, *cloud);
}