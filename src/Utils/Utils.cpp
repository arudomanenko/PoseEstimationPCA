#include "Utils.h"
#include <pcl/common/centroid.h>

static pcl::ExtractIndices<pcl::PointXYZ> extract;

pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::SelectByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                  pcl::PointIndices::Ptr &indexes, bool isInvert)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr desiredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    extract.setInputCloud(cloud);
    extract.setIndices(indexes);
    extract.setNegative(isInvert);
    extract.filter(*desiredCloud);

    return desiredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::SelectByIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                     pcl::PointIndices &indexes, bool isInvert)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr desiredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(const auto &ind : indexes.indices)
    {
        desiredCloud->push_back((*cloud)[ind]);
    }

    return desiredCloud;
}
