#include "Visualizer.h"

void Visualizer::UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (!viewer->updatePointCloud(cloud, cloudId))
    {
        viewer->addPointCloud(cloud, cloudId);
        viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudId);
    }
}

void Visualizer::AddRotations(std::vector<Eigen::Affine3f> &transformations) {
    for(auto &t : transformations) {
        viewer->addCoordinateSystem(0.1f, t);
    }
}

void Visualizer::SpinOnce()
{
    viewer->spinOnce(100);
}
