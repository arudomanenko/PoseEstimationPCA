#ifndef WOODPLANKDETECTOR_VISUALIZER_H
#define WOODPLANKDETECTOR_VISUALIZER_H

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <string>
#include <utility>
#include <vector>


/**
    * @class Visualizer
    * Class for point cloud visualization
*/
class Visualizer
{
public:
    /**
        * Constructor for Visualizer
        * @param cloudId - identification string of cloud
        * @param pointsSize - size of points
    */
    explicit Visualizer(std::string cloudId, int pointsSize)
    : cloudId(std::move(cloudId)), pointSize(pointsSize),
    viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"))
    {
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.5);
        viewer->initCameraParameters();
    }

    /**
        * Update current point cloud or (if absent) add it
        * @param cloud - pointer to cloud
    */
    void UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /**
        * Adding system coordinates of cloud for orientations visualization
        * @param transformations - vector of affine transformations of clusters
    */
    void AddRotations(std::vector<Eigen::Affine3f> &transformations);

    /**
        * Make one tick of pcl visualizer
    */
    void SpinOnce();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::string cloudId;
    int pointSize;
};


#endif //WOODPLANKDETECTOR_VISUALIZER_H
