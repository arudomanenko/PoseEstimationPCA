#ifndef WOODPLANKDETECTOR_FRAMECONVERTER_H
#define WOODPLANKDETECTOR_FRAMECONVERTER_H

#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
    * @class FrameConverter
    * Class for converting frames to point cloud
*/
class FrameConverter {
public:
    /**
    * Constructor for FrameConverter
    */
    explicit FrameConverter() : cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
    pcl::PointCloud<pcl::PointXYZ>::Ptr ConvertToPointCloud(const rs2::points &points);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif //WOODPLANKDETECTOR_FRAMECONVERTER_H
