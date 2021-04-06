#include "FrameConverter.h"
#include <vector>

pcl::PointCloud<pcl::PointXYZ>::Ptr FrameConverter::ConvertToPointCloud(const rs2::points &points)
{
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto vertices = points.get_vertices();

    for (int i = 0; i < points.size(); ++i)
    {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;
    }

    return cloud;
}