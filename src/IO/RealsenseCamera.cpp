#include "RealsenseCamera.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void RealsenseCamera::Run(){
    if (isRunning)
    {
        return;
    }

    pipe.start();
    isRunning = true;

    WarmUp();
}

void RealsenseCamera::Stop() {
    pipe.stop();
    isRunning = false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RealsenseCamera::GetData()
{
    GetFramePointCloud();
    return convertedPointCloud;
}

void RealsenseCamera::GetFramePointCloud()
{
    frameset = pipe.wait_for_frames();

    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    postprocessor.Apply(depth);

    points = pointCloud.calculate(depth);

    convertedPointCloud = frameConverter.ConvertToPointCloud(points);
}

void RealsenseCamera::WarmUp() const
{
    rs2::frameset tmp;
    for (int i = 0; i < warmUpIterations; i++)
    {
        tmp = pipe.wait_for_frames();
    }
}
