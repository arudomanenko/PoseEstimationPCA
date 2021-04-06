#ifndef WOODPLANKDETECTOR_REALSENSECAMERA_H
#define WOODPLANKDETECTOR_REALSENSECAMERA_H

#include "../Processing/FrameConverter.h"
#include "../Processing/Filters/DepthFilter.h"
#include <librealsense2/rs.hpp>
#include <iostream>

/**
    * @class RealsenseCamera
    * Class for grabbing frame from RealsenseCamera
*/
class RealsenseCamera {
public:
    int warmUpIterations;

    /**
     * Constructor for RealsenseCamera class
     * @param warmUpIterations - number of non-catching frames for warm up camera
     */
    explicit RealsenseCamera(int warmUpIterations = 100) :
        pipe(rs2::pipeline()), warmUpIterations(warmUpIterations), isRunning(false),
        convertedPointCloud(new pcl::PointCloud<pcl::PointXYZ>) {}

    /**
     * Start camera capturing after warming up
     */
    void Run();

    /**
     * Stop camera capturing
     */
    void Stop();

    /**
     * Get converted to point cloud frames
     * @return pointer to frame converted to point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetData();

private:
    FrameConverter frameConverter;
    DepthFilter postprocessor;

    rs2::pipeline pipe;
    rs2::frameset frameset;
    rs2::pointcloud pointCloud;
    rs2::points points;

    pcl::PointCloud<pcl::PointXYZ>::Ptr convertedPointCloud;

    bool isRunning;

    /**
     * Convert frame to point cloud
     */
    void GetFramePointCloud();

    /**
     * Run camera and skip grabbing N frames
     */
    void WarmUp() const;
};


#endif //WOODPLANKDETECTOR_REALSENSECAMERA_H
