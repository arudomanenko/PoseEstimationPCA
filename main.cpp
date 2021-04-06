#include "src/IO/RealsenseCamera.h"
#include "src/VisualizingTools/Visualizer.h"
#include "src/Processing/Filters/PointCloudFilter.h"
#include "src/Pipeline/FloorRemover.h"
#include "src/Pipeline/PoseEstimator.h"

#ifndef RUN_WITH_CAMERA
#include <pcl/io/pcd_io.h>
#endif


int main(int argc, char * argv[]) {
    RansacOptions ransacOptions = {0.02, 5000};
    OutlierRemovalOptions removalOptions = {0.05, 50};
    ClusteringOptions clusteringOptions = {0.02, 100, 25000};

    float xyzLeafSize = 0.01f;
    std::vector<float> leafSize = {xyzLeafSize, xyzLeafSize, xyzLeafSize};

    RealsenseCamera camera;
    PointCloudFilter pcFilter(removalOptions, leafSize);
    FloorRemover planesRemover(ransacOptions);
    PoseEstimator detector(clusteringOptions);

    Visualizer vis = Visualizer("Cloud", 3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

#ifdef RUN_WITH_CAMERA
    camera.Run();
    pointCloud = camera.GetData();
#else
    pcl::io::loadPCDFile<pcl::PointXYZ> ("test_image.pcd", *pointCloud);
#endif

    pcFilter.DownSample(pointCloud);
    pointCloud = planesRemover.RemoveFloor(pointCloud);
    pcFilter.RemoveOutliers(pointCloud);

    auto clusters = detector.GetClusters(pointCloud);
    auto transformations = detector.DefineTransformations();

#ifdef RUN_WITH_CAMERA
        camera.Stop();
#endif

    vis.UpdateCloud(pointCloud);
    vis.AddRotations(transformations);

    while(true)
    {
        vis.SpinOnce();
    }

    return 0;
}
