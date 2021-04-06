cmake_minimum_required(VERSION 3.1.0)

project(PoseEstimationPCA)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable (PoseEstimationPCA
        main.cpp
        src/IO/RealsenseCamera.cpp
        src/IO/RealsenseCamera.h
        src/VisualizingTools/Visualizer.h
        src/VisualizingTools/Visualizer.cpp
        src/Processing/FrameConverter.h
        src/Processing/FrameConverter.cpp
        src/Processing/Filters/DepthFilter.h
        src/Processing/Filters/DepthFilter.cpp
        src/Processing/Filters/PointCloudFilter.h
        src/Processing/Filters/PointCloudFilter.cpp
        src/Utils/Utils.h
        src/Utils/Utils.cpp
        src/Pipeline/FloorRemover.cpp
        src/Pipeline/PoseEstimator.h
        src/Pipeline/PoseEstimator.cpp)

find_package(realsense2 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS io common visualization filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set_property(TARGET PoseEstimationPCA PROPERTY CXX_STANDARD 14)
target_link_libraries(PoseEstimationPCA ${realsense2_LIBRARY} ${PCL_LIBRARIES} pcl_sample_consensus pcl_segmentation)
set_target_properties (PoseEstimationPCA PROPERTIES FOLDER "Examples")

install(TARGETS PoseEstimationPCA RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})