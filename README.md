# PoseEstimationPCA
## Motivation
The aim of this project is to try to apply Principal Component Analysis (PCA) for sloving pose estimation problem on point clouds.

## Algoritm
1) Get the depth frame from camera (I use intel realsense d415, where RGB channel is absent)
2) Convert depth frame to point cloud
3) Remove floor
4) Cluster the resulting point cloud
5) Compute PCA and find centroid for each cluster
6) Combite rotation from PCA and translation from centriod togehter to get the result transformation

## Result
