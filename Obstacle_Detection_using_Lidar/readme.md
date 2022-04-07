# Obstacle Detection using Lidar

This package provides utilities to process a sequence of lidar point cloud data using C++.

### Lidar and Point Cloud Data

Light Detection and Ranging (Lidar) is a remote sensing method. The lidar sensor emits thousands of laser rays at different angles, which get reflected off of obstacles and are detected using receivers.

Each lidar reflection forms a point and the set of all lidar reflections measured forms a point cloud.

The Lidar data is stored in a Point Cloud Data (PCD) format. Each .pcd file contains (x, y, z, I) data where x= x-coordinate, y= y-coordinate, z= z-coordinate, and I = laser intensity value obtained from the laser reflection off of obstacle.

### Processing of PCD

The processing of data involves:
(i) Filtering
(ii) Segmentation
(iii) Clustering

---
 
**Filtering**: Filtering is carried out to downsample the point cloud data. This package uses voxel grid filtering. 

**Segmentation**: Segementation is used to segment or separate the point cloud of obstacles from the point cloud corresponding to the ground plane. This package uses RANdom SAmple Consensus (RANSAC) algorithm to carry out segmentation

**Clustering**: Clustering involves forming clusters of obstacle point clouds corresponding to each obstacle. This package uses kd-tree (3d-tree) data structure out of the obstacle point cloud data, and searches for the closest neighbors within a tolerance using Euclidean Clustering algorithm.

### 
Library: PCL https://github.com/PointCloudLibrary/pcl
Forked from: https://github.com/udacity/SFND_Lidar_Obstacle_Detection

