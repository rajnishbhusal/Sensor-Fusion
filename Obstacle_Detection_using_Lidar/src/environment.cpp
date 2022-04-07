/* \author Rajnish Bhusal*/
// Obstacle Detection for Self Driving Cars using Lidar and point cloud data from PCL
// Filtering: Voxel 
// Segmentation: Ransac Algorithm
// Clustering: Euclidean (kd-tree based searching)

#include "render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "myOwnProcessPointClouds.h"
#include "myOwnProcessPointClouds.cpp"


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, myOwnProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    bool render_pointCloud = false; // to view pointclouds
    bool render_obst = false; // to view obstacle shapes
    bool render_plane = true; // to view road
    bool render_clusters = true; // to view different colors of clusters
    bool render_box = true; // to view blocks or boxes of clusters

    // ----------------------------------------------------
    // -----            Filtering           -----
    // ----------------------------------------------------

    // steps for filtering using filtercloud in processPointClouds.cpp
    // filter resolution = 0.3 m (how close points do you want)
    // Eigen::Vector4f(-10, -5, -3, 1)----> min crop box relative to the origin (backwards, right, up)
    // Eigen::Vector4f(40, 6, 10, 1) ---> max crop box relative to the origin (frontwards, left, down)
    // 

    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5.5, -2.5, 1), Eigen::Vector4f(35, 5.5, 1, 1));

    if (render_pointCloud){
        renderPointCloud(viewer, inputCloud, "inputCloud"); // use renderPointCloud if you want to see the point clouds only.
    }

    // ----------------------------------------------------
    // -----        Ransac  Segmentation          -----
    // ----------------------------------------------------

    // SegmentPlane(cloud, int maxIterations, float distanceThreshold)
    // SegmentPlane(inputCloud, 100, 0.2) 
    
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.myOwnRansac3d(inputCloud, 25, 0.3);

    if (render_obst){
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }

    if (render_plane){
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }

    // ----------------------------------------------------
    // -----           Euclidean Clustering          -----
    // ----------------------------------------------------

    // segmentCloud.first = obstacle segment
    // minSize = minimum number of points for forming a cluster
    // maxSize = maximum number of points for forming a cluster
    // Clustering(cloud, float clusterTolerance, int minSize, int maxSize)
    // Clustering(segmentCloud.first, 0.5, 3, 650)
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.5, 10, 650);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
      if (render_clusters){
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
      }

      if (render_box){
          Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
      }
      
      ++clusterId;
    }

}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS; //XY
    initCamera(setAngle, viewer);

    // For streaming point clouds
    // Call City Block after defining a processor in stack and input cloud in main 
    // (use '.' to access functions for the processor since it is in stack)
    // (not on heap with new keyword which uses '->')

    // Create point cloud processor with (X,Y,Z,I)

    myOwnProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    // Import or Stream Data Files
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    // Declare a null pointer to the inputCloudI
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    // Run until the viewer is manually closed
    while (!viewer->wasStopped ())
    {   
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load PCD and run obstacle detection process using cityBlock
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}