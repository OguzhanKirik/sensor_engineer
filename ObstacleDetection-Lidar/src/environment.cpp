/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include<memory>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    double groundSlope= 0.0;
    std::unique_ptr<Lidar> lidarSensor = std::make_unique<Lidar>(cars,groundSlope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidarSensor->scan();
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud =
        pointProcessor.FilterCloud(cloud, 0.2f, Eigen::Vector4f(-10, -6, -2, 1), Eigen::Vector4f(30, 6, 3, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
        pointProcessor.SegmentPlane(filteredCloud, 50, 0.2f);

    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
        pointProcessor.Clustering(segmentCloud.first, 1.0f, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        renderBox(viewer, pointProcessor.BoundingBox(cluster), clusterId);
        ++clusterId;
    }
  
}
/*****************************************************************/
/************ use cityBlock to test on real Lidar data *******/
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // hyperparameters
  // filter params
  float filterRes = 0.4;
  Eigen::Vector4f minPoint(-10, -6.5, -2, 1);
  Eigen::Vector4f maxPoint(30, 6.5, 1, 1);
  // segment params
  int maxIter = 40;
  float distanceThreshold = 0.3;
  // cluster params
  float clusterTolerance = 0.5;
  int minClusterSize = 10;
  int maxClusterSize = 140;

  // Filter cloud, to reduce omputational cost
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);

  // Step 1. Segment the filtered cloud into two parts, road and obstacles.
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
      pointProcessorI->SegmentPlane(filterCloud, maxIter, distanceThreshold);

// renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  // Step 2. Cluster the obstacle cloud.
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minClusterSize, maxClusterSize);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

        // Step 3. Find bounding boxes for the clusters
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
  }

}
/*****************************************************************/


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
    bool useCityBlock = true;
    CameraAngle setAngle = useCityBlock ? FPS : XY;
    initCamera(setAngle, viewer);

    if (!useCityBlock)
    {
        simpleHighway(viewer);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        }
        return 0;
    }

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        ++streamIterator;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}
