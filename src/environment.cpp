/**
* @file landbasedrobot.cpp
* @authors
* Sudharsan
*
* @version 1.0
*
* @section LICENSE
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* @section DESCRIPTION
*
*  This is implementation file for environment.cpp
*/

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer){

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

void streamerInbuilt(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputPCD, bool render_cluster, bool render_box)
{
    /* -------------------------------------------------- */
    /* ---> Open 3D viewer and display simple highway<--- */
    /* -------------------------------------------------- */

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputPCD, 0.2, Eigen::Vector4f (-10, -5, -3, 1), Eigen::Vector4f (30, 7, 1, 1));

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 10, 0.18);

    renderPointCloud(viewer, segmentCloud.first, "obstacles", Color( 1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "road", Color(1, 1, 1));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.3, 50, 1500);
    std::cout<<"Clustering Ok!!"<<std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 0, 1), Color(0, 1, 0), Color(0, 1, 1), Color(1, 0, 1), Color(1, 1, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (render_cluster){
            std::cout<<"Cluster size: ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacle Cloud"+std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box){
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++ clusterId;
    }
}

template<typename PointT>
void streamerCustom(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<PointT>* pointProcessorI,  const typename pcl::PointCloud<PointT>::Ptr& inputPCD, bool render_cluster, bool render_box)
{
    /* ---------------------------------------------------------------------------- */
    /* ---> Lidar Obstacle detection custom code for segmentation & clustering <--- */
    /* ---------------------------------------------------------------------------- */

    //Down-sampling using filter function
    typename pcl::PointCloud<PointT>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputPCD, 0.2, Eigen::Vector4f (-20, -5, -3, 1), Eigen::Vector4f (30, 7, 1, 1));

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentCloud = pointProcessorI->CustomSegmentation(filteredCloud, 35, 0.18);

    renderPointCloud(viewer, segmentCloud.first, "Road", Color( 0, 1, 0));
//    renderPointCloud(viewer, segmentCloud.second, "Obstacles", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->CustomClustering(segmentCloud.second, 0.45, 10, 800);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 0, 1), Color(0, 1, 0), Color(0, 1, 1), Color(1, 0, 1), Color(1, 1, 1)};

    for (auto cluster : cloudClusters)
    {
        if (render_cluster){
            std::cout<<"Cluster size: ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstacle Cloud"+std::to_string(clusterId), colors[clusterId]);
        }
        if (render_box){
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++ clusterId;
    }
}

int main (int argc, char** argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI{new ProcessPointClouds<pcl::PointXYZI>};
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputPCDI;

    while (!viewer->wasStopped ())
    {
        //clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load pcd and run obstacle detection process
        inputPCDI = pointProcessorI->loadPcd((*streamIterator).string());
//        streamerInbuilt(viewer, pointProcessorI, inputPCDI, true, true);
        streamerCustom(viewer, pointProcessorI, inputPCDI, true, true);

        if (++ streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}