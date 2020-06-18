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
*  This is implementation file for procesPointClouds.h
*/
#include "processPointClouds.h"
#include "KdTree.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr filteredCloud {new pcl::PointCloud<PointT>};

    pcl::VoxelGrid<PointT> voxGrid;
    voxGrid.setInputCloud(cloud);
    voxGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxGrid.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    pcl::CropBox<PointT> region{true};
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roofRegion(true);
    roofRegion.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roofRegion.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roofRegion.setInputCloud(cloudRegion);
    roofRegion.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
    typename pcl::PointCloud<PointT>::Ptr obsCloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr roadCloud {new pcl::PointCloud<PointT>};
    typename pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*roadCloud);

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obsCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    typename pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size() == 0 ) {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ece;
    ece.setClusterTolerance(clusterTolerance);
    ece.setMinClusterSize(minSize);
    ece.setMaxClusterSize(maxSize);
    ece.setSearchMethod(tree);
    ece.setInputCloud(cloud);
    ece.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster {new pcl::PointCloud<PointT>};

        for (int index : getIndices.indices)
            cloud_cluster->points.push_back (cloud->points[index]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in ascending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


/* ------------------------------------------------------------------------------- */
/* ---> Lidar Obstacle detection custom methods for segmentation & clustering <--- */
/* ------------------------------------------------------------------------------- */

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::CustomRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    auto startTime = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--){
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand()%(cloud->points.size()));

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;

        itr ++;

        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;

        itr ++;

        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
        float d = -(a*x1 + b*y1 + c*z1);

        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index)>0)
                continue;

            auto point = cloud->points[index];
            float x = point.x;
            float y = point.y;
            float z = point.z;
            float dist = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);

            if (dist <= distanceTol)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    if (inliersResult.size() == 0 )
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC 3d took: " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    //RANSAC3D
    std::unordered_set<int> inliers = CustomRansac3D(cloud, maxIterations, distanceThreshold);

    //Segmentation
    typename pcl::PointCloud<PointT>::Ptr cloudInliers{new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers{new pcl::PointCloud<PointT>};

    for(int index = 0; index < cloud->points.size(); index++) {
        auto point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult {cloudInliers, cloudOutliers};

    return segResult;
}

//
//template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {
//
//    // Time clustering process
//    auto startTime = std::chrono::steady_clock::now();
//
//    KdTree<PointT>* tree {new KdTree<PointT>};
//
//    for (int i=0; i<cloud->points.size(); i++)
//        tree->insert(cloud->points[i], i);
//
//    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = EuclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
//
//    auto endTime = std::chrono::steady_clock::now();
//    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
//
//    return clusters;
//}
//

//template<typename PointT>
//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize) {
//
//    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
//    std::vector<bool> processed(cloud->points.size(), false);
//
//    for (int i=0; i<cloud->points.size(); ++i) {
//        if (!processed[i]){
//            typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};
//            Proximity(i, cloud, cluster, processed, tree, distanceTol);
//
//            if(cluster->points.size() >= minSize && cluster->points.size() <= maxSize) {
//                cluster->width = cluster->points.size();
//                cluster->height = 1;
//                cluster->is_dense = true;
//                clusters.push_back(cluster);
//            }
//        }
//    }
//    return clusters;
//
//}
//
//
//template<typename PointT>
//void ProcessPointClouds<PointT>::Proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol) {
//    processed[index] = true;
//    PointT point = cloud->points[index];
//    cluster->points.push_back(point);
//
//    std::vector<int> nearbys = tree->search(point, distanceTol);
//
//    for (int id : nearbys) {
//        if (!processed[id])
//            Proximity(id, cloud, cluster, processed, tree, distanceTol);
//    }
//}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusteringResults;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    KdTree* tree = new KdTree;

    std::vector<float> point;
    std::vector<std::vector<float>> pointCloud;

    for (int i=0; i<cloud->points.size(); i++)
    {
        std::vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        pointCloud.push_back(point);
        tree->insert(point, i);

    }

    std::vector<std::vector<int>> clusters = euclideanCluster(pointCloud, tree, clusterTolerance);

    for (auto clusterIDs : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster {new pcl::PointCloud<PointT>};

        for (int index : clusterIDs)
            cloud_cluster->points.push_back (cloud->points[index]);

        if (cloud_cluster->points.size()>=minSize && cloud_cluster->points.size()<=maxSize)
        {
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusteringResults.push_back(cloud_cluster);
        }

    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusteringResults;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, std::vector<int>& cluster,const std::vector<std::vector<float>> points, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{

    processed[index] = true;
    cluster.push_back(index);
    std::vector<int> nearbys = tree->search(points[index], distanceTol);

    for (int id : nearbys)
    {
        if (!processed[id])
            proximity(id, cluster, points, processed, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed(points.size(), false);

    for (int i=0; i<points.size(); ++i)
    {
        if (processed[i])
            continue;
        std::vector<int> cluster;
        proximity(i,cluster, points, processed, tree, distanceTol);
        clusters.push_back(cluster);
    }

    return clusters;

}