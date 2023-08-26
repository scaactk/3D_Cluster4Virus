//
// Created by scaactk on 8/24/2023.
//

#ifndef PCL_TEST_DBSCAN_KDTREE_HPP
#define PCL_TEST_DBSCAN_KDTREE_HPP


#include "PCL_TEST_HEADER.h"
#include <cmath>
#include "algorithm"
#include "vector"
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#endif //PCL_TEST_DBSCAN_KDTREE_HPP


bool expand_cluster(MyPointCloud &cloud, const pcl::KdTreeFLANN<PointT> &treeFlann, size_t query, size_t clusterID,
                    float_t eps, int min, std::array<uint8_t, 3> color) {

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float_t> pointRadiusSquaredDistance;

    // include query point itself
    if (treeFlann.radiusSearch(query, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0) < min) {
        // find noise
        cloud.points[query].clusterID = -1;
        return false;
    }
    else {
        // set cluster id
        for (int i: pointIdxRadiusSearch) {
            cloud.points[i].clusterID = clusterID;
            cloud.points[i].r = color[0];
            cloud.points[i].g = color[1];
            cloud.points[i].b = color[2];
        }
        // delete current queried point
        pointIdxRadiusSearch.erase(std::remove(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end(), query),
                                   pointIdxRadiusSearch.end());

        // check other points which have been added in this cluster
        while (!pointIdxRadiusSearch.empty()) {
            size_t ptr = pointIdxRadiusSearch.front();
            std::vector<int> temp_idx;
            std::vector<float_t> temp_dis;
            if (treeFlann.radiusSearch(ptr, eps, temp_idx, temp_dis, 0) > min) {
                for (int i : temp_idx){
                    // find new required point
                    // -1 means noise before
                    if (cloud.points[i].clusterID==-1){
                        // do not need to check whether this point has neighbour again
                        cloud.points[i].clusterID = clusterID; // border point
                        cloud.points[i].r = color[0];
                        cloud.points[i].g = color[1];
                        cloud.points[i].b = color[2];
                    }

                    // 0 means new point as default
                    else if(cloud[i].clusterID==0){
                        // add to local_cluster check list
                        pointIdxRadiusSearch.push_back(i); // center point
                        cloud.points[i].clusterID = clusterID;
                        cloud.points[i].r = color[0];
                        cloud.points[i].g = color[1];
                        cloud.points[i].b = color[2];
                    }
                    else{ // already in local_cluster check list
                        continue;
                    }
                }
            }
            // remove front ,next will be the new front
            pointIdxRadiusSearch.erase(std::remove(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end(), ptr), pointIdxRadiusSearch.end());

        }
    }
    return true;
}

// kdtree.radiusSearch(searchPoint1, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)
size_t dbscan_kdtree(MyPointCloud &cloud, const pcl::KdTreeFLANN<PointT> &treeFlann, float_t eps, int min) {
    size_t clusterID = 1;
    std::array<uint8_t, 3> color{};
    generate_color(color);

    for (size_t i = 0; i < cloud.size(); i++) {
        if (i % 100 == 0) {
            std::cout << "i is " << i << std::endl;
        }

        if (cloud.points[i].clusterID == 0) {
            if (expand_cluster(cloud, treeFlann, i, clusterID, eps, min, color)) {
                clusterID++;
                generate_color(color);
            }
        }
    }
    return clusterID -1;
}


