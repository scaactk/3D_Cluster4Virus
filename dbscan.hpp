#include "PCL_TEST_HEADER.h"
#include <cmath>
#include "algorithm"
#include "vector"

const static inline float_t distance(float_t x1, float_t y1, float_t z1, float_t x2, float_t y2, float_t z2) {
    float_t dx = x2 - x1;
    float_t dy = y2 - y1;
    float_t dz = z2 - z1;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

const inline size_t region_query(const MyPointCloud &cloud, size_t query, std::vector<size_t> &local_cluster, float_t eps) {
    for (size_t i = 0; i < cloud.size(); i++) {
        if (distance(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
                     cloud.points[query].x, cloud.points[query].y, cloud.points[query].z) < eps) {
            local_cluster.push_back(i); // index
        }
    }
    return local_cluster.size();
}

bool expand_cluster(MyPointCloud &cloud, size_t query, int clusterID, float_t eps, int min) {
    std::vector<size_t> local_cluster;

    size_t local_cluster_size = region_query(cloud, query, local_cluster, eps);
    if (local_cluster_size < min) {
        // find noise
        cloud.points[query].clusterID = -1;
        return false;
    }
    else {
        // set cluster id
        for (size_t i = 0; i < local_cluster.size(); i++) {
            cloud[local_cluster[i]].clusterID = clusterID;
        }
        // delete current queried point
        local_cluster.erase(std::remove(local_cluster.begin(), local_cluster.end(), query), local_cluster.end());

        // check other points which have been added in this cluster
        while (!local_cluster.empty()) {
//            std::cout<< "query is "<< query << std::endl;
//            std::cout<< local_cluster.size()<< std::endl;
            size_t ptr = local_cluster.front();
            std::vector<size_t> temp_cluster;

            // if new temp_cluster size of current point from local_cluster > min ,,,
            // is this needed? if the current point is on the border
            // answer: yes, this is the requirement of dbscan, the algorithm is defined as this,
            // each point's density is high enough
            if (region_query(cloud, ptr, temp_cluster, eps)> min){
                for (size_t i=0; i<temp_cluster.size(); i++){
                    // find new required point
                    // -1 means noise before
                    if (cloud[temp_cluster[i]].clusterID==-1){
                        // do not need to check whether this point has neighbour again
                        //local_cluster.push_back(temp_cluster[i]);
                        cloud.points[temp_cluster[i]].clusterID = clusterID; // border point
                    }
                    // 0 means new point as default
                    else if(cloud[temp_cluster[i]].clusterID==0){
                        // add to local_cluster check list
                        local_cluster.push_back(temp_cluster[i]); // center point
                        cloud.points[temp_cluster[i]].clusterID = clusterID;
                    }
                    else{ // already in local_cluster check list
                        continue;
                    }
                }
            }
            // remove front ,next will be the new front
            local_cluster.erase(std::remove(local_cluster.begin(), local_cluster.end(), ptr), local_cluster.end());

        }

    }

    return true;
}


size_t dbscan(MyPointCloud & cloud, float_t eps, int min) {
    int clusterID = 1;

    //std::vector<int> output(size); // if only given size, all element for int vector is 0
    for (size_t i = 0; i < cloud.size(); i++) {
        if(i%100==0){
            std::cout<< "i is "<< i << std::endl;
        }

        if (cloud.points[i].clusterID==0) {
            if (expand_cluster(cloud, i, clusterID, eps, min)) {
                clusterID++;
            }
        }
    }
    return clusterID - 1;
}

//int main(){
//    // test
//    std::vector<Point> points(10);
//
//    points[0].x = 20; points[0].y = 21;
//    points[1].x = 20; points[1].y = 25;
//    points[2].x = 28; points[2].y = 22;
//    points[3].x = 30; points[3].y = 52;
//    points[4].x = 26; points[4].y = 70;
//    points[5].x = 30; points[5].y = 75;
//    points[6].x = 0; points[6].y = 70;
//    points[7].x = 70; points[7].y = 50;
//    points[8].x = 67; points[8].y = 69;
//    points[9].x = 80; points[9].y = 35;
//
//    std::vector<int> labels;
//
//    int num = dbscan(points, labels, 20.0, 3);
//
//    std::cout<< "cluster size is "<< num << std::endl;
//
//    for(int i = 0; i < (int)points.size(); i++){
//        std::cout<<"Point("<<points[i].x<<", "<<points[i].y<<"): "<<labels[i]<<std::endl;
//    }
//
//    return 0;
//}
