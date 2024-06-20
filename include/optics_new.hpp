//
// Created by scaactk on 8/29/2023.
//

#ifndef PCL_TEST_OPTICS_NEW_HPP
#define PCL_TEST_OPTICS_NEW_HPP

#include "PCL_TEST_HEADER.h"
#include <vector>
#include "iostream"

#endif //PCL_TEST_OPTICS_NEW_HPP

static inline void
sortIdxDist_new(std::vector<int> &pointIdxRadiusSearch, std::vector<float> &pointRadiusSquaredDistance) {
    // 创建一个索引向量，用于排序, 保持pointIdx和pointDist的同步
    std::vector<int> idx(pointRadiusSquaredDistance.size());
    std::iota(idx.begin(), idx.end(), 0);

    // 使用自定义的比较函数进行排序
    std::sort(idx.begin(), idx.end(),
              [&pointRadiusSquaredDistance](int i1, int i2) {
                  return pointRadiusSquaredDistance[i1] < pointRadiusSquaredDistance[i2]; // 升序
              });

    // 使用排序后的索引向量来获取排序后的结果
    std::vector<int> sortedIndices(pointIdxRadiusSearch.size());
    std::vector<float> sortedDistances(pointRadiusSquaredDistance.size());
    for (int i = 0; i < idx.size(); ++i) {
        sortedIndices[i] = pointIdxRadiusSearch[idx[i]];
        sortedDistances[i] = pointRadiusSquaredDistance[idx[i]];
    }
    for (int i = 0; i < idx.size(); ++i) {
        pointIdxRadiusSearch[i] = sortedIndices[i];
        pointRadiusSquaredDistance[i] = sortedDistances[i];
    }
}

int update_new(std::vector<bool> &processed, std::vector<float> &reachability_distance, std::vector<int>& neighbour_idx,
           std::vector<float>& neighbour_dist, std::vector<int> &seeds_idx, int min) {
    float core_dist = neighbour_dist[min-1];
    for (int i=0; i < neighbour_idx.size(); i++) {
        if (processed[neighbour_idx[i]] == false) {
            float new_reach_dist = std::max(core_dist, neighbour_dist[i]);
            if (reachability_distance[neighbour_idx[i]] == 0) { // point is not in seeds
                reachability_distance[neighbour_idx[i]] = new_reach_dist;
                seeds_idx.push_back(neighbour_idx[i]);
            } else {
                reachability_distance[neighbour_idx[i]] = std::min(reachability_distance[neighbour_idx[i]],
                                                                   new_reach_dist);
            }
        }
    }
    //std::cout<<seeds_idx.size()<<endl;
    return seeds_idx.size();
}

static int extract_id_new(MyPointCloud &cloud, const std::vector<int> &ordered_sequence, const std::vector<float> &output_dist, float filter){
    int clusterID = 0;
    bool pre = true;
    bool cur = true;
    for (int i=0; i<output_dist.size(); i++){
        if (output_dist[i] < filter){
            cur = true;
            if (cur != pre){
                clusterID += 1;
            }
            if (i==0){
                clusterID += 1;
            }
            cloud.points[ordered_sequence[i]].clusterID = clusterID;
            pre = cur;
        }
        else{
            cur = false;
            cloud.points[ordered_sequence[i]].clusterID = -1; // noise point
            pre = cur;
        }
    }
    return clusterID;
}

static bool write2file_new(const std::string& folderPath, const std::vector<int> &ordered_sequence, const std::vector<float> &output_dist){
    std::ofstream file(folderPath+"/output.csv");
    if (!file.is_open()) {
        std::cout << "Unable to open file";
        return false;
    }

    for (size_t i = 0; i < ordered_sequence.size(); ++i) {
        // std::sqrt(result_distance[i])
        file << ordered_sequence[i] << "," << output_dist[i] << "\n";
    }

    file.close();
    return true;
}

static bool write2file_idx_point(MyPointCloud &cloud, const std::string& folderPath, const std::vector<int> &ordered_sequence, const std::vector<float> &output_dist){
    std::ofstream file(folderPath+"/idx_point.csv");
    if(!file.is_open()){
        std::cout << "Unable to open file";
        return false;
    }

    for(size_t i=0; i < ordered_sequence.size(); ++i){
        file << ordered_sequence[i] << "," << output_dist[i] << ","
        << cloud.points[ordered_sequence[i]].x << ","
        << cloud.points[ordered_sequence[i]].y << ","
        << cloud.points[ordered_sequence[i]].z << "\n";
    }
}

std::tuple<std::vector<int>, std::vector<float>> optics_new(MyPointCloud &cloud, const pcl::KdTreeFLANN<PointT> &treeFlann, float eps, int min, const std::string& folderPath) {
    eps = eps * eps;
    std::vector<bool> processed(cloud.size(), false);
    std::vector<int> ordered_sequence;
    std::vector<float> output_dist(cloud.size(), 0);
    std::vector<float> reachability_distance(cloud.size(), 0);

    for (int query = 0; query < cloud.size(); query++) {
        if (query % 100 == 0) {
            std::cout << "query is " << query << std::endl;
        }
        std::vector<int> neighbour_idx;
        std::vector<float> neighbour_dist;
        if (processed[query] == false) {
            treeFlann.radiusSearch(query, eps, neighbour_idx, neighbour_dist);
            processed[query] = true;
            ordered_sequence.push_back(query);
            output_dist[query] = reachability_distance[query];
            if (neighbour_idx.size() >= min - 1) {
                sortIdxDist_new(neighbour_idx, neighbour_dist);
                std::vector<int> seeds_idx;
                update_new(processed, reachability_distance, neighbour_idx, neighbour_dist, seeds_idx, min);
                while (!seeds_idx.empty()) {
                    //std::cout << seeds_idx.size() << std::endl;
                    std::vector<int> temp_idx;
                    std::vector<float> temp_dist;
                    int ptr = seeds_idx.front();
                    treeFlann.radiusSearch(ptr, eps, temp_idx, temp_dist);
                    processed[ptr] = true;
                    ordered_sequence.push_back(ptr);
                    //std::cout << ordered_sequence.size() << std::endl;
                    output_dist[ptr] = reachability_distance[ptr];
                    seeds_idx.erase(std::remove(seeds_idx.begin(), seeds_idx.end(), ptr), seeds_idx.end());
                    if (temp_idx.size() >= min - 1) {
                        update_new(processed, reachability_distance, temp_idx, temp_dist, seeds_idx, min);
                        std::vector<float> seeds_dist;
                        for (int i: seeds_idx) {
                            seeds_dist.push_back(reachability_distance[i]);
                        }
                        sortIdxDist_new(seeds_idx, seeds_dist);
                        std::vector<float>().swap(seeds_dist); //clean
                        std::vector<int>().swap(temp_idx);
                        std::vector<float>().swap(temp_dist);
                    }
                }
            }
        }
    }
    std::vector<float> result_distance;
    for (int i : ordered_sequence){
        result_distance.push_back(std::sqrt(output_dist[i]));
    }
    std::cout << result_distance.size() << std::endl;

    // write2file_new(folderPath, ordered_sequence, result_distance);
    write2file_idx_point(cloud, folderPath, ordered_sequence, result_distance);
    // int cluster_number = extract_id_new(cloud, ordered_sequence, result_distance, 11);

    return std::make_tuple(ordered_sequence, result_distance);
}
