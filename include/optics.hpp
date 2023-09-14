//
// Created by scaactk on 8/26/2023.
//
#ifndef PCL_TEST_OPTICS_HPP
#define PCL_TEST_OPTICS_HPP

#include "PCL_TEST_HEADER.h"
#include <vector>
#include "iostream"

#endif


static inline void sortIdxDist(std::vector<int> &pointIdxRadiusSearch, std::vector<float> &pointRadiusSquaredDistance) {
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

bool findCluster(MyPointCloud &cloud, const ::pcl::KdTreeFLANN<PointT> &treeFlann, std::vector<bool> &processed,
                 std::vector<int> &ordered_sequence, std::vector<float> &ordered_distance, int query, float eps,
                 int min) {

    // radiusSearch result include query point itself
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance; // 平方和
    if (treeFlann.radiusSearch(query, eps, pointIdxRadiusSearch, pointRadiusSquaredDistance, 0) <= min) {
        // find noise
        // 噪声点如何定义 reachable distance?
        ordered_sequence.push_back(query);
        ordered_distance[query] = eps*eps;
        processed[query] = true;
        return false;
    } else {
        // core_points, set core_dist and reachable_dist
        sortIdxDist(pointIdxRadiusSearch, pointRadiusSquaredDistance); // 按dist升序排列
        //std::cout<<pointRadiusSquaredDistance[min]<<std::endl;
        float core_dist = pointRadiusSquaredDistance[min];
        for (int i = 0; i < pointIdxRadiusSearch.size(); i++) {
            if (pointRadiusSquaredDistance[i] <= core_dist) { //核心圈内，自身也算
                // std::cout<<"aaa"<<std::endl;
                ordered_distance[pointIdxRadiusSearch[i]] = core_dist;
            } else { // 当前核心的最小核心圈外
                //std::cout<<"bbb"<<std::endl;
                ordered_distance[pointIdxRadiusSearch[i]] = pointRadiusSquaredDistance[i];
            }
        }

        ordered_sequence.push_back(query);
        processed[query] = true;
        // delete current queried point
        // erase delete elements by location or range of location, such as the pointIdxRadiusSearch.end()
        // remove do the move action, move selected number to the end, others will be ahead
        pointIdxRadiusSearch.erase(std::remove(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end(), query),
                                   pointIdxRadiusSearch.end());

        // find neighbourhood
        while (!pointIdxRadiusSearch.empty()) {
            int ptr = pointIdxRadiusSearch.front();
            std::vector<int> temp_idx;
            std::vector<float> temp_dist;

            // 无论当前最近的点是不是核心点，都输出到ordered_sequence中
            // 但是要check这个点是不是在之前的一次 "大搜索->findCluster外的循环" 中已经被处理了
            if (processed[ptr] == true) {
                pointIdxRadiusSearch.erase(std::remove(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end(), ptr),
                                           pointIdxRadiusSearch.end());
            } else {
                if (treeFlann.radiusSearch(ptr, eps, temp_idx, temp_dist, 0) > min) {
                    sortIdxDist(temp_idx, temp_dist); // 可能包含已经processed的点
                    core_dist = temp_dist[min];
                    for (int i = 0; i < temp_idx.size(); i++) { // 自身也得改,可能是新的密度中心， i=0开始
                        float new_dist = std::max(core_dist, temp_dist[i]);
                        if (processed[temp_idx[i]] == false) {
                            if (ordered_distance[temp_idx[i]] == 0) { // new neighbour
                                ordered_distance[temp_idx[i]] = new_dist;
                                pointIdxRadiusSearch.push_back(i); // add to check list 只插入idx 会不会导致idx和dist不一样长？
                            } else { //old point
                                // 保留较小的reachable_dist
                                // 这里的写法，如果是已经输出到ordered_sequence的点，reachable_dist也会被修改
                                ordered_distance[temp_idx[i]] = std::min(ordered_distance[temp_idx[i]], new_dist);
                            }
                        }
                    }
                }
                else{ // cluster 的边界点
                    ordered_distance[ptr] = eps*eps;
                }

                ordered_sequence.push_back(ptr);
                processed[ptr] = true;
                pointIdxRadiusSearch.erase(std::remove(pointIdxRadiusSearch.begin(), pointIdxRadiusSearch.end(), ptr),
                                           pointIdxRadiusSearch.end());
            }

        }


    }
    return true;
}

static int extract_id(MyPointCloud &cloud, const std::vector<int> &ordered_sequence, const std::vector<float> &result_distance, float filter){
    int clusterID = 0;
    bool pre = true;
    bool cur = true;
    for (int i=0; i<result_distance.size(); i++){
        if (result_distance[i] < filter){
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

static bool write2file(const std::string& folderPath, const std::vector<int> &ordered_sequence, const std::vector<float> &result_distance){
    std::ofstream file(folderPath+"/output.csv");
    if (!file.is_open()) {
        std::cout << "Unable to open file";
        return false;
    }

    for (size_t i = 0; i < ordered_sequence.size(); ++i) {
        // std::sqrt(result_distance[i])
        file << ordered_sequence[i] << "," << result_distance[i] << "\n";
    }

    file.close();
    return true;
}

int optics(MyPointCloud &cloud, const pcl::KdTreeFLANN<PointT> &treeFlann, float eps, int min) {

    //生成容器时，同时指定元素个数，就能够减少空间额外分配的次数
//    std::vector<float> core_dists(cloud.size());
//    std::vector<float> reach_dists(cloud.size());
    std::vector<bool> processed(cloud.size(), false);
    std::vector<int> ordered_sequence; // 按输出顺序排列
    std::vector<float> ordered_distance(cloud.size(), 0.0); // 从0-n排列

    for (int i = 0; i < cloud.size(); i++) {
        if (i % 100 == 0) {
            std::cout << "i is " << i << std::endl;
        }

        if (processed[i] == false) {
            findCluster(cloud, treeFlann, processed, ordered_sequence, ordered_distance, i, eps, min);
        }
    }
//    // check size
//    std::cout << cloud.size() << std::endl;
//    std::cout << ordered_sequence.size() << std::endl;
//    int non_zero_count = std::count_if(ordered_distance.begin(), ordered_distance.end(),
//                                       [](float value) { return value != 0.0f; });
//    std::cout << non_zero_count << std::endl;
    std::vector<float> result_distance;
    for (int i : ordered_sequence){
        result_distance.push_back(std::sqrt(ordered_distance[i]));
    }

    // 赋值id
    int clusterNumber = extract_id(cloud, ordered_sequence, result_distance, 50);

    std::string folderPath = "C:\\Users\\tjut_\\Desktop\\20230818_VLP samples";
    write2file(folderPath, ordered_sequence, result_distance);

    return clusterNumber;
}
