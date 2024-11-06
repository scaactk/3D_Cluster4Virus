#ifndef PCL_TEST_CLUSTER_PROCESSOR_HPP
#define PCL_TEST_CLUSTER_PROCESSOR_HPP

#include "PCL_TEST_HEADER.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <map>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Alpha_shape_vertex_base_3<K> Vb;
typedef CGAL::Alpha_shape_cell_base_3<K> Fb;
typedef CGAL::Triangulation_data_structure_3<Vb,Fb> Tds;
typedef CGAL::Delaunay_triangulation_3<K,Tds> Triangulation_3;
typedef CGAL::Alpha_shape_3<Triangulation_3> Alpha_shape_3;
typedef K::Point_3 Point;

struct ClusterMetrics {
    int cluster_id;
    int point_count;
    float volume;
    float density;
    PointT centroid;
    std::array<uint8_t, 3> color;  // 存储cluster的颜色
};

class ClusterProcessor {
public:
    static int process_clusters(MyPointCloud &cloud, 
                              const std::vector<int> &ordered_sequence, 
                              const std::vector<float> &output_dist, 
                              float filter) {
        // 给点分配cluster ID
        int clusterNumber = assign_cluster_ids(cloud, ordered_sequence, output_dist, filter);
        
        // 计算每个cluster的指标
        auto metrics = calculate_metrics(cloud);
        
        // 根据metrics中存储的颜色给点着色
        apply_colors(cloud, metrics);
        
        // 保存结果
        save_results(cloud, metrics);
        
        return clusterNumber;
    }

    static bool load_processed_cloud(MyPointCloud &cloud, const std::string &filepath) {
        if (pcl::io::loadPCDFile(filepath, cloud) == -1) {
            return false;
        }
        return true;
    }

private:
    static int assign_cluster_ids(MyPointCloud &cloud, 
                                const std::vector<int> &ordered_sequence, 
                                const std::vector<float> &output_dist, 
                                float filter) {
        int clusterID_Index = 0;
        bool pre = true;
        bool cur = true;
        
        for (int i = 0; i < output_dist.size(); i++) {
            if (output_dist[i] < filter) {
                cur = true;
                if (cur != pre) clusterID_Index += 1;
                if (i == 0) clusterID_Index += 1;
                cloud.points[ordered_sequence[i]].clusterID = clusterID_Index;
                pre = cur;
            } else {
                cur = false;
                cloud.points[ordered_sequence[i]].clusterID = -1;
                pre = cur;
            }
        }
        return clusterID_Index;
    }

    static std::vector<ClusterMetrics> calculate_metrics(const MyPointCloud &cloud) {
        std::vector<ClusterMetrics> metrics;
        std::map<int, std::vector<int>> cluster_points;
        
        // 按cluster ID分组
        for (int i = 0; i < cloud.points.size(); i++) {
            if (cloud.points[i].clusterID > 0) {
                cluster_points[cloud.points[i].clusterID].push_back(i);
            }
        }
        
        // 计算每个cluster的指标
        // cluster set: {id:{points}}
        for (const auto& cluster : cluster_points) {
            ClusterMetrics metric;
            metric.cluster_id = cluster.first;
            metric.point_count = cluster.second.size();

            // 生成随机颜色
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(50, 256);
            metric.color = {static_cast<uint8_t>(dis(gen)),
                          static_cast<uint8_t>(dis(gen)),
                          static_cast<uint8_t>(dis(gen))};

            // 计算质心
            for (int idx : cluster.second) {
                metric.centroid.x += cloud.points[idx].x;
                metric.centroid.y += cloud.points[idx].y;
                metric.centroid.z += cloud.points[idx].z;
            }
            metric.centroid.x /= metric.point_count;
            metric.centroid.y /= metric.point_count;
            metric.centroid.z /= metric.point_count;

            // 计算体积（使用Alpha Shape）
            std::vector<Point> cgal_points;
            for (int idx : cluster.second) {
                cgal_points.push_back(Point(
                    cloud.points[idx].x,
                    cloud.points[idx].y,
                    cloud.points[idx].z
                ));
            }

            Alpha_shape_3 as(cgal_points.begin(), cgal_points.end());
            float optimal_alpha = *as.find_optimal_alpha(1);
            as.set_alpha(optimal_alpha);
            metric.volume = as.alpha_volume();
            metric.density = metric.point_count / metric.volume;

            metrics.push_back(metric);
        }
        
        return metrics;
    }

    static void apply_colors(MyPointCloud &cloud, const std::vector<ClusterMetrics> &metrics) {
        // 先将所有点设为灰色（噪声点）
        for (auto& point : cloud.points) {
            if (point.clusterID < 0) {
                point.r = point.g = point.b = 50;
                point.a = 50;
            }
        }
        
        // 给每个cluster的点着色
        for (const auto& metric : metrics) {
            for (auto& point : cloud.points) {
                if (point.clusterID == metric.cluster_id) {
                    point.r = metric.color[0];
                    point.g = metric.color[1];
                    point.b = metric.color[2];
                }
            }
        }
    }

    static void save_results(const MyPointCloud &cloud, 
                           const std::vector<ClusterMetrics> &metrics) {
        // 保存点云
        pcl::io::savePCDFile("clustered_cloud.pcd", cloud);
        
        // 保存指标
        std::ofstream metrics_file("cluster_metrics.csv");
        metrics_file << "Cluster ID,Point Count,Volume,Density,Centroid X,Centroid Y,Centroid Z\n";
        for (const auto& metric : metrics) {
            metrics_file << metric.cluster_id << ","
                        << metric.point_count << ","
                        << metric.volume << ","
                        << metric.density << ","
                        << metric.centroid.x << ","
                        << metric.centroid.y << ","
                        << metric.centroid.z << "\n";
        }
        metrics_file.close();
    }
};

#endif //PCL_TEST_CLUSTER_PROCESSOR_HPP 